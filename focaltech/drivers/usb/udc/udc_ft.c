/*
 * Copyright (c) 2025 Focaltech Systems CO.,Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * USB device controller (UDC) driver ft
 *
 * This is a ft driver for a device controller driver using the UDC API.
 * Please use it as a starting point for a driver implementation for your
 * USB device controller. Maintaining a common style, terminology and
 * abbreviations will allow us to speed up reviews and reduce maintenance.
 * Copy UDC driver ft, remove all unrelated comments and replace the
 * copyright notice with your own.
 *
 * Typically, a driver implementation contains only a single source file,
 * but the large list of e.g. register definitions should be in a separate
 * .h file.
 *
 * If you want to define a helper macro, check if there is something similar
 * in include/zephyr/sys/util.h or include/zephyr/usb/usb_ch9.h that you can use.
 * Please keep all identifiers and logging messages concise and clear.
 */

#include <soc.h>
#include <stdio.h>
#include <string.h>

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/kernel.h>

#include <zephyr/drivers/usb/udc.h>
#include <zephyr/dt-bindings/usb/usb.h>

#include <zephyr/drivers/reset.h>
#include <zephyr/sys/util.h>

#include <usb/udc/udc_common.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/pm/policy.h>
// LOG_MODULE_REGISTER(udc_ft, LOG_LEVEL_DBG);
LOG_MODULE_REGISTER(usb_device_init, LOG_LEVEL_ERR);
// LOG_MODULE_REGISTER(udc_ft, LOG_LEVEL_WRN);
// LOG_MODULE_REGISTER(udc_ft, LOG_LEVEL_INF);

#include "cpm_drv.h"
#include "usb_drv.h"

enum udc_ft_msg_type
{
    // FT_EVT_TRANSFER
    /* EP0 INTERRUPT */
    FT_UDC_MSG_TYPE_EP0,
    /* OUT transaction for specific EP completed */
    FT_UDC_MSG_TYPE_OUT,
    /* IN transaction for specific EP completed */
    FT_UDC_MSG_TYPE_IN,
    /* Re-activate queued transfer for specific EP */
    FT_UDC_MSG_TYPE_XFER,
    /*Do a fake in event */
    FT_UDC_MSG_TYPE_STATUS_IN,
    /* USB connect*/
    FT_UDC_MSG_TYPE_CONN,
    /* USB disconnect */
    FT_UDC_MSG_TYPE_DISCON,
    /* USB reset */
    FT_UDC_MSG_TYPE_RESET,
    /* USB suspend */
    FT_UDC_MSG_TYPE_SUSUPEND,
    /* USB resume */
    FT_UDC_MSG_TYPE_RESUME,
    /* S/W reconnect */
    FT_UDC_MSG_TYPE_SW_RECONN,
};

struct udc_ft_msg
{
    enum udc_ft_msg_type type;
    union {
        struct
        {
            uint8_t packet[8];
        } setup;
        struct
        {
            uint8_t ep;
        } out;
        struct
        {
            uint8_t ep;
        } in;
        struct
        {
            uint8_t ep;
        } xfer;
    };
};

/*
 * Structure for holding controller configuration items that can remain in
 * non-volatile memory. This is usually accessed as
 *   const struct ft_udc_config *config = dev->config;
 */
struct udc_ft_config
{
    FT_USBD_Type *base;
    size_t num_of_eps;
    struct udc_ep_config *ep_cfg_in;
    struct udc_ep_config *ep_cfg_out;
    struct reset_dt_spec reset;
    uint32_t clk_src;
    void (*irq_config_func)(const struct device *dev);
    uint32_t irq_num;
    void (*make_thread)(const struct device *dev);
    int speed_idx;
};

/*
 * Structure to hold driver private data.
 * Note that this is not accessible via dev->data, but as
 *   struct ft_udc_data *priv = udc_get_private(dev);
 */
struct udc_ft_data
{
    struct k_thread thread_data;
    struct k_msgq *msgq;
    struct k_mutex hal_mutex;
    uint32_t ctrlout_tailroom;
    uint8_t setup[8];
    uint16_t sof_num;
    uint8_t addr; /* Host assigned USB device address */
    uint8_t tailrom;
    bool enum_done;
#ifdef CONFIG_PM
    atomic_t pm_lock;
#endif
#ifdef CONFIG_UDC_FT_DMA
    struct k_sem status_sem;
#endif
};

#define DT_DRV_COMPAT ft_ft90_usb

/**
 * @brief usb clock handling
 */
static int udc_ft_clock_request(const struct device *dev, bool on)
{
    const struct udc_ft_config *cfg = dev->config;
    int ret;

    __ASSERT_NO_MSG(dev != NULL);

    /* enable clock for subsystem */
    const struct device *clk = DEVICE_DT_GET(DT_NODELABEL(rcc));

    if (on)
    {
        ret = clock_control_on(clk, (clock_control_subsys_t *)&cfg->clk_src);
    }
    else
    {
        ret = clock_control_off(clk, (clock_control_subsys_t *)&cfg->clk_src);
    }

    return ret;
}

#ifdef CONFIG_PM
static void udc_ft_pm_policy_lock_get(const struct device *dev)
{
    struct udc_ft_data *priv = udc_get_private(dev);
    if (atomic_test_and_set_bit(&priv->pm_lock, 0) == 0)
    {
	pm_device_runtime_get(dev);
	pm_policy_state_lock_get(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);
    }
}
static void udc_ft_pm_policy_lock_put(const struct device *dev)
{
    struct udc_ft_data *priv = udc_get_private(dev);

    if (atomic_test_and_clear_bit(&priv->pm_lock, 0) == 1)
    {	
	pm_device_runtime_put(dev);
        pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);
    }
}
#endif
/**
 * @brief USBC PHY
 *
 * @param[in] src_type
 *- CPM_USBPHY_AUTO_DET,
 *- CPM_USBPHY_INTER_OSC,
 *- CPM_USBPHY_EXTER_OSC,
 * @return @ref NONE
 */

void CPM_UsbPhyInit(uint8_t src_type)
{
    int time_out;
    // select the osc mode [31:30]

    CPM->CHIPCFGR = (CPM->CHIPCFGR & 0x3FFFFFFF) | ((src_type & 0x03) << 30);

    if (src_type != 0x02)
    {
        CPM->OSCESTIMER = (CPM->OSCESTIMER & 0xFFFF0000) | 0x3000;
        CPM->OCSR = CPM->OCSR | CPM_OCSR_OSCEXT_CLK_EN;
    }

    // Enable PHY Power Switch
    CPM->CHIPCFGR = CPM->CHIPCFGR | CPM_CHIPCFGR_USBPHY_POWER_SWITCH_EN;

    // delay at least 10ms, sys clk is 320MHz, in release obj, one clock_cycle is 6 clock
    // so delay(x) = (6 * x / 120)us = 0.05x us, we use 11ms here.so 11ms = (6 * x / 120)=>x=220000
    // DelayMS(11);
    k_sleep(K_MSEC(11));

    // Enable PHY Regulator
    // CCM->PHYPA &= ~0xFF;
    *(volatile uint16_t *)0x40001006 = (*(volatile uint16_t *)0x40001006) & 0xFF00;

    // delay at least 10us
    k_sleep(K_USEC(15));

    if (src_type != 0x02)
    {
        time_out = 200; // 2ms，
        while (time_out--)
        {
            if (CPM->OCSR & CPM_OCSR_OSCEXT_STABLE)
                break;
            k_sleep(K_USEC(10));
        }
    }

    // delay at least 10ms
    k_sleep(K_MSEC(11));

    CPM->CHIPCFGR = CPM->CHIPCFGR & ~CPM_CHIPCFGR_USBPHY_CONF_SOFTWARE_MASK;

    // delay at least 1ms
    k_sleep(K_MSEC(1));

    CPM->CHIPCFGR = CPM->CHIPCFGR & ~CPM_CHIPCFGR_USBPHY_PLL_SOFTWARE_MASK;

    // delay at least 1ms
    k_sleep(K_MSEC(1));

    CPM->CHIPCFGR = CPM->CHIPCFGR & ~CPM_CHIPCFGR_USBPHY_RESET_SIGNAL_MASK;

    // delay at least 1ms
    k_sleep(K_MSEC(1));
}

/**
 * @brief USBC PHY
 *
 * @param[in] NONE
 * @return @ref NONE
 */
void CPM_UsbPhyDeinit(void)
{
    // delay at least 1ms
    k_sleep(K_MSEC(1));

    // delay at least 10ms
    CPM->CHIPCFGR = CPM->CHIPCFGR | CPM_CHIPCFGR_USBPHY_RESET_SIGNAL_MASK;
    k_sleep(K_MSEC(1));

    // delay at least 1ms
    CPM->CHIPCFGR = CPM->CHIPCFGR | CPM_CHIPCFGR_USBPHY_PLL_SOFTWARE_MASK;
    k_sleep(K_MSEC(1));

    // delay at least 1ms
    CPM->CHIPCFGR = CPM->CHIPCFGR | CPM_CHIPCFGR_USBPHY_CONF_SOFTWARE_MASK;
    k_sleep(K_MSEC(10));

    // delay at least 10us
    k_sleep(K_USEC(15));

    // delay at least 10ms, sys clk is 320MHz, in release obj, one clock_cycle is 6 clock
    // so delay(x) = (6 * x / 120)us = 0.05x us, we use 11ms here.so 11ms = (6 * x / 120)=>x=220000
    k_sleep(K_MSEC(11));

    // Disable PHY Power Switch
    CPM->CHIPCFGR = CPM->CHIPCFGR & ~CPM_CHIPCFGR_USBPHY_POWER_SWITCH_EN; //_cpm_set_usbphy_power_switch_dis;
    k_sleep(K_MSEC(1));

    CPM->OCSR = CPM->OCSR & ~CPM_OCSR_OSCEXT_CLK_EN; //_cpm_set_oscext_clk_dis;
    k_sleep(K_MSEC(1));
}

int udc_ft_hal_lock(const struct device *dev)
{
    int err;
    struct udc_ft_data *priv = udc_get_private(dev);
    err = k_mutex_lock(&priv->hal_mutex, K_FOREVER);
    if (err)
    {
        LOG_ERR("hal_lock err :%x", err);
    }
    return err;
}

int udc_ft_hal_unlock(const struct device *dev)
{
    int err;
    struct udc_ft_data *priv = udc_get_private(dev);
    err = k_mutex_unlock(&priv->hal_mutex);
    if (err)
    {
        LOG_ERR("unlock err :%x", err);
    }

    return err;
}

/*
 *Power management callback to suspend or resume GPIO hardware
 *during system sleep or runtime idle
 */

static int udc_ft_pm_action(const struct device *dev, enum pm_device_action action)
{
    switch (action)
    {
    case PM_DEVICE_ACTION_RESUME:
        udc_ft_clock_request(dev, true);
        break;
    case PM_DEVICE_ACTION_SUSPEND:
        udc_ft_clock_request(dev, false);
        break;
    // case PM_DEVICE_ACTION_TURN_OFF:
    // case PM_DEVICE_ACTION_TURN_ON:
    //	break;
    default:
        return -ENOTSUP;
    }
    return 0;
}

static volatile uint8_t *udc_ft_get_fifo_ptr(FT_USBD_Type *USBx, unsigned int ep_idx)
{
    volatile uint32_t *ptr;

    ptr = &USBx->FIFO_EP0;
    ptr += ep_idx; /* Pointer math: multiplies ep by sizeof(uint32_t) */
    return (volatile uint8_t *)ptr;
}

#ifdef CONFIG_UDC_FT_DMA
void udc_ft_dma_write_packet(const struct device *dev, uint8_t ep_idx, uint8_t *buffer, uint16_t len)
{
    struct udc_ft_data *priv = udc_get_private(dev);
    const struct udc_ft_config *config = dev->config;
    FT_USBD_Type *const USBx = config->base;
    int res = -1;

    LOG_INF("udc_ft_dma_write_packet ep_idx:0x%02x, len:0x%02x", ep_idx, len);

    k_sem_reset(&priv->status_sem);

    USBx->USB_DMAReg[ep_idx].DMA_ADDR = (uint32_t)buffer;
    USBx->USB_DMAReg[ep_idx].DMA_COUNT = len;
    USBx->USB_DMAReg[ep_idx].DMA_CNTL =
        DEV_CNTL_DMAEN | DEV_CNTL_EP(ep_idx) | DEV_CNTL_DIRECTION_READ | DEV_CNTL_INTERE | DEV_CNTL_BURSTMODE(0);

    res = k_sem_take(&priv->status_sem, K_FOREVER);
    if (res != 0)
    {
        LOG_ERR("Write Packet :%u", res);
    }
    /*disable dma*/
    USBx->USB_DMAReg[ep_idx].DMA_CNTL = 0;
    return;
}

void udc_ft_dma_read_packet(const struct device *dev, uint8_t ep_idx, uint8_t *buffer, uint16_t len)
{
    struct udc_ft_data *priv = udc_get_private(dev);
    const struct udc_ft_config *config = dev->config;
    FT_USBD_Type *const USBx = config->base;
    int res = -1;

    LOG_INF("udc_ft_dma_read_packet ep_idx:0x%02x, len:0x%02x", ep_idx, len);

    k_sem_reset(&priv->status_sem);

    USBx->USB_DMAReg[ep_idx].DMA_ADDR = (uint32_t)buffer;
    USBx->USB_DMAReg[ep_idx].DMA_COUNT = len;
    USBx->USB_DMAReg[ep_idx].DMA_CNTL = DEV_CNTL_DMAEN | DEV_CNTL_EP(ep_idx) | DEV_CNTL_INTERE | DEV_CNTL_BURSTMODE(0);

    res = k_sem_take(&priv->status_sem, K_FOREVER);
    if (res != 0)
    {
        LOG_ERR("Read Packet :%u", res);
    }

    /*disable dma*/
    USBx->USB_DMAReg[ep_idx].DMA_CNTL = 0;
    return;
}
#endif /*CONFIG_UDC_FT_DMA*/
static void udc_ft_write_packet(volatile uint8_t *fifo_ptr, uint8_t *data_ptr, uint16_t len)
{
    int i;
    for (i = 0; i < len; i++)
    {
        *fifo_ptr = *data_ptr++;
    }
}

static void udc_ft_read_packet(volatile uint8_t *fifo_ptr, uint8_t *data_ptr, uint16_t len)
{
    int i;
    for (i = 0; i < len; i++)
    {
        *data_ptr++ = *fifo_ptr;
    }
}

static int ft_udc_xfer_out(const struct device *dev, uint8_t ep, bool strict)
{
    const struct udc_ft_config *config = dev->config;
    struct udc_ep_config *ep_cfg = udc_get_ep_cfg(dev, ep);
    struct net_buf *buf;
    uint8_t ep_idx;
    uint8_t saved_idx;
    uint8_t csr_l;
    FT_USBD_Type *const USBx = config->base;

    if (!USB_EP_DIR_IS_OUT(ep))
    {
        LOG_ERR("Invalid EP address 0x%02x for data out", ep);
        return -EINVAL;
    }
    if (udc_ep_is_busy(ep_cfg))
    {
        if (strict)
        {
            LOG_ERR("EP 0x%02x busy", ep);
            return -EAGAIN;
        }
        return 0;
    }

    buf = udc_buf_peek(ep_cfg);
    if (buf == NULL)
    {
        if (strict)
        {
            LOG_ERR("No buffer queued for EP 0x%02x", ep);
            return -ENODATA;
        }
        return 0;
    }
    ep_idx = USB_EP_GET_IDX(ep);
    udc_ep_set_busy(ep_cfg, true);

    udc_ft_hal_lock(dev);
    saved_idx = USBx->EINDEX;
    ;
    USBx->EINDEX = ep_idx;
    if (ep == USB_CONTROL_EP_OUT)
    {
        csr_l = USBx->E0CSR_L;
        if (csr_l & DEV_CSR0_SENTSTALL)
        {
            csr_l &= ~DEV_CSR0_SENTSTALL;
        }
        if (csr_l & DEV_CSR0_RXPKTRDY)
        {
            csr_l |= DEV_CSR0_SERVICE_RXPKTRDY;
        }
        USBx->E0CSR_L = DEV_CSR0_SERVICE_RXPKTRDY;
    }
    else
    {
        csr_l = USBx->RXCSR_L;
        // Clear sentstall and restart data toggle.
        if (csr_l & DEV_RXCSR_SENT_STALL)
        {
            // clear SendStall bit
            csr_l &= ~DEV_RXCSR_SEND_STALL;

            csr_l |= DEV_RXCSR_CLR_DATA_TOG;
            // set ClrDataTog
            USBx->RXCSR_L = csr_l;
        }
        csr_l &= ~DEV_RXCSR_RXPKTRDY;
        USBx->RXCSR_L = csr_l;
    }
    USBx->EINDEX = saved_idx;
    udc_ft_hal_unlock(dev);
    return 0;
}

static int ft_ctrl_feed_dout(const struct device *dev, const uint16_t length)
{
    struct udc_ft_data *priv = udc_get_private(dev);
    struct udc_ep_config *ep_cfg = udc_get_ep_cfg(dev, USB_CONTROL_EP_OUT);
    struct net_buf *buf;
    uint8_t ep;

    ep = USB_CONTROL_EP_OUT;
    buf = udc_ctrl_alloc(dev, USB_CONTROL_EP_OUT, length);
    if (buf == NULL)
    {
        LOG_ERR("ft_ctrl_feed_dout NULL");
        return -ENOMEM;
    }
    priv->ctrlout_tailroom = length;
    k_fifo_put(&ep_cfg->fifo, buf);
    ft_udc_xfer_out(dev, ep, true);
    return 0;
}
static int ft_udc_reset_interrupt_ep_netbuf(const struct device *dev, struct udc_ep_config *ep_cfg, int ret)
{
    struct net_buf *buf;
    int err = 0;
    buf = udc_buf_get_all(ep_cfg);
    if (buf)
    {
        err = udc_submit_ep_event(dev, buf, ret);
        if (err)
        {

            LOG_ERR("udc submit err %d", err);
        }
    }

    return err;
}
static int ft_udc_xfer_in(const struct device *dev, uint8_t ep, bool strict) // ready to send data from Host
{
    const struct udc_ft_config *config = dev->config;
    struct udc_ep_config *ep_cfg = udc_get_ep_cfg(dev, ep);
    struct net_buf *buf;
    uint8_t *data_ptr;
    volatile uint8_t *fifo_ptr;
    uint32_t data_len;
    uint8_t ep_idx = USB_EP_GET_IDX(ep);
    uint8_t saved_idx;
    uint8_t csr_l;
    bool is_interrupt_ep = (USB_EP_TYPE_INTERRUPT == ep_cfg->attributes);

    FT_USBD_Type *const USBx = config->base;

    if (!USB_EP_DIR_IS_IN(ep))
    {
        LOG_ERR("Invalid EP address 0x%02x for data in", ep);
        return -EINVAL;
    }

    if (is_interrupt_ep && udc_ep_is_busy(ep_cfg))
    {

        udc_ft_hal_lock(dev);

        saved_idx = USBx->EINDEX;
        USBx->EINDEX = ep_idx;

        udc_ep_set_busy(ep_cfg, false);
        if (!(USBx->TXCSR_L & DEV_TXCSR_TXPKTRDY))
        {
            LOG_DBG("fake busy ep,ignore\n");
        }
        else
        {
            USBx->TXCSR_L = DEV_TXCSR_CLR_DATA_TOG;
            LOG_ERR("real busy,reset interrupt ep\n");
        }

        USBx->EINDEX = saved_idx;

        udc_ft_hal_unlock(dev);
    }

    buf = udc_buf_peek(ep_cfg);
    if (buf == NULL || buf->data == NULL)
    {
        /* No DATA IN request */
        LOG_ERR("No DATA IN request,ep=%x", ep);
        return 0;
    }

    data_ptr = buf->data;
    data_len = buf->len;
    data_len = MIN(config->ep_cfg_in[ep_idx].caps.mps, buf->len);

    if (udc_ep_is_busy(ep_cfg))
    {
        LOG_ERR("EPI 0x%02x busy", ep);
        ft_udc_reset_interrupt_ep_netbuf(dev, ep_cfg, 0);
        // udc_ep_set_busy(ep_cfg, false);
        return 0;
    }
    udc_ep_set_busy(ep_cfg, true);

    udc_ft_hal_lock(dev);

    saved_idx = USBx->EINDEX;
    USBx->EINDEX = ep_idx;
    if (ep == USB_CONTROL_EP_IN)
    {
        csr_l = USBx->E0CSR_L;
        if (csr_l & DEV_CSR0_TXPKTRDY)
        {
            LOG_ERR("DATA is not sent out1\n");
            USBx->EINDEX = saved_idx;
            udc_ft_hal_unlock(dev);
            return 0;
        }
    }
    else
    {
        csr_l = USBx->TXCSR_L;
        if (csr_l & DEV_TXCSR_TXPKTRDY)
        {
            LOG_ERR("DATA is not sent out2\n");
            USBx->EINDEX = saved_idx;

            ft_udc_reset_interrupt_ep_netbuf(dev, ep_cfg, 0);
            udc_ft_hal_unlock(dev);

            return 0;
        }
    }

    if (data_len)
    {
        if (csr_l & DEV_TXCSR_FIFO_NOT_EMPTY)
        {
            LOG_WRN("DATA is not sent out");
        }
        fifo_ptr = udc_ft_get_fifo_ptr(USBx, ep_idx);
#ifdef CONFIG_UDC_FT_DMA
        if (ep_idx == 0)
            udc_ft_write_packet(fifo_ptr, data_ptr, data_len);
        else
            udc_ft_dma_write_packet(dev, ep_idx, data_ptr, data_len);
#else  /* CONFIG_UDC_FT_DMA */

        udc_ft_write_packet(fifo_ptr, data_ptr, data_len);
	
#ifdef FT_SHOW_USB_CMD
        if(ep&&ep!=0x80&&data_len<64){
            for(uint8_t i=0;i<data_len;i++){
                printk("%x ",data_ptr[i]);
            }
            printk("-ep%x-\n",ep);
	}
#endif
	

#endif /* CONFIG_UDC_FT_DMA */

        net_buf_pull(buf, data_len);

        if (ep == USB_CONTROL_EP_IN)
        {
            if (data_len < config->ep_cfg_in[ep_idx].caps.mps)
            {
                USBx->E0CSR_L = DEV_CSR0_TXPKTRDY | DEV_CSR0_DATAEND;
            }
            else
            {
                USBx->E0CSR_L = DEV_CSR0_TXPKTRDY;
            }
        }
        else
        {
            USBx->TXMAXP = data_len;
            if (csr_l & DEV_TXCSR_SENT_SATLL)
            {
                USBx->TXCSR_L = DEV_TXCSR_TXPKTRDY | DEV_TXCSR_CLR_DATA_TOG;
            }
            else
            {
                USBx->TXCSR_L = DEV_TXCSR_TXPKTRDY;
            }

            if (is_interrupt_ep)
            {
                ft_udc_reset_interrupt_ep_netbuf(dev, ep_cfg, 0);
            }
        }
    }
    else if (udc_ep_buf_has_zlp(buf))
    {
        udc_ep_buf_clear_zlp(buf);
        if (ep == USB_CONTROL_EP_IN)
        {
            LOG_INF("Wrong ZLP %x", ep);
            USBx->E0CSR_L = DEV_CSR0_TXPKTRDY | DEV_CSR0_DATAEND;
        }
        else
        {
            USBx->TXMAXP = 0;
            if (csr_l & DEV_TXCSR_SENT_SATLL)
            {
                USBx->TXCSR_L = DEV_TXCSR_TXPKTRDY | DEV_TXCSR_CLR_DATA_TOG;
            }
            else
            {
                USBx->TXCSR_L = DEV_TXCSR_TXPKTRDY;
            }
        }
    }
    else
    {
        LOG_INF("Len 0 pkt :%x", ep);
        if (ep == USB_CONTROL_EP_IN)
        {
            // Nothing is done
            USBx->E0CSR_L = DEV_CSR0_TXPKTRDY | DEV_CSR0_DATAEND;
        }
        else
        {
            USBx->TXMAXP = 0;
            if (csr_l & DEV_TXCSR_SENT_SATLL)
            {
                USBx->TXCSR_L = DEV_TXCSR_TXPKTRDY | DEV_TXCSR_CLR_DATA_TOG;
            }
            else
            {
                USBx->TXCSR_L = DEV_TXCSR_TXPKTRDY;
            }
        }
    }
    USBx->EINDEX = saved_idx;
    udc_ft_hal_unlock(dev);
    return 0;
}

static inline int ft_udc_msg_handle_out(const struct device *dev, struct udc_ft_msg *msg)
{
    const struct udc_ft_config *config = dev->config;
    struct udc_ep_config *ep_cfg;
    struct net_buf *buf;
    uint8_t *data_ptr;
    volatile uint8_t *fifo_ptr;
    uint8_t ep;
    uint8_t ep_idx;
    uint8_t saved_idx;
    uint32_t data_len;
    uint16_t read_count;
    int err;
    FT_USBD_Type *const USBx = config->base;

    if (msg->type != FT_UDC_MSG_TYPE_OUT)
    {
        LOG_ERR("Wrong msg type int ft_udc_msg_handle_out");
        return -ENOMSG;
    }

    ep = msg->out.ep;
    ep_idx = USB_EP_GET_IDX(ep);
    ep_cfg = udc_get_ep_cfg(dev, ep);

    udc_ep_set_busy(ep_cfg, false);

    buf = udc_buf_peek(ep_cfg);
    if (buf == NULL)
    {
        LOG_ERR("No buffer queued for ep=0x%02x", ep);
        return -ENODATA;
    }

    if (!!ep_cfg->stat.halted)
    {
        return 0;
    }

    data_len = net_buf_tailroom(buf); // ready to receive data_len data
    data_ptr = net_buf_tail(buf);

    udc_ft_hal_lock(dev);

    saved_idx = USBx->EINDEX;
    ;
    USBx->EINDEX = ep_idx;
    read_count = USBx->RXCOUNTR;
    LOG_INF("EP :%x, space:%x, read_count:%x", ep_idx, data_len, read_count);
    data_len = MIN(data_len, read_count);

    fifo_ptr = udc_ft_get_fifo_ptr(USBx, ep_idx);
#ifdef CONFIG_UDC_FT_DMA
    udc_ft_dma_read_packet(dev, ep_idx, data_ptr, data_len);
#else  /*CONFIG_UDC_FT_DMA*/
    udc_ft_read_packet(fifo_ptr, data_ptr, data_len);

#ifdef FT_SHOW_USB_CMD     
    if(ep&&ep!=0x80&&data_len<64){
            for(uint8_t i=0;i<data_len;i++){
                printk("%x ",data_ptr[i]);
            }
            printk("-ep%x-\n",ep);
    }
#endif

#endif /*CONFIG_UDC_FT_DMA*/
    USBx->EINDEX = saved_idx;
    udc_ft_hal_unlock(dev);
    net_buf_add(buf, data_len);

    buf = udc_buf_get(ep_cfg);
    err = udc_submit_ep_event(dev, buf, 0);
    if (err < 0)
    {
        LOG_ERR("udc_submit_ep_event failed for ep=0x%02x: %d", ep, err);
        return err;
    }
    buf = udc_buf_peek(ep_cfg);
    if (buf)
    {
        ft_udc_xfer_out(dev, ep, false);
    }
    return 0;
}

static int ft_udc_msg_handle_in(const struct device *dev, struct udc_ft_msg *msg)
{
    struct udc_ep_config *ep_cfg;
    struct net_buf *buf;
    uint8_t ep;
    uint8_t ep_idx;
    int err;

    if (msg->type != FT_UDC_MSG_TYPE_IN)
    {
        LOG_ERR("Wrong msg type int ft_udc_msg_handle_in");
        return -ENOMSG;
    }

    ep = msg->in.ep;
    ep_idx = USB_EP_GET_IDX(ep);
    ep_cfg = udc_get_ep_cfg(dev, ep);

    udc_ep_set_busy(ep_cfg, false);
    buf = udc_buf_peek(ep_cfg);
    if (buf == NULL)
    {
        LOG_DBG("No DATA IN request");
        /* No DATA IN request */
        return 0;
    }

    if (!!ep_cfg->stat.halted)
    {
        return 0;
    }

    if (buf->len || udc_ep_buf_has_zlp(buf))
    {
        ft_udc_xfer_in(dev, ep, false);
        return 0;
    }

    buf = udc_buf_get(ep_cfg);

    err = udc_submit_ep_event(dev, buf, 0);
    if (err < 0)
    {
        LOG_ERR("udc_submit_ep_event failed for ep=0x%02x: %d", ep, err);
        return err;
    }
    return 0;
}

int ft_udc_msg_handle_xfer(const struct device *dev, struct udc_ft_msg *msg)
{
    uint8_t ep;
    int err;

    if (msg->type != FT_UDC_MSG_TYPE_XFER)
    {
        LOG_ERR("Wrong msg type int ft_udc_xfer_out");
        return -ENOMSG;
    }
    ep = msg->xfer.ep;
    if (USB_EP_DIR_IS_IN(ep))
    {
        err = ft_udc_xfer_in(dev, ep, false);
    }
    else
    {
        err = ft_udc_xfer_out(dev, ep, false);
    }
    return err;
}

static int ft_udc_handle_ep0_tx(const struct device *dev)
{
    uint8_t ep = USB_CONTROL_EP_IN;
    struct net_buf *buf;
    struct udc_ep_config *ep_cfg = udc_get_ep_cfg(dev, USB_CONTROL_EP_IN);

    udc_ep_set_busy(ep_cfg, false);

    buf = udc_buf_peek(ep_cfg);
    if (buf == NULL)
    {
        return 0;
    }

    LOG_INF("tx process buf->len:%x, buf->size:%x  zlp:%x\n", buf->len, buf->size, udc_ep_buf_has_zlp(buf));
    if (buf->len || udc_ep_buf_has_zlp(buf))
    {
        ft_udc_xfer_in(dev, ep, false);
        return 0;
    }

    /* To submit the peeked buffer */
    buf = udc_buf_get(ep_cfg);

    if (udc_ctrl_stage_is_status_in(dev) || udc_ctrl_stage_is_no_data(dev))
    {
        /* Status stage finished, notify upper layer */
        udc_ctrl_submit_status(dev, buf);
    }

    if (udc_ctrl_stage_is_data_in(dev))
    {
        /*
         * s-in-[status] finished, release buffer.
         * Since the controller supports auto-status we cannot use
         * if (udc_ctrl_stage_is_status_out()) after state update.
         */
        net_buf_unref(buf);
    }

    /* Update to next stage of control transfer */
    udc_ctrl_update_stage(dev, buf);
    return 0;
}

static int ft_udc_handle_ep0_rx(const struct device *dev) // a packet is got
{
    const struct udc_ft_config *config = dev->config;
    struct udc_ft_data *priv = udc_get_private(dev);
    struct udc_ep_config *ep_cfg = udc_get_ep_cfg(dev, USB_CONTROL_EP_OUT);
    struct net_buf *buf;
    uint8_t ep = USB_CONTROL_EP_OUT;
    uint8_t ep_idx;
    uint8_t saved_idx;
    uint32_t data_len;
    uint16_t read_count;
    uint8_t *data_ptr;
    volatile uint8_t *fifo_ptr;
    FT_USBD_Type *const USBx = config->base;

    udc_ep_set_busy(ep_cfg, false); // allow next transfer

    buf = udc_buf_peek(ep_cfg);
    if (buf == NULL)
    {
        LOG_ERR("No buffer queued for ep=0x%02x", ep);
        return -ENODATA;
    }

    ep_idx = USB_EP_GET_IDX(ep);
    data_len = priv->ctrlout_tailroom;
    data_ptr = net_buf_tail(buf);

    udc_ft_hal_lock(dev);
    saved_idx = USBx->EINDEX;
    ;
    USBx->EINDEX = ep_idx;
    read_count = USBx->E0COUNTR;

    if (read_count > data_len || read_count > net_buf_tailroom(buf))
    {
        LOG_ERR("Buffer queued for ep=0x%02x cannot accommodate packet", ep);
        USBx->EINDEX = saved_idx;
        udc_ft_hal_unlock(dev);
        return -ENOBUFS;
    }

    data_len = MIN(data_len, read_count);
    fifo_ptr = udc_ft_get_fifo_ptr(USBx, ep_idx);
    udc_ft_read_packet(fifo_ptr, data_ptr, data_len);
    net_buf_add(buf, data_len);
    USBx->EINDEX = saved_idx;
    udc_ft_hal_unlock(dev);
    priv->ctrlout_tailroom -= data_len;
    if (priv->ctrlout_tailroom > 0)
    {
        buf = udc_buf_peek(ep_cfg);
        if (buf)
        {
            ft_udc_xfer_out(dev, ep, false);
        }
        return 0;
    }

    buf = udc_buf_get(ep_cfg);
    if (buf == NULL)
    {
        LOG_ERR("ep 0x%02x ok, queue is empty", ep);
        return 0;
    }

    /*
     * In case s-in-status, controller supports auto-status therefore we
     * do not have to call udc_ctrl_stage_is_status_out().
     */
    udc_ctrl_update_stage(dev, buf);

    if (udc_ctrl_stage_is_status_in(dev))
    {
        udc_ctrl_submit_s_out_status(dev, buf);
    }

    return 0;
}

static int ft_udc_handle_ep0_setup(const struct device *dev) // a in packet is sent to host
{
    struct udc_ft_data *priv = udc_get_private(dev);
    struct usb_setup_packet *setup = (void *)priv->setup;
    uint8_t *data_ptr;
    struct net_buf *buf;
    int err;
    struct udc_ep_config *cfg_out = udc_get_ep_cfg(dev, USB_CONTROL_EP_OUT);
    struct udc_ep_config *cfg_in = udc_get_ep_cfg(dev, USB_CONTROL_EP_IN);

    /* Make sure there isn't any obsolete data stage buffer queued */
    buf = udc_buf_get_all(cfg_out);
    if (buf)
    {
        net_buf_unref(buf);
        LOG_WRN("Release EP_OUT-setup");
    }

    buf = udc_buf_get_all(cfg_in);
    if (buf)
    {
        net_buf_unref(buf);
        LOG_INF("Release EP_IN-setup");
    }

    udc_ep_set_busy(cfg_out, false);
    udc_ep_set_busy(cfg_in, false);

    buf = udc_ctrl_alloc(dev, USB_CONTROL_EP_OUT, 8);
    if (buf == NULL)
    {
        LOG_ERR("allocate error for Setup");
        return -ENOMEM;
    }
    udc_ep_buf_set_setup(buf);
    data_ptr = net_buf_tail(buf);
    bytecpy(data_ptr, (uint8_t *)priv->setup, 8);
    net_buf_add(buf, 8);

    // LOG_WRN("setup:%02x %02x : %02x %02x %02x %02x %02x %02x",data_ptr[0], data_ptr[1], data_ptr[2],
    // data_ptr[3],data_ptr[4],data_ptr[5],data_ptr[6],data_ptr[7]);

    /* Update to next stage of CTRL transfer */
    udc_ctrl_update_stage(dev, buf);

    // USB_REQTYPE_DIR_TO_DEVICE
    if ((USB_REQTYPE_GET_DIR(setup->bmRequestType) == USB_REQTYPE_DIR_TO_DEVICE) &&
        (USB_REQTYPE_GET_TYPE(setup->bmRequestType) == USB_REQTYPE_TYPE_STANDARD) &&
        (USB_REQTYPE_GET_RECIPIENT(setup->bmRequestType) == USB_REQTYPE_RECIPIENT_DEVICE) &&
        (setup->bRequest == USB_SREQ_SET_ADDRESS))
    {

        priv->addr = setup->wValue;
        LOG_INF("SET_ADDRESS %x", setup->wValue);
    }

    if (udc_ctrl_stage_is_data_out(dev))
    { // data out
        /*  Allocate and feed buffer for DATA OUT stage */
        err = ft_ctrl_feed_dout(dev, udc_data_stage_length(buf));
        if (err == -ENOMEM)
        {
            LOG_ERR("ft_udc_handle_ep0_setup err:%x", err);
            udc_submit_ep_event(dev, buf, err);
        }
        // LOG_WRN("EP0 OUT:%d", udc_data_stage_length(buf));
    }
    else if (udc_ctrl_stage_is_data_in(dev))
    {
        udc_ctrl_submit_s_in_status(dev);
        priv->ctrlout_tailroom = 0;
    }
    else
    {
        udc_ctrl_submit_s_status(dev); // generate a 0 len IN trans
        priv->ctrlout_tailroom = 0;
    }
    return 0;
}

// ep0
static int ft_udc_msg_handle_ep0(const struct device *dev)
{
    const struct udc_ft_config *config = dev->config;
    const struct udc_ft_data *priv = udc_get_private(dev);
    struct udc_ep_config *ep_cfg;
    struct udc_data *data = dev->data;
    struct net_buf *buf;
    volatile uint8_t *fifo_ptr;
    uint16_t read_count;
    uint8_t saved_idx;
    uint8_t ep_idx;
    uint8_t ep = USB_CONTROL_EP_IN;
    uint8_t csr_l;
    uint8_t csr_l_hex;
    struct usb_setup_packet *setup = (void *)priv->setup;
    int err;
    FT_USBD_Type *const USBx = config->base;

    ep_idx = USB_EP_GET_IDX(ep);
    udc_ft_hal_lock(dev);
    saved_idx = USBx->EINDEX;
    USBx->EINDEX = ep_idx;
    csr_l = USBx->E0CSR_L;
    csr_l_hex = csr_l;
    LOG_DBG("-- handle_ep0 ep0_st:%x, stage:%x\n", csr_l_hex, data->stage);
    if (csr_l & DEV_CSR0_SENTSTALL)
    {
        LOG_DBG("ft_udc_msg_handle_ep0 stalled");
    }

    if (csr_l & DEV_CSR0_SETUPEND)
    { // control transfer data process is done
        USBx->E0CSR_L = DEV_CSR0_SERVICE_SETUPEND;
        LOG_ERR("EP0 SETUPEND :%x, stage:%x\n", csr_l, data->stage);
        USBx->EINDEX = saved_idx;
        udc_ft_hal_unlock(dev);
        return 0;
    }

    if (csr_l & DEV_CSR0_RXPKTRDY)
    {
        if (udc_ctrl_stage_is_data_in(dev))
        {
            // LOG_INF("RX INT udc_ctrl_stage_is_data_in, status-out");
            err = ft_udc_handle_ep0_tx(dev);
            // status-out/status-out->setup
        }
        if (udc_ctrl_stage_is_no_data(dev))
        {
            /* Status stage finished, notify upper layer */
            LOG_ERR("RX INT FAKE STATUS is missed,%u, ep0_st:%x", data->stage, csr_l_hex);
        }
        // 1. setup
        if (udc_ctrl_stage_is_data_out(dev))
        {
            // LOG_INF("RX INT udc_ctrl_stage_is_data_out, status-in");
            USBx->EINDEX = saved_idx;
            udc_ft_hal_unlock(dev);
            ft_udc_handle_ep0_rx(dev);
            udc_ft_hal_lock(dev);
            saved_idx = USBx->EINDEX;
        }
        else
        {
            read_count = USBx->E0COUNTR;
            if (read_count != 8)
            {
                USBx->EINDEX = saved_idx;
                udc_ft_hal_unlock(dev);
                return 0;
            }
            LOG_INF("RX INT Ep0 setup, stage: %u, count:%x ep0_st:%x", data->stage, read_count, csr_l_hex);
            fifo_ptr = udc_ft_get_fifo_ptr(USBx, ep_idx);
            udc_ft_read_packet(fifo_ptr, (uint8_t *)setup, 8);
            ft_udc_handle_ep0_setup(dev);
        }
    }
    else
    {
        if (udc_ctrl_stage_is_data_in(dev))
        {
            LOG_INF("TX INT udc_ctrl_stage_is_data_in, status-out");
            // status[out]
            ft_udc_handle_ep0_tx(dev);
        }
        else
        {
            if (udc_ctrl_stage_is_data_out(dev))
            {
                LOG_DBG("TX INT udc_ctrl_stage_is_data_out, status-in :%d", priv->ctrlout_tailroom);
            }
            else if (udc_ctrl_stage_is_no_data(dev))
            {
                LOG_ERR("TX INT  udc_ctrl_stage_is_no_data, In occurs");
            }
            else
            {
                LOG_INF("TX INT Ep0 In done, In occurs, %u, ep0_st:%x", data->stage, csr_l_hex);
            }
            ep_cfg = udc_get_ep_cfg(dev, USB_CONTROL_EP_IN);
            buf = udc_buf_get(ep_cfg);
            if (unlikely(buf == NULL))
            {
                LOG_DBG("ep 0x%02x queue is empty in ep0 process", USB_CONTROL_EP_IN);
            }
            else
            {
                net_buf_unref(buf);
                LOG_INF("TX INT IN transfer finished, release the buffer");
            }
        }
    }
    USBx->EINDEX = saved_idx;
    udc_ft_hal_unlock(dev);
    return 0;
}

static int ft_udc_msg_handle_fake_status_in(const struct device *dev, struct udc_ft_msg *msg)
{
    const struct udc_ft_config *config = dev->config;
    struct net_buf *buf;
    uint8_t saved_idx;
    uint8_t ep = USB_CONTROL_EP_IN;
    uint8_t ep_idx;
    struct udc_ep_config *ep_cfg = udc_get_ep_cfg(dev, USB_CONTROL_EP_IN);
    FT_USBD_Type *const USBx = config->base;

    buf = udc_buf_get(ep_cfg);
    if (unlikely(buf == NULL))
    {
        LOG_DBG("ep 0x%02x queue is empty", USB_CONTROL_EP_IN);
        return 0;
    }

    if (udc_ctrl_stage_is_status_in(dev) || udc_ctrl_stage_is_no_data(dev))
    {
        /* Status stage finished, notify upper layer */
        udc_ctrl_submit_status(dev, buf);
    }

    if (udc_ctrl_stage_is_data_in(dev))
    {
        /*
         * s-in-[status] finished, release buffer.
         * Since the controller supports auto-status we cannot use
         * if (udc_ctrl_stage_is_status_out()) after state update.
         */
        net_buf_unref(buf);
    }

    // k_sleep(K_MSEC(10));

    /* Update to next stage of control transfer */
    udc_ctrl_update_stage(dev, buf);

    ep_idx = USB_EP_GET_IDX(ep);
    udc_ft_hal_lock(dev);
    saved_idx = USBx->EINDEX;
    USBx->EINDEX = ep_idx;
    USBx->E0CSR_L = DEV_CSR0_SERVICE_RXPKTRDY | DEV_CSR0_DATAEND;
    USBx->EINDEX = saved_idx;
    udc_ft_hal_unlock(dev);

    return 0;
}

static int ft_udc_send_msg(const struct device *dev, const struct udc_ft_msg *msg)
{
    const struct udc_ft_data *priv = udc_get_private(dev);
    int err;
    err = k_msgq_put(priv->msgq, msg, K_NO_WAIT);
    if (err < 0)
    {
        LOG_ERR("Message queue overflow again");
    }
    return err;
}

static inline void ft_udc_reset_addr(const struct device *dev)
{
    const struct udc_ft_config *config = dev->config;
    struct udc_ft_data *priv = udc_get_private(dev);
    FT_USBD_Type *const USBx = config->base;

    USBx->FADDRR = 0x00;
    priv->addr = 0;
}

static int ft_udc_msg_handle_connect(const struct device *dev, struct udc_ft_msg *msg)
{
    const struct udc_ft_config *config = dev->config;
    USBC_Connect(config->base);

    udc_submit_event(dev, UDC_EVT_VBUS_READY, 0);
    return 0;
}

static int ft_udc_msg_handle_disconnect(const struct device *dev, struct udc_ft_msg *msg)
{
    const struct udc_ft_config *config = dev->config;
    USBC_Disconnect(config->base);

    udc_submit_event(dev, UDC_EVT_VBUS_REMOVED, 0);
    return 0;
}

static int ft_udc_msg_handle_reset(const struct device *dev, struct udc_ft_msg *msg)
{
    struct udc_ft_data *priv = udc_get_private(dev);
    const struct udc_ft_config *config = dev->config;
    unsigned int key;

    k_msgq_purge(priv->msgq);
    key = irq_lock();
    priv->enum_done = false;
    irq_unlock(key);
#ifdef CONFIG_PM
    udc_ft_pm_policy_lock_get(dev);
#endif
    udc_submit_event(dev, UDC_EVT_RESET, 0);

    for (int i = 0; i < config->num_of_eps; i++)
    {
        if (config->ep_cfg_in[i].stat.halted)
        {
            LOG_INF("RESET ep0_in halted:%x ", config->ep_cfg_in[i].stat.halted);
            config->ep_cfg_in[i].stat.halted = false;
        }
        if (config->ep_cfg_out[i].stat.halted)
        {
            LOG_INF("RESET ep0_in halted:%x ", config->ep_cfg_out[i].stat.halted);
            config->ep_cfg_out[i].stat.halted = false;
        }
    }

    /* reset top half processing*/
    udc_ft_hal_lock(dev);
    /* Re-Enable control endpoints */
    USBC_BusReset(config->base, config->speed_idx);
    udc_ft_hal_unlock(dev);

    /* UDC stack would handle bottom-half processing,
     * including reset device address (udc_set_address),
     * un-configure device (udc_ep_disable), etc.
     */

    return 0;
}

static int ft_udc_msg_handle_suspend(const struct device *dev, struct udc_ft_msg *msg)
{
    unsigned int key;
    struct udc_ft_data *priv = udc_get_private(dev);
    printk("usb suspend\n");

    LOG_WRN("SUSPEND");
    if (!priv->enum_done)
    {
        return 0;
    }
    key = irq_lock();
    
    /* UDC stack would handle bottom-half processing */
    if (!udc_is_suspended(dev) && udc_is_enabled(dev))
    {
        LOG_WRN("Enter SUSPEND State");
        udc_set_suspended(dev, true);
        udc_submit_event(dev, UDC_EVT_SUSPEND, 0);
    }
    irq_unlock(key);
#ifdef CONFIG_PM
    ft_pm_enter_deep_sleep(true);
    udc_ft_pm_policy_lock_put(dev);
#endif
    return 0;
}

static int ft_udc_msg_handle_resume(const struct device *dev, struct udc_ft_msg *msg)
{
#ifdef CONFIG_PM
    ft_pm_enter_deep_sleep(false);
    udc_ft_pm_policy_lock_get(dev);
#endif
    /* UDC stack would handle bottom-half processing */
    if (udc_is_suspended(dev) && udc_is_enabled(dev))
    {
        udc_set_suspended(dev, false);
        udc_submit_event(dev, UDC_EVT_RESUME, 0);
    }
    return 0;
}

/*
 * You can use one thread per driver instance model or UDC driver workqueue,
 * whichever model suits your needs best. If you decide to use the UDC workqueue,
 * enable Kconfig option UDC_WORKQUEUE and remove the handler below and
 * caller from the UDC_FOCALTECH_DEVICE_DEFINE macro.
 */
static ALWAYS_INLINE void ft_thread_handler(void *const arg)
{
    const struct device *dev = (const struct device *)arg;
    struct udc_ft_data *priv = udc_get_private(dev);
    int err;
    struct udc_ft_msg msg;

    // LOG_DBG("Driver %p thread started", dev);
    LOG_DBG("Driver %p thread started", dev);
    while (true)
    {
        k_msgq_get(priv->msgq, &msg, K_FOREVER);
        err = 0;
        // LOG_INF("--handle message enter, type:%x\n", msg.type);

        udc_lock_internal(dev, K_FOREVER);

        switch (msg.type)
        {
        case FT_UDC_MSG_TYPE_CONN:
            err = ft_udc_msg_handle_connect(dev, &msg);
            break;

        case FT_UDC_MSG_TYPE_DISCON:
            err = ft_udc_msg_handle_disconnect(dev, &msg);
            break;

        case FT_UDC_MSG_TYPE_RESET:
            err = ft_udc_msg_handle_reset(dev, &msg);
            LOG_INF("ft_udc_msg_handle_reset \n");
            break;

        case FT_UDC_MSG_TYPE_SUSUPEND:
            LOG_INF("ft_udc_msg_handle_suspend\n");
            err = ft_udc_msg_handle_suspend(dev, &msg);
            break;

        case FT_UDC_MSG_TYPE_RESUME:
            LOG_INF("ft_udc_msg_handle_resume\n");
            err = ft_udc_msg_handle_resume(dev, &msg);
            break;

        case FT_UDC_MSG_TYPE_EP0:
            err = ft_udc_msg_handle_ep0(dev);
            break;

        case FT_UDC_MSG_TYPE_OUT:
            err = ft_udc_msg_handle_out(dev, &msg);
            break;

        case FT_UDC_MSG_TYPE_IN:
            err = ft_udc_msg_handle_in(dev, &msg);
            break;

        case FT_UDC_MSG_TYPE_XFER:
            err = ft_udc_msg_handle_xfer(dev, &msg);
            break;

        case FT_UDC_MSG_TYPE_STATUS_IN:
            err = ft_udc_msg_handle_fake_status_in(dev, &msg);
            break;

        case FT_UDC_MSG_TYPE_SW_RECONN:
            LOG_INF("wrong message ft_thread_handler\n");
            // err = ft_udc_msg_handle_sw_reconn(dev, &msg);
            break;

        default:
            __ASSERT_NO_MSG(false);
        }

        udc_unlock_internal(dev);

        if (err)
        {
            LOG_ERR("------ft_thread_handler get a message type :%x, err :%x\n", msg.type, err);
            udc_submit_event(dev, UDC_EVT_ERROR, err);
        }
        // LOG_INF("--handle message exit, type:%x\n", msg.type);
        // k_msleep(1000);
    }
}
static inline void ft_usb_resume_event(const struct device *dev)
{
#ifdef CONFIG_PM
    udc_ft_pm_policy_lock_get(dev);
#endif
    if (udc_is_suspended(dev) && udc_is_enabled(dev))
    {
        udc_set_suspended(dev, false);
        udc_submit_event(dev, UDC_EVT_RESUME, 0);
    }
}

// * @fn static inline uint8_t sys_read8(mm_reg_t addr);

static void ft_udbd_isr(const struct device *dev)
{
    const struct udc_ft_config *config = dev->config;
    struct udc_ft_data *priv = udc_get_private(dev);
    uint8_t ep;
    uint8_t ep_idx = 1;
    struct udc_ft_msg msg = {0};
    uint8_t usbd_intrusb;
    uint16_t usbd_inttx;
    uint16_t usbd_intrx;
    FT_USBD_Type *const USBx = config->base;

#ifdef CONFIG_UDC_FT_DMA
    uint32_t usbd_intdma = USBx->DMA_INTR;
#endif

    usbd_intrusb = USBx->INTRUSB;
    usbd_inttx = USBx->INTRTX;
    usbd_intrx = USBx->INTRRX;

    // LOG_DBG("isr:%x tx:%x rx:%x, rxe:%x, txe:%x", usbd_intrusb,usbd_inttx,usbd_intrx, USBx->INTRRXE, USBx->INTRTXE);
    if (usbd_intrusb & USB_INTERRUPT_CONNECT)
    {
        msg.type = FT_UDC_MSG_TYPE_CONN;
        ft_udc_send_msg(dev, &msg);
        LOG_INF("----------------connect------------------------");
        LOG_INF("USB connect");
    }

    if (usbd_intrusb & USB_INTERRUPT_DISCON)
    {
        msg.type = FT_UDC_MSG_TYPE_DISCON;
        LOG_INF("----------------disconnect------------------------");
        ft_udc_send_msg(dev, &msg);
        LOG_INF("USB disconnect");
    }

    if (usbd_intrusb & USB_INTERRUPT_RESET)
    {
        msg.type = FT_UDC_MSG_TYPE_RESET;
        ft_udc_send_msg(dev, &msg);
    }
    /* USB suspend */
    if (usbd_intrusb & USB_INTERRUPT_SUSPEND)
    {
        // disable usb py
        msg.type = FT_UDC_MSG_TYPE_SUSUPEND;
        ft_udc_send_msg(dev, &msg);
    }
    /* USB resume */
    if (usbd_intrusb & USB_INTERRUPT_RESUME)
    {
        ft_usb_resume_event(dev);
    }
    /* USB SOF */
    if (usbd_intrusb & USB_INTERRUPT_SOF)
    {
        priv->sof_num = USBx->FNUMR;
        // udc_submit_event(dev, UDC_EVT_SOF, 0);
        ft_usb_resume_event(dev);
    }

    /* Handle EP0 interrupt */
    if (usbd_inttx & USB_INTERRUPT_EP0)
    {

        usbd_inttx &= ~0x1U;
        if (priv->addr)
        {
            USBx->FADDRR = priv->addr;
            priv->addr = 0;
            priv->enum_done = true;
        }
        // ft_udc_msg_handle_ep0(dev);
        msg.type = FT_UDC_MSG_TYPE_EP0;
        msg.out.ep = USB_CONTROL_EP_OUT;
        ft_udc_send_msg(dev, &msg);
    }
    // IN_EP
    ep_idx = 1;
    while (usbd_inttx)
    {
        if (usbd_inttx & (1 << ep_idx))
        {
            ep = ep_idx | USB_EP_DIR_IN;
            msg.type = FT_UDC_MSG_TYPE_IN;
            msg.in.ep = ep;
            ft_udc_send_msg(dev, &msg);
            usbd_inttx &= ~(1 << ep_idx);
        }
        ep_idx++;
    }

    /*OUT_EP*/
    ep_idx = 1;
    while (usbd_intrx)
    {
        if (usbd_intrx & (1 << ep_idx))
        {
            ep = ep_idx | USB_EP_DIR_OUT;
            msg.type = FT_UDC_MSG_TYPE_OUT;
            msg.out.ep = ep;
            ft_udc_send_msg(dev, &msg);
            usbd_intrx &= ~(1 << ep_idx);
        }
        ep_idx++;
    }

    /*DMA*/
#ifdef CONFIG_UDC_FT_DMA
    ep_idx = 1;
    while (usbd_intdma)
    {
        if (usbd_intdma & (1 << ep_idx))
        {
            usbd_intdma &= ~(1 << ep_idx);
            k_sem_give(&priv->status_sem);
        }
        ep_idx++;
    }
#endif /*CONFIG_UDC_FT_DMA*/
    return;
}

/*
 * This is called in the context of udc_ep_enqueue() and must
 * not block. The driver can immediately claim the buffer if the queue is empty,
 * but usually it is offloaded to a thread or workqueue to handle transfers
 * in a single location. Please refer to existing driver implementations
 * for examples.
 */

static int udc_ft_ep_enqueue(const struct device *dev, struct udc_ep_config *const ep_cfg, struct net_buf *buf)
{
    struct udc_ft_msg msg = {0};
    bool is_interrupt_ep = (USB_EP_TYPE_INTERRUPT == ep_cfg->attributes);

    udc_ft_hal_lock(dev);

    udc_buf_put(ep_cfg, buf);

    LOG_DBG("%p enqueue [%p:%p] halted:%u, ep:%x, len:%x", dev, buf, ep_cfg, ep_cfg->stat.halted, ep_cfg->addr,
            buf->len);

    /*If the net_buf addresses do not match, it indicates that the previous interrupt transfer data failed to be
      transmitted. Perform a reset on the interrupt endpoint's net_buf.*/

    if (is_interrupt_ep)
    {
        if (udc_buf_peek(ep_cfg) != buf)
        {
            LOG_ERR("endpoint buf check err!!!,buf->len=%d,ep=%x,halt=%d\n", buf->len, ep_cfg->addr,
                    ep_cfg->stat.halted);

            ft_udc_reset_interrupt_ep_netbuf(dev, ep_cfg, -ECONNREFUSED);
        }
    }

    if (ep_cfg->addr == USB_CONTROL_EP_IN && buf->len == 0)
    {
        const struct udc_buf_info *bi = udc_get_buf_info(buf);
        if (bi->status)
        {
            /* Controller automatically performs status IN stage */
            msg.type = FT_UDC_MSG_TYPE_STATUS_IN;
            msg.xfer.ep = USB_CONTROL_EP_IN;
            ft_udc_send_msg(dev, &msg);
            udc_ft_hal_unlock(dev);
            return 0;
        }
    }
    if (!ep_cfg->stat.halted)
    {
        msg.type = FT_UDC_MSG_TYPE_XFER;
        msg.xfer.ep = ep_cfg->addr;
        ft_udc_send_msg(dev, &msg);
    }

    udc_ft_hal_unlock(dev);
    return 0;
}

/*
 * This is called in the context of udc_ep_dequeue()
 * and must remove all requests from an endpoint queue
 * Successful removal should be reported to the higher level with
 * ECONNABORTED as the request result.
 * It is up to the request owner to clean up or reuse the buffer.
 */
static int udc_ft_ep_dequeue(const struct device *dev, struct udc_ep_config *const ep_cfg)
{
    const struct udc_ft_config *config = dev->config;
    struct net_buf *buf;
    LOG_INF("Deque:%x", ep_cfg->addr);
    udc_ft_hal_lock(dev);
    USBC_EpxReset(config->base, ep_cfg->addr);
    udc_ft_hal_unlock(dev);

    ep_cfg->stat.halted = false;
    buf = udc_buf_get_all(ep_cfg);
    if (buf)
    {
        udc_submit_ep_event(dev, buf, -ECONNABORTED);
    }
    udc_ep_set_busy(ep_cfg, false);
    LOG_DBG("dequeue ep 0x%02x", ep_cfg->addr);

    return 0;
}

/*
 * Configure and make an endpoint ready for use.
 * This is called in the context of udc_ep_enable() or udc_ep_enable_internal(),
 * the latter of which may be used by the driver to enable control endpoints.
 */
static int udc_ft_ep_enable(const struct device *dev, struct udc_ep_config *const ep_cfg)
{
    const struct udc_ft_config *config = dev->config;
    int status;
    FT_USBD_Type *const USBx = config->base;
    uint8_t saved_idx = USBx->EINDEX;

    udc_ft_hal_lock(dev);
    LOG_INF("Enable ep 0x%02x", ep_cfg->addr);
    // ep_cfg->stat.enabled = true;

    USBx->EINDEX = USB_EP_GET_IDX(ep_cfg->addr);
    if (ep_cfg->addr & 0x80)
        USBx->TXCSR_L = DEV_TXCSR_TXPKTRDY | DEV_TXCSR_CLR_DATA_TOG;

    USBx->EINDEX = saved_idx;
    status = USBC_EpxOpen(config->base, ep_cfg->addr, ep_cfg->mps, ep_cfg->attributes, config->speed_idx);
    udc_ft_hal_unlock(dev);
    if (status)
    {
        LOG_ERR("USBC_EpxOpen failed(0x%02x), %d", ep_cfg->addr, status);
        return -EIO;
    }
    return 0;
}

/*
 * Opposite function to udc_ft_ep_enable(). udc_ep_disable_internal()
 * may be used by the driver to disable control endpoints.
 */
static int udc_ft_ep_disable(const struct device *dev, struct udc_ep_config *const ep_cfg)
{
    const struct udc_ft_config *config = dev->config;
    int status = 0;

    LOG_DBG("Disalbe ep 0x%02x", ep_cfg->addr);
    udc_ft_hal_lock(dev);
    // ep_cfg->stat.enabled = false;
    status = USBC_EpxClose(config->base, ep_cfg->addr);
    udc_ft_hal_unlock(dev);
    if (status)
    {
        LOG_ERR("USBC_EpxClose failed(0x%02x), %d", ep_cfg->addr, status);
        return -EIO;
    }
    return 0;
}

/* Halt endpoint. Halted endpoint should respond with a STALL handshake. */
static int udc_ft_ep_set_halt(const struct device *dev, struct udc_ep_config *const ep_cfg)
{
    const struct udc_ft_config *config = dev->config;
    if (ep_cfg->stat.halted == true)
    {
        LOG_INF("ep 0x%02x is already as halt", ep_cfg->addr);
        return 0;
    }
    LOG_DBG("Set halt ep 0x%02x", ep_cfg->addr);
    udc_ft_hal_lock(dev);
    // HW set ep stall
    if (ep_cfg->addr == USB_CONTROL_EP_OUT || ep_cfg->addr == USB_CONTROL_EP_IN)
    {
        USBC_EP0SendStall(config->base);
    }
    else
    {
        ep_cfg->stat.halted = true;
        USBC_EPxSendStall(config->base, ep_cfg->addr);
    }
    udc_ft_hal_unlock(dev);
    LOG_INF("Set halt ep 0x%02x done", ep_cfg->addr);
    return 0;
}

/*
 * Opposite to halt endpoint. If there are requests in the endpoint queue,
 * the next transfer should be prepared.
 */
static int udc_ft_ep_clear_halt(const struct device *dev, struct udc_ep_config *const ep_cfg)
{
    const struct udc_ft_config *config = dev->config;
    struct udc_ft_msg xfer_msg = {0};

    LOG_INF("Clear halt ep 0x%02x", ep_cfg->addr);
    udc_ft_hal_lock(dev);
    if (ep_cfg->addr == USB_CONTROL_EP_OUT || ep_cfg->addr == USB_CONTROL_EP_IN)
    {
        USBC_EP0ClearSendStall(config->base);
        LOG_INF("EP0 ClearHalt");
        if (udc_buf_peek(ep_cfg))
        {
            LOG_INF("EP0 ClearHalt, Resend");
            xfer_msg.type = FT_UDC_MSG_TYPE_XFER;
            xfer_msg.xfer.ep = ep_cfg->addr;
            ft_udc_send_msg(dev, &xfer_msg);
        }
    }
    else
    {
        USBC_EPxClearSendStall(config->base, ep_cfg->addr);
        if (udc_buf_peek(ep_cfg))
        {
            xfer_msg.type = FT_UDC_MSG_TYPE_XFER;
            xfer_msg.xfer.ep = ep_cfg->addr;
            ft_udc_send_msg(dev, &xfer_msg);
        }
    }
    ep_cfg->stat.halted = false;
    udc_ft_hal_unlock(dev);
    LOG_DBG("Clear halt ep 0x%02x done", ep_cfg->addr);
    return 0;
}

static int udc_ft_set_address(const struct device *dev, const uint8_t addr)
{
    struct udc_ft_data *priv = udc_get_private(dev);
    /*
     * If the status stage already finished (which depends entirely on when
     * the host sends IN token) then NRF_USBD->USBADDR will have the same
     * address, otherwise it won't (unless new address is unchanged).
     *
     * Store the address so the driver can detect address mismatches
     * between USB stack and USBD peripheral. The mismatches can occur if:
     *   * SW has high enough latency in SETUP handling, or
     *   * Host did not issue Status Stage after Set Address request
     *
     * The SETUP handling latency is a problem because the Set Address is
     * automatically handled by device. Because whole Set Address handling
     * can finish in less than 21 us, the latency required (with perfect
     * timing) to hit the issue is relatively short (2 ms Set Address
     * recovery interval + negligible Set Address handling time). If host
     * sends new SETUP before SW had a chance to read the Set Address one,
     * the Set Address one will be overwritten without a trace.
     */

    LOG_DBG("Set new address %u for %p", addr, dev);
    priv->addr = addr;

    return 0;
}

static int udc_ft_host_wakeup(const struct device *dev)
{
    const struct udc_ft_config *config = dev->config;
    FT_USBD_Type *const USBx = config->base;

    LOG_WRN("Remote wakeup");
    /*When the device is operating in the suspended mode,
    a recovery signal is generated through the CPU set 1.
    After 10ms (up to 15ms) the CPU should clear the bit
    and close the recovery signal.
    0= Turn off the recovery signal
    1= to produce the recovery signal BIT 2*/

    if (udc_is_suspended(dev))
    {
        USBx->UCSR |= USB_POWER_RESUME;
        k_sleep(K_MSEC(10));
        USBx->UCSR &= ~(USB_POWER_RESUME);
    }

    return 0;
}

/* Return actual USB device speed */
static enum udc_bus_speed udc_ft_device_speed(const struct device *dev)
{
    struct udc_data *data = dev->data;
    return data->caps.hs ? UDC_BUS_SPEED_HS : UDC_BUS_SPEED_FS;
}

static int udc_ft_enable(const struct device *dev)
{
    const struct udc_ft_config *config = dev->config;
    LOG_DBG("Enable device %p", dev);

    udc_ft_hal_lock(dev);
    USBC_Init(config->base, config->speed_idx);
    udc_ft_hal_unlock(dev);

    /*enable ep0*/
    if (udc_ep_enable_internal(dev, USB_CONTROL_EP_OUT, USB_EP_TYPE_CONTROL, 64, 0))
    {
        LOG_ERR("Failed to enable control endpoint");
        return -EIO;
    }

    if (udc_ep_enable_internal(dev, USB_CONTROL_EP_IN, USB_EP_TYPE_CONTROL, 64, 0))
    {
        LOG_ERR("Failed to enable control endpoint");
        return -EIO;
    }

    /*enable ep0 interupt*/
    USBC_Enbale_Irq_Ep0(config->base);

    return 0;
}

static int udc_ft_disable(const struct device *dev)
{
    const struct udc_ft_config *config = dev->config;
    udc_ft_hal_lock(dev);

    // Disable soft connect, Clear interrupt status
    USBC_DeInit(config->base, config->speed_idx);
    udc_ft_hal_unlock(dev);
    LOG_DBG("Disable device %p", dev);

    if (udc_ep_disable_internal(dev, USB_CONTROL_EP_OUT))
    {
        LOG_ERR("Failed to disable control endpoint");
        return -EIO;
    }

    if (udc_ep_disable_internal(dev, USB_CONTROL_EP_IN))
    {
        LOG_ERR("Failed to disable control endpoint");
        return -EIO;
    }

    /*disable ep0 interupt*/
    USBC_Disbale_Irq_Ep0(config->base);

    // clock off
    // pm_policy_state_lock_put(PM_STATE_STANDBY, PM_ALL_SUBSTATES);

    // irq_disable(config->irq_num);
    return 0;
}

/*
 * Prepare and configure most of the parts, if the controller has a way
 * of detecting VBUS activity it should be enabled here.
 * Only udc_ft_enable() makes device visible to the host.
 */
static int udc_ft_init(const struct device *dev)
{
    // struct udc_ft_data *priv = udc_get_private(dev);
    const struct udc_ft_config *config = dev->config;

    LOG_INF("Init device");
    /* USB device address defaults to 0 */
    ft_udc_reset_addr(dev);

    /* Initialize USBD H/W */
    udc_ft_hal_lock(dev);
    USBC_Init(config->base, config->speed_idx);
    udc_ft_hal_unlock(dev);
    /* Initialize all EP H/W contexts */
    irq_enable(config->irq_num);

    return 0;
}

/* Shut down the controller completely */
static int udc_ft_shutdown(const struct device *dev)
{
    const struct udc_ft_config *config = dev->config;
    struct udc_ft_data *priv = udc_get_private(dev);
    LOG_DBG("udc_ft_shutdown device %p\n", dev);

    /*close interrupt*/
    irq_disable(config->irq_num);

    /* DeInit USBD H/W */
    udc_ft_hal_lock(dev);
    USBC_DeInit(config->base, config->speed_idx);
    udc_ft_hal_unlock(dev);
    /* Purge message queue */
    k_msgq_purge(priv->msgq);
    return 0;
}

/*
 * This is called once to initialize the controller and endpoints
 * capabilities, and register endpoint structures.
 */
static int udc_ft_driver_preinit(const struct device *dev)
{
    const struct udc_ft_config *config = dev->config;
    struct udc_data *data = dev->data;
    uint16_t mps = 1023;
    struct udc_ft_data *priv = udc_get_private(dev);
    int err;

    CPM_UsbPhyDeinit();
    k_sleep(K_MSEC(50));

#ifdef CONFIG_PM
    atomic_set(&priv->pm_lock, 0);
    udc_ft_pm_policy_lock_get(dev);
#endif
    /*
     * You do not need to initialize it if your driver does not use
     * udc_lock_internal() / udc_unlock_internal(), but implements its
     * own mechanism.
     */
    k_mutex_init(&data->mutex);
    k_mutex_init(&priv->hal_mutex);

    data->caps.rwup = true;
    data->caps.can_detect_vbus = false;
    // data->caps.can_detect_vbus = true;
    // data->caps.rwup = false;
    // data->caps.addr_before_status = true;
    data->caps.out_ack = true;
    data->caps.mps0 = UDC_MPS0_64; // 3
    if (config->speed_idx == 2)
    { // high_speed
        data->caps.hs = true;
        mps = 512;
    }
    else
    { // full speed
        data->caps.hs = 0;
        mps = 64;
    }

    for (int i = 0; i < config->num_of_eps; i++)
    {
        config->ep_cfg_out[i].caps.out = 1;
        if (i == 0)
        {
            config->ep_cfg_out[i].caps.control = 1;
            config->ep_cfg_out[i].caps.mps = 64;
            // config->ep_cfg_out[i].caps.mps = 8;
        }
        else
        {
            config->ep_cfg_out[i].caps.bulk = 1;
            config->ep_cfg_out[i].caps.interrupt = 1;
            config->ep_cfg_out[i].caps.iso = 1;
            config->ep_cfg_out[i].caps.mps = mps;
        }

        config->ep_cfg_out[i].addr = USB_EP_DIR_OUT | i;
        err = udc_register_ep(dev, &config->ep_cfg_out[i]);
        if (err != 0)
        {
            LOG_ERR("Failed to register endpoint");
            return err;
        }
    }

    for (int i = 0; i < config->num_of_eps; i++)
    {
        config->ep_cfg_in[i].caps.in = 1;
        if (i == 0)
        {
            config->ep_cfg_in[i].caps.control = 1;
            config->ep_cfg_in[i].caps.mps = 64;
            // config->ep_cfg_in[i].caps.mps = 8;
        }
        else
        {
            config->ep_cfg_in[i].caps.bulk = 1;
            config->ep_cfg_in[i].caps.interrupt = 1;
            config->ep_cfg_in[i].caps.iso = 1;
            config->ep_cfg_in[i].caps.mps = mps;
        }

        config->ep_cfg_in[i].addr = USB_EP_DIR_IN | i;
        err = udc_register_ep(dev, &config->ep_cfg_in[i]);
        if (err != 0)
        {
            LOG_ERR("Failed to register endpoint");
            return err;
        }
    }

    CPM_UsbPhyInit(0x02);

    udc_ft_hal_lock(dev);
    USBC_BusReset(config->base, config->speed_idx);
    udc_ft_hal_unlock(dev);

    config->make_thread(dev);
    k_sleep(K_MSEC(10));

    config->irq_config_func(dev);

    // LOG_INF("Device %p (max. speed %d)", dev, config->speed_idx);
    LOG_DBG("Device %p , base :%p, (max. speed %d)\n", dev, config->base, config->speed_idx);

    return pm_device_driver_init(dev, udc_ft_pm_action);
    // return 0;
}

static void udc_ft_lock(const struct device *dev)
{

    udc_lock_internal(dev, K_FOREVER);
}

static void udc_ft_unlock(const struct device *dev)
{
    udc_unlock_internal(dev);
}

/*
 * UDC API structure.
 * Note, you do not need to implement basic checks, these are done by
 * the UDC common layer udc_common.c
 */
static const struct udc_api udc_ft_api = {
    .lock = udc_ft_lock,
    .unlock = udc_ft_unlock,
    .device_speed = udc_ft_device_speed,
    .init = udc_ft_init,
    .enable = udc_ft_enable,
    .disable = udc_ft_disable,
    .shutdown = udc_ft_shutdown,
    .set_address = udc_ft_set_address,
    .host_wakeup = udc_ft_host_wakeup,
    .ep_enable = udc_ft_ep_enable,
    .ep_disable = udc_ft_ep_disable,
    .ep_set_halt = udc_ft_ep_set_halt,
    .ep_clear_halt = udc_ft_ep_clear_halt,
    .ep_enqueue = udc_ft_ep_enqueue,
    .ep_dequeue = udc_ft_ep_dequeue,
};

/*
 * A UDC driver should always be implemented as a multi-instance
 * driver, even if your platform does not require it.
 */

#define UDC_FT_DEVICE_DEFINE(n)                                                                                        \
    static void udc_ft_irq_config_func_##n(const struct device *dev)                                                   \
    {                                                                                                                  \
        IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), ft_udbd_isr, DEVICE_DT_INST_GET(n), 0);                 \
                                                                                                                       \
        /*irq_enable(DT_INST_IRQN(n));       */                                                                        \
    }                                                                                                                  \
                                                                                                                       \
    K_THREAD_STACK_DEFINE(udc_ft_stack_##n, CONFIG_UDC_FT_STACK_SIZE);                                                 \
                                                                                                                       \
    static void udc_ft_thread_##n(void *dev, void *arg1, void *arg2)                                                   \
    {                                                                                                                  \
        ft_thread_handler(dev);                                                                                        \
    }                                                                                                                  \
                                                                                                                       \
    static void udc_ft_make_thread_##n(const struct device *dev)                                                       \
    {                                                                                                                  \
        struct udc_ft_data *priv = udc_get_private(dev);                                                               \
                                                                                                                       \
        k_thread_create(&priv->thread_data, udc_ft_stack_##n, K_THREAD_STACK_SIZEOF(udc_ft_stack_##n),                 \
                        udc_ft_thread_##n, (void *)dev, NULL, NULL, K_PRIO_COOP(CONFIG_UDC_FT_THREAD_PRIORITY),        \
                        K_ESSENTIAL, K_NO_WAIT);                                                                       \
        k_thread_name_set(&priv->thread_data, dev->name);                                                              \
    }                                                                                                                  \
                                                                                                                       \
    static struct udc_ep_config ep_cfg_out_##n[DT_INST_PROP(n, num_bidir_endpoints)];                                  \
    static struct udc_ep_config ep_cfg_in_##n[DT_INST_PROP(n, num_bidir_endpoints)];                                   \
                                                                                                                       \
    static const struct udc_ft_config udc_ft_dev_config_##n = {.base = (FT_USBD_Type *)DT_INST_REG_ADDR(n),            \
                                                               .num_of_eps = DT_INST_PROP(n, num_bidir_endpoints),     \
                                                               .ep_cfg_in = ep_cfg_in_##n,                             \
                                                               .ep_cfg_out = ep_cfg_out_##n,                           \
                                                               .irq_num = DT_INST_IRQN(n),                             \
                                                               .make_thread = udc_ft_make_thread_##n,                  \
                                                               .clk_src = DT_INST_CLOCKS_CELL(n, id),                  \
                                                               .reset = RESET_DT_SPEC_INST_GET(n),                     \
                                                               .speed_idx =                                            \
                                                                   DT_ENUM_IDX(DT_DRV_INST(n), maximum_speed),         \
                                                               .irq_config_func = udc_ft_irq_config_func_##n};         \
                                                                                                                       \
    K_MSGQ_DEFINE(udc_ft_msgq_##n, sizeof(struct udc_ft_msg), CONFIG_UDC_FT_MAX_QMESSAGES, 4);                         \
                                                                                                                       \
    static struct udc_ft_data udc_priv_##n = {                                                                         \
        .msgq = &udc_ft_msgq_##n,                                                                                      \
        .enum_done = false,                                                                                            \
        IF_ENABLED(CONFIG_UDC_FT_DMA, (.status_sem = Z_SEM_INITIALIZER(udc_priv_##n.status_sem, 0, 1), ))};            \
                                                                                                                       \
    static struct udc_data udc_data_##n = {                                                                            \
        .mutex = Z_MUTEX_INITIALIZER(udc_data_##n.mutex),                                                              \
        .priv = &udc_priv_##n,                                                                                         \
    };                                                                                                                 \
    PM_DEVICE_DT_INST_DEFINE(n, udc_ft_pm_action);                                                                     \
    DEVICE_DT_INST_DEFINE(n, udc_ft_driver_preinit, PM_DEVICE_DT_INST_GET(n), &udc_data_##n, &udc_ft_dev_config_##n,   \
                          POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &udc_ft_api);

DT_INST_FOREACH_STATUS_OKAY(UDC_FT_DEVICE_DEFINE)
