/*
 * Copyright (c) 2025 Focaltech Systems CO.,Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ft_ft90_spi

#include <errno.h>
#include <soc.h>
#include <spi_drv.h>
#include <spi_reg.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/pm/policy.h>
#include <zephyr/sys/sys_io.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(spi_ft);

#include <spi/spi_context.h>

struct spi_ft_config
{
    struct spi_config spi_config;
    SPI_TypeDef *base;
    uint32_t clkid;
    struct reset_dt_spec reset;
    int guard_time;
    const struct pinctrl_dev_config *pcfg;
#ifdef CONFIG_SPI_FOCALTECH_INTERRUPT
    void (*irq_configure)();
#endif
};

struct spi_ft_data
{
    struct spi_context ctx;
};

#define DEV_CFG(dev) ((const struct spi_ft_config *)(dev)->config)
#define DEV_DATA(dev) ((struct spi_ft_data *)(dev)->data)

static void spi_ft_complete(const struct device *dev, int status);
static int spi_ft_pm_action(const struct device *dev, enum pm_device_action action)
{
    const struct spi_ft_config *cfg = dev->config;
    const struct device *const clk = DEVICE_DT_GET(DT_NODELABEL(rcc));
    int err;
    printk("action=%x\n", action);

    LOG_INF("spi_ft_pm_action SPI:%p", cfg->base);
    switch (action)
    {
    case PM_DEVICE_ACTION_RESUME:
        /* enable clock */

        err = clock_control_on(clk, (clock_control_subsys_t *)&cfg->clkid);
        if (err != 0)
        {
            LOG_ERR("Could not enable SPI clock");
            return err;
        }
        break;
    case PM_DEVICE_ACTION_SUSPEND:
        /* Stop device clock. */
        printk("sp sus\n");
        err = clock_control_off(clk, (clock_control_subsys_t *)&cfg->clkid);
        if (err != 0)
        {
            LOG_ERR("Could not disable SPI clock");
            return err;
        }
        break;
    default:
        return -ENOTSUP;
    }

    return 0;
}
static int spi_ft_init(const struct device *dev)
{
    const struct device *clk = DEVICE_DT_GET(DT_NODELABEL(rcc));
    const struct spi_ft_config *cfg = dev->config;
    struct spi_ft_data *data = dev->data;
    // struct pinctrl_dev_config *pcfg;

    int ret = 0;
    // printf("spi_ft_init addr :%x\n", cfg->base);
    ret = clock_control_on(clk, (clock_control_subsys_t *)&cfg->clkid);
    if (ret < 0)
    {
        LOG_ERR("Could not enable SPI clock");
        return ret;
    }
    ret = reset_line_toggle_dt(&cfg->reset);
    if (ret < 0)
    {
        LOG_ERR("SPI resets failed");
        return ret;
    }
    if (cfg->pcfg)
    {
        /* Configure dt provided device signals when available */
        ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
        if (ret < 0)
        {
            LOG_ERR("SPI pinctrl setup failed (%d), addr:%p \n", ret, cfg->base);
            return ret;
        }
    }

#ifdef CONFIG_SPI_FOCALTECH_INTERRUPT
    cfg->irq_configure(dev);
#endif

    spi_context_unlock_unconditionally(&data->ctx);

    pm_device_driver_init(dev, spi_ft_pm_action);

    return pm_device_runtime_enable(dev);
}

/*
 * The function to get Scaler and Prescaler for corresponding registers
 * to configure the baudrate for the transmission. The real frequency is
 * computated to ensure it will always equal or the nearest approximation
 * lower to the expected one.
 */
static void spi_ft_get_good_frequency(uint32_t clock_frequency, uint32_t requested_baud, uint8_t *get_prescaler,
                                      uint8_t *get_scaler, uint32_t *good_frequency)
{
    uint8_t scaler;
    uint8_t prescaler;

    uint32_t low, high;
    uint32_t curr_freq;

    uint32_t best_freq = 0U;
    // SPI_FOCALTECH_NUM_PRESCALER
    static const uint8_t prescaler_arr[8] = {1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U};

    // SPI_FOCALTECH_NUM_SCALER
    static const uint16_t scaller_arr[8] = {2U, 4U, 8U, 16U, 32U, 64U, 128U, 256U};

    // printf("sys is :%x, baud is %x\n", clock_frequency, requested_baud);
    for (prescaler = 0U; prescaler < 8; prescaler++)
    {
        low = 0U;
        // high = SPI_FOCALTECH_NUM_SCALER - 1U;
        high = 7U;
        /* Implement golden section search algorithm */
        do
        {
            scaler = (low + high) / 2U;
            curr_freq = clock_frequency * 1U / (prescaler_arr[prescaler] * scaller_arr[scaler]);
            if (curr_freq > requested_baud)
            {
                low = scaler;
                continue;
            }
            else
            {
                high = scaler;
            }

            if ((requested_baud - best_freq) > (requested_baud - curr_freq))
            {
                best_freq = curr_freq;
                *get_prescaler = prescaler;
                *get_scaler = scaler;
            }

            if (best_freq == requested_baud)
            {
                break;
            }
        } while ((high - low) > 1U);

        // printf("step1 : scaler %x, prescaler:%x, low :%x, high:%x\n", scaler, prescaler, low, high);
        if ((high - low) <= 1U)
        {

            if (high == scaler)
            {
                /* use low value */
                scaler = low;
            }
            else
            {
                scaler = high;
            }

            curr_freq = clock_frequency * 1U / (prescaler_arr[prescaler] * scaller_arr[scaler]);

            if (curr_freq <= requested_baud)
            {

                if ((requested_baud - best_freq) > (requested_baud - curr_freq))
                {
                    best_freq = curr_freq;
                    *get_prescaler = prescaler;
                    *get_scaler = scaler;
                }
            }
        }
        if (best_freq == requested_baud)
        {
            break;
        }
    }
    *good_frequency = best_freq;
    // best_baud->frequency = best_freq;
    return;
}
/*
static UINT8 spi_baudrate_map(uint32_t frequency)
{
    uint32_t sys_freq = sys_clock_hw_cycles_per_sec()/2;
    LOG_INF("sys_freq: %x ,frequency :%x\n", sys_freq, frequency);

    if(frequency <= sys_freq/256)
        return SPI_BaudRatePrescaler_256;
    else if(frequency <= sys_freq/64)
        return SPI_BaudRatePrescaler_64;
    else if(frequency <= sys_freq/40)
        return SPI_BaudRatePrescaler_40;
    else if(frequency <= sys_freq/20)
        return SPI_BaudRatePrescaler_20;
    else if(frequency <= sys_freq/10)
        return SPI_BaudRatePrescaler_10;
    else if(frequency <= sys_freq/8)
        return SPI_BaudRatePrescaler_8;
    else if(frequency <= sys_freq/6)
        return SPI_BaudRatePrescaler_6;
    else if(frequency <= sys_freq/4)
        return SPI_BaudRatePrescaler_4;
    else if(frequency <= sys_freq/2)
        return SPI_BaudRatePrescaler_2;
    else
        return SPI_BaudRatePrescaler_error;
}
*/

int spi_transceive_init(const struct device *dev, const struct spi_config *config)
{
    const struct spi_ft_config *cfg = dev->config;
    // ruct spi_ft_data *data = dev->data;
    uint8_t tmp_data;
    SPI_TypeDef *SPIx = cfg->base;
    uint32_t sys_freq = sys_clock_hw_cycles_per_sec() / 2;
    uint8_t spr = 0;
    uint8_t sppr = 0;
    uint32_t good_frequency;

    //	//disable SPI
    SPI_Disable(cfg->base);
    SPIx->SPIPURD = 0x00; // disable pullup/down
    // tmp_data = spi_baudrate_map(config->frequency);
    // SPIx->SPIBR = tmp_data;
    if (config->operation & SPI_OP_MODE_SLAVE)
    {
        SPIx->SPIBR = 0x00;   // default div2
        SPIx->SPIPURD = 0x01; // pullup for spi slave port
    }
    else
    {
        spi_ft_get_good_frequency(sys_freq, config->frequency, &sppr, &spr, &good_frequency);
        SPIx->SPIBR = ((sppr & 0x07) << 4) | (spr & 0x07);
        // printf("sys_freq:%d, baud is%d, good_freq is %d, sppr is %x, spr is : %x\n",sys_freq, config->frequency,
        // good_frequency, sppr, spr); SPIx->SPIPURD = 0x01;   //pullup for spi port
    }

    if ((config->operation & SPI_FRAME_FORMAT_TI) == SPI_FRAME_FORMAT_TI)
    {
        SPIx->SPIFR |= SPIFR_FFSEL_MASK;
    }
    else
    {
        SPIx->SPIFR &= ~SPIFR_FFSEL_MASK;
    }

    if (config->operation & SPI_MODE_CPOL)
    {
        SPIx->SPICR1 |= SPICR1_CPOL_MASK;
    }
    else
    {
        SPIx->SPICR1 &= ~SPICR1_CPOL_MASK;
    }
    if (config->operation & SPI_MODE_CPHA)
    {
        SPIx->SPICR1 |= SPICR1_CPHA_MASK;
    }
    else
    {
        SPIx->SPICR1 &= ~SPICR1_CPHA_MASK;
    }

    if (config->operation & SPI_TRANSFER_LSB)
    {
        SPIx->SPICR1 |= SPICR1_LSBFE_MASK;
    }
    else
    {
        SPIx->SPICR1 &= ~SPICR1_LSBFE_MASK;
    }

    if (config->operation & SPI_OP_MODE_SLAVE)
    {
        SPIx->SPICR1 &= ~SPICR1_MSTR_MASK;

        // SPIx->SPIDDR &= ~(SPIDDR_SS_MASK | SPIDDR_SCK_MASK | SPIDDR_MOSI_MASK);  //SPI_Direction_1Line_RxOrTx
        SPIx->SPIDDR = SPIDDR_MISO_MASK; // SPI DATA Direction: ss is input  sck is input  MOSI is input MISO is output
    }
    else
    {
        SPIx->SPICR1 |= SPICR1_MSTR_MASK;
        // SS high Active
        // SPIx->SPIPORT |= SPIPORT_SS_MASK;
        SPI_CSHigh(cfg->base);

        // SPI DATA Direction: ss is output  sck is output  MOSI is output MISO is input
        SPIx->SPIDDR = SPIDDR_SS_MASK | SPIDDR_SCK_MASK | SPIDDR_MOSI_MASK;
    }

    if (SPI_WORD_SIZE_GET(config->operation) <= 8)
    {
        tmp_data = SPIx->SPIFR & (~SPIFR_FMSZ_MASK);
        tmp_data |= 0x07;
        SPIx->SPIFR = tmp_data;
    }
    else
    {
        tmp_data = SPIx->SPIFR & (~SPIFR_FMSZ_MASK);
        tmp_data |= 0x0F;
        SPIx->SPIFR = tmp_data;
    }

    SPIx->SPIFR &= ~SPIFR_GTE_MASK;
    SPIx->SPICR2 &= ~SPICR2_GT_MASK; // SPI_LINES_SINGLE Only

    if (cfg->guard_time)
    {
        SPIx->SPIFR |= SPIFR_GTE_MASK;
        SPIx->SPICR2 |= ((cfg->guard_time & 0x03F) << 2);
        // printf("guard_time is %x\n", cfg->guard_time);
    }

    // SPIx->SPICR2 |= SPICR2_SPC0_MASK;  //	//SPI_Direction_1Line_RxOrTx
    SPIx->SPICR2 &= ~SPICR2_SPC0_MASK; // SPI_LINES_SINGLE Only
    // SPIx->SPICR2 &= ~SPICR2_GT_MASK      //set GT time
    // SPIx->SPICR2 |= (0x09<<2);           //GT=(GT[5:3]+1)*2^(GT[2:0]+1)
    // SPIx->SPICR2 |= (((0x05<<0)|(0x03<<3))<<2);

    // set_fifo_threshold
    if (config->operation & SPI_OP_MODE_SLAVE)
    {
        SPIx->SPITXFCR = SPITXFCR_TXFCLR_MASK | (0x07); // tx_threshold = 7
        SPIx->SPIRXFCR = SPIRXFCR_RXFCLR_MASK;          // tx_threshold = 0
        SPIx->SPITXFCR &= ~SPITXFCR_TXFCLR_MASK;
        SPIx->SPIRXFCR &= ~SPIRXFCR_RXFCLR_MASK;
    }
    else
    {
        SPIx->SPITXFCR = SPITXFCR_TXFCLR_MASK; // tx_threshold = 0
        SPIx->SPIRXFCR = SPIRXFCR_RXFCLR_MASK; // tx_threshold = 0
        SPIx->SPITXFCR &= ~SPITXFCR_TXFCLR_MASK;
        SPIx->SPIRXFCR &= ~SPIRXFCR_RXFCLR_MASK;
    }

    // Clear SPI FIFO TX/RX Status
    SPIx->SPISRHW = 0xffff;

    return 0;
}

static void spi_ft_frame_shift_s(const struct spi_ft_config *cfg, struct spi_ft_data *data)
{
    const uint8_t frame_size = SPI_WORD_SIZE_GET(data->ctx.config->operation);
    SPI_TypeDef *SPIx = cfg->base;
    uint16_t tx_frame = 0U;
    uint16_t rx_frame = 0U;
    uint16_t status;

    status = SPIx->SPISRHW;

    /*rx not empty*/
    if (!(status & SPISR_RXFEMP_MASK) && spi_context_rx_buf_on(&data->ctx))
    {
        if (frame_size <= 8)
        {
            rx_frame = SPIx->SPIDR;
            UNALIGNED_PUT(rx_frame, (uint8_t *)data->ctx.rx_buf);
            spi_context_update_rx(&data->ctx, 1, 1);
        }
        else
        {
            rx_frame = SPIx->SPIDRHW;
            UNALIGNED_PUT(rx_frame, (uint16_t *)data->ctx.rx_buf);
            spi_context_update_rx(&data->ctx, 2, 1);
        }
    }

    /*slave tx, not full*/
    if (!(status & SPISR_TXFFULL_MASK) && spi_context_tx_on(&data->ctx))
    {
        if (frame_size <= 8)
        {
            tx_frame = UNALIGNED_GET((uint8_t *)(data->ctx.tx_buf));
            SPIx->SPIDR = (uint8_t)tx_frame;
            spi_context_update_tx(&data->ctx, 1, 1);
        }
        else
        {
            tx_frame = UNALIGNED_GET((uint16_t *)(data->ctx.tx_buf));
            SPIx->SPIDRHW = (uint16_t)tx_frame;
            spi_context_update_tx(&data->ctx, 2, 1);
        }
    }

    if (!spi_context_tx_on(&data->ctx))
    {
        SPI_IrqDisableTx(SPIx);
    }
}

static void spi_ft_read_next_frame(const struct spi_ft_config *cfg, struct spi_ft_data *data)
{
    const uint8_t frame_size = SPI_WORD_SIZE_GET(data->ctx.config->operation);
    SPI_TypeDef *SPIx = cfg->base;
    uint16_t rx_frame = 0;

    if (frame_size <= 8)
    {
        rx_frame = SPIx->SPIDR;
        if (spi_context_rx_buf_on(&data->ctx))
        {
            UNALIGNED_PUT(rx_frame, (uint8_t *)data->ctx.rx_buf);
        }
        spi_context_update_rx(&data->ctx, 1, 1);
    }
    else
    {
        rx_frame = SPIx->SPIDRHW;
        if (spi_context_rx_buf_on(&data->ctx))
        {
            UNALIGNED_PUT(rx_frame, (uint16_t *)data->ctx.rx_buf);
        }
        spi_context_update_rx(&data->ctx, 2, 1);
    }
}

static void spi_ft_send_next_frame(const struct spi_ft_config *cfg, struct spi_ft_data *data)
{
    const uint8_t frame_size = SPI_WORD_SIZE_GET(data->ctx.config->operation);
    SPI_TypeDef *SPIx = cfg->base;
    uint32_t tx_frame = 0;

    if (frame_size <= 8)
    {
        if (spi_context_tx_buf_on(&data->ctx))
        {
            tx_frame = UNALIGNED_GET((uint8_t *)(data->ctx.tx_buf));
        }
        SPIx->SPIDR = (uint8_t)tx_frame;
        spi_context_update_tx(&data->ctx, 1, 1);
    }
    else
    {
        if (spi_context_tx_buf_on(&data->ctx))
        {
            tx_frame = UNALIGNED_GET((uint16_t *)(data->ctx.tx_buf));
        }
        SPIx->SPIDRHW = (uint16_t)tx_frame;
        spi_context_update_tx(&data->ctx, 2, 1);
    }
    if (!spi_context_tx_buf_on(&data->ctx))
    {
        SPI_IrqDisableTx(SPIx);
    }
}

static void spi_ft_frame_shift_m(const struct spi_ft_config *cfg, struct spi_ft_data *data)
{
    uint16_t status;
    SPI_TypeDef *SPIx = cfg->base;
    status = SPIx->SPISRHW;
    // printf("misr st:%x\n", status);
    /*RX FIFO no empty*/
    if (!(status & SPISR_RXFEMP_MASK))
    {
        spi_ft_read_next_frame(cfg, data);
    }

    /*TX FIFO empty*/
    if (status & SPISR_TXFEMP_MASK)
    {
        spi_ft_send_next_frame(cfg, data);
    }
}

static int spi_ft_shift_frames(const struct spi_ft_config *cfg, struct spi_ft_data *data)
{
    uint16_t operation = data->ctx.config->operation;

    if (SPI_OP_MODE_GET(operation) == SPI_OP_MODE_MASTER)
    {
        spi_ft_frame_shift_m(cfg, data);
    }
    else
    {
        spi_ft_frame_shift_s(cfg, data);
    }
    return SPI_GetStatusErr(cfg->base);
}

static bool spi_ft_transfer_ongoing(struct spi_ft_data *data)
{
    return spi_context_tx_on(&data->ctx) || spi_context_rx_on(&data->ctx);
}

static int spi_ft_configure(const struct device *dev, const struct spi_config *config)
{
    struct spi_ft_data *data = dev->data;

    if (spi_context_configured(&data->ctx, config))
        return 0;

    spi_transceive_init(dev, config);

    data->ctx.config = config;
    return 0;
}

static int spi_ft_transceive_impl(const struct device *dev, const struct spi_config *config,
                                  const struct spi_buf_set *tx_bufs, const struct spi_buf_set *rx_bufs,
                                  bool asynchronous, spi_callback_t cb, void *userdata)
{
    struct spi_ft_data *data = dev->data;
    const struct spi_ft_config *cfg = dev->config;
    int ret = 0;

    if (!tx_bufs && !rx_bufs)
    {
        return 0;
    }

#ifndef CONFIG_SPI_FOCALTECH_INTERRUPT
    if (asynchronous)
    {
        return -ENOTSUP;
    }
#endif /* CONFIG_SPI_FOCALTECH_INTERRUPT */

    // spi_ft_pm_action(dev,PM_DEVICE_ACTION_RESUME);

    /* context setup */
    spi_context_lock(&data->ctx, asynchronous, cb, userdata, config);

    /* set configuration */
    ret = spi_ft_configure(dev, config);
    if (ret)
    {
        goto out;
    }

    /* Set buffers info */
    if (SPI_WORD_SIZE_GET(config->operation) <= 8)
    {
        spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, 1);
    }
    else
    {
        spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, 2);
    }

    /* enable spi*/
    SPI_Enable(cfg->base);

    /* cs true*/
    if (SPI_OP_MODE_GET(config->operation) == SPI_OP_MODE_MASTER)
    {
        SPI_CSLow(cfg->base);
    }

    /*enable interrupt*/
#ifdef CONFIG_SPI_FOCALTECH_INTERRUPT
    if (spi_context_tx_on(&data->ctx))
    {
        SPI_IrqEnableTxErrors(cfg->base);
    }
    if (spi_context_rx_on(&data->ctx))
    {
        SPI_IrqEnableRx(cfg->base);
        SPI_IrqEnableRxErrors(cfg->base);
    }

    SPI_IrqEnableTx(cfg->base);
    SPI_IrqEnable(cfg->base);

    /*wait until spi_context_complete*/
    ret = spi_context_wait_for_completion(&data->ctx);
#else // cpu normal
    /* transceive data */
    do
    {
        ret = spi_ft_shift_frames(cfg, data);
        if (ret != 0)
        {
            LOG_INF("cpu normal cfg->base:%x SPIx->SPISRHW: %x ret:%x\n", cfg->base, SPIx->SPISRHW, ret);
            break;
        }
    } while (spi_ft_transfer_ongoing(data));

    spi_ft_complete(dev, ret);

#ifdef CONFIG_SPI_SLAVE
    if (spi_context_is_slave(&data->ctx) && !ret)
    {
        ret = data->ctx.recv_frames;
    }
#endif /* CONFIG_SPI_SLAVE */

#endif /* CONFIG_SPI_FOCALTECH_INTERRUPT */

out:
    spi_context_release(&data->ctx, ret);
    //	spi_ft_pm_action(dev,PM_DEVICE_ACTION_SUSPEND);

    return ret;
}

static int spi_ft_transceive(const struct device *dev, const struct spi_config *config,
                             const struct spi_buf_set *tx_bufs, const struct spi_buf_set *rx_bufs)
{
    return spi_ft_transceive_impl(dev, config, tx_bufs, rx_bufs, false, NULL, NULL);
}

#ifdef CONFIG_SPI_ASYNC
/* API implementation: transceive_async */
static int spi_ft_transceive_async(const struct device *dev, const struct spi_config *config,
                                   const struct spi_buf_set *tx_bufs, const struct spi_buf_set *rx_bufs,
                                   spi_callback_t cb, void *userdata)
{
    return spi_ft_transceive_impl(dev, config, tx_bufs, rx_bufs, true, cb, userdata);
}
#endif /* CONFIG_SPI_ASYNC */

static int spi_ft_release(const struct device *dev, const struct spi_config *config)
{
    struct spi_ft_data *data = DEV_DATA(dev);
    const struct spi_ft_config *cfg = dev->config;

    spi_context_unlock_unconditionally(&data->ctx);

    SPI_Disable(cfg->base);

    //	spi_ft_pm_action(dev,PM_DEVICE_ACTION_SUSPEND);
    return 0;
}

static void spi_ft_complete(const struct device *dev, int status)
{
    struct spi_ft_data *data = dev->data;
    const struct spi_ft_config *cfg = dev->config;
    const uint8_t frame_size = SPI_WORD_SIZE_GET(data->ctx.config->operation);
    uint32_t sys_freq = sys_clock_hw_cycles_per_sec();
    int guard_time;
    int i;
    guard_time = frame_size * sys_freq / (data->ctx.config->frequency * 6); // 1*frames， 6instructs for each loop
#ifdef CONFIG_SPI_FOCALTECH_INTERRUPT
    /*disable interrupt*/
    SPI_IrqDisable(cfg->base);
    SPI_IrqDisableRx(cfg->base);
    SPI_IrqDisableTx(cfg->base);
    SPI_IrqDisableRxErrors(cfg->base);
    SPI_IrqDisableTxErrors(cfg->base);
#endif

    /* Flush RX buffer */
    if (frame_size <= 8)
    {
        SPI_RxReadAll(cfg->base, 8);
    }
    else
    {
        SPI_RxReadAll(cfg->base, 16);
    }

    // wait till spi is not busy, all tx is send out
    if (SPI_OP_MODE_GET(data->ctx.config->operation) == SPI_OP_MODE_MASTER)
    {
        SPI_TxTranserDone(cfg->base);
        for (i = 0; i < guard_time; i++)
        {
            __asm("nop");
            __asm("nop");
        }
        SPI_CSHigh(cfg->base);
    }
#ifdef CONFIG_SPI_FOCALTECH_INTERRUPT
    spi_context_complete(&data->ctx, dev, status);
#endif
}

static void spi_ft_isr(struct device *dev)
{
    const struct spi_ft_config *cfg = dev->config;
    struct spi_ft_data *data = dev->data;
    int err = 0;
    err = SPI_GetStatusErr(cfg->base);
    if (err)
    {
        spi_ft_complete(dev, err);
        LOG_ERR("spi error status detected, err = %x", err);
        return;
    }

    if (spi_ft_transfer_ongoing(data))
    {
        err = spi_ft_shift_frames(cfg, data);
        if (err)
        {
            LOG_ERR("spi_ft_shift_frames err = %x", err);
        }
    }

    if (err || !spi_ft_transfer_ongoing(data))
    {
        spi_ft_complete(dev, err);
    }
}

static const struct spi_driver_api spi_ft_driver_api = {
    .transceive = spi_ft_transceive,
    .release = spi_ft_release,
#ifdef CONFIG_SPI_ASYNC
    .transceive_async = spi_ft_transceive_async,
#endif /* CONFIG_SPI_ASYNC */
};

#define FOCALTECH_IRQ_CONFIGURE(idx)                                                                                   \
    static void spi_ft_irq_configure_##idx(void)                                                                       \
    {                                                                                                                  \
        IRQ_CONNECT(DT_INST_IRQN(idx), DT_INST_IRQ(idx, priority), spi_ft_isr, DEVICE_DT_INST_GET(idx), 0);            \
        irq_enable(DT_INST_IRQN(idx));                                                                                 \
    }

#define SPI_FOCALTECH_DEVICE(idx)                                                                                      \
    IF_ENABLED(CONFIG_SPI_FOCALTECH_INTERRUPT, (FOCALTECH_IRQ_CONFIGURE(idx)));                                        \
    static struct spi_ft_data spi_ft_data_##idx = {SPI_CONTEXT_INIT_LOCK(spi_ft_data_##idx, ctx),                      \
                                                   SPI_CONTEXT_INIT_SYNC(spi_ft_data_##idx, ctx)};                     \
    static const struct spi_ft_config spi_ft_dev_cfg_##idx = {                                                         \
        .base = (SPI_TypeDef *)DT_INST_REG_ADDR(idx),                                                                  \
        .clkid = DT_INST_CLOCKS_CELL(idx, id),                                                                         \
        .reset = RESET_DT_SPEC_INST_GET(idx),                                                                          \
        .guard_time = DT_INST_PROP(idx, guard_time),                                                                   \
        IF_ENABLED(CONFIG_SPI_FOCALTECH_INTERRUPT, (.irq_configure = spi_ft_irq_configure_##idx))};                    \
    DEVICE_DT_INST_DEFINE(idx, spi_ft_init, NULL, &spi_ft_data_##idx, &spi_ft_dev_cfg_##idx, POST_KERNEL,              \
                          CONFIG_SPI_INIT_PRIORITY, (void *)&spi_ft_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SPI_FOCALTECH_DEVICE);
