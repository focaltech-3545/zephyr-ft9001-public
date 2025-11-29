/*
 * Copyright (c) 2025 Focaltech Systems CO.,Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ft_ft90_usart

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/sys_io.h>

#include <soc.h>
#include <uart_drv.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/pm/policy.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(uart_ft);

struct uart_ft_config
{
    UART_TypeDef *base;
    uint32_t clkid;
    struct reset_dt_spec reset;
    const struct pinctrl_dev_config *pcfg;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
    uart_irq_config_func_t irq_config_func;
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
};

enum uart_ft_pm_lock
{
    UART_FT_PM_LOCK_TX,
    UART_FT_PM_LOCK_TX_POLL,
    UART_FT_PM_LOCK_RX,
    UART_FT_PM_LOCK_COUNT,
};

struct uart_ft_data
{
    /* uart config */
    struct uart_config *uart_cfg;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
    uart_irq_callback_user_data_t cb; /**< Callback function pointer */
    void *cb_data;                    /**< Callback function arg */
#endif                                /* CONFIG_UART_INTERRUPT_DRIVEN */
#ifdef CONFIG_PM
    ATOMIC_DEFINE(pm_lock, UART_FT_PM_LOCK_COUNT);
#endif /*CONFIG_PM*/
};

#define DEV_CFG(dev) ((const struct uart_ft_config *)(dev)->config)
#define DEV_DATA(dev) ((struct uart_ft_data *)(dev)->data)

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void uart_ft_isr(const struct device *dev);
#endif /*CONFIG_UART_INTERRUPT_DRIVEN*/

static int uart_ft_pm_action(const struct device *dev, enum pm_device_action action)
{
    int err = 0;
    const struct uart_ft_config *cfg = dev->config;
    const struct device *const clk = DEVICE_DT_GET(DT_NODELABEL(rcc));

    if (action == PM_DEVICE_ACTION_RESUME)
    {
        err = clock_control_on(clk, (clock_control_subsys_t *)&cfg->clkid);
    }
    else if (IS_ENABLED(CONFIG_PM_DEVICE) && (action == PM_DEVICE_ACTION_SUSPEND))
    {
        err = clock_control_off(clk, (clock_control_subsys_t *)&cfg->clkid);
    }
    else
    {
        return -ENOTSUP;
    }
    return 0;
}

/**
 * @brief Get PM lock on low power states
 *
 * @param dev  UART device struct
 * @param lock UART PM lock type
 *
 * @return true if lock was taken, false otherwise
 */
static bool uart_ft_pm_lock_get(const struct device *dev, enum uart_ft_pm_lock lock)
{
#ifdef CONFIG_PM
    struct uart_ft_data *data = dev->data;
    bool was_locked = atomic_test_and_set_bit(data->pm_lock, lock);

    if (!was_locked)
    {
        /* Lock out low-power states that would interfere with UART traffic */
        pm_policy_state_lock_get(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);
    }
    return !was_locked;
#endif
    return false;
}

/**
 * @brief Release PM lock on low power states
 *
 * @param dev  UART device struct
 * @param lock UART PM lock type
 *
 * @return true if lock was released, false otherwise
 */
static bool uart_ft_pm_lock_put(const struct device *dev, enum uart_ft_pm_lock lock)
{
#ifdef CONFIG_PM
    struct uart_ft_data *data = dev->data;
    bool was_locked = atomic_test_and_clear_bit(data->pm_lock, lock);
    if (was_locked)
    {
        /* Unlock low-power states that would interfere with UART traffic */
        pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);
    }
    return was_locked;
#endif /*CONFIG_PM*/
    return false;
}

static int uart_ft_init(const struct device *dev)
{
    UART_InitTypeDef UART_InitStruct;
    const struct device *clk = DEVICE_DT_GET(DT_NODELABEL(rcc));
    const struct uart_ft_config *config = DEV_CFG(dev);
    const struct uart_ft_data *data = DEV_DATA(dev);
    int ret = 0;

    ret = clock_control_on(clk, (clock_control_subsys_t *)&config->clkid);
    if (ret < 0)
    {
        LOG_ERR("Could not enable UART clock");
        return ret;
    }

    ret = reset_line_toggle_dt(&config->reset);
    if (ret < 0)
    {
        LOG_ERR("UART resets failed");
        return ret;
    }

    if (config->pcfg)
    {
        /* Configure dt provided device signals when available */
        ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
        if (ret < 0)
        {
            LOG_ERR("UART pinctrl setup failed (%d), addr:%p \n", ret, config->base);
            return ret;
        }
    }
    /*init uart, interrupt mode*/
    UART_InitStruct.UART_BaudRate = data->uart_cfg->baudrate;
    UART_InitStruct.UART_Mode = UART_INT_MODE;

    if (UART_CFG_PARITY_ODD == data->uart_cfg->parity)
        UART_InitStruct.UART_Parity = UART_PARITY_ODD;
    else if (UART_CFG_PARITY_EVEN == data->uart_cfg->parity)
        UART_InitStruct.UART_Parity = UART_PARITY_EVE;
    else if (UART_CFG_PARITY_NONE == data->uart_cfg->parity)
        UART_InitStruct.UART_Parity = UART_PARITY_NONE;
    else
        return -ENOTSUP;

    /*only support 1bit stopbits*/
    if (UART_CFG_STOP_BITS_1 != data->uart_cfg->stop_bits)
        return -ENOTSUP;
    UART_InitStruct.UART_StopBits = 1;

    if (UART_CFG_DATA_BITS_9 == data->uart_cfg->data_bits)
        UART_InitStruct.UART_FrameLength = UART_DATA_FRAME_LEN_11BIT;
    else if (UART_CFG_DATA_BITS_8 == data->uart_cfg->data_bits)
        UART_InitStruct.UART_FrameLength = UART_DATA_FRAME_LEN_10BIT;
    else
        return -ENOTSUP;

    /*NO support flow ctrl*/
    if (UART_CFG_FLOW_CTRL_NONE != data->uart_cfg->flow_ctrl)
        return -ENOTSUP;

    /*get sysy frequency and Initialize HW*/
    uint32_t sys_freq = sys_clock_hw_cycles_per_sec() / 2;
    UART_Init(config->base, &UART_InitStruct, sys_freq);

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
    config->irq_config_func(dev);
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

    return pm_device_driver_init(dev, uart_ft_pm_action);
}

static int uart_ft_poll_in(const struct device *dev, unsigned char *c)
{
    const struct uart_ft_config *config = DEV_CFG(dev);
    int ret = 0;
    if (UART_PollIn(config->base, c) != 0)
        ret = -EPERM;

    return ret;
}

static void uart_ft_poll_out(const struct device *dev, unsigned char c)
{
    const struct uart_ft_config *config = DEV_CFG(dev);

    UART_PollOut(config->base, c);
    return;
}

static int uart_ft_err_check(const struct device *dev)
{
    int ret = 0;
    const struct uart_ft_config *config = DEV_CFG(dev);
    ret = UART_IrqErrCheck(config->base);
    return ret;
}

#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
static int uart_ft_configure(const struct device *dev, const struct uart_config *cfg)
{
    const struct uart_ft_data *data = DEV_DATA(dev);
    struct uart_config *uart_cfg = data->uart_cfg;
    const struct uart_ft_config *config = DEV_CFG(dev);
    UART_InitTypeDef UART_InitStruct;

    /*uart_config, interrupt mode*/
    UART_InitStruct.UART_BaudRate = cfg->baudrate;
    UART_InitStruct.UART_Mode = UART_INT_MODE;

    /*uart_parity*/
    if (cfg->parity == UART_CFG_PARITY_EVEN)
        UART_InitStruct.UART_Parity = UART_PARITY_EVE;
    else if (cfg->parity == UART_CFG_PARITY_ODD)
        UART_InitStruct.UART_Parity = UART_PARITY_ODD;
    else if (cfg->parity == UART_CFG_PARITY_NONE)
        UART_InitStruct.UART_Parity = UART_PARITY_NONE;
    else
        return -ENOTSUP;

    /*only support 1bit stopbits*/
    if (cfg->stop_bits != UART_CFG_STOP_BITS_1)
        return -ENOTSUP;

    /*HW only support 8bit and 9bit data len*/
    if (cfg->data_bits == UART_CFG_DATA_BITS_8)
        UART_InitStruct.UART_FrameLength = UART_DATA_FRAME_LEN_10BIT;
    else if (cfg->data_bits == UART_CFG_DATA_BITS_9)
        UART_InitStruct.UART_FrameLength = UART_DATA_FRAME_LEN_11BIT;
    else
        return -ENOTSUP;

    /*NO support flow ctrl*/
    if (cfg->flow_ctrl != UART_CFG_FLOW_CTRL_NONE)
        return -ENOTSUP;

    /*set HW*/
    uint32_t sys_freq = sys_clock_hw_cycles_per_sec() / 2;
    UART_Configure(config->base, &UART_InitStruct, sys_freq);

    /*copy uart_config to dev->data*/
    *uart_cfg = *cfg;

    return 0;
}

static int uart_ft_config_get(const struct device *dev, struct uart_config *cfg)
{
    const struct uart_ft_data *data = DEV_DATA(dev);
    const struct uart_config *uart_cfg = data->uart_cfg;

    cfg->baudrate = uart_cfg->baudrate;
    cfg->parity = uart_cfg->parity;
    cfg->stop_bits = uart_cfg->stop_bits;
    cfg->data_bits = uart_cfg->data_bits;
    cfg->flow_ctrl = uart_cfg->flow_ctrl;

    return 0;
}
#endif /*CONFIG_UART_USE_RUNTIME_CONFIGURE*/

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static int uart_ft_fifo_fill(const struct device *dev, const uint8_t *tx_data, int size)
{
    const struct uart_ft_config *config = DEV_CFG(dev);
    int num_tx = 0;
    int ret = -1;
    for (num_tx = 0; num_tx < size; num_tx++)
    {
        ret = UART_FifoFill(config->base, tx_data[num_tx]);
        if (0 != ret)
            break;
    }

    return num_tx;
}

static int uart_ft_fifo_read(const struct device *dev, uint8_t *rx_data, const int size)
{
    const struct uart_ft_config *config = DEV_CFG(dev);
    int num_rx = 0U;

    num_rx = UART_FifoRead(config->base, rx_data, size);

    return (int)num_rx;
}

static void uart_ft_irq_tx_enable(const struct device *dev)
{
    const struct uart_ft_config *config = DEV_CFG(dev);

    (void)uart_ft_pm_lock_get(dev, UART_FT_PM_LOCK_TX);
    UART_IrqTxEnable(config->base);

    return;
}

static void uart_ft_irq_tx_disable(const struct device *dev)
{
    const struct uart_ft_config *config = DEV_CFG(dev);

    UART_IrqTxDisable(config->base);
    (void)uart_ft_pm_lock_put(dev, UART_FT_PM_LOCK_TX);

    return;
}

static int uart_ft_irq_tx_ready(const struct device *dev)
{
    int ret;
    const struct uart_ft_config *config = DEV_CFG(dev);

    ret = UART_IrqTxReady(config->base);
    if (0 != ret)
        return 0;
    else
        return 1;
}

static void uart_ft_irq_rx_enable(const struct device *dev)
{
    const struct uart_ft_config *config = DEV_CFG(dev);

    (void)uart_ft_pm_lock_get(dev, UART_FT_PM_LOCK_RX);
    UART_IrqRxEnable(config->base);

    return;
}

static void uart_ft_irq_rx_disable(const struct device *dev)
{
    const struct uart_ft_config *config = DEV_CFG(dev);

    UART_IrqRxDisable(config->base);
    (void)uart_ft_pm_lock_put(dev, UART_FT_PM_LOCK_RX);

    return;
}

static int uart_ft_irq_tx_complete(const struct device *dev)
{
    int ret;
    const struct uart_ft_config *config = DEV_CFG(dev);

    ret = UART_IrqTxComplete(config->base);
    if (0 != ret)
        return 0;
    else
        return 1;
}

static int uart_ft_irq_rx_ready(const struct device *dev)
{
    int ret;
    const struct uart_ft_config *config = DEV_CFG(dev);

    ret = UART_IrqRxReady(config->base);
    if (0 != ret)
        return 0;
    else
        return 1;
}

static void uart_ft_irq_err_enable(const struct device *dev)
{
    const struct uart_ft_config *config = DEV_CFG(dev);
    /* Not yet used in zephyr */
    UART_IrqErrEnable(config->base);

    return;
}

static void uart_ft_irq_err_disable(const struct device *dev)
{
    const struct uart_ft_config *config = DEV_CFG(dev);

    /* Not yet used in zephyr */
    UART_IrqErrDisable(config->base);

    return;
}

/* Interrupt driven pending status function */
static int uart_ft_irq_is_pending(const struct device *dev)
{
    int ret;
    const struct uart_ft_config *config = DEV_CFG(dev);

    ret = UART_IrqIsPending(config->base);
    if (0 != ret)
        return -EPERM;
    else
        return 1;
}

static int uart_ft_irq_update(const struct device *dev)
{
    return 1;
}

static void uart_ft_irq_callback_set(const struct device *dev, uart_irq_callback_user_data_t cb, void *cb_data)
{
    struct uart_ft_data *const dev_data = DEV_DATA(dev);

    dev_data->cb = cb;
    dev_data->cb_data = cb_data;

    return;
}

/**
 * @brief Interrupt service routine.
 *
 * This simply calls the callback function, if one exists.
 *
 * Note: CC32XX UART Tx interrupts when ready to send; Rx interrupts when char
 * received.
 *
 * @param arg Argument to ISR.
 *
 * @return N/A
 */
static void uart_ft_isr(const struct device *dev)
{
    struct uart_ft_data *const dev_data = DEV_DATA(dev);
    if (dev_data->cb)
    {
        dev_data->cb(dev, dev_data->cb_data);
    }
    /*
     * RX/TX interrupt should have been implicitly cleared by Zephyr UART
     * clients calling uart_fifo_read() or uart_fifo_write().
     * Still, clear any error interrupts here, as they're not yet handled.
     */
}
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

static const struct uart_driver_api uart_ft_driver_api = {
    .poll_in = uart_ft_poll_in,
    .poll_out = uart_ft_poll_out,
    .err_check = uart_ft_err_check,
#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
    .configure = uart_ft_configure,
    .config_get = uart_ft_config_get,
#endif /* CONFIG_UART_USE_RUNTIME_CONFIGURE */
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
    .fifo_fill = uart_ft_fifo_fill,
    .fifo_read = uart_ft_fifo_read,
    .irq_tx_enable = uart_ft_irq_tx_enable,
    .irq_tx_disable = uart_ft_irq_tx_disable,
    .irq_tx_ready = uart_ft_irq_tx_ready,
    .irq_rx_enable = uart_ft_irq_rx_enable,
    .irq_rx_disable = uart_ft_irq_rx_disable,
    .irq_tx_complete = uart_ft_irq_tx_complete,
    .irq_rx_ready = uart_ft_irq_rx_ready,
    .irq_err_enable = uart_ft_irq_err_enable,
    .irq_err_disable = uart_ft_irq_err_disable,
    .irq_is_pending = uart_ft_irq_is_pending,
    .irq_update = uart_ft_irq_update,
    .irq_callback_set = uart_ft_irq_callback_set,
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
};

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
#define FOCALTECH_UART_IRQ_HANDLER(idx)                                                                                \
    static void uart_ft_cfg_func_##idx(const struct device *dev)                                                       \
    {                                                                                                                  \
        IRQ_CONNECT(DT_INST_IRQN(idx), DT_INST_IRQ(idx, priority), uart_ft_isr, DEVICE_DT_INST_GET(idx), 0);           \
        irq_enable(DT_INST_IRQN(idx));                                                                                 \
    }
#define FOCALTECH_UART_IRQ_HANDLER_FUNC_INIT(idx) .irq_config_func = uart_ft_cfg_func_##idx
#else /* CONFIG_UART_INTERRUPT_DRIVEN */
#define FOCALTECH_UART_IRQ_HANDLER(idx)
#define FOCALTECH_UART_IRQ_HANDLER_FUNC_INIT(idx)
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

#define UART_FOCALTECH_DEVICE(idx)                                                                                     \
    FOCALTECH_UART_IRQ_HANDLER(idx)                                                                                    \
    static struct uart_config uart_cfg_##idx = {                                                                       \
        .baudrate = DT_INST_PROP(idx, current_speed),                                                                  \
        .parity = UART_CFG_PARITY_NONE,                                                                                \
        .stop_bits = UART_CFG_STOP_BITS_1,                                                                             \
        .data_bits = UART_CFG_DATA_BITS_8,                                                                             \
        .flow_ctrl = UART_CFG_FLOW_CTRL_NONE,                                                                          \
    };                                                                                                                 \
    static struct uart_ft_data uart_ft_dev_data_##idx = {                                                              \
        .uart_cfg = &uart_cfg_##idx,                                                                                   \
    };                                                                                                                 \
    static const struct uart_ft_config uart_ft_dev_cfg_##idx = {                                                       \
        .base = (UART_TypeDef *)DT_INST_REG_ADDR(idx),                                                                 \
        .clkid = DT_INST_CLOCKS_CELL(idx, id),                                                                         \
        .reset = RESET_DT_SPEC_INST_GET(idx),                                                                          \
        .pcfg = COND_CODE_1(DT_INST_NODE_HAS_PROP(idx, pinctrl_0), (PINCTRL_DT_INST_DEV_CONFIG_GET(idx)), (NULL)),     \
        FOCALTECH_UART_IRQ_HANDLER_FUNC_INIT(idx)};                                                                    \
    PM_DEVICE_DT_INST_DEFINE(idx, uart_ft_pm_action);                                                                  \
    DEVICE_DT_INST_DEFINE(idx, uart_ft_init, PM_DEVICE_DT_INST_GET(idx), &uart_ft_dev_data_##idx,                      \
                          &uart_ft_dev_cfg_##idx, PRE_KERNEL_1, CONFIG_SERIAL_INIT_PRIORITY,                           \
                          (void *)&uart_ft_driver_api);

DT_INST_FOREACH_STATUS_OKAY(UART_FOCALTECH_DEVICE);
