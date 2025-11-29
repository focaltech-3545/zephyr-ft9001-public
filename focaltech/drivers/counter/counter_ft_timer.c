/*
 * Copyright (c) 2025 Focaltech Systems CO.,Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ft_ft90_timer

#include <tc_drv.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

LOG_MODULE_REGISTER(counter_ft_timer);
struct counter_ft_ch_data
{
    counter_alarm_callback_t callback;
    void *user_data;
};

struct counter_ft_data
{
    counter_top_callback_t top_cb;
    void *top_user_data;
    uint32_t guard_period;
    atomic_t cc_int_pending;
    uint32_t freq;
    bool read_flag;
    bool alarm_flag;
    struct counter_ft_ch_data alarm[];
};

struct counter_ft_config
{
    struct counter_config_info counter_info;
    TC_TypeDef *base;
    uint16_t clkid;
    struct reset_dt_spec reset;
    uint16_t prescaler;
    void (*irq_config)(const struct device *dev);
    void (*set_irq_pending)(void);
    uint32_t (*get_irq_pending)(void);
};

/**
 * @brief Start the timer device.
 *
 * - dev: Timer device.
 * @return 0 on success.
 */
static int counter_ft_timer_start(const struct device *dev)
{
    LOG_INF("Device is running");
    return 0;
}

/**
 * @brief Stop the timer device.
 *
 *- dev: Timer device.
 * @return -ENOTSUP ,the device doesn't support stopping the
 *                   counter.
 */
static int counter_ft_timer_stop(const struct device *dev)
{
    LOG_ERR("Device doesn't support stopping the counter! ! !");
    return -ENOTSUP;
}

/**
 * @brief Get the current timer counter value.
 *
 * This function retrieves the current counter value of the timer and stores
 * it in the provided ticks pointer.
 *
 * - dev: Timer device.
 * -ticks: Pointer to a variable where the current counter value will be stored.
 *
 * @return 0 on success.
 */
static int counter_ft_timer_get_value(const struct device *dev, uint32_t *ticks)
{
    uint32_t temp_v;
    uint32_t temp_c;
    struct counter_ft_data *data = dev->data;
    const struct counter_ft_config *cfg = dev->config;
    struct counter_ft_data *data_ft = dev->data;

    /* Low-power mode timing calculation */
    temp_v = Timer_GetLoadvalue(cfg->base);
    temp_c = Timer_GetCount(cfg->base);

    if (temp_v >= temp_c)
    {
        *ticks = temp_v - temp_c;
    }
    else
    {
        *ticks = 0;
    }

    return 0;
}

/**
 * @brief Set an alarm on the timer.
 *
 * This function configures an alarm to trigger when the timer counter reaches
 * a specified value. When the alarm triggers, the configured callback function
 * will be executed.
 *
 * @param dev Pointer to the timer device structure.
 * @param chan_id Alarm channel identifier (reserved for future use, should be 0).
 * @param alarm_cfg Pointer to the alarm configuration structure containing:
 *        - ticks: Number of ticks at which to trigger the alarm (support relative)
 *        - callback: Function pointer to be called when alarm triggers
 *        - user_data: User context data passed to callback function
 *        - flags: Configuration flags (e.g., COUNTER_ALARM_CFG_ABSOLUTE for
 *                 absolute time, COUNTER_ALARM_CFG_PERIODIC for periodic alarm)
 *
 *
 * @retval 0 Alarm successfully set.
 */
static int counter_ft_timer_set_alarm(const struct device *dev, uint8_t chan, const struct counter_alarm_cfg *alarm_cfg)
{
    const struct counter_ft_config *cfg = dev->config;
    struct counter_ft_data *data = dev->data;
    struct counter_ft_ch_data *chdata = &data->alarm[chan];
    if (data->alarm_flag)
    {
        LOG_ERR("Alarm channel 0 already in use. Cancel first before setting a new one.");
        return -EBUSY;
    }

    Timer_IntDisable(cfg->base);
#ifdef CONFIG_CORTEX_M_SYSTICK_LPM_TIMER_COUNTER
    /* Set timer to Doze mode for low-power operation */
    Timer_SetMode(cfg->base, 3);
#else
    /* Set timer to Normal mode for standard operation */
    Timer_SetMode(cfg->base, 0);
#endif

    if (alarm_cfg->ticks > (cfg->counter_info).max_top_value)
    {
        Timer_SetAlarmValue(cfg->base, (cfg->counter_info).max_top_value);
    }
    else
    {
        Timer_SetAlarmValue(cfg->base, alarm_cfg->ticks);
    }

    Timer_IntFlagClear(cfg->base);

    data->alarm_flag = true;
    chdata->callback = alarm_cfg->callback;
    chdata->user_data = alarm_cfg->user_data;
    Timer_IntEnable(cfg->base);

    return 0;
}
/**
 * @brief Cancel a previously set alarm on the timer.
 *
 * This function cancels an active alarm configuration on the specified channel.
 * After cancellation, the alarm will not trigger and any associated callback
 * will not be executed. If the alarm was periodic, the periodic behavior
 * will be stopped.
 *
 * @param dev Pointer to the timer device structure.
 * @param chan_id Alarm channel identifier (reserved for future use, should be 0).
 *
 *
 * @retval 0 Alarm successfully cancelled or no alarm was active.
 */
static int counter_ft_timer_cancel_alarm(const struct device *dev, uint8_t chan)
{
    const struct counter_ft_config *cfg = dev->config;
    struct counter_ft_data *data = dev->data;

    data->alarm_flag = false;
    Timer_IntFlagClear(cfg->base);
    Timer_IntDisable(cfg->base);
    data->alarm[chan].callback = NULL;

    return 0;
}
/*
 * @brief Get the current top value of the timer.
 *
 * This function returns the top value (i.e., the value at which the timer overflows
 * or reloads) that is currently set for the timer.
 *
 * - dev: Timer device.
 * @return -ENOTSUP ,the device doesn't support getting top
 *                   counter.
 */
static uint32_t counter_ft_timer_get_top_value(const struct device *dev)
{
    LOG_ERR("Device does not support getting top value! ! !");

    return -ENOTSUP;
}
/*
 * Set the auto-reload (top) value for the timer device.
 * - dev: Timer device.
 * - top_cfg: Contains the reload value and the interrupt callback function.
 *
 * This function sets the timer's auto-reload value and configures the interrupt callback function.
 *
 * * - dev: Timer device.
 * @return -ENOTSUP ,the device doesn't support setting top
 *                   counter.
 */
static int counter_ft_timer_set_top_value(const struct device *dev, const struct counter_top_cfg *top_cfg)
{
    LOG_ERR("Device does not support setting top value! ! !");

    return -ENOTSUP;
}
/*
 * Get the pending interrupt status for the timer device.
 * - dev: Timer device.
 *
 * This function retrieves the current pending interrupt status for the timer device
 * by calling the device-specific method to check if there are any interrupts pending.
 * It returns the interrupt status, where a non-zero value indicates that an interrupt is pending.
 */
static uint32_t counter_ft_timer_get_pending_int(const struct device *dev)
{
    const struct counter_ft_config *cfg = dev->config;

    return cfg->get_irq_pending();
}
/**
 * @brief Get the current frequency of the timer.
 *
 * This function returns the operating frequency of the timer in Hz,
 * which is typically used for converting between timer ticks and real time.
 *
 * - dev: Timer device.
 * @return Timer frequency in Hz.
 */
static uint32_t counter_ft_timer_get_freq(const struct device *dev)
{
    struct counter_ft_data *data = dev->data;

    return data->freq;
}
/**
 * @brief Get the guard period value for the timer.
 *
 * This function retrieves the currently configured guard period value, which
 * represents the minimum time interval required between successive alarm
 * operations to ensure proper behavior.
 *
 * The guard period is used to prevent:
 * - Race conditions during alarm configuration
 * - Improper alarm setting too close to counter overflow/underflow
 * - Hardware settling time violations
 *
 * @param dev Pointer to the timer device structure.
 * @param flags Reserved for future use, should be 0.
 *
 * @return The current guard period value in timer ticks.
 * @return 0 if guard period is not supported or no guard period is set.
 */
static uint32_t counter_ft_timer_get_guard_period(const struct device *dev, uint32_t flags)
{
    LOG_ERR("Device does not support getting guard period value! ! !");

    return 0;
}
/**
 * @brief Set the guard period for the timer.
 *
 * This function configures the guard period value that provides a safety
 * margin to prevent alarm operations from being set too close to counter
 * rollover points or in timing-critical regions.
 *
 * @param dev Pointer to the timer device structure.
 * @param ticks Guard period value in timer ticks.
 * @param flags Reserved for future use, should be 0.
 *
 * @retval 0 Guard period successfully set.
 * @retval -ENOSYS if function or flags are not supported.
 */
static int counter_ft_timer_set_guard_period(const struct device *dev, uint32_t guard, uint32_t flags)
{
    LOG_ERR("Device does not support setting guard period value! ! !");

    return -ENOSYS;
}
/*
 * Timer interrupt service routine handler.
 * - dev: Timer device.
 * This function handles interrupt status, clears the interrupt flag,
 * and invokes registered callbacks based on the interrupt status.
 */
static void irq_handler(const struct device *dev)
{
    const struct counter_ft_config *cfg = dev->config;
    struct counter_ft_data *data = dev->data;
    struct counter_ft_ch_data *chdata = &data->alarm[0];
    counter_top_callback_t tcb = data->top_cb;
    counter_alarm_callback_t acb = chdata->callback;

    Timer_IntFlagClear(cfg->base);
    if (data->alarm_flag)
    {
        Timer_IntDisable(cfg->base);
        data->alarm_flag = false;
        chdata->callback = NULL;
        if (acb != NULL)
        {
            acb(dev, 0, Timer_GetCount(cfg->base), chdata->user_data);
        }
    }

    else
    {
        if (tcb != NULL)
        {
            tcb(dev, data->top_user_data);
        }
    }
}
/**
 * @brief Initialize the timer device.
 *
 *  - dev: Timer device.
 * @return 0 on success, negative error code on failure.
 */
static int counter_ft_timer_init(const struct device *dev)
{
    const struct counter_ft_config *cfg = dev->config;
    struct counter_ft_data *data = dev->data;
    const struct device *clk = DEVICE_DT_GET(DT_NODELABEL(rcc));
    int ret = 0;

    ret = clock_control_on(clk, (clock_control_subsys_t *)&cfg->clkid);
    if (ret < 0)
    {
        LOG_ERR("TC:%p could not enable tc clock", cfg->base);
        return ret;
    }

    ret = reset_line_toggle_dt(&cfg->reset);
    if (ret < 0)
    {
        LOG_ERR("TC:%p could not reset.", cfg->base);
        return ret;
    }

    data->freq = (uint32_t)(cfg->counter_info.freq / cfg->prescaler);
    Timer_RstDisable(cfg->base);
    Timer_IntDisable(cfg->base);

    Timer_SetPrescaler(cfg->base, 11 - LOG2CEIL(cfg->prescaler));
    Timer_SetLoadvalue(cfg->base, cfg->counter_info.max_top_value);

    cfg->irq_config(dev);

    return 0;
}
/*
 * Timer driver API structure that defines the timer operations.
 */
static const struct counter_driver_api counter_api = {
    .start = counter_ft_timer_start,
    .stop = counter_ft_timer_stop,
    .get_value = counter_ft_timer_get_value,
    .set_alarm = counter_ft_timer_set_alarm,
    .cancel_alarm = counter_ft_timer_cancel_alarm,
    .set_top_value = counter_ft_timer_set_top_value,
    .get_pending_int = counter_ft_timer_get_pending_int,
    .get_top_value = counter_ft_timer_get_top_value,
    .get_guard_period = counter_ft_timer_get_guard_period,
    .set_guard_period = counter_ft_timer_set_guard_period,
    .get_freq = counter_ft_timer_get_freq,
};
#define TIMER_IRQ_CONFIG(n)                                                                                            \
    static void irq_config_##n(const struct device *dev)                                                               \
    {                                                                                                                  \
        IRQ_CONNECT(DT_INST_IRQ_BY_IDX(n, 0, irq), DT_INST_IRQ_BY_IDX(n, 0, priority), irq_handler,                    \
                    DEVICE_DT_INST_GET(n), 0);                                                                         \
        irq_enable(DT_INST_IRQ_BY_IDX(n, 0, irq));                                                                     \
    }                                                                                                                  \
    static void set_irq_pending_##n(void)                                                                              \
    {                                                                                                                  \
        (NVIC_SetPendingIRQ(DT_INST_IRQ_BY_IDX(n, 0, irq)));                                                           \
    }                                                                                                                  \
    static uint32_t get_irq_pending_##n(void)                                                                          \
    {                                                                                                                  \
        return NVIC_GetPendingIRQ(DT_INST_IRQ_BY_IDX(n, 0, irq));                                                      \
    }

#define FOCALTECH_TIMER_INIT(n)                                                                                        \
    TIMER_IRQ_CONFIG(n);                                                                                               \
    static struct counter_ft_data_##n                                                                                  \
    {                                                                                                                  \
        struct counter_ft_data data;                                                                                   \
        struct counter_ft_ch_data alarm[DT_INST_PROP(n, channels)];                                                    \
    } timer_data_##n = {0};                                                                                            \
    static const struct counter_ft_config timer_config_##n = {                                                         \
        .counter_info = {.max_top_value = COND_CODE_1(DT_INST_PROP(n, is_32bit), (UINT32_MAX), (UINT16_MAX)),          \
                         .flags = 1,                                                                                   \
                         .freq = DT_INST_PROP(n, clock_frequency),                                                     \
                         .channels = DT_INST_PROP(n, channels)},                                                       \
        .base = (TC_TypeDef *)DT_INST_REG_ADDR(n),                                                                     \
        .clkid = DT_INST_CLOCKS_CELL(n, id),                                                                           \
        .reset = RESET_DT_SPEC_INST_GET(n),                                                                            \
        .prescaler = DT_INST_PROP(n, prescaler),                                                                       \
        .irq_config = irq_config_##n,                                                                                  \
        .set_irq_pending = set_irq_pending_##n,                                                                        \
        .get_irq_pending = get_irq_pending_##n,                                                                        \
    };                                                                                                                 \
                                                                                                                       \
    DEVICE_DT_INST_DEFINE(n, counter_ft_timer_init, NULL, &timer_data_##n, &timer_config_##n, PRE_KERNEL_1,            \
                          CONFIG_COUNTER_INIT_PRIORITY, &counter_api);

DT_INST_FOREACH_STATUS_OKAY(FOCALTECH_TIMER_INIT);
