/*
 * SPDX-License-Identifier: Apache-2.0
 */


#define DT_DRV_COMPAT ft_ft90_timer

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

#include <tc_drv.h>

struct counter_ft_ch_data {
	counter_alarm_callback_t callback;
	void *user_data;
};

struct counter_ft_data {
	counter_top_callback_t top_cb;
	void *top_user_data;
	uint32_t guard_period;
	atomic_t cc_int_pending;
	uint32_t freq;
	struct counter_ft_ch_data alarm[];
};

struct counter_ft_config {
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
 * This function enables and starts the timer by calling the low-level
 * Timer_Start function on the specified timer base address.
 *
 * - dev: Timer device.
 * @return 0 on success.
 */
static int counter_ft_timer_start(const struct device *dev)
{
	const struct counter_ft_config *cfg = dev->config;

	Timer_Start(cfg->base);

	return 0;
}

/**
 * @brief Stop the timer device.
 *
 * This function disables the running timer by calling the low-level
 * Timer_Stop function using the timer base address from the configuration.
 *
 *- dev: Timer device.
 * @return 0 on success.
 */
static int counter_ft_timer_stop(const struct device *dev)
{
	const struct counter_ft_config *cfg = dev->config;

	Timer_Stop(cfg->base);

	return 0;
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
	const struct counter_ft_config *cfg = dev->config;
	*ticks = Timer_Get_Count(cfg->base);
	return 0;
}

/*
 * @brief Get the current top value of the timer.
 *
 * This function returns the top value (i.e., the value at which the timer overflows
 * or reloads) that is currently set for the timer.
 *
 * - dev: Timer device.
 * @return The current top (reload) value of the timer.
 */
static uint32_t counter_ft_timer_get_top_value(const struct device *dev)
{
	const struct counter_ft_config *cfg = dev->config;

	return Timer_Get_Loadvalue(cfg->base);
}

/*
 * Set the auto-reload (top) value for the timer device.
 * - dev: Timer device.
 * - top_cfg: Contains the reload value and the interrupt callback function.
 *
 * This function sets the timer's auto-reload value and configures the interrupt callback function.
 */
static int counter_ft_timer_set_top_value(const struct device *dev, const struct counter_top_cfg *top_cfg)
{
	const struct counter_ft_config *cfg = dev->config;
	struct counter_ft_data *data = dev->data;

	Timer_Int_Disable(cfg->base);
	Timer_Set_Loadvalue(cfg->base, top_cfg->ticks);
	Timer_IntFlag_Clear(cfg->base);

	data->top_cb = top_cfg->callback;
	data->top_user_data = top_cfg->user_data;

	if (top_cfg->callback) {
		Timer_Int_Enable(cfg->base);
	}

	return 0;
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

static uint32_t counter_ft_timer_get_guard_period(const struct device *dev, uint32_t flags)
{
	struct counter_ft_data *data = dev->data;

	return data->guard_period;
}

static int counter_ft_timer_set_guard_period(const struct device *dev, uint32_t guard, uint32_t flags)
{
	struct counter_ft_data *data = dev->data;

	__ASSERT_NO_MSG(guard < counter_ft_timer_get_top_value(dev));

	data->guard_period = guard;
	return 0;
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

	counter_top_callback_t cb = data->top_cb;

	Timer_IntFlag_Clear(cfg->base);

	cb(dev, data->top_user_data);

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
	if (ret < 0) {
		return ret;
	}

	ret = reset_line_toggle_dt(&cfg->reset);
	if (ret < 0) {
		return ret;
	}

	data->freq = 128 * 1000 / (2048 >> (cfg->prescaler));
	Timer_Rst_Disable(cfg->base);
	Timer_Int_Disable(cfg->base);

	cfg->irq_config(dev);
	Timer_Set_Prescaler(cfg->base, cfg->prescaler);
	Timer_Set_Loadvalue(cfg->base, cfg->counter_info.max_top_value);

	return 0;
}

/*
 * Timer driver API structure that defines the timer operations.
 */
static const struct counter_driver_api counter_api = {
	.start = counter_ft_timer_start,
	.stop = counter_ft_timer_stop,
	.get_value = counter_ft_timer_get_value,
	.set_top_value = counter_ft_timer_set_top_value,
	.get_pending_int = counter_ft_timer_get_pending_int,
	.get_top_value = counter_ft_timer_get_top_value,
	.get_guard_period = counter_ft_timer_get_guard_period, 
	.set_guard_period = counter_ft_timer_set_guard_period, 
	.get_freq = counter_ft_timer_get_freq,
};

#define TIMER_IRQ_CONFIG(n)                                                                        \
	static void irq_config_##n(const struct device *dev)                                       \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQ_BY_IDX(n, 0, irq),                                   \
			    DT_INST_IRQ_BY_IDX(n, 0, priority),     \
			    irq_handler,                 \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQ_BY_IDX(n, 0, irq));                                   \
	}                                                                                          \
	static void set_irq_pending_##n(void)                                                      \
	{                                                                                          \
		(NVIC_SetPendingIRQ(DT_INST_IRQ_BY_IDX(n, 0, irq)));                         \
	}                                                                                          \
	static uint32_t get_irq_pending_##n(void)                                                  \
	{                                                                                          \
		return NVIC_GetPendingIRQ(DT_INST_IRQ_BY_IDX(n, 0, irq));                    \
	}

#define FOCALTECH_TIMER_INIT(n)                                                                         \
	TIMER_IRQ_CONFIG(n);									  \
	static struct counter_ft_data_##n {                                                      \
		struct counter_ft_data data;                                                     \
		struct counter_ft_ch_data alarm[DT_INST_PROP(n, channels)];                      \
	} timer_data_##n = {0};                                                                    \
	static const struct counter_ft_config timer_config_##n = {                               \
		.counter_info = {                                                                  \
		.max_top_value = COND_CODE_1(DT_INST_PROP(n, is_32bit), (UINT32_MAX), (UINT16_MAX)),\
		.flags = 0,						   \
		.freq = 0, .channels = DT_INST_PROP(n, channels)},				   \
		.base = (TC_TypeDef *)DT_INST_REG_ADDR(n),							   \
		.clkid = DT_INST_CLOCKS_CELL(n, id),						   \
		.reset = RESET_DT_SPEC_INST_GET(n),						   \
		.prescaler = DT_INST_PROP(n, prescaler),					   \
		.irq_config = irq_config_##n,							   \
		.set_irq_pending = set_irq_pending_##n,						   \
		.get_irq_pending = get_irq_pending_##n,						   \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, counter_ft_timer_init, NULL, &timer_data_##n,                   \
			      &timer_config_##n, PRE_KERNEL_1, CONFIG_COUNTER_INIT_PRIORITY,       \
			      &counter_api);

DT_INST_FOREACH_STATUS_OKAY(FOCALTECH_TIMER_INIT);
