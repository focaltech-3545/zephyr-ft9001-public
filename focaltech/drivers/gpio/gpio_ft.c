/*
 * Copyright (c) 2025-2026, Focaltech Systems CO.,Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ft_ft90_gpio

#include <eport_drv.h>
#include <errno.h>
#include <soc.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_utils.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/pm/policy.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/util_macro.h>
LOG_MODULE_REGISTER(gpio_ft);

static struct k_spinlock gpio_ft_global_lock;
struct gpio_ft_config
{
    /* gpio_driver_config must be the first element */
    struct gpio_driver_config common;
    EPORT_TypeDef *base;
    const struct pinctrl_dev_config *pcfg;
    uint32_t clkid;
    struct reset_dt_spec reset;
    uint32_t gpio_mask;
    uint32_t ngpios;
};

struct gpio_ft_data
{
    /* gpio_driver_data must be the first element */
    struct gpio_driver_data common;
    /* Port ISR callback list */
    sys_slist_t callbacks;
    gpio_flags_t Con_Flags[8];
};

/*
 * Configure input/output mode for a GPIO pin. The pin refers to the nth pin of the
 * nth EPORT device, where numbering starts from 0. For example, GIN24 corresponds
 * to the pin of EPORT3 with pin number 0.
 */
static int gpio_ft_configure(const struct device *port, gpio_pin_t pin, gpio_flags_t flags)
{
    int ret;
    uint32_t temp_val;
    k_spinlock_key_t key;
    const struct gpio_ft_config *config = port->config;
    struct gpio_ft_data *ftdata = port->data;

    // Check if the pin is valid
    if (((config->gpio_mask & BIT(pin)) == 0) || pin >= (config->ngpios))
    {
        LOG_ERR("Invalid pin! You should select another pin.");
        return -ENOTSUP; // Invalid pin
    }
    ftdata->Con_Flags[pin] = flags;
    // Check for conflicting flags (cannot be both input and output)
    if (((flags & GPIO_INPUT) != 0U) && ((flags & GPIO_OUTPUT) != 0U))
    {
        LOG_ERR("Cannot be configured as both input and output simultaneously.");
        return -ENOTSUP; // Conflicting flags
    }

    ret = pm_device_runtime_get(port);
    if (ret < 0)
    {
        LOG_ERR("pm_device_runtime_get err:%x", ret);
        return ret;
    }

    if ((flags & GPIO_OUTPUT) != 0U)
    {

        /*
         * Register operations are atomic, but do the sequence under
         * lock so it serializes.
         */
        // Global lock
        key = k_spin_lock(&gpio_ft_global_lock);
        // Configure as output mode
        ret = EPORT_ConfigGpio(config->base, pin, 1);

        // Check for open-drain or push-pull output mode
        if ((flags & GPIO_OPEN_DRAIN) != 0U)
        {
            ret = EPORT_OutputMode(config->base, pin, 1);
        }
        else
        {
            ret = EPORT_OutputMode(config->base, pin, 0);
        }

        // Set initial output value (high or low)
        if ((flags & GPIO_OUTPUT_INIT_HIGH) != 0U)
        {
            temp_val = EPORT_ReadGpioDatas(config->base) | (BIT(pin) & (config->gpio_mask));
            EPORT_WriteGpioDatas(config->base, temp_val);
        }
        else if ((flags & GPIO_OUTPUT_INIT_LOW) != 0U)
        {
            temp_val = EPORT_ReadGpioDatas(config->base) & (~(BIT(pin) & (config->gpio_mask)));
            EPORT_WriteGpioDatas(config->base, temp_val);
        }
        // Release global lock
        k_spin_unlock(&gpio_ft_global_lock, key);
    }
    else
    {

        // Global lock
        key = k_spin_lock(&gpio_ft_global_lock);
        // Configure as input mode
        ret = EPORT_ConfigGpio(config->base, pin, 0);

        // Check for pull-up or pull-down input configuration
        if ((flags & GPIO_PULL_UP) != 0U)
        {
            ret = EPORT_PullConfig(config->base, pin, 1);
        }
        else if ((flags & GPIO_PULL_DOWN) != 0U)
        {
            ret = EPORT_PullConfig(config->base, pin, 0);
        }
        else
        {
            // Disable both pull-up and pull-down
            ret = EPORT_PullConfig(config->base, pin, 2);
        }
        // Release global lock
        k_spin_unlock(&gpio_ft_global_lock, key);
    }

    return pm_device_runtime_put(port);
}

#if defined(CONFIG_GPIO_GET_CONFIG)
/**
 * @brief Get configuration of pin
 */
static int gpio_ft_get_config(const struct device *port, gpio_pin_t pin, gpio_flags_t *flags)
{
    const struct gpio_ft_config *config = port->config;
    struct gpio_ft_data *ftdata = port->data;
    // Check if the pin is valid
    if (((config->gpio_mask & BIT(pin)) == 0) || pin >= (config->ngpios))
    {
        LOG_ERR("Invalid pin! You should select another pin.");
        return -ENOTSUP; // Invalid pin
    }
    int err;

    err = pm_device_runtime_get(port);
    if (err < 0)
    {
        return err;
    }

    if ((((ftdata->Con_Flags[pin]) & GPIO_INPUT)) == (((ftdata->Con_Flags[pin]) & GPIO_OUTPUT)))
    {
        *flags = 0;
        return 0;
    }
    else if ((((ftdata->Con_Flags[pin]) & GPIO_OUTPUT)))
    {
        ftdata->Con_Flags[pin] = (ftdata->Con_Flags[pin]) & (0xFFF3FFFF);
        if (EPORT_ReadGpioDatas(config->base) & BIT(pin))
        {
            *flags = (ftdata->Con_Flags[pin]) | GPIO_OUTPUT_INIT_HIGH;
        }
        else
        {
            *flags = (ftdata->Con_Flags[pin]) | GPIO_OUTPUT_INIT_LOW;
        }
    }
    else
    {
        *flags = ftdata->Con_Flags[pin];
    }
    pm_device_runtime_put(port);
    return 0;
}
#endif /* CONFIG_GPIO_GET_CONFIG */

/*
 * Read the value of Eportx
 */
static int gpio_ft_port_get_raw(const struct device *port, uint32_t *value)
{
    const struct gpio_ft_config *config = port->config;

    *value = EPORT_ReadGpioDatas(config->base);

    return 0;
}

/*
 * Write values only to the pins that are high in the 'pins' mask. For example, if pins is 0x1,
 * it writes to the first pin. If pins is 0x5, it writes to the first and third pins.
 */
static int gpio_ft_port_set_masked_raw(const struct device *port, gpio_port_pins_t pins, gpio_port_value_t value)
{
    const struct gpio_ft_config *config = port->config;
    k_spinlock_key_t key;
    uint32_t temp_val =
        (EPORT_ReadGpioDatas(config->base) & (~(pins & (config->gpio_mask)))) | (pins & value & (config->gpio_mask));
    // Global lock
    key = k_spin_lock(&gpio_ft_global_lock);
    EPORT_WriteGpioDatas(config->base, temp_val);
    // Release global lock
    k_spin_unlock(&gpio_ft_global_lock, key);
    return 0;
}

/*
 * Write value 1 to the pins that are high in the 'pins' mask.
 * For example, if pins is 0x1, it writes 1 to the first pin.
 * If pins is 0x5, it writes 1 to the first and third pins.
 */
static int gpio_ft_port_set_bits_raw(const struct device *port, gpio_port_pins_t pins)
{
    const struct gpio_ft_config *config = port->config;
    k_spinlock_key_t key;
    uint32_t temp_val = EPORT_ReadGpioDatas(config->base) | (pins & (config->gpio_mask));
    // Global lock
    key = k_spin_lock(&gpio_ft_global_lock);
    EPORT_WriteGpioDatas(config->base, temp_val);
    // Release global lock
    k_spin_unlock(&gpio_ft_global_lock, key);
    return 0;
}

/*
 * Write value 0 to the pins that are high in the 'pins' mask.
 * For example, if pins is 0x1, it writes 0 to the first pin.
 * If pins is 0x5, it writes 0 to the first and third pins.
 */
static int gpio_ft_port_clear_bits_raw(const struct device *port, gpio_port_pins_t pins)
{
    const struct gpio_ft_config *config = port->config;
    k_spinlock_key_t key;
    uint32_t temp_val = EPORT_ReadGpioDatas(config->base) & (~(pins & (config->gpio_mask)));
    // Global lock
    key = k_spin_lock(&gpio_ft_global_lock);
    EPORT_WriteGpioDatas(config->base, temp_val);
    // Release global lock
    k_spin_unlock(&gpio_ft_global_lock, key);
    return 0;
}

/*
 * Write the inverse of the original value to the pins that are high in the 'pins' mask.
 * For example, if pins is 0x1, it writes to the first pin.
 * If the original value is 1, it writes 0, and if the original value is 0, it writes 1.
 * If pins is 0x5, it writes to the first and third pins.
 */
static int gpio_ft_port_toggle_bits(const struct device *port, gpio_port_pins_t pins)
{
    const struct gpio_ft_config *config = port->config;
    k_spinlock_key_t key;
    uint32_t temp_val = EPORT_ReadGpioDatas(config->base) ^ (pins & (config->gpio_mask));
    // Global lock
    key = k_spin_lock(&gpio_ft_global_lock);
    EPORT_WriteGpioDatas(config->base, temp_val);
    // Release global lock
    k_spin_unlock(&gpio_ft_global_lock, key);
    return 0;
}

/*
 * Configure the interrupt mode for the nth device's nth pin, with n starting from 0.
 */
static int gpio_ft_pin_interrupt_configure(const struct device *port, gpio_pin_t pin, enum gpio_int_mode mode,
                                           enum gpio_int_trig trig)
{
    const struct gpio_ft_config *config = port->config;
    EPORT_TypeDef *base = config->base;
    k_spinlock_key_t key;

    if (((config->gpio_mask & BITS(pin)) == 0) || pin >= (config->ngpios)) // If the pin is invalid, return error
    {
        LOG_ERR("Invalid pin! You should select another pin.");
        return -ENOTSUP;
    }
    // Global lock
    key = k_spin_lock(&gpio_ft_global_lock);
    if ((mode == GPIO_INT_MODE_EDGE) && (trig == GPIO_INT_TRIG_LOW))
    {
        EPORT_ITTypeConfig(base, pin, FALLING_EDGE_INT);
    }
    else if ((mode == GPIO_INT_MODE_EDGE) && (trig == GPIO_INT_TRIG_HIGH))
    {
        EPORT_ITTypeConfig(base, pin, RISING_EDGE_INT);
    }
    else if ((mode == GPIO_INT_MODE_EDGE) && (trig == GPIO_INT_TRIG_BOTH))
    {
        EPORT_ITTypeConfig(base, pin, RISING_FALLING_EDGE_INT);
    }
    else if ((mode == GPIO_INT_MODE_LEVEL) && (trig == GPIO_INT_TRIG_HIGH))
    {
        EPORT_ITTypeConfig(base, pin, HIGH_LEVEL_INT);
    }
    else if ((mode == GPIO_INT_MODE_LEVEL) && (trig == GPIO_INT_TRIG_LOW))
    {
        EPORT_ITTypeConfig(base, pin, LOW_LEVEL_INT);
    }
    else
    {
        EPORT_ITConfig(base, pin, 0);
    }
    // Release global lock
    k_spin_unlock(&gpio_ft_global_lock, key);
    return 0;
}

/*
 * This function manages GPIO callback registration and removal.
 * - port: The GPIO device.
 * - cb: The callback structure.
 * - set: True to add the callback, false to remove it.
 * Returns: Status of callback management.
 */
static int gpio_ft_manage_callback(const struct device *port, struct gpio_callback *cb, bool set)
{
    struct gpio_ft_data *data = port->data;
    return gpio_manage_callback(&data->callbacks, cb, set);
}

/*
 * GPIO interrupt service routine (ISR) handler.
 * - port: The GPIO device.
 * This function handles interrupt status, clears the interrupt flag,
 * and invokes registered callbacks based on the interrupt status.
 */
void gpio_ft_port_isr(const struct device *port, int irq_num)
{
    const struct gpio_ft_config *config = port->config;
    struct gpio_ft_data *data = port->data;
    EPORT_ClrStatus(config->base, BIT(irq_num));               // Clear the interrupt status
    gpio_fire_callbacks(&data->callbacks, port, BIT(irq_num)); // Fire relevant callbacks
}

/*
 * GPIO driver API structure that defines the GPIO operations.
 */
static const struct gpio_driver_api gpio_ft_driver_api = {
    .pin_configure = gpio_ft_configure,
#if defined(CONFIG_GPIO_GET_CONFIG)
    .pin_get_config = gpio_ft_get_config,
#endif
    .port_get_raw = gpio_ft_port_get_raw,
    .port_set_masked_raw = gpio_ft_port_set_masked_raw,
    .port_set_bits_raw = gpio_ft_port_set_bits_raw,
    .port_clear_bits_raw = gpio_ft_port_clear_bits_raw,
    .port_toggle_bits = gpio_ft_port_toggle_bits,
    .pin_interrupt_configure = gpio_ft_pin_interrupt_configure,
    .manage_callback = gpio_ft_manage_callback,
};

/*
 *Power management callback to suspend or resume GPIO hardware
 *during system sleep or runtime idle
 */
static int gpio_ft_pm_action(const struct device *port, enum pm_device_action action)
{
    // int err=0;
    // const struct gpio_ft_config* cfg = port->config;
    printk("gpaction =%d\n", action);

    switch (action)
    {
    case PM_DEVICE_ACTION_RESUME:
        break;
    case PM_DEVICE_ACTION_SUSPEND:
        break;
    case PM_DEVICE_ACTION_TURN_OFF:
    case PM_DEVICE_ACTION_TURN_ON:
        break;
    default:
        return -ENOTSUP;
    }
    return 0;
}
/*
 * Macro to connect GPIO interrupts. It sets up the ISR for each GPIO pin interrupt.
 * - i: Interrupt index.
 * - n: The device index.
 */

#define GPIO_IRQ_CONNECT(i, n)                                                                                         \
    IRQ_CONNECT(DT_INST_IRQ_BY_IDX(n, i, irq), DT_INST_IRQ_BY_IDX(n, i, priority), gpio_ft_port_isr_##n##_##i,         \
                DEVICE_DT_INST_GET(n), 0);                                                                             \
    irq_enable(DT_INST_IRQ_BY_IDX(n, i, irq));

#define GPIO_GEN_ISR(i, n)                                                                                             \
    static void gpio_ft_port_isr_##n##_##i(const void *arg)                                                            \
    {                                                                                                                  \
        const struct device *port = arg;                                                                               \
        gpio_ft_port_isr(port, i);                                                                                     \
    }

/*
 * Macro for defining and initializing a GPIO device based on the device tree configuration.
 * - n: The device index.
 */
#define GPIO_FOCALTECH_DEVICE(n)                                                                                       \
    static int gpio_ft_##n##_init(const struct device *port);                                                          \
    LISTIFY(DT_INST_PROP_OR(n, ngpios, 8), GPIO_GEN_ISR, (;), n)                                                       \
    static const struct gpio_ft_config gpio_ft_##n##_config = {                                                        \
        .common =                                                                                                      \
            {                                                                                                          \
                .port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(n),                                                   \
            },                                                                                                         \
        .base = (EPORT_TypeDef *)DT_INST_REG_ADDR(n),                                                                  \
        .pcfg = COND_CODE_1(DT_INST_NODE_HAS_PROP(n, pinctrl_0), (PINCTRL_DT_INST_DEV_CONFIG_GET(n)), (NULL)),         \
        .gpio_mask = DT_INST_PROP_OR(n, mask, 0xFF),                                                                   \
        .ngpios = DT_INST_PROP_OR(n, ngpios, 8),                                                                       \
        .clkid = DT_INST_CLOCKS_CELL(n, id),                                                                           \
        .reset = RESET_DT_SPEC_INST_GET(n),                                                                            \
    };                                                                                                                 \
                                                                                                                       \
    static struct gpio_ft_data gpio_ft_##n##_data;                                                                     \
    PM_DEVICE_DT_INST_DEFINE(n, gpio_ft_pm_action);                                                                    \
    DEVICE_DT_INST_DEFINE(n, gpio_ft_##n##_init, PM_DEVICE_DT_INST_GET(n), &gpio_ft_##n##_data, &gpio_ft_##n##_config, \
                          PRE_KERNEL_1, CONFIG_GPIO_INIT_PRIORITY, &gpio_ft_driver_api);                               \
                                                                                                                       \
    static int gpio_ft_##n##_init(const struct device *port)                                                           \
    {                                                                                                                  \
        LISTIFY(DT_INST_PROP_OR(n, ngpios, 8), GPIO_IRQ_CONNECT, (), n)                                                \
        const struct gpio_ft_config *config = port->config;                                                            \
        const struct device *clk = DEVICE_DT_GET(DT_NODELABEL(rcc));                                                   \
        int ret = 0;                                                                                                   \
        ret = clock_control_on(clk, (clock_control_subsys_t *)&config->clkid);                                         \
        if (ret < 0)                                                                                                   \
        {                                                                                                              \
            LOG_ERR("GPIO:%p could not enable gpio clock", config->base);                                              \
            return ret;                                                                                                \
        }                                                                                                              \
        ret = reset_line_toggle_dt(&config->reset);                                                                    \
        if (ret < 0)                                                                                                   \
        {                                                                                                              \
            LOG_ERR("GPIO:%p could not reset.", config->base);                                                         \
            return ret;                                                                                                \
        }                                                                                                              \
        return IS_ENABLED(CONFIG_PM_DEVICE) ? pm_device_driver_init(port, gpio_ft_pm_action) : 0;                      \
    }

/*
 * Macro to iterate through all the GPIO devices defined in the device tree and initialize them.
 */
DT_INST_FOREACH_STATUS_OKAY(GPIO_FOCALTECH_DEVICE)
