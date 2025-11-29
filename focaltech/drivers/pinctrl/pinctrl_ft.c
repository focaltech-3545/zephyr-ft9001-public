/*
 * Copyright (c) 2025 Focaltech Systems CO.,Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/init.h>

/**
 * @brief Initialize PINCTRL
 *
 * This function enables pinctrl clock and configures the I/O compensation if
 * available and enabled in Devicetree.
 *
 * @retval 0 Always
 */
static int pinctrl_init(void)
{
    uint16_t clkid = DT_CLOCKS_CELL(DT_NODELABEL(pinctrl), id);

    clock_control_on(DEVICE_DT_GET(DT_NODELABEL(rcc)), (clock_control_subsys_t)&clkid);
    return 0;
}

SYS_INIT(pinctrl_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
/**
 * @brief Configure a pin.
 *
 * @param pin The pin to configure.
 */
static void configure_pin(pinctrl_soc_pin_t pin)
{
    uint32_t port, pin_bit, bit_val, reg_val;
    volatile uint32_t *reg;

    port = FT90_REGISTER_GET(pin); // reg address
    pin_bit = FT90_BIT_GET(pin);   // bit position
    bit_val = FT90_VALUE_GET(pin); // the value is written
    reg = (volatile uint32_t *)port;
    reg_val = *reg; // read the value of reg
    if (bit_val == 0)
    {
        reg_val &= BIT0_MASK(pin_bit);
    }
    else
    {
        reg_val |= BIT1_MASK(pin_bit);
        ;
    }

    *reg = reg_val; // write the value
}

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt, uintptr_t reg)
{
    ARG_UNUSED(reg);
    if (pin_cnt == 0U)
    {
        return -EINVAL;
    }

    /* configure all pins */
    for (uint8_t i = 0U; i < pin_cnt; i++)
    {
        configure_pin(pins[i]);
    }

    return 0;
}
