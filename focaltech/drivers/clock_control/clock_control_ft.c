/*
 * Copyright (c) 2022 Focaltech Systems CO.,Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ft_ft90_rcc
#include <stdint.h>

#include <zephyr/arch/cpu.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>

/** RCU offset (from id cell) */
#define FOCALTECH_CLOCK_ID_OFFSET(id) (((id) >> 6U) & 0xFFU)
/** RCU configuration bit (from id cell) */
#define FOCALTECH_CLOCK_ID_BIT(id) ((id) & 0x1FU)

struct clock_control_ft_config
{
    uint32_t base;
};

static int clock_control_ft_on(const struct device *dev, clock_control_subsys_t sys)
{
    const struct clock_control_ft_config *config = dev->config;
    uint16_t id = *(uint16_t *)sys;

    sys_set_bit(config->base + FOCALTECH_CLOCK_ID_OFFSET(id), FOCALTECH_CLOCK_ID_BIT(id));

    return 0;
}

static int clock_control_ft_off(const struct device *dev, clock_control_subsys_t sys)
{
    const struct clock_control_ft_config *config = dev->config;
    uint16_t id = *(uint16_t *)sys;

    sys_clear_bit(config->base + FOCALTECH_CLOCK_ID_OFFSET(id), FOCALTECH_CLOCK_ID_BIT(id));

    return 0;
}

static enum clock_control_status clock_control_ft_get_status(const struct device *dev, clock_control_subsys_t sys)
{
    const struct clock_control_ft_config *config = dev->config;
    uint16_t id = *(uint16_t *)sys;

    if (sys_test_bit(config->base + FOCALTECH_CLOCK_ID_OFFSET(id), FOCALTECH_CLOCK_ID_BIT(id)) != 0)
    {
        return CLOCK_CONTROL_STATUS_ON;
    }

    return CLOCK_CONTROL_STATUS_OFF;
}

static const struct clock_control_driver_api clock_control_ft_api = {
    .on = clock_control_ft_on,
    .off = clock_control_ft_off,
    //	.get_rate = clock_control_gd32_get_rate,
    .get_status = clock_control_ft_get_status,
};

static const struct clock_control_ft_config config = {
    .base = DT_REG_ADDR(DT_NODELABEL(rcc)),
};

DEVICE_DT_INST_DEFINE(0, NULL, NULL, NULL, &config, PRE_KERNEL_1, CONFIG_CLOCK_CONTROL_INIT_PRIORITY,
                      &clock_control_ft_api);
