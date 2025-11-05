/*
 * Copyright (c) 2025 focaltech.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ft_ft90_trng

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/entropy.h>
#include <zephyr/random/random.h>
#include <zephyr/init.h>
#include <stdio.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(entropy_ft_trng, LOG_LEVEL_DBG);

#include "trng_drv.h"

static int entropy_ft_trng_get_entropy(const struct device *dev,
					 uint8_t *buffer,
					 uint16_t length)
{
	//printf("%s\n", __func__);
	int count = length / sizeof(uint32_t);
	int remain = length % sizeof(uint32_t);

	for (int i = 0; i < count; i++)
	{
		uint32_t data = random_get_data();
		memcpy(buffer + (i * sizeof(uint32_t)), &data, sizeof(uint32_t));
	}

	if (remain > 0)
	{
		uint32_t data = random_get_data();
		memcpy(buffer + (count * sizeof(uint32_t)), &data, sizeof(uint32_t));
	}
	
	return 0;
}

static const struct entropy_driver_api entropy_ft_trng_api_funcs = {
	.get_entropy = entropy_ft_trng_get_entropy
};

static int entropy_ft_trng_init(const struct device *dev)
{
	//printf("%s\n", __func__);
	random_init();
	return 0;
}

#ifdef CONFIG_PM_DEVIC
static int entropy_ft_trng_pm_action(const struct device *dev, enum pm_device_action action)
{
	switch (action) {
	case PM_DEVICE_ACTION_SUSPEND:
		random_init();
		return 0;
	case PM_DEVICE_ACTION_RESUME:
		random_deinit();
		return 0;
	default:
		return -ENOTSUP;
	}
}

PM_DEVICE_DT_INST_DEFINE(0, entropy_ft_trng_pm_action);
#endif


DEVICE_DT_INST_DEFINE(0,
		    entropy_ft_trng_init, NULL, NULL, NULL,
		    POST_KERNEL, CONFIG_ENTROPY_INIT_PRIORITY,
		    &entropy_ft_trng_api_funcs); 

