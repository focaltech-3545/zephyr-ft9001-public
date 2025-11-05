/*
 * Copyright (c) 2024 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <soc.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/sys/byteorder.h>
#include <string.h>
#include <libGetSN.h>
#include "reset_drv.h"

#define FT_DEVICE_ID_LEN	8

ssize_t z_impl_hwinfo_get_device_id(uint8_t *buffer, size_t length)
{
	if (!buffer || length == 0) {
		return 0;
	}

	uint8_t uid[32] = {0};

	LIB_SN_Read(uid);
	memcpy(buffer, uid, FT_DEVICE_ID_LEN);

	return FT_DEVICE_ID_LEN;
}

extern uint8_t Get_RESET_GetStatus(void);
int z_impl_hwinfo_get_reset_cause(uint32_t *cause)
{
	if (!cause) {
		return -EINVAL;
	}

	uint32_t flags = 0;
	uint8_t status = Get_RESET_GetStatus();

	if (status & (BIT(4) | BIT(7))) {   /* BIT4: WDT, BIT7: TC Reset */
		flags |= RESET_WATCHDOG;
	}
	if (status & BIT(5)) {              /* Software reset */
		flags |= RESET_SOFTWARE;
	}
	if (status & BIT(3)) {              /* Power-On Reset */
		flags |= RESET_POR;
	}
	if (status & BIT(6)) {              /* VD Reset */
		flags |= RESET_BROWNOUT;
	}

	*cause = flags;
	return 0;
}

int z_impl_hwinfo_clear_reset_cause(void)
{
	return -ENOSYS;
}

int z_impl_hwinfo_get_supported_reset_cause(uint32_t *supported)
{
	if (!supported) {
        return -EINVAL;
    }

    *supported = (RESET_PIN
               |  RESET_WATCHDOG
               |  RESET_SOFTWARE
               |  RESET_POR
               |  RESET_BROWNOUT);

	return 0;
}
