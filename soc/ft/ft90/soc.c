/*
 * Copyright (c) 2021 Focaltech Systems CO.,Ltd
 * SPDX-License-Identifier: Apache-2.0
 */

#include <soc.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/linker/linker-defs.h>

#include <ft_trace.h>

void ft_Sys_Init(void);

void soc_early_init_hook(void)
{
    /* Configure the Vector Table location */
    SCB->VTOR = ((unsigned int)_vector_start) & 0xFFFFFE00;

    ft_Sys_Init();


#if defined(CONFIG_HAS_FT9001LIB)

    xip_clock_switch(1); // switch clocks
    // xip_reback_boot();
#endif
}
