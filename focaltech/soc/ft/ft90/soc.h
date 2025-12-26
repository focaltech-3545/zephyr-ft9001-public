/*
 * Copyright (c) 2021 Focaltech Systems CO.,Ltd
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _SOC_ARM_FOCALTECH_FT90_SOC_H_
#define _SOC_ARM_FOCALTECH_FT90_SOC_H_

#ifndef _ASMLANGUAGE

#ifdef CONFIG_HAS_FT9002LIB
#include <ft9002s.h>
#include <system_ft9002s.h>
#endif

#ifdef CONFIG_HAS_FT9001LIB
#include <ft9001.h>
#include <libft9001.h>
#include <system_ft9001.h>
#endif

#define PUSHW0 ((1) << 25)
#define PUSHW1 ((1) << 27)

void __attribute__((section(".ramfunc"))) xip_clock_switch(uint32_t clk_div);
void __attribute__((section(".ramfunc"))) xip_reback_boot(void);

void ft_pm_enter_deep_sleep(bool enable);
void DRV_DCACHE_Push(uint32_t way);

#endif /* _ASMLANGUAGE */

#endif /* _SOC_ARM_FOCALTECH_FT90_SOC_H_ */
