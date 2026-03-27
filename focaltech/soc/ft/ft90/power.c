/*
 * Copyright (c) 2025 Focaltech Systems CO.,Ltd
 * SPDX-License-Identifier: Apache-2.0
 */

#include "eport_drv.h"
#include "lowpower_drv.h"
#include <core_cm4.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/pm/pm.h>
#include <zephyr/sys_clock.h>
#include <zephyr/types.h>

LOG_MODULE_REGISTER(soc_power, CONFIG_SOC_LOG_LEVEL);

#define FT_WAIT_DEEP_SLEEP_TIMEOUT (500) //10s
static uint64_t ft_enter_deep_sleep_time = 0;

static void ft_enable_wakeup_irq_source()
{
    //EPORT_ITConfig((EPORT_TypeDef*)DT_REG_ADDR(DT_NODELABEL(eport5)),0,1);
}

void ft_pm_enter_deep_sleep(bool enable)
{
    if(enable){
	ft_enter_deep_sleep_time = k_uptime_get();

    }else{
	ft_enter_deep_sleep_time = 0;
    }


}

typedef void(*SSID_FUNC)(char);
void HAL_Enable_Four_Line_Mode(unsigned char ssi_id)
{
    uint32_t Enable_Four_Line_Mode[43] = { 
                0x4601b53f, 0x4d242200, 0xcd38447d, 0x4502e9cd, 
                0x1e4b9301, 0xf854ac01, 0xf3ef0023, 0x461a8310, 
                0xf101b672, 0x2401030f, 0xf303fa04, 0x0413f101, 
                0x40a52501, 0x4c19432b, 0x439c6be4, 0x63dc4b17, 
                0x93006a83, 0x60832300, 0x30f4f8d0, 0x0303f003, 
                0x6803b96b, 0x0340f403, 0x1f00f5b3, 0x6803d007, 
                0x0340f423, 0x68036003, 0x0300f443, 0x23016003, 
                0xf1016083, 0x2401030f, 0xf303fa04, 0x0413f101, 
                0x40a52501, 0x4c05432b, 0x43236be4, 0x63e34c03, 
                0x8810f382, 0x0000bd3f, 0x00000094, 0x40001000, 
                0x13000000, 0x17000000, 0x1b000000, };

    SSID_FUNC ssid_func = (SSID_FUNC)((unsigned int)Enable_Four_Line_Mode | 0x1);

    ssid_func(ssi_id);
}
static void ft_enter_sleep_prepare()
{

#ifdef CONFIG_CROS_EC_RW
        random_deinit();
        
        HAL_Enable_Four_Line_Mode(1);
        LP_LowpowerIn();
        __enable_irq();
       
#endif
        k_cpu_idle();

        void ft_sys_wake_up(void);
        ft_sys_wake_up();
        //printk("exit low power\n");
        LP_LowpowerOut();
        random_init();
}

/* Power state manage */

void pm_state_set(enum pm_state state, uint8_t substate_id)
{
    ARG_UNUSED(substate_id);

    bool enter_sleep = false;

    if (ft_enter_deep_sleep_time)
    {

        if (k_uptime_get() - ft_enter_deep_sleep_time > FT_WAIT_DEEP_SLEEP_TIMEOUT)
        { // wait 10s
            ft_enter_deep_sleep_time = 0;
            enter_sleep = true;
        }
    }

    switch (state)
    {

    case PM_STATE_SUSPEND_TO_IDLE:

        if (enter_sleep)
        {
            enter_sleep=false;
            //printk("enter Low power from suspend\n");
            ft_enable_wakeup_irq_source();
            ft_enter_deep_sleep_time = 0;
            SCB->SCR |= (SCB_SCR_SLEEPDEEP_Msk);
            ft_enter_sleep_prepare();
        }
        else
        {
            
           // k_cpu_idle();
        }

        break;
    case PM_STATE_STANDBY:

        //printk("PM_STATE_STANDBY\n");
        //LOG_DBG("entering PM state standby");
        //k_cpu_idle();
        break;
    case PM_STATE_SOFT_OFF:

        //printk("PM_STATE_STANDBY\n");
        //LOG_DBG("entering PM state soft off");
       // k_cpu_idle();
        break;
    default:
        //k_cpu_idle();
        //LOG_DBG("Unsupported power state %u", state);
        break;
    }
}

/* Exit low-power mode */
void pm_state_exit_post_ops(enum pm_state state, uint8_t substate_id)
{
    ARG_UNUSED(state);
    ARG_UNUSED(substate_id);

    switch (state)
    {
    case PM_STATE_SUSPEND_TO_IDLE:

        LP_LowpowerOut();
        //printk("suspend exit\n");

        break;
    default:
        break;
    }
    irq_unlock(0);
}
