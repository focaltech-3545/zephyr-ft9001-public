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

#define FT_WAIT_DEEP_SLEEP_TIMEOUT (10*1000) //10s
static uint64_t ft_enter_deep_sleep_time = 0;

static void ft_enable_wakeup_irq_source()
{

    // EPORT_ITConfig((EPORT_TypeDef*)DT_REG_ADDR(DT_NODELABEL(eport5)),0,1);
}

void ft_pm_enter_deep_sleep(bool enable)
{
    if(enable){
	ft_enter_deep_sleep_time = k_uptime_get();

    }else{
	ft_enter_deep_sleep_time = 0;
    }


}
static void ft_enter_sleep_prepare()
{

#ifdef CONFIG_CROS_EC_RW
        random_deinit();
        __disable_irq();
        __set_BASEPRI(0);
        __ISB();
            LP_LowpowerIn();
        __enable_irq();
        __ISB();
#endif
        k_cpu_idle();

        void ft_sys_wake_up(void);
        ft_sys_wake_up();
        printk("exit low power\n");
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
    case PM_STATE_RUNTIME_IDLE:

        if (enter_sleep)
        {
            enter_sleep=false;
            printk("enter Low power from runtime\n");
            ft_enable_wakeup_irq_source();
            ft_enter_deep_sleep_time = 0;
            ft_enter_sleep_prepare();
        }
        else
        {

            k_cpu_idle();
        }

        break;
    case PM_STATE_SUSPEND_TO_IDLE:

        if (enter_sleep)
        {
            enter_sleep=false;
            printk("enter Low power from suspend\n");
            ft_enable_wakeup_irq_source();
            ft_enter_deep_sleep_time = 0;
            ft_enter_sleep_prepare();
        }
        else
        {
            k_cpu_idle();
        }

        break;
    case PM_STATE_STANDBY:

        printk("PM_STATE_STANDBY\n");
        LOG_DBG("entering PM state standby");
        k_cpu_idle();
        break;
    case PM_STATE_SOFT_OFF:

        printk("PM_STATE_STANDBY\n");
        LOG_DBG("entering PM state soft off");
        k_cpu_idle();
        break;
    default:

        LOG_DBG("Unsupported power state %u", state);
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
