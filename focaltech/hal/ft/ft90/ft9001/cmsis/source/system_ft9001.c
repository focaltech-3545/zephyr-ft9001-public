/**************************************************************************//**
 * @file     system_ARMCM4.c
 * @brief    CMSIS Device System Source File for
 *           ARMCM4 Device Series
 * @version  V2.00
 * @date     18. August 2015
 ******************************************************************************/
/* Copyright (c) 2011 - 2015 ARM LIMITED

   All rights reserved.
   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
   - Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   - Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
   - Neither the name of ARM nor the names of its contributors may be used
     to endorse or promote products derived from this software without
     specific prior written permission.
   *
   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
   ---------------------------------------------------------------------------*/

#include <stdio.h>
#include "ft9001.h"
#include "system_ft9001.h"
#include "libft9001.h"

#include "wdt_reg.h"
#include "tc_reg.h"

#include "cpm_drv.h"
#include "cache_drv.h"

unsigned int SystemCoreClock; 
void Sys_SysClkConfig();
void Sys_wdt_close(void);
void Sys_tc_close(void);

void SystemInit (void)
{
    Sys_wdt_close();
    Sys_tc_close();
#if (__FPU_USED == 1)
    SCB->CPACR |= ((3UL << 10 * 2) | /* set CP10 Full Access */
                   (3UL << 11 * 2)); /* set CP11 Full Access */
#endif

#ifdef UNALIGNED_SUPPORT_DISABLE
    SCB->CCR |= SCB_CCR_UNALIGN_TRP_Msk;
#endif

    /* 初始化时钟 */
    Sys_SysClkConfig();

    //开启EFLASH加速模块
    DRV_DCACHE_Init(CACHE_Through, CACHE_Through, CACHE_Through, CACHE_Through, CACHE_Off, CACHE2_BASE_ADDR);
    DRV_ICACHE_Init(CACHE_Through, CACHE_Through, CACHE_Through, CACHE_Through, CACHE_Off, CACHE_BASE_ADDR);
    
    //CPM_ClearPADWKINTCR();      //清除默认中断唤醒源
    
    //CPM_VCC5V_Bypass(); //芯片3.3V或1.8V供电时，对功耗要求高，建议打开此接口，对功耗要求不高可以不打开; 芯片5V供电时必须屏蔽此接口,否则可能坏芯片！！！

    // 恢复ROM启动模式，开发完成后建议屏蔽此接口或换用其他方式恢复ROM启动
    //Reback_Boot();
    //DelayMS(1000);
    /*配置串口作为debug用，时钟根据不同环境配置不同的时钟参数*/
    //UART_Debug_Init(SCI1, g_ips_clk, 115200);

    // UART打印版本信息
    //Printf_Version();
}

void Sys_SysClkConfig()
{
    //Trim CLock
    LIB_CPM_OscSwitch(OSC_320M_HZ);

    //Select 320M clock
    DRV_CPM_SystemClkOSC320MSelect();

    //DRV_CPM_SetSystemClkDiv(pClkInit->SysClkDiv);
//    DRV_XIP_SYSCLKSwitch((pClkInit->SysClkDiv + 1), 2, 1);

    //set IPS clock
    DRV_CPM_SetIpsClkDiv(1);

    //Enable low power if sys_fren<60M，trim_vol=0.9V
    //DRV_CPM_SystemClkVrefTrim(CPM_VREF_TRIM_090);

}


void SystemCoreClockUpdate(void)
{
    SystemCoreClock = 160*1000*1000;  //160M
}

void Sys_wdt_close(void)
{
    WDT_TypeDef *pWDT;

    pWDT  =  (WDT_TypeDef *)(WDT_BASE_ADDR);
    pWDT->WDT_WCR &= ~WDT_EN; // Close WDT
}


void Sys_tc_close(void)
{
    TC_TypeDef *pTC;

    pTC = (TC_TypeDef*)(TC_BASE_ADDR);
    pTC->TCCR = 0; // Close TC
}

void Sys_delay_us(int count)
{
    int i;
    for(i=0; i<count; i++)
    {
        __asm("nop");
        __asm("nop");
        __asm("nop");
        __asm("nop");
        __asm("nop");
        __asm("nop");
        __asm("nop");
        __asm("nop");
        __asm("nop");
        __asm("nop");

        __asm("nop");
        __asm("nop");
        __asm("nop");
        __asm("nop");
        __asm("nop");
        //__asm("nop");
        //__asm("nop");
        //__asm("nop");
        //__asm("nop");
        //__asm("nop");
    }
}
