/*
 * Copyright (c) 2022 Focaltech Systems CO.,Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <inttypes.h>
#include <kernel_internal.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
// #include <zephyr/arch/common/exc_handle.h>
#include <string.h>
#include <zephyr/linker/linker-defs.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/barrier.h>

#include "system_ft9001.h"

// extern char _ramfunc_start[];
// extern char _ramfunc_end[];
// extern char _ramfunc_load_start[]; // Flash 中的源地址
uint32_t var_sram_data = 10U;

void copy_ramfunc(void)
{
    size_t size = __ramfunc_end - __ramfunc_start;
    memcpy(__ramfunc_start, __ramfunc_load_start, size);

    // delay a while
    __asm("nop");
    __asm("nop");
    __asm("nop");
    __asm("nop");
    __asm("nop");
    __asm("nop");
    __asm("nop");
    __asm("nop");
}

void __attribute__((section(".ramfunc"))) xip_clock_switch(uint32_t clk_div)
{
    *(volatile unsigned int *)(0x40004008) =
        ((*(volatile unsigned int *)(0x40004008)) & 0xFFFFFFF0) | (clk_div & 0x0F);           // set parameters
    *(volatile unsigned int *)(0x40004018) = *(volatile unsigned int *)(0x40004018) | (0x02); // update

    // delay a while
    __asm("nop");
    __asm("nop");
    __asm("nop");
    __asm("nop");
    __asm("nop");
    __asm("nop");
    __asm("nop");
    __asm("nop");
}

#if 0
void __attribute__((section(".ramfunc"))) xip_reback_boot(void)
{
    uint32_t primask_bit;
    //uint8_t  return_value;
    struct   str_flash {
        uint8_t    SsiId;            //1,2,3
        uint8_t    SysDiv;
        uint8_t    IsQpiMode;      
        uint16_t   StandBaudr; 
        uint16_t   QuadBaudr; 
        uint32_t   RxSampleDelay;

        uint8_t    Cmd;        //later
        uint32_t   Value;      //later

        uint8_t    IsMaskInterrupt;
        //------------------------------------
        uint8_t    ProgramMode;
        uint32_t   Addr;       //later
        uint16_t   Len;        //later
        uint32_t   Buf;        //later
        uint32_t   Delay;       
        uint32_t   Timeout;     
        };
    struct str_flash  ssi_cfg[3];
    int i;
    uint8_t (*xip_flash_erase)(struct str_flash *p_ssi_para);
    
    primask_bit = __get_PRIMASK();  /**< backup PRIMASK bit */
    __disable_irq();                  /**< Disable all interrupts by setting PRIMASK bit on Cortex*/

    //if I2C_SDA is low, reback boot
    if (!((*(unsigned char *)0x40017009) & 0x02))
    {
        //Erase the first sector, addr:0x1000_0000, size:0x1000
        //the first sector has data only
        //disable interrupts
        primask_bit = __get_PRIMASK();  /**< backup PRIMASK bit */
        __disable_irq();                  /**< Disable all interrupts by setting PRIMASK bit on Cortex*/
        __DSB();

        /*        step1, DRV_DCACHE_Invalidate, DCACHE2:0x4005_5000       */
        if((*(volatile uint32_t *)0x40055000) & 0x01)  //cache enable, CACHE_CCR
        {
            //addr = 0x1000_0000, size = 0x1000      
            //size = (((addr & 0x0f) + size) + 0x0f) & ~0x0f;
            //addr = addr & ~0x0f;
            //DCACHE->CACHE_CPEA = addr;
            //DCACHE->CACHE_CPES = size | PAGE_CACHE_CLEAN_GO;
            //while (DCACHE->CACHE_CPES & PAGE_CACHE_CLEAN_GO) ;
            *(volatile uint32_t *)(0x40055000+0x180) = 0x10000000;
            *(volatile uint32_t *)(0x40055000+0x184) = 0x10001001;
            while((*(volatile uint32_t *)(0x40055000+0x184)) & 0x01);
        }
         
        /*        step2, Sys_SSI_Flash_Sector_Erase */
        for(i=0; i<3; i++)
        {
            ssi_cfg[i].SsiId = 1;  //
            ssi_cfg[i].Cmd  = 0x00000020;      //erase
            ssi_cfg[i].Addr = 0x10000000;

            ssi_cfg[i].SysDiv = 2;
            ssi_cfg[i].IsQpiMode = 0;
            ssi_cfg[i].StandBaudr = 0x0002;
            ssi_cfg[i].QuadBaudr = 0x0002;
            ssi_cfg[i].RxSampleDelay = 0x00000001;

            ssi_cfg[i].IsMaskInterrupt = 1;
            ssi_cfg[i].ProgramMode = 0;           //dma_mode or not
            ssi_cfg[i].Len =0;                    //program len
            ssi_cfg[i].Buf =0;                    //program buf addr
            ssi_cfg[i].Delay = 10; 
            ssi_cfg[i].Timeout = 0xffffffff;
        }

        //ssi_ops.xip_sys_clk_switch = (uint8_t (*)(SSI_ParaTypeDef *))     (((uint32_t *)(0x04007658 | 1)));
        //ssi_ops.ssi_open_xip = (void (*)(SSI_ParaTypeDef *))              (((uint32_t *)(0x040069d4 | 1)));
        //ssi_ops.ssi_close_xip = (void (*)(SSI_ParaTypeDef *))             (((uint32_t *)(0x0400665c | 1)));
        //ssi_ops.xip_enter_qpi = (uint8_t (*)(SSI_ParaTypeDef *))          (((uint32_t *)(0x0400731c | 1)));
        //ssi_ops.xip_exit_qpi = (uint8_t (*)(SSI_ParaTypeDef *))           (((uint32_t *)(0x040073d4 | 1)));
        //ssi_ops.ssi_flash_erase = (uint8_t (*)(SSI_ParaTypeDef *))        (((uint32_t *)(0x0400719c | 1)));
        //ssi_ops.ssi_flash_program = (uint8_t (*)(SSI_ParaTypeDef *))      (((uint32_t *)(0x040071fe | 1)));
        //ssi_ops.xip_flash_erase = (uint8_t (*)(SSI_ParaTypeDef *))        (((uint32_t *)(0x0400747a | 1)));
        //ssi_ops.xip_flash_program = (uint8_t (*)(SSI_ParaTypeDef *))      (((uint32_t *)(0x0400752e | 1)));
    


        xip_flash_erase = (uint8_t (*)(struct str_flash *))(((uint32_t *)(0x0400747a | 1)));
        xip_flash_erase(&ssi_cfg[0]);

        __set_PRIMASK(primask_bit);     /**< Restore PRIMASK bit*/
        while(1);
    }
}

/*
void __attribute__((section(".ramfunc"))) function_in_sram(void)
{
	printf("Address of %s %p\n", __func__, &function_in_sram);
	printf("Address of var_sram_data %p (%d)\n", &var_sram_data, var_sram_data);
while(1);
}
*/

#endif
