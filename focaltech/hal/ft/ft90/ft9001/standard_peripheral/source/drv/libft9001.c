/**
  **********************************************************************************
             Copyright(c) 2020 Focaltech Co. Ltd.
                      All Rights Reserved
  **********************************************************************************
  * @file    libft9001.c
  * @author  xukai
  * @version V1.0
  * @date    2021.12.09
  * @brief   系统时钟切换函数.
  *
  */

//#include "ft9001_reg.h"

#include "libft9001.h"
#include "type.h"
#include "cpm_reg.h"
#include "cpm_drv.h"

#define SYSCLK_SEL_MASK (0x3)
#define SYSCLK_SEL_OSC160M (0x01)
#define CLKSEL_ST_OSC160M (0x0200)

/*-----LIB VERSION------------------------------------------------------------------*/

uint8_t LIB_VERSION[] = "V1.1.0";

uint8_t * Lib_GetVersion(void)
{
    return LIB_VERSION;
}

/*-----LIB VERSION END---------------------------------------------------------------*/
void LIB_CPM_OscSwitch(int osc_sel)
{
    unsigned int tmp,trim_en_addr,trim_value_addr;
    unsigned char part;

    CPM->CSWCFGR &= ~(SYSCLK_SEL_MASK);

    //=========================================================================
    tmp = ((*(volatile unsigned int *)(0x40004000 + 0x5c)) & 0x3FFFFFFF);

    (*(volatile unsigned int *)(0x40004000 + 0x5c)) = (tmp);
    (*(volatile unsigned int *)(0x40004000 + 0x5c)) = (tmp|0x40000000);
    (*(volatile unsigned int *)(0x40004000 + 0x5c)) = (tmp|0x80000000);
    (*(volatile unsigned int *)(0x40004000 + 0x5c)) = (tmp|0xc0000000);
    (*(volatile unsigned int *)(0x40004000 + 0x5c)) = tmp | (1<<19);

    if( osc_sel == OSC_320M_HZ )
    {
        //判断哪个opt区有效,默认选择part2
        if(0x55aa55aa == *(unsigned int * )0x082007F8)//otp part2有效
        {
            part = 2;
        }
        else if(0x55aa55aa == *(unsigned int * )0x082007F4)//otp part1有效
        {
            part = 1;
        }
        else if(0x55aa55aa == *(unsigned int * )0x082007F0)//otp part0有效
        {
            part = 0;
        }
        else
        {
            part = 2;
        }

        trim_en_addr = 0x08200704 + part*0x60;
        trim_value_addr =  0x08200700 + part*0x60;
        //OSC320MHz
        if((*(volatile unsigned int *)(trim_en_addr)) == 0x77658320)
        {
            (*(volatile unsigned int *)(0x40004000 + 0x68)) = (*(volatile unsigned int *)(trim_value_addr));
        }
    }
    else if( osc_sel == OSC_360M_HZ )
    {
        //OSC360MHz
        tmp = (*(volatile unsigned int *)(0x082000ec));
        if((tmp & 0xff000000) == 0x92000000)
        {
            (*(volatile unsigned int *)(0x40004000 + 0x68)) = tmp;
        }
    }
    else if ( osc_sel == OSC_400M_HZ )
    {
        //OSC400MHz
        tmp = (*(volatile unsigned int *)(0x082000e4));
        if((tmp & 0xff000000) == 0x92000000)
        {
            (*(volatile unsigned int *)(0x40004000 + 0x68)) = tmp;
        }
    }

    CPM->CSWCFGR |= SYSCLK_SEL_OSC160M;
    while(CLKSEL_ST_OSC160M != (CPM->CSWCFGR & CLKSEL_ST_OSC160M));
}

uint8_t LIB_sysclock400_check(void)
{
    if( 0xFFFF == (*(volatile unsigned int *)(0x082000e4)))
    {
        return 1;
    }
    return 0;
}