/**
    **********************************************************************************
             Copyright(c) 2020 China Core Co. Ltd.
                      All Rights Reserved
  **********************************************************************************
  * @file    cpm_drv.c
  * @author  Product application department
  * @version V1.0
  * @date    2020.03.12
        1. nothing.
  @endverbatim
  */

#include "cpm_drv.h"
#include "cpm_reg.h"

void DRV_CPM_SystemClkOSC320MSelect(void)
{
    uint32_t data;
    CPM->OCSR |= CPM_OCSR_OSC320M_CLK_EN;
    while(!(CPM->OCSR & CPM_OCSR_OSC320M_STABLE));

    data = CPM->CSWCFGR & CPM_CSWCFGR_SOC_CLK_SOURCE_MASK;
    data |= 1;
    CPM->CSWCFGR |= data;
    while(!(CPM->CSWCFGR & CPM_CSWCFGR_OSC320M_SELECT));
}

void DRV_CPM_SetIpsClkDiv(uint32_t div)
{
    uint32_t data;
    CPM->CDIVENR |= CPM_CDIVENR_IPS_CLK_DIV_EN;

    data = CPM->PCDIVR1 & CPM_PCDIVR_IPS_DIV_MASK;
    data |= (div << CPM_PCDIVR_IPS_DIV_SHIFT_MASK);
    CPM->PCDIVR1 |= data;
    CPM->CDIVUPDR |= CPM_CDIVUPDR_PERIPHERAL_DIV_UPDATE;
}



/************************ (C) COPYRIGHT C*Core *****END OF FILE**********************/
