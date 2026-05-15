// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// File name    : cpm_drv.h
// Version      : V0.1
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#ifndef __CPM_DRV_H__
#define __CPM_DRV_H__

#include "type.h"
#include "memmap.h"
#include "cpm_reg.h"




/**
* @brief  USBPHY
*
*/
typedef enum
{
    CPM_USBPHY_AUTO_DET = 0x00,
    CPM_USBPHY_INTER_OSC = 0x02,
    CPM_USBPHY_EXTER_OSC = 0x03
} CPM_UsbPhySrcTypeDef;

/**
* @brief  
*
*/
typedef enum
{
    CPM_VREF_TRIM_090 = 0x10, /**< 0.90V          */
    CPM_VREF_TRIM_105 = 0x00, /**< 1.05V          */
    CPM_VREF_TRIM_110 = 0x01, /**< 1.10V          */
    CPM_VREF_TRIM_115 = 0x02, /**< 1.15V          */
    CPM_VREF_TRIM_121 = 0x03  /**< 1.21V          */
} CPM_VrefTrimValueTypeDef;

/**
* @brief  
*
*/
typedef enum
{
    CPM_SYSCLK_OSC8M = 0,
    CPM_SYSCLK_OSC320M,
    CPM_SYSCLK_USBPHY240M,
    CPM_SYSCLK_OSCEXT
} CPM_SysClkSelTypeDef;






#define CPM ((CPM_TypeDef *)(CPM_BASE_ADDR))

extern void DRV_CPM_SystemClkOSC320MSelect(void);
extern void DRV_CPM_SetIpsClkDiv(uint32_t div);


#endif /* __CPM_DRV_H__ */
