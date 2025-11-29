/**
  ******************************************************************************
             Copyright(c) 2025 Focaltech Co. Ltd.
                      All Rights Reserved
  ******************************************************************************
  * @file    libft9001.h
  * @author  Product application department
  * @version V1.0
  * @date    2020.03.25
  * @brief   Header file of FT9001 库函数接口.
  * @note 库接口包括
  *   - get sn 接口
  *   - clk switch 接口
  * 
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LIBFT9001_H
#define __LIBFT9001_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "type.h"

typedef enum
{
    OSC_320M_HZ = 0,
    OSC_360M_HZ,
    OSC_400M_HZ
} CPM_SysClkTrimTypeDef;

#ifdef FILE_OUT
/**@defgroup DRV 3 HAL driver
  *
  *@{
  */

/** @defgroup DRV_LIB LIB FT9001
  *
  *@{
  */
#endif
/*** 宏定义 *******************************************************************/
#ifdef FILE_OUT
/** @defgroup DRV_LIB_Exported_Macros Exported Macros
  *
  * @{
  */
#endif
#define LOCK_JTAG_KEY 0xA5A5A5A5 /**<  */

/**
  * @}
  */

/*** 结构体、枚举变量定义 *****************************************************/
#ifdef FILE_OUT
/** @defgroup DRV_LIB_Exported_Types Exported Types
  *
  * @{
  */
#endif

/**
  * @}
  */

/*** 全局变量声明 **************************************************************/
#ifdef FILE_OUT
/** @defgroup DRV_LIB_Exported_Variables Exported Variables
  *
  * @{
  */
#endif

/**
  * @}
  */

/*** 函数声明 ******************************************************************/
#ifdef FILE_OUT
/** @defgroup DRV_LIB_Exported_Functions Exported Functions
  * @{
  */
#endif

    /* 获取库的版本 */
    extern uint8_t *Lib_GetVersion(void);

    /* EFLASH 函数接口 */
    //extern uint8_t LIB_EFLASH_Disboot(void);

    //extern uint8_t LIB_EFLASH_RecoveryToBoot(void);

    /* 获取芯片SN号函数接口，长度8字节 */
    extern void LIB_GetSerialNumber(uint8_t *buff);

    /* 时钟源切换接口 */
    extern void LIB_CPM_OscSwitch(int32_t osc_sel);

    extern uint8_t LIB_sysclock400_check(void);
    /**
  * @}
  */

    /**
  *@}
  */

    /**
  *@}
  */

#ifdef __cplusplus
}
#endif

#endif /* __LIBFT9001_H */

/************************ (C) COPYRIGHT Focaltech *****END OF FILE****/
