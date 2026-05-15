/**
  ******************************************************************************
             Copyright(c) 2020 China Core Co. Ltd.
                      All Rights Reserved
  ******************************************************************************
  * @file    cache_drv.h
  * @author  Product application department
  * @version V1.0
  * @date    2020.02.25
  * @brief   Header file of CACHE module.
  *
  ******************************************************************************
*/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CACHE_DRV_C__
#define __CACHE_DRV_C__

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "cache_reg.h"

#ifdef FILE_OUT
/**@defgroup DRV 3 HAL driver
  *
  *@{
  */

/** @defgroup DRV_CACHE CACHE
  *
  *@{
  */
#endif
/*** 宏定义 *******************************************************************/
#ifdef FILE_OUT
/** @defgroup DRV_CACHE_Exported_Macros Exported Macros
  *
  * @{
  */
#endif

#define PAGE_CACHE_CLEAN_GO 0x00000001

#define INVW1           ((1) << 26)
#define INVW0           ((1) << 24)
#define PUSHW0          ((1) << 25)
#define PUSHW1          ((1) << 27)
#define GO              (((uint32_t)1) << 31)
#define ENWRBUF         ((1) << 1)
#define ENCACHE         ((1) << 0)

#define LGO             (0x01)

#define CACHE_LINE_SIZE 0x10
#define CACHE_LINE_MASK 0x0f

#define R0_WT_WB        ((1) << 0)
#define R0_ENCACHE      ((1) << 1)
#define R2_WT_WB        ((1) << 4)
#define R2_ENCACHE      ((1) << 5)
#define RAM0_WT_WB      ((1) << 12)
#define RAM0_ENCACHE    ((1) << 13)

#define WRITE_BACK          (0xff)
#define WRITE_THROUGH       (0xaa)
#define SPIM_WRITE_BACK     (0x3f)
#define SPIM_WRITE_THROUGH  (0x2a)

#define BOOT_CACHEOFF   (0x00ffffff)
#define ROM_CACHEOFF    (0xfffcffff)
#define SPIM1_CACHEOFF  (0xffff00ff)
#define SPIM2_CACHEOFF  (0xff00ffff)
#define SPIM3_CACHEOFF  (0xffffff00)

#define BOOT_CACHE_SHIFT    (24)
#define ROM_CACHE_SHIFT     (16)
#define SPIM1_CACHE_SHIFT   (8)
#define SPIM2_CACHE_SHIFT   (16)
#define SPIM3_CACHE_SHIFT   (0)

/**
  * @}
  */

/*** *****************************************************/
#ifdef FILE_OUT
/** @defgroup DRV_CACHE_Exported_Types Exported Types
  *
  * @{
  */
#endif
    /**
* @brief  CACHE 模式定义
*
*/
typedef enum
{
    CACHE_Off = 0,
    CACHE_Through,
    CACHE_Back,
} CACHE_ComTypeDef;

/**
* @brief  CACHE 初始化定义
*
*/
typedef struct
{
    CACHE_ComTypeDef BOOT;
    CACHE_ComTypeDef ROM;
    CACHE_ComTypeDef SPIM1;
    CACHE_ComTypeDef SPIM2;
    CACHE_ComTypeDef SPIM3;
} CACHE_InitTypeDef;

/**
  * @}
  */

/*** **************************************************************/
#ifdef FILE_OUT
/** @defgroup DRV_CACHE_Exported_Variables Exported Variables
  *
  * @{
  */
#endif

/**
  * @}
  */

/*** ******************************************************************/
#ifdef FILE_OUT
/** @defgroup DRV_CACHE_Exported_Functions Exported Functions
  * @{
  */
#endif
extern void DRV_ICACHE_Init(CACHE_ComTypeDef boot,
                            CACHE_ComTypeDef rom,
                            CACHE_ComTypeDef spim1,
                            CACHE_ComTypeDef spim2,
                            CACHE_ComTypeDef spim3,
                            uint32_t base);
extern void DRV_DCACHE_Init(CACHE_ComTypeDef boot,
                            CACHE_ComTypeDef rom,
                            CACHE_ComTypeDef spim1,
                            CACHE_ComTypeDef spim2,
                            CACHE_ComTypeDef spim3,
                            uint32_t base);
extern void DRV_ICACHE_Invalidate(uint32_t addr, uint32_t size, uint32_t base);
extern void DRV_DCACHE_Invalidate(uint32_t addr, uint32_t size, uint32_t base);

/**
  *@}
  */

#ifdef __cplusplus
}
#endif

#endif /* __CACHE_DRV_H */

/************************ (C) COPYRIGHT C*Core *****END OF FILE*************/
