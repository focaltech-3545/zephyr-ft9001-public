/*
 * Copyright (c) 2021 Focaltech Systems CO.,Ltd
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _FT_SYS_H_
#define _FT_SYS_H_

#include "libft9001.h"

struct str_flash
{
    uint8_t SsiId; // 1,2,3
    uint8_t SysDiv;
    uint8_t IsQpiMode;
    uint16_t StandBaudr;
    uint16_t QuadBaudr;
    uint32_t RxSampleDelay;

    uint8_t Cmd;    // later
    uint32_t Value; // later

    uint8_t IsMaskInterrupt;
    //------------------------------------
    uint8_t ProgramMode;
    uint32_t Addr; // later
    uint16_t Len;  // later
    uint32_t Buf;  // later
    uint32_t Delay;
    uint32_t Timeout;
};

typedef enum
{
    CPM_SYSCLK_OSC8M = 0,
    CPM_SYSCLK_OSC320M,
    CPM_SYSCLK_USBPHY240M,
    CPM_SYSCLK_OSCEXT
} CPM_SysClkSelTypeDef;

typedef enum
{
    CLK_DIV_1 = 0,
    CLK_DIV_2,
    CLK_DIV_3,
    CLK_DIV_4,
    CLK_DIV_5,
    CLK_DIV_6,
    CLK_DIV_7,
    CLK_DIV_8,
    CLK_DIV_9,
    CLK_DIV_10,
    CLK_DIV_11,
    CLK_DIV_12,
    CLK_DIV_13,
    CLK_DIV_14,
    CLK_DIV_15,
    CLK_DIV_16,
    CLK_DIV_17,
    CLK_DIV_18,
    CLK_DIV_19,
    CLK_DIV_20
} CPM_SysClkDivTypeDef, CPM_IpsClkDivTypeDef;

#if 0
typedef enum
{
    OSC_320M_HZ = 0,
    OSC_360M_HZ,
    OSC_400M_HZ
} CPM_SysClkTrimTypeDef;
#endif	

typedef struct
{
    CPM_SysClkSelTypeDef SysClkSource;
    CPM_SysClkTrimTypeDef SysClkTrim;
    CPM_SysClkDivTypeDef SysClkDiv;
    CPM_IpsClkDivTypeDef IpsClkDiv;

} SYS_ClkInitTypeDef;

#define __IO volatile /*!< Defines 'read / write' permissions              */

typedef struct
{
    __IO uint32_t SLPCFGR; /**< 00 sleep configuration register                     */
    __IO uint32_t SLPCR;   /**< 04 sleep control register                           */
    __IO uint32_t SCDIVR;  /**< 08 system clock divider register                    */
    __IO uint32_t PCDIVR1; /**< 0C speripheral clock divider register 1             */

    __IO uint32_t PCDIVR2;  /**< 10 speripheral clock divider register 2             */
    __IO uint32_t RESERVED; /**< 14 speripheral clock divider register 3             */
    __IO uint32_t CDIVUPDR; /**< 18 clock divider update register                    */
    __IO uint32_t CDIVENR;  /**< 1C clock divider enable register                    */

    __IO uint32_t OCSR;     /**< 20 oscillator control and status register           */
    __IO uint32_t CSWCFGR;  /**< 24 clock switch config register                     */
    __IO uint32_t CTICKR;   /**< 28 core tick timer register                         */
    __IO uint32_t CHIPCFGR; /**< 2C chip config register                             */

    __IO uint32_t PWRCR;      /**< 30 power control register                            */
    __IO uint32_t SLPCNTR;    /**< 34 sleep counter register                            */
    __IO uint32_t WKPCNTR;    /**< 38 wake up counter register                          */
    __IO uint32_t MULTICGTCR; /**< 3C multiple clock gate control register              */

    __IO uint32_t SYSCGTCR;   /**< 40 system clock gate control register                */
    __IO uint32_t AHB3CGTCR;  /**< 44 ahb3 clock gate control registe                   */
    __IO uint32_t ARITHCGTCR; /**< 48 arith clock gate control registe                  */
    __IO uint32_t IPSCGTCR;   /**< 4C ips clock gate control registe                    */

    __IO uint32_t VCCGTRIMR; /**< 50 vcc general trim register                         */
    __IO uint32_t VCCLTRIMR; /**< 54 vcc lv detect trim register                       */
    __IO uint32_t VCCVTRIMR; /**< 58 vcc vref trim register                            */
    __IO uint32_t VCCCTMR;   /**< 5C vcc core test mode register                       */

    __IO uint32_t O8MTRIMR;   /**< 60 osc8mhz trim register                             */
    __IO uint32_t RESERVED1;  /**< 64                                                   */
    __IO uint32_t O400MTRIMR; /**< 68 osc320mhz trim register                           */
    __IO uint32_t CARDTRIMR;  /**< 6C card ldo trim register                            */

    __IO uint32_t OSCLSTIMER; /**< 70 oscl stable time register                         */
    __IO uint32_t OSCHSTIMER; /**< 74 osch stable time register                         */
    __IO uint32_t OSCESTIMER; /**< 78 osce stable time register                         */
    __IO uint32_t PWRSR;      /**< 7C power status register                             */

    __IO uint32_t EPORTSLPCFGR; /**< 80                                             */
    __IO uint32_t EPORTCGTR;    /**< 84                                             */
    __IO uint32_t EPORTRSTCR;   /**< 88                                             */
    __IO uint32_t RTCTRIMR;     /**< 8C rtc trim register                                 */

    __IO uint32_t PADWKINTCR;  /**< 90 pad wakeup interrupt control register             */
    __IO uint32_t WKPFILTCNTR; /**< 94 wakeup filter counter register                    */
    __IO uint32_t CARDPOCR;    /**< 98 card power on counter register                    */
    __IO uint32_t RTCSTIMER;   /**< 9C rtc 32k stable time register                      */

    __IO uint32_t MPDSLPCR;     /**< A0 memory power down sleep control register          */
    __IO uint32_t RESERVED2[2]; /**< A4 A8                                                   */
    __IO uint32_t MULTIRSTCR;   /**< AC multiple reset control register                   */

    __IO uint32_t SYSRSTCR;    /**< B0 system reset control register                     */
    __IO uint32_t AHB3RSTCR;   /**< B4 ahb3 reset control register                       */
    __IO uint32_t ARITHRSTTCR; /**< B8 arith reset control register                      */
    __IO uint32_t IPRSTCR;     /**< BC ips reset control register                        */

    __IO uint32_t SLPCFGR2;     /**< C0 sleep config register 2                           */
    __IO uint32_t RESERVED3[3]; /**< C4 C8 CC                                             */

    __IO uint32_t PDNCNTR;      /**< D0 power down counter register                       */
    __IO uint32_t PONCNTR;      /**< D4 power on counter register                         */
    __IO uint32_t PCDIVR4;      /**< D8                           */
    __IO uint32_t RESERVED4;    /**< DC wake up source control register                   */
    __IO uint32_t PLLNFCCFGR;   /**< E0  NFC PLL config  register                             */
    __IO uint32_t PLLNFCSTIMER; /**< E4 wake up source control register                   */

} CPM_TypeDef;

#define CPM_BASE_ADDR (0x40004000)
#define CPM ((CPM_TypeDef *)(CPM_BASE_ADDR))
#define CPM_VCCGTRIMR_CORE_VOLTAGE_MASK (((uint32_t)1 << 15))
#define CPM_VCCCTMR_OVERWRITE_VCCGTRIMR_TRIM_EN (((uint32_t)1 << 23))
#define CPM_OCSR_OSC320M_CLK_EN (((uint32_t)1 << 3))
#define CPM_OCSR_OSC320M_STABLE (((uint32_t)1 << 11))
#define CPM_CSWCFGR_SOC_CLK_SOURCE_MASK ((uint32_t)(0xFFFFFFFC))
#define CPM_CSWCFGR_OSC320M_SELECT (((uint32_t)1 << 9))
#define CPM_CDIVENR_IPS_CLK_DIV_EN (((uint32_t)1 << 0))
#define CPM_CDIVUPDR_PERIPHERAL_DIV_UPDATE (((uint32_t)1 << 0))
#define CPM_PCDIVR_IPS_DIV_MASK ((uint32_t)(0xFFFFFFF0))
#define CPM_PCDIVR_IPS_DIV_SHIFT_MASK ((uint32_t)(0))

extern void LIB_CPM_OscSwitch(int32_t osc_sel);
typedef enum
{
    CPM_VREF_TRIM_090 = 0x10, /**< 0.90V          */
    CPM_VREF_TRIM_105 = 0x00, /**< 1.05V          */
    CPM_VREF_TRIM_110 = 0x01, /**< 1.10V          */
    CPM_VREF_TRIM_115 = 0x02, /**< 1.15V          */
    CPM_VREF_TRIM_121 = 0x03  /**< 1.21V          */
} CPM_VrefTrimValueTypeDef;

/*****************************************************************************/
#define _bit_set(value, bit) ((value) |= (bit))
#define _bit_clr(value, bit) ((value) &= ~(bit))
#define _bit_get(value, bit) ((value) & (bit))
#define _reg_write(reg, val) ((reg) = (val))
#define _reg_read(reg) ((reg))
#define _reg_clear(reg) ((reg) = (0x0))
#define _reg_modify(reg, clearmask, setmask) _reg_write((reg), (((_reg_read(reg)) & (clearmask)) | (setmask)))
#define _reg_chk(value, bit) ((value) & (bit))

#define _cpm_write_core_test_mode_value(val) _reg_write(CPM->VCCCTMR, (val))
#define _cpm_set_ldo_trim_value(val) _reg_modify(CPM->VCCGTRIMR, 0xFFFFFFF3, ((val) & 0x03) << 2)

#define _cpm_get_core_test_mode_value _reg_read(CPM->VCCCTMR)

#define _cpm_set_core_voltage_0V9_on _bit_set(CPM->VCCGTRIMR, CPM_VCCGTRIMR_CORE_VOLTAGE_MASK)
#define _cpm_set_core_voltage_0V9_off _bit_clr(CPM->VCCGTRIMR, CPM_VCCGTRIMR_CORE_VOLTAGE_MASK)

#define _cpm_set_overwrite_vccgtrimr_trim_en _bit_set(CPM->VCCCTMR, CPM_VCCCTMR_OVERWRITE_VCCGTRIMR_TRIM_EN)
#define _cpm_set_osc320m_clk_en _bit_set(CPM->OCSR, CPM_OCSR_OSC320M_CLK_EN)
#define _cpm_set_osc320m_clk_dis _bit_clr(CPM->OCSR, CPM_OCSR_OSC320M_CLK_EN)
#define _cpm_get_osc320m_stable_flag _reg_chk(CPM->OCSR, CPM_OCSR_OSC320M_STABLE)

#define _cpm_set_soc_clk_osc320m_en _reg_modify(CPM->CSWCFGR, CPM_CSWCFGR_SOC_CLK_SOURCE_MASK, 1)
#define _cpm_get_osc320m_select_flag _reg_chk(CPM->CSWCFGR, CPM_CSWCFGR_OSC320M_SELECT)

#define _cpm_set_ips_clk_div_en _bit_set(CPM->CDIVENR, CPM_CDIVENR_IPS_CLK_DIV_EN)
#define _cpm_set_ips_clk_div_dis _bit_clr(CPM->CDIVENR, CPM_CDIVENR_IPS_CLK_DIV_EN)
#define _cpm_update_peripheral_clk_div _bit_set(CPM->CDIVUPDR, CPM_CDIVUPDR_PERIPHERAL_DIV_UPDATE)

#define _cpm_chk_sys_clk_src(index) _reg_chk(CPM->CSWCFGR, (index))
#define _cpm_get_sys_clk_value _reg_read(CPM->SCDIVR)

#define _cpm_chk_ips_clk_div_en _reg_chk(CPM->CDIVENR, CPM_CDIVENR_IPS_CLK_DIV_EN)

#define _cpm_get_peripheral1_clk_div_value _reg_read(CPM->PCDIVR1)

#define _cpm_set_peripheral_ips_clk_div(div)                                                                           \
    _reg_modify(CPM->PCDIVR1, CPM_PCDIVR_IPS_DIV_MASK, (div) << CPM_PCDIVR_IPS_DIV_SHIFT_MASK)

#define DRV_SYS_OSC_CLK_400M (400 * 1000 * 1000) /**< 400Mhz*/
#define DRV_SYS_OSC_CLK_360M (360 * 1000 * 1000) /**< 360Mhz*/
#define DRV_SYS_OSC_CLK_320M (320 * 1000 * 1000) /**< 320Mhz*/
#define DRV_SYS_OSC_CLK_240M (240 * 1000 * 1000) /**< 240Mhz*/
#define DRV_SYS_OSC_CLK_12M (12 * 1000 * 1000)   /**< 12Mhz*/
#define DRV_SYS_OSC_CLK_8M (8 * 1000 * 1000)     /**< 8Mhz*/

typedef struct
{
    __IO uint8_t BDL;       // 0x00
    __IO uint8_t BDH;       // 0x01
    __IO uint8_t CR2;       // 0x02
    __IO uint8_t CR1;       // 0x03
    __IO uint8_t SR2;       // 0x04
    __IO uint8_t SR1;       // 0x05
    __IO uint8_t DRL;       // 0x06
    __IO uint8_t DRH;       // 0x07
    __IO uint8_t PORT;      // 0x08
    __IO uint8_t PURD;      // 0x09
    __IO uint8_t BRDF;      // 0x0a
    __IO uint8_t DDR;       // 0x0b
    __IO uint8_t IRCR;      // 0x0c
    __IO uint8_t TR;        // 0x0d
    __IO uint8_t FCR;       // 0x0e
    __IO uint8_t IRDR;      // 0x0f
    __IO uint8_t DCR;       // 0x10
    __IO uint8_t FSR;       // 0x11
    __IO uint8_t RXTOCTR;   // 0x12
    __IO uint8_t FCR2;      // 0x13
    __IO uint8_t RESERVED1; // 0x14
    __IO uint8_t FSR2;      // 0x15

} UART_TypeDef;

#define UART2_BASE_ADDR (0x40014000) /**< UART2  base address  */
#define UART3_BASE_ADDR (0x4001D000) /**< UART2  base address  */
#define UART2 ((UART_TypeDef *)UART2_BASE_ADDR)
#define UART3 ((UART_TypeDef *)UART3_BASE_ADDR)
#define UART_RE ((uint8_t)(1 << 2)) /**<*/
#define UART_CR2_TE_MASK 0x08

typedef struct
{
    __IO uint32_t CACHE_CCR;                                                        /**< 0x00  Control      */
    __IO uint32_t CACHE_CLCR;                                                       /**< 0x04  Line Control */
    __IO uint32_t CACHE_CSAR;                                                       /**< 0x08  Addr status  */
    __IO uint32_t CACHE_CCVR;                                                       /**< 0x0C  R/W value    */
    __IO uint32_t RESERVED1[4];                                                     /**< 0x10~0x1C */
    __IO uint32_t CACHE_CACR;                                                       /**< 0x20 */
    __IO uint32_t CACHE_CSACR;                                                      /**< 0x24 */
    __IO uint32_t RESERVED2[6];                                                     /**< 0x28~0x3C */

    __IO uint32_t CACHE_CSPI1S0HA;                                                  /**< 0x40 SPI1 */
    __IO uint32_t CACHE_CSPI1S1HA;                                                  /**< 0x44 */
    __IO uint32_t CACHE_CSPI1S2HA;                                                  /**< 0x48 */
    __IO uint32_t CACHE_CSPI1S3HA;                                                  /**< 0x4C */
    __IO uint32_t CACHE_CSPI1S0LA;                                                  /**< 0x50 */
    __IO uint32_t CACHE_CSPI1S1LA;                                                  /**< 0x54 */
    __IO uint32_t CACHE_CSPI1S2LA;                                                  /**< 0x58 */
    __IO uint32_t CACHE_CSPI1S3LA;                                                  /**< 0x5C */

    __IO uint32_t CACHE_CSPI2S0HA;                                                  /**< 0x60 SPI2 */
    __IO uint32_t CACHE_CSPI2S1HA;                                                  /**< 0x64 */
    __IO uint32_t CACHE_CSPI2S2HA;                                                  /**< 0x68 */
    __IO uint32_t CACHE_CSPI2S3HA;                                                  /**< 0x6C */
    __IO uint32_t CACHE_CSPI2S0LA;                                                  /**< 0x70 */
    __IO uint32_t CACHE_CSPI2S1LA;                                                  /**< 0x74 */
    __IO uint32_t CACHE_CSPI2S2LA;                                                  /**< 0x78 */
    __IO uint32_t CACHE_CSPI2S3LA;                                                  /**< 0x7C */

    __IO uint32_t CACHE_CSPI3S0HA;                                                  /**< 0x80 SPI3 */
    __IO uint32_t CACHE_CSPI3S1HA;                                                  /**< 0x84 */
    __IO uint32_t CACHE_CSPI3S2HA;                                                  /**< 0x88 */
    __IO uint32_t CACHE_CSPI3S3HA;                                                  /**< 0x8C */
    __IO uint32_t CACHE_CSPI3S0LA;                                                  /**< 0x90 */
    __IO uint32_t CACHE_CSPI3S1LA;                                                  /**< 0x94 */
    __IO uint32_t CACHE_CSPI3S2LA;                                                  /**< 0x98 */
    __IO uint32_t CACHE_CSPI3S3LA;                                                  /**< 0x9C */
    __IO uint32_t CACHE_CROMRS0HA;                                                  /**< 0xA0 ROM */
    __IO uint32_t CACHE_CROMRS1HA;                                                  /**< 0xA4 */
    __IO uint32_t CACHE_CROMRS2HA;                                                  /**< 0xA8 */
    __IO uint32_t CACHE_CROMRS3HA;                                                  /**< 0xAC */
    __IO uint32_t CACHE_CROMRS0LA;                                                  /**< 0xB0 */
    __IO uint32_t CACHE_CROMRS1LA;                                                  /**< 0xB4 */
    __IO uint32_t CACHE_CROMRS2LA;                                                  /**< 0xB8 */
    __IO uint32_t CACHE_CROMRS3LA;                                                  /**< 0xBC */


    __IO uint32_t RESERVED3[48];                                                    /**< 0xC0~0x17C */

    __IO uint32_t CACHE_CPEA;                                                       /**< 0x180 Page clear address */
    __IO uint32_t CACHE_CPES;                                                       /**< 0x184 Page clear size    */
    __IO uint32_t CACHE_CCG;                                                        /**< 0x188 clock gating */
} CACHE_TypeDef;
#define CACHE_BASE_ADDR                     (0x40051000)
#define CACHE2_BASE_ADDR                    (0x40055000)
#define ICACHE ((CACHE_TypeDef *)(CACHE_BASE_ADDR))
#define DCACHE ((CACHE_TypeDef *)(CACHE2_BASE_ADDR))
typedef enum
{
    CACHE_Off = 0,
    CACHE_Through,
    CACHE_Back,
} CACHE_ComTypeDef;


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


#define TC_BASE_ADDR (0x40006000)
#define TC_RN				(1<<0)

#ifndef _ASMLANGUAGE

#ifdef CONFIG_HAS_FT9002LIB
#include <ft9002s.h>
#include <system_ft9002s.h>
#endif

#ifdef CONFIG_HAS_FT9001LIB
#include <ft9001.h>
#include <system_ft9001.h>
#endif

#endif /* _ASMLANGUAGE */

#endif /* _SOC_ARM_FOCALTECH_FT90_SOC_H_ */
