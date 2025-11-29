/**  
  **********************************************************************************
             Copyright(c) 2025 Focaltech Co. Ltd.
                      All Rights Reserved
  **********************************************************************************
  */
  
#ifndef __CPM_REG_H__
#define __CPM_REG_H__

#include "type.h"

/**
* @brief CLOCK and POWER
*/
typedef struct
{
    __IO uint32_t SLPCFGR;                                                          /**< 00 sleep configuration register                     */
    __IO uint32_t SLPCR;                                                            /**< 04 sleep control register                           */
    __IO uint32_t SCDIVR;                                                           /**< 08 system clock divider register                    */
    __IO uint32_t PCDIVR1;                                                          /**< 0C speripheral clock divider register 1             */

    __IO uint32_t PCDIVR2;                                                          /**< 10 speripheral clock divider register 2             */
    __IO uint32_t RESERVED;                                                         /**< 14 speripheral clock divider register 3             */
    __IO uint32_t CDIVUPDR;                                                         /**< 18 clock divider update register                    */
    __IO uint32_t CDIVENR;                                                          /**< 1C clock divider enable register                    */

    __IO uint32_t OCSR;                                                             /**< 20 oscillator control and status register           */
    __IO uint32_t CSWCFGR;                                                          /**< 24 clock switch config register                     */
    __IO uint32_t CTICKR;                                                           /**< 28 core tick timer register                         */
    __IO uint32_t CHIPCFGR;                                                         /**< 2C chip config register                             */

    __IO uint32_t PWRCR;                                                            /**< 30 power control register                            */
    __IO uint32_t SLPCNTR;                                                          /**< 34 sleep counter register                            */
    __IO uint32_t WKPCNTR;                                                          /**< 38 wake up counter register                          */
    __IO uint32_t MULTICGTCR;                                                       /**< 3C multiple clock gate control register              */

    __IO uint32_t SYSCGTCR;                                                         /**< 40 system clock gate control register                */
    __IO uint32_t AHB3CGTCR;                                                        /**< 44 ahb3 clock gate control registe                   */
    __IO uint32_t ARITHCGTCR;                                                       /**< 48 arith clock gate control registe                  */
    __IO uint32_t IPSCGTCR;                                                         /**< 4C ips clock gate control registe                    */

    __IO uint32_t VCCGTRIMR;                                                        /**< 50 vcc general trim register                         */
    __IO uint32_t VCCLTRIMR;                                                        /**< 54 vcc lv detect trim register                       */
    __IO uint32_t VCCVTRIMR;                                                        /**< 58 vcc vref trim register                            */
    __IO uint32_t VCCCTMR;                                                          /**< 5C vcc core test mode register                       */

    __IO uint32_t O8MTRIMR;                                                         /**< 60 osc8mhz trim register                             */
    __IO uint32_t RESERVED1;                                                        /**< 64                                                   */
    __IO uint32_t O400MTRIMR;                                                       /**< 68 osc320mhz trim register                           */
    __IO uint32_t CARDTRIMR;                                                        /**< 6C card ldo trim register                            */

    __IO uint32_t OSCLSTIMER;                                                       /**< 70 oscl stable time register                         */
    __IO uint32_t OSCHSTIMER;                                                       /**< 74 osch stable time register                         */
    __IO uint32_t OSCESTIMER;                                                       /**< 78 osce stable time register                         */
    __IO uint32_t PWRSR;                                                            /**< 7C power status register                             */

    __IO uint32_t EPORTSLPCFGR;                                                     /**< 80                                             */
    __IO uint32_t EPORTCGTR;                                                        /**< 84                                             */
    __IO uint32_t EPORTRSTCR;                                                       /**< 88                                             */
    __IO uint32_t RTCTRIMR;                                                         /**< 8C rtc trim register                                 */

    __IO uint32_t PADWKINTCR;                                                       /**< 90 pad wakeup interrupt control register             */
    __IO uint32_t WKPFILTCNTR;                                                      /**< 94 wakeup filter counter register                    */
    __IO uint32_t CARDPOCR;                                                         /**< 98 card power on counter register                    */
    __IO uint32_t RTCSTIMER;                                                        /**< 9C rtc 32k stable time register                      */

    __IO uint32_t MPDSLPCR;                                                         /**< A0 memory power down sleep control register          */
    __IO uint32_t RESERVED2[2];                                                     /**< A4 A8                                                   */
    __IO uint32_t MULTIRSTCR;                                                       /**< AC multiple reset control register                   */

    __IO uint32_t SYSRSTCR;                                                         /**< B0 system reset control register                     */
    __IO uint32_t AHB3RSTCR;                                                        /**< B4 ahb3 reset control register                       */
    __IO uint32_t ARITHRSTTCR;                                                      /**< B8 arith reset control register                      */
    __IO uint32_t IPRSTCR;                                                          /**< BC ips reset control register                        */

    __IO uint32_t SLPCFGR2;                                                         /**< C0 sleep config register 2                           */
    __IO uint32_t RESERVED3[3];                                                     /**< C4 C8 CC                                             */

    __IO uint32_t PDNCNTR;                                                          /**< D0 power down counter register                       */
    __IO uint32_t PONCNTR;                                                          /**< D4 power on counter register                         */
    __IO uint32_t PCDIVR4;                                                          /**< D8                           */
    __IO uint32_t RESERVED4;                                                        /**< DC wake up source control register                   */
    __IO uint32_t PLLNFCCFGR;                                                       /**< E0  NFC PLL config  register                             */
    __IO uint32_t PLLNFCSTIMER;                                                     /**< E4 wake up source control register                   */

} CPM_TypeDef;

/*** CPM **********************************************/

/*sleep config register:SLPCFGR 0x0000 ~ 0x0003*/
#if 1
#define CPM_SLPCFGR_LOW_POWER_MODE          ((uint32_t)(0 << 30))
#define CPM_SLPCFGR_RETENTION_MODE          (((uint32_t)1 << 30))
#define CPM_SLPCFGR_DEEP_SLEEP_MODE         (((uint32_t)1 << 30))
#define CPM_SLPCFGR_HIBERNATION_MODE        (((uint32_t)1 << 31))
#define CPM_SLPCFGR_EPORT4_MODULE_CLOCK_SLEEP_EN (((uint32_t)1 << 28))              /**< eport4 module clock enable when stop*/
#define CPM_SLPCFGR_EPORT3_MODULE_CLOCK_SLEEP_EN (((uint32_t)1 << 27))              /**< eport3 module clock enable when stop*/
#define CPM_SLPCFGR_EPORT2_MODULE_CLOCK_SLEEP_EN (((uint32_t)1 << 26))              /**< eport2 module clock enable when stop*/
#define CPM_SLPCFGR_EPORT1_MODULE_CLOCK_SLEEP_EN (((uint32_t)1 << 22))              /**< eport1 module clock enable when stop*/
#define CPM_SLPCFGR_EPORT0_MODULE_CLOCK_SLEEP_EN (((uint32_t)1 << 21))              /**< eport0 module clock enable when stop*/
#define CPM_SLPCFGR_OSCEXT_SLEEP_EN              (((uint32_t)1 << 20))              /**< OSCEXT enable when stop*/
#define CPM_SLPCFGR_PMU128K_SLEEP_EN             (((uint32_t)1 << 19))              /**< PMU128K enable when stop*/
#define CPM_SLPCFGR_RTC32K_SLEEP_EN              (((uint32_t)1 << 16))              /**< RTC32K enable when stop*/
#define CPM_SLPCFGR_VDD33_LDO_ENTER_LOWPOWER_EN  (((uint32_t)1 << 18))              /**< VDD LDO enter lowpower when stop*/
#define CPM_SLPCFGR_FLASH_LDO_ENTER_LOWPOWER_EN  (((uint32_t)1 << 17))              /**< FLASH LDO enter lowpower when stop*/
#define CPM_SLPCFGR_CARD0_LDO_POWEROFF           (((uint32_t)1 << 11))              /**< Card0 LDO poweroff*/
#define CPM_SLPCFGR_CARD0_LDO_VOTAGE_OUT_1V8     ((uint32_t)(0 << 8))
#define CPM_SLPCFGR_CARD0_LDO_VOTAGE_OUT_3V0     (((uint32_t)1 << 8))
#define CPM_SLPCFGR_CARD0_LDO_VOTAGE_OUT_3V3     ((uint32_t)(3 << 8))
//#define CPM_SLPCFGR_CARD0_LDO_VOTAGE_OUT_5V0          ((uint32_t)(3<<8))
#define CPM_SLPCFGR_FLASH_LDO_ENTER_LOWPOWER_POWEROFF   (((uint32_t)1 << 7))        /**< FLASH LDO poweroff when stop*/
#define CPM_SLPCFGR_FLASH_IP_ENTER_LOWPOWER_EN          (((uint32_t)1 << 4))        /**< FLASH IP enter lowpower when stop*/

/*sleep control register:SLPCR 0x0004 ~ 0x0007*/
#define CPM_SLPCR_SLEEP_CONF_MODE           (((uint32_t)1 << 29))

#else
#define CPM_SLPCFGR_SLEEP_MODE_POS          (30U)                               /*!< CPM SLPCFGR: SLEEP MODE Position */
#define CPM_SLPCFGR_SLEEP_MODE_MASK         (0xC0000000U)                       /*!< CPM SLPCFGR: SLEEP MODE Mask */
#define CPM_SLPCFGR_LOW_POWER_MODE          (0U << 31)
#define CPM_SLPCFGR_HIBERNATION_MODE        (1U << 31)
#define CPM_SLPCFGR_LP_DIS_EN_POS           (29U)                               /*!< CPM SLPCFGR: Lowpower Disable Position */
#define CPM_SLPCFGR_LP_DIS_EN_MASK          (1U << CPM_SLPCFGR_LP_DIS_EN_POS)   /*!< CPM SLPCFGR: Lowpower Disable Mask */
#define CPM_SLPCFGR_EXTFLASH_IPSLP_POS      (26U)                               /*!< CPM SLPCFGR: Lowpower Disable Position */
#define CPM_SLPCFGR_EXTFLASH_IPSLP_MASK     (0x1C000000U)                       /*!< CPM SLPCFGR: Lowpower Disable Mask */
#define CPM_SLPCFGR_EXTFLASH_IPSLP_SSI4_MASK (1U << CPM_SLPCFGR_EXTFLASH_IPSLP_POS)
#define CPM_SLPCFGR_EXTFLASH_IPSLP_SSI5_MASK (1U << 27U)
#define CPM_SLPCFGR_EXTFLASH_IPSLP_SSI6_MASK (1U << 28U)
#define CPM_SLPCFGR_MCC_SLPEN_POS           (23U)                                           /*!< CPM SLPCFGR: MCC Sleep Enable Position */
#define CPM_SLPCFGR_MCC_SLPEN_MASK          (1U << CPM_SLPCFGR_MCC_SLPEN_POS)               /*!< CPM SLPCFGR: MCC Sleep Enable Mask */
#define CPM_SLPCFGR_HP_READY_WKPWAIT_POS    (22U)                                           /*!< CPM SLPCFGR: Wait HPLDO READY Position */
#define CPM_SLPCFGR_HP_READY_WKPWAIT_MASK   (1U << CPM_SLPCFGR_HP_READY_WKPWAIT_POS)        /*!< CPM SLPCFGR: Wait HPLDO READY Mask */
#define CPM_SLPCFGR_WAKEUP_NM_SLPWAIT_POS   (21U)                                           /*!< CPM SLPCFGR: Wait WAKEUP NM Position */
#define CPM_SLPCFGR_WAKEUP_NM_SLPWAIT_MASK  (1U << CPM_SLPCFGR_WAKEUP_NM_SLPWAIT_POS)       /*!< CPM SLPCFGR: Wait WAKEUP NM Mask */
#define CPM_SLPCFGR_OSCEXT_SLEEP_EN_POS     (20U)                                           /*!< CPM SLPCFGR: OSCEXT SLEEP Enable Position */
#define CPM_SLPCFGR_OSCEXT_SLEEP_EN         (1U << CPM_SLPCFGR_OSCEXT_SLEEP_EN_POS)         /*!< CPM SLPCFGR: OSCEXT SLEEP Enable Mask */
#define CPM_SLPCFGR_PMU128K_SLEEP_EN_POS    (19U)                                           /*!< CPM SLPCFGR: PMU128K SLEEP Enable Position */
#define CPM_SLPCFGR_PMU128K_SLEEP_EN_MASK   (1U << CPM_SLPCFGR_PMU128K_SLEEP_EN_POS)        /*!< CPM SLPCFGR: PMU128K SLEEP Enable Mask */
#define CPM_SLPCFGR_RTC32K__SLEEP_EN_POS    (16U)                                           /*!< CPM SLPCFGR: RTC32K SLEEP Enable Position */
#define CPM_SLPCFGR_RTC32K_SLEEP_EN_MASK    (1U << CPM_SLPCFGR_RTC32K__SLEEP_EN_POS)        /*!< CPM SLPCFGR: RTC32K SLEEP Enable Mask */
#define CPM_SLPCFGR_CARD0_LDO_POWEROFF_POS  (11)                                            /*!< CPM SLPCFGR: CARD0 LDO POWEROFF Position */
#define CPM_SLPCFGR_CARD0_LDO_POWEROFF_MASK (1U << CPM_SLPCFGR_CARD0_LDO_POWEROFF_POS)      /*!< CPM SLPCFGR: CARD0 LDO POWEROFF Mask */
#define CPM_SLPCFGR_CARD0_LDO_POWER_POS     (8)                                             /*!< CPM SLPCFGR: CARD0 LDO POWER Position */
#define CPM_SLPCFGR_CARD0_LDO_POWER_MASK    (0x00000300U)                                   /*!< CPM SLPCFGR: CARD0 LDO POWER Mask */
#define CPM_SLPCFGR_CARD0_LDO_VOTAGE_OUT_1V8 (0U << CPM_SLPCFGR_CARD0_LDO_POWER_POS)
#define CPM_SLPCFGR_CARD0_LDO_VOTAGE_OUT_3V0 (1U << CPM_SLPCFGR_CARD0_LDO_POWER_POS)
#define CPM_SLPCFGR_CARD0_LDO_VOTAGE_OUT_3V3 (3U << CPM_SLPCFGR_CARD0_LDO_POWER_POS)
#define CPM_SLPCFGR_OTP_IPSLP_POS           (4U)                                            /*!< CPM SLPCFGR: OPT IP SLEEP Position */
#define CPM_SLPCFGR_OTP_IPSLP_MASK          (1U << CPM_SLPCFGR_OTP_IPSLP_POS)               /*!< CPM SLPCFGR: OPT IP SLEEP Mask */
#define CPM_SLPCFGR_SSI_IDLE_WKPWAIT_POS    (1U)                                            /*!< CPM SLPCFGR: WAKEUP WAIT SSI IDLE Position */
#define CPM_SLPCFGR_SSI_IDLE_WKPWAIT_MASK   (1U << CPM_SLPCFGR_SSI_IDLE_WKPWAIT_POS)        /*!< CPM SLPCFGR: WAKEUP WAIT SSI IDLE Mask */
#define CPM_SLPCFGR_SSI_IDLE_SLPWAIT_POS    (0U)                                            /*!< CPM SLPCFGR: SLEEP WAIT SSI IDLE Position */
#define CPM_SLPCFGR_SSI_IDLE_SLPWAIT_MASK   (1U << CPM_SLPCFGR_SSI_IDLE_SLPWAIT_POS)        /*!< CPM SLPCFGR: SLEEP WAIT SSI IDLE Mask */

/*sleep control register:SLPCR 0x0004 ~ 0x0007*/
#define CPM_SLPCR_SLP_CFG_KEY_POS           (30U)                                           /*!< CPM SLPCR: SLEEP CONFIG KEY Position */
#define CPM_SLPCR_SLP_CFG_KEY_MASK          (0xC0000000U)                                   /*!< CPM SLPCR: SLEEP CONFIG KEY Mask */
#define CPM_SLPCR_SLP_CFG_MODE_POS          (29U)                                           /*!< CPM SLPCR: SLEEP CONFIG MODE Position */
#define CPM_SLPCR_SLP_CFG_MODE_MASK         (1U << CPM_SLPCR_SLP_CFG_MODE_POS)              /*!< CPM SLPCR: SLEEP CONFIG MODE Mask */
#endif

/*system clock divider register:SCDIVR 0x0008 ~ 0x000B*/
#define CPM_SCDIVR_CLKOUT_DIV_MASK          ((uint32_t)(0xFF00FFFF))                /**< */
#define CPM_SCDIVR_CLKOUT_DIV_SHIFT_MASK    (((uint32_t)16))                        /**<  */
#define CPM_SCDIVR_TRACE_DIV_MASK           ((uint32_t)(0xFFFF00FF))                /**< */
#define CPM_SCDIVR_TRACE_DIV_SHIFT_MASK     ((uint32_t)(8))                         /**<  */
#define CPM_SCDIVR_SYS_DIV_MASK             ((uint32_t)(0xFFFFFF00))                /**< */
#define CPM_SCDIVR_SYS_DIV_SHIFT_MASK       ((uint32_t)(0))                         /**< */

/*periphal clock divider register 1:PCDIVR1 0x000C ~ 0x000F*/
#define CPM_PCDIVR_ARITH_DIV_MASK           ((uint32_t)(0xFFFF0FFF))                /**< */
#define CPM_PCDIVR_ARITH_DIV_SHIFT_MASK     (((uint32_t)12))                        /**< */
#define CPM_PCDIVR_AHB3_DIV_MASK            ((uint32_t)(0xFFFFF0FF))                /**< */
#define CPM_PCDIVR_AHB3_DIV_SHIFT_MASK      ((uint32_t)(8))                         /**< */
#define CPM_PCDIVR_IPS_DIV_MASK             ((uint32_t)(0xFFFFFFF0))                /**< */
#define CPM_PCDIVR_IPS_DIV_SHIFT_MASK       ((uint32_t)(0))                         /**< */

/*periphal clock divider register 2:PCDIVR2 0x0010 ~ 0x0013*/
#define CPM_PCDIVR_TC_DIV_MASK              ((uint32_t)(0x0FFFFFFF))                /**< */
#define CPM_PCDIVR_TC_DIV_SHIFT_MASK        ((uint32_t)(28))                        /**<  */
#define CPM_PCDIVR_MESH_DIV_MASK            ((uint32_t)(0xF0FFFFFF))                /**< */
#define CPM_PCDIVR_MESH_DIV_SHIFT_MASK      ((uint32_t)(24))                        /**<  */
#define CPM_PCDIVR_ADC_DIV_MASK             ((uint32_t)(0xFFFF0FFF))                /**< */
#define CPM_PCDIVR_ADC_DIV_SHIFT_MASK       (((uint32_t)12))                        /**< */
#define CPM_PCDIVR_MCC_ADR_DIV_MASK         ((uint32_t)(0xFFFFF1FF))                /**< */
#define CPM_PCDIVR_MCC_ADR_DIV_SHIFT_MASK   ((uint32_t)(9))                         /**< */
#define CPM_PCDIVR_MCC_DIV_MASK             ((uint32_t)(0xFFFFFE00))                /**< */
#define CPM_PCDIVR_MCC_DIV_SHIFT_MASK       ((uint32_t)(0))                         /**< */

/*periphal clock divider register 3:PCDIVR3 0x0010 ~ 0x0013*/
/*clock divider update register£ºCDIVUPDR 0x0018 ~ 0x001B*/
#define CPM_CDIVUPDR_SYS_DIV_UPDATE         (((uint32_t)1 << 1))                    /**< sys clk update*/
#define CPM_CDIVUPDR_PERIPHERAL_DIV_UPDATE  (((uint32_t)1 << 0))                    /**< peripheral clk update*/

/*clock divider enable register£ºCDIVENR 0x001C ~ 0x001F*/
#define CPM_CDIVENR_I2S_S_DIVEN             (((uint32_t)1 << 23))
#define CPM_CDIVENR_I2S_M_DIVEN             (((uint32_t)1 << 22))
#define CPM_CDIVENR_CLKOUT_CLK_DIV_EN       (((uint32_t)1 << 15))
#define CPM_CDIVENR_TRACE_CLK_DIV_EN        (((uint32_t)1 << 14))
#define CPM_CDIVENR_TC_CLK_DIV_EN           (((uint32_t)1 << 13))
#define CPM_CDIVENR_MESH_CLK_DIV_EN         (((uint32_t)1 << 12))
#define CPM_CDIVENR_ADC_CLK_DIV_EN          (((uint32_t)1 << 10))
#define CPM_CDIVENR_MCC_CLK_DIV_EN          (((uint32_t)1 << 8))
#define CPM_CDIVENR_ARITH_CLK_DIV_EN        (((uint32_t)1 << 3))
#define CPM_CDIVENR_AHB3_CLK_DIV_EN         (((uint32_t)1 << 2))
#define CPM_CDIVENR_IPS_CLK_DIV_EN          (((uint32_t)1 << 0))

/*oscillator control and status register£ºOCSR 0x0020 ~ 0x0023*/
#define CPM_OCSR_TRNG_OSCEN                 ((uint32_t)(0xF << 24))                 /**< */
#define CPM_OCSR_PLLNFC_STABLE              (((uint32_t)1 << 15))                   /**< */
#define CPM_OCSR_PMU2K_STABLE               (((uint32_t)1 << 14))                   /**< */
#define CPM_OCSR_RTC32K_STABLE              (((uint32_t)1 << 13))                   /**< */
#define CPM_OCSR_OSCEXT_STABLE              (((uint32_t)1 << 12))                   /**< */
#define CPM_OCSR_OSC320M_STABLE             (((uint32_t)1 << 11))                   /**< */
#define CPM_OCSR_USBPHY240M_STABLE          (((uint32_t)1 << 10))                   /**< */
#define CPM_OCSR_PMU128K_STABLE             (((uint32_t)1 << 9))                    /**< */
#define CPM_OCSR_OSC8M_STABLE               (((uint32_t)1 << 8))                    /**< */
#define CPM_OCSR_PLLNFC_EN                  (((uint32_t)1 << 7))
#define CPM_OCSR_PMU2K_CLK_EN               (((uint32_t)1 << 6))
#define CPM_OCSR_RTC32K_CLK_EN              (((uint32_t)1 << 5))
#define CPM_OCSR_OSCEXT_CLK_EN              (((uint32_t)1 << 4))
#define CPM_OCSR_OSC320M_CLK_EN             (((uint32_t)1 << 3))
#define CPM_OCSR_USBPHY240M_CLK_EN          (((uint32_t)1 << 2))
#define CPM_OCSR_PMU128K_CLK_EN             (((uint32_t)1 << 1))
#define CPM_OCSR_OSC8M_CLK_EN               (((uint32_t)1 << 0))

/*clock switch config register: CSWCFGR 0x0024 ~ 0x0027*/
#define CPM_CSWCFGR_OSC8M_SELECT            (((uint32_t)1 << 8))                    /**< */
#define CPM_CSWCFGR_OSC320M_SELECT          (((uint32_t)1 << 9))                    /**< */
#define CPM_CSWCFGR_USBPHY240M_SELECT       (((uint32_t)1 << 10))                   /**< */
#define CPM_CSWCFGR_OSCEXT_SELECT           (((uint32_t)1 << 11))                   /**< */
#define CPM_CSWCFGR_SOC_CLK_SOURCE_MASK     ((uint32_t)(0xFFFFFFFC))                /**< */
#define CPM_CSWCFGR_CLKOUT_SOURCE_SYS       ((uint32_t)(0 << 24))                   /**< */
#define CPM_CSWCFGR_CLKOUT_SOURCE_ARITH     (((uint32_t)1 << 24))                   /**< */
#define CPM_CSWCFGR_CLKOUT_SOURCE_PLLNFC    ((uint32_t)(2 << 24))                   /**< */
#define CPM_CSWCFGR_CLKOUT_SOURCE_OSCL      ((uint32_t)(3 << 24))                   /**< */

/*core tick timer register:CTICKR 0x0028 ~ 0x002B*/
#define CPM_CTICKR_REFERENCE_CLK_SELECT_MASK (((uint32_t)1 << 25))                  /**< */
#define CPM_CTICKR_SKEW_EN                  (((uint32_t)1 << 24))                   /**< */

/*chip config register:CHIPCFGR:CHIPCFGR 0x002C ~ 0x002F*/
#define CPM_CHIPCFGR_USBPHY_OSC_MODE_AUTO   ((uint32_t)(0 << 30))                   /**< auto dection oscillator*/
#define CPM_CHIPCFGR_USBPHY_OSC_MODE_AUTO_FOR_SIMU (((uint32_t)1 << 30))            /**< auto dection oscillator for fast simulation*/
#define CPM_CHIPCFGR_USBPHY_OSC_MODE_INTER  ((uint32_t)(2 << 30))                   /**< select internal oscillator*/
#define CPM_CHIPCFGR_USBPHY_OSC_MODE_EXTER  ((uint32_t)(3 << 30))                   /**< select external oscillator*/
#define CPM_CHIPCFGR_USBPHY_CONF_SOFTWARE_MASK  (((uint32_t)1 << 29))
#define CPM_CHIPCFGR_USBPHY_PLL_SOFTWARE_MASK   (((uint32_t)1 << 28))
#define CPM_CHIPCFGR_USBPHY_RESET_SIGNAL_MASK   (((uint32_t)1 << 25))
#define CPM_CHIPCFGR_USBPHY_POWER_SWITCH_EN     (((uint32_t)1 << 24))
#define CPM_CHIPCFGR_USBPHY_IP_SOFTWARE_MASK    (((uint32_t)1 << 23))
#define CPM_CHIPCFGR_PCI_H2L_ISOLATION_SEL_MASK (((uint32_t)1 << 17))
#define CPM_CHIPCFGR_PCI_H2L_ISOLATION_EN   (((uint32_t)1 << 16))
#define CPM_CHIPCFGR_RTC1S_CLK_GATE_EN      (((uint32_t)1 << 14))
#define CPM_CHIPCFGR_RTC1K_CLK_GATE_EN      (((uint32_t)1 << 13))
#define CPM_CHIPCFGR_RTC32K_CLK_GATE_EN     (((uint32_t)1 << 12))
#define CPM_CHIPCFGR_RTC32K_ISOLATION_EN    (((uint32_t)1 << 11))
#define CPM_CHIPCFGR_RIM_ARST_MASK          (((uint32_t)1 << 10))
#define CPM_CHIPCFGR_RIM_RST_MASK           (((uint32_t)1 << 9))
#define CPM_CHIPCFGR_RIM_SOFTRST_MASK       (((uint32_t)1 << 8))
#define CPM_CHIPCFGR_USBPHY_12M_EN_MASK     (((uint32_t)1 << 3))
#define CPM_CHIPCFGR_OSCEXT_PAD_TE          (((uint32_t)1 << 2))
#define CPM_CHIPCFGR_OSCEXT_PAD_SF          ((uint32_t)(3 << 0))

/*power control register:PWRCR 0x0030 ~ 0x0033*/
#define CPM_PWRCR_VCC_IO_LATCH_CLR_MASK     ((uint32_t)1 << 31)
#define CPM_PWRCR_VCC_IO_LATCH_SET_MASK     ((uint32_t)1 << 30)
#define CPM_PWRCR_VCC_3V3_LV_DETECT_RESET_EN    (((uint32_t)1 << 29))
#define CPM_PWRCR_VCARD0_INTERFACE_ISOLATION_EN (((uint32_t)1 << 25))
#define CPM_PWRCR_VCC_3V3_LVD_POWERDOWN_MASK    (((uint32_t)1 << 23))
#define CPM_PWRCR_CARD0_LV_DETECT_RESET_EN  (((uint32_t)1 << 9))
#define CPM_PWRCR_VCC_3V3_LV_DETECT_INT_EN  (((uint32_t)1 << 15))
#define CPM_PWRCR_CARD0_LV_DETECT_INT_EN    (((uint32_t)1 << 13))
#define CPM_PWRCR_CARD0_IE_EN_FAIL          (((uint32_t)1 << 11))
#define CPM_PWRCR_CARD0_RE_LVD              (((uint32_t)1 << 9))
#define CPM_PWRCR_VCC_OE_LVDT33             (((uint32_t)1 << 7))
#define CPM_PWRCR_CARD0_OE_LVD              (((uint32_t)1 << 5))
#define CPM_PWRCR_VCC_3V3_LV_DETECT_EN      (((uint32_t)1 << 3))
#define CPM_PWRCR_CARD0_LV_DETECT_EN        (((uint32_t)1 << 1))

/*sleep counter register:SLPCNTR 0x0034~ 0x0037*/

/*wake up counter register:WKPCNTR 0x0038~ 0x003B*/

/*multiple clock gate control register:MULTICGTCR £º0x003C ~ 0x003F*/
/*system clock gate control register:SYSCGTCR 0x0040 ~ 0x0043*/
/*ahb3 clock gate control register:AHB3CGTCR 0x0044 ~ 0x0047*/
/*arith clock gate control register:ARITHCGTCR 0x0048 ~ 0x004B*/
/*ips clock gate control register:IPSCGTCR 0x004C ~ 0x004F*/

/*vcc general trim register:VCCGTRIMR 0x0050 ~ 0x0053*/
#define CPM_VCCGTRIMR_DISCHARGE_EN          (((uint32_t)1 << 30))                   /**< when this bit is set,discharge vd33 when chip switch to poff2 mode*/
#define CPM_VCCGTRIMR_2KHZ_CLK_GATE_EN      (((uint32_t)1 << 23))
#define CPM_VCCGTRIMR_CORE_VOLTAGE_MASK     (((uint32_t)1 << 15))                   /**< when this bit is set, the core voltage is 0.9V*/
#define CPM_VCCGTRIMR_VCC_LATCH_AUTO_SET_MASK (((uint32_t)1 << 13))
#define CPM_VCCGTRIMR_VCC_LATCH_AUTO_CLR_MASK (((uint32_t)1 << 12))
#define CPM_VCCGTRIMR_VCC_LATCH_AUTO_PORCLR (((uint32_t)1 << 11))
#define CPM_VCCGTRIMR_TEST_BIAS_CURRENT_EN  (((uint32_t)1 << 7))                    /**< test the bias current enable signal*/

/*vcc lv detect trim register:VCCLTRIMR 0x0054 ~ 0x0057*/
#define CPM_VCCLTRIMR_OTP_LVDT_MASK         (((uint32_t)1 << 25))
#define CPM_VCCLTRIMR_COARSE_LVD_MODULE_EN  (((uint32_t)1 << 24))

/*vcc vref trim register:VCCVTRIMR 0x0058 ~ 0x005B*/
#define CPM_VCCVTRIMR_SLEEP_CONF_REG_PROTECT_EN     (((uint32_t)1 << 31))
#define CPM_VCCVTRIMR_POFF2_WAKEUP_SOURCE_USBDET    (((uint32_t)1 << 16))
#define CPM_VCCVTRIMR_VREF_STABLE_MASK              (((uint32_t)1 << 11))
#define CPM_VCCVTRIMR_VREF_TRIM_EN                  (((uint32_t)1 << 10))
#define CPM_VCCVTRIMR_VREF_TRIM_VALUE_LOAD_BIT      (((uint32_t)1 << 9))
#define CPM_VCCVTRIMR_STORE_VREF_VOLTAGE_VALUE_EN   (((uint32_t)1 << 8))

/*vcc core test mode register:VCCCTMR 0x005C ~ 0x005F*/
#define CPM_VCCCTMR_OVERWRITE_CSWCFGR_TRIM_EN       (((uint32_t)1 << 29))
#define CPM_VCCCTMR_OVERWRITE_RTCTRIMR_TRIM_EN      (((uint32_t)1 << 28))
#define CPM_VCCCTMR_OVERWRITE_RTCSTIMER_TRIM_EN     (((uint32_t)1 << 26))
#define CPM_VCCCTMR_OVERWRITE_CARDTRIMR_TRIM_EN     (((uint32_t)1 << 24))
#define CPM_VCCCTMR_OVERWRITE_VCCGTRIMR_TRIM_EN     (((uint32_t)1 << 23))
#define CPM_VCCCTMR_OVERWRITE_VCCLTRIMR_TRIM_EN     (((uint32_t)1 << 22))
#define CPM_VCCCTMR_OVERWRITE_VCCVTRIMR_TRIM_EN     (((uint32_t)1 << 21))
#define CPM_VCCCTMR_OVERWRITE_O8MTRIMR_TRIM_EN      (((uint32_t)1 << 20))
#define CPM_VCCCTMR_OVERWRITE_O320MTRIMR_TRIM_EN    (((uint32_t)1 << 19))
#define CPM_VCCCTMR_OVERWRITE_OSCLSTIMER_TRIM_EN    (((uint32_t)1 << 18))
#define CPM_VCCCTMR_OVERWRITE_OSCHSTIMER_TRIM_EN    (((uint32_t)1 << 17))
#define CPM_VCCCTMR_OVERWRITE_OSCESTIMER_TRIM_EN    (((uint32_t)1 << 16))
#define CPM_VCCCTMR_OVERWRITE_ARITHCGTCR_TRIM_EN    (((uint32_t)1 << 13))
#define CPM_VCCCTMR_OVERWRITE_SCDIVR_TRIM_EN        (((uint32_t)1 << 11))
#define CPM_VCCCTMR_OVERWRITE_PCDIVR_TRIM_EN        (((uint32_t)1 << 10))
#define CPM_VCCCTMR_OVERWRITE_OCSR_TRIM_EN          (((uint32_t)1 << 9))
#define CPM_VCCCTMR_CPU_CORE_TEST_MODE_EN           (((uint32_t)1 << 7))
#define CPM_VCCCTMR_SOFT_POR                        (((uint32_t)1 << 3))
#define CPM_VCCCTMR_OFF_MODE2                       (((uint32_t)1 << 2))
#define CPM_VCCCTMR_EN_LP                           (((uint32_t)1 << 0))

/*osc8mhz trim register:O8MTRIMR 0x0060 ~ 0x0063*/


/*card ldo trim trgister:CARDTRIMR 0x006C ~ 0x006F*/
#define CPM_CARDTRIMR_WAKEUP_FILTER_EN                  (((uint32_t)1 << 30))
#define CPM_CARDTRIMR_WAKEUP_FILTER_BYPASS_EN           (((uint32_t)1 << 29))
#define CPM_CARDTRIMR_WAKEUP_FILTER_CLK_GATE_EN         (((uint32_t)1 << 28))
#define CPM_CARDTRIMR_WAKEUP_VDD33_PSWEN                (((uint32_t)1 << 24))
#define CPM_CARDTRIMR_WAKEUP_ANALOG_FILTER_BYPASS_EN    (((uint32_t)1 << 15))
#define CPM_CARDTRIMR_CARD0_REDUCE                      (((uint32_t)1 << 13))

/*oscl stable time register:OSCLSTIMER 0x0070 ~ 0x0073*/
/*osch stable time register:OSCHSTIMER 0x0074 ~ 0x0077*/
/*osce stable time register:OSCESTIMER 0x0078 ~ 0x007B*/

/*power status register:PWRSR 0x007C ~ 0x007F*/
#define CPM_PWRSR_VCARD_ISOLATION_FLAG      (((uint32_t)1 << 26))
#define CPM_PWRSR_VCC3V3_LVD_FLAG           (((uint32_t)1 << 23))
#define CPM_PWRSR_CARD0_LVD_FLAG            (((uint32_t)1 << 21))
#define CPM_PWRSR_VCC3V3_LVD_REAL_TIME_FLAG (((uint32_t)1 << 19))
#define CPM_PWRSR_CARD0_LVD_REAL_TIME_FLAG  (((uint32_t)1 << 17))
#define CPM_PWRSR_CARD0_LVD_FAIL_FLAG       (((uint32_t)1 << 15))
#define CPM_PWRSR_VCC_HIGH_POWER_READY_FLAG (((uint32_t)1 << 3))
#define CPM_PWRSR_CARD0_READY_FLAG          (((uint32_t)1 << 2))

/*eport sleep control register:EPORTSLPCFGR 0x0080 ~ 0x0083*/
/*eport clock gate control register:EPORTCGTR 0x0084 ~ 0x0087*/
/*eport reset control register:EPORTRSTCR 0x0088 ~ 0x008B*/

/*rtc trim register:RTCTRIMR 0x008C~ 0x008F*/
#define CPM_RTCTRIMR_RTC_TRIM_EN            (((uint32_t)1 << 31))
#define CPM_RTCTRIMR_RTC_TRIM_LOAD_EN       (((uint32_t)1 << 30))

/*pad wakeup interrupt control register:PADWKINTCR 0x0090~ 0x0093*/
#define CPM_PADWKINTCR_RIM_ARST_RT                      (((uint32_t)1 << 31))
#define CPM_PADWKINTCR_DBG_PWRUP_RT                     (((uint32_t)1 << 30))
#define CPM_PADWKINTCR_ISORST_RT                        (((uint32_t)1 << 29))
#define CPM_PADWKINTCR_TCRST_RT                         (((uint32_t)1 << 28))
#define CPM_PADWKINTCR_PCI_ATIMER_WAKEUP_SRC_STATUS     (((uint32_t)1 << 27))
#define CPM_PADWKINTCR_PCI_DET_WAKEUP_SRC_STATUS        (((uint32_t)1 << 26))
#define CPM_PADWKINTCR_WAKE_WAKEUP_SRC_STATUS           (((uint32_t)1 << 25))
#define CPM_PADWKINTCR_USB_DET_WAKEUP_SRC_STATUS        (((uint32_t)1 << 24))
#define CPM_PADWKINTCR_RIM_ARST_EN                      (((uint32_t)1 << 23))
#define CPM_PADWKINTCR_DBG_PWRUP_EN                     (((uint32_t)1 << 22))
#define CPM_PADWKINTCR_ISORST_EN                        (((uint32_t)1 << 21))
#define CPM_PADWKINTCR_TCRST_EN                         (((uint32_t)1 << 20)
#define CPM_PADWKINTCR_PCI_ATIMER_WAKEUP_SRC_EN         (((uint32_t)1 << 19))
#define CPM_PADWKINTCR_PCI_DET_WAKEUP_SRC_EN            (((uint32_t)1 << 18))
#define CPM_PADWKINTCR_WAKE_WAKEUP_SRC_EN               (((uint32_t)1 << 17))
#define CPM_PADWKINTCR_USB_DET_WAKEUP_SRC_EN            (((uint32_t)1 << 16))
#define CPM_PADWKINTCR_RIM_ARST_INTM                    (((uint32_t)1 << 15))
#define CPM_PADWKINTCR_DBG_PWRUP_INTM                   (((uint32_t)1 << 14))
#define CPM_PADWKINTCR_ISORST_INTM                      (((uint32_t)1 << 13))
#define CPM_PADWKINTCR_TCRST_INTM                       (((uint32_t)1 << 12))
#define CPM_PADWKINTCR_PCI_ATIMER_WAKEUP_SRC_INT_EN     (((uint32_t)1 << 11))
#define CPM_PADWKINTCR_PCI_DET_WAKEUP_SRC_INT_EN        (((uint32_t)1 << 10))
#define CPM_PADWKINTCR_WAKE_WAKEUP_SRC_INT_EN           (((uint32_t)1 << 9))
#define CPM_PADWKINTCR_USB_DET_WAKEUP_SRC_INT_EN        (((uint32_t)1 << 8))
#define CPM_PADWKINTCR_RIM_ARST_STAT                    (((uint32_t)1 << 7))
#define CPM_PADWKINTCR_DBG_PWRUP_STAT                   (((uint32_t)1 << 6))
#define CPM_PADWKINTCR_ISORST_STAT                      (((uint32_t)1 << 5))
#define CPM_PADWKINTCR_TCRST_STAT                       (((uint32_t)1 << 4))
#define CPM_PADWKINTCR_PCI_ATIMER_WAKEUP_SRC_INT_FLAG   (((uint32_t)1 << 3))
#define CPM_PADWKINTCR_PCI_DET_WAKEUP_SRC_INT_FLAG      (((uint32_t)1 << 2))
#define CPM_PADWKINTCR_WAKE_WAKEUP_SRC_INT_FLAG         (((uint32_t)1 << 1))
#define CPM_PADWKINTCR_USB_DET_WAKEUP_SRC_INT_FLAG      (((uint32_t)1 << 0))

/*wakeup filter counter register:FILTCNTR 0x0094~ 0x0097*/
/*card power on counter register:CARDPOCR 0x0098~ 0x009B*/
/*rtc 32k stable time register:RTCSTIMER 0x009C~ 0x009F*/
/*mem power down sleep control register:MPDSLPCR 0x00A0 ~ 0x00A3*/
/*multiple reset control register:MULTIRSTCR 0x00AC ~ 0x00AF*/
/*system reset control register:SYSRSTCR 0x00B0~ 0x00B3*/
/*ahb3 reset control register:AHB3RSTCR 0x00B4~ 0x00B7*/
/*arith reset control register:ARITHRSTTCR 0x00B8 ~ 0x00BB*/
/*ips reset control register:IPRSTCR 0x00BC ~ 0x00BF*/

/*sleep config register 2:SLPCFGR2 0x00C0 ~ 0x00C3*/
#define CPM_SLPCFGR2_RIM_ARST_INTM_SGL                  (((uint32_t)1 << 23))
#define CPM_SLPCFGR2_DBG_PWRUP_INTM_SGL                 (((uint32_t)1 << 22))
#define CPM_SLPCFGR2_ISORST_INTM_SGL                    (((uint32_t)1 << 21))
#define CPM_SLPCFGR2_TCRST_INTM_SGL                     (((uint32_t)1 << 20))
#define CPM_SLPCFGR2_PCI_ATIMER_WAKEUP_SRC_SGL_INT_EN   (((uint32_t)1 << 19))
#define CPM_SLPCFGR2_PCI_DET_WAKEUP_SRC_SGL_INT_EN      (((uint32_t)1 << 18))
#define CPM_SLPCFGR2_WAKE_WAKEUP_SRC_SGL_INT_EN         (((uint32_t)1 << 17))
#define CPM_SLPCFGR2_USB_DET_WAKEUP_SRC_SGL_INT_EN      (((uint32_t)1 << 16))
#define CPM_SLPCFGR2_TRNG_SLPEN                         (((uint32_t)1 << 15))
#define CPM_SLPCFGR2_OTP_IDLE_WKPWAIT                   (((uint32_t)1 << 14))
#define CPM_SLPCFGR2_OTP_IDLE_SLPWAIT                   (((uint32_t)1 << 13))
#define CPM_SLPCFGR2_CACHE_IDLE_SLPWAIT                 (((uint32_t)1 << 12))
#define CPM_SLPCFGR2_CORE_F_CLK_SLEEP_EN                (((uint32_t)1 << 11))
#define CPM_SLPCFGR2_CLKOUT_CLK_SLEEP_EN                (((uint32_t)1 << 10))
#define CPM_SLPCFGR2_CPM_IPS_CLK_SLEEP_EN               (((uint32_t)1 << 9))
#define CPM_SLPCFGR2_TC_CLK_SLEEP_EN                    (((uint32_t)1 << 8))
#define CPM_SLPCFGR2_PMURTC_SLPEN                       (((uint32_t)1 << 7))
#define CPM_SLPCFGR2_CAN_IDLE_SLPWAIT                   (((uint32_t)1 << 6))

/*power down counter register:PDNCNTR 0x00D0 ~ 0x00D3*/

/*power down counter register:PONCNTR 0x00D4 ~ 0x00D7*/

/*periphal clock divider register 4:PCDIVR4 0x00D8 ~ 0x00DB*/

/*pll nfc config register 4:PLLNFCCFGR 0x00E0 ~ 0x00E3*/
#define CPM_PLLNFCCFGR_TEST_PORT_EN         (((uint32_t)1 << 26))
#define CPM_PLLNFCCFGR_TEST_EN              (((uint32_t)1 << 25))
#define CPM_PLLNFCCFGR_EN_XTAL              (((uint32_t)1 << 24))

/*pll nfc time stable register 4:PLLNFCCFGR 0x00E4 ~ 0x00E7*/

#endif /* __CPM_REG_H__ */
