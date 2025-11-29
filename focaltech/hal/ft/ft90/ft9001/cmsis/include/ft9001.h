/**
  ******************************************************************************
  * @file    ft9001.h
  * @author  MCD Application Team
  * @version V2.4.2
  * @date    13-November-2024
  * @brief   CMSIS FT9001 Device Peripheral Access Layer Header File.
  *
  *          This file contains:
  *           - Data structures and the address mapping for all peripherals
  *           - Peripheral's registers declarations and bits definition
  *           - Macros to access peripheral’s registers hardware
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright(c) 2025 Focaltech Co. Ltd.
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of Focaltech nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/** @addtogroup CMSIS_Device
  * @{
  */

/** @addtogroup ft9001
  * @{
  */

#ifndef __FT9001_H
#define __FT9001_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/** @addtogroup Configuration_section_for_CMSIS
  * @{
  */

/**
  * @brief Configuration of the Cortex-M4 Processor and Core Peripherals
  */

#define __CM4_REV                 0x0001  /*!< Core revision r0p1*/


#define __MPU_PRESENT             1       /*!< ft9001 provides an MPU*/



#define __NVIC_PRIO_BITS          3       /*!< ft9001 uses 3 Bits for the Priority Levels*/
#define __NVIC_Group              2


#define __Vendor_SysTickConfig    0       /*!< Set to 1 if different SysTick Config is used*/


#define __FPU_PRESENT             1      /*!< FPU present*/


/**
  * @}
  */

/** @addtogroup Peripheral_interrupt_number_definition
  * @{
  */

/**
 * @brief FT9001 Interrupt Number Definition, according to the selected device
 *        in @ref Library_configuration_section
 */
typedef enum
{
/******  Cortex-M4 Processor Exceptions Numbers ****************************************************************/
  NonMaskableInt_IRQn         = -14,    /*!< 2 Non Maskable Interrupt                                          */
  MemoryManagement_IRQn       = -12,    /*!< 4 Cortex-M4 Memory Management Interrupt                           */
  BusFault_IRQn               = -11,    /*!< 5 Cortex-M4 Bus Fault Interrupt                                   */
  UsageFault_IRQn             = -10,    /*!< 6 Cortex-M4 Usage Fault Interrupt                                 */
  SVCall_IRQn                 = -5,     /*!< 11 Cortex-M4 SV Call Interrupt                                    */
  DebugMonitor_IRQn           = -4,     /*!< 12 Cortex-M4 Debug Monitor Interrupt                              */
  PendSV_IRQn                 = -2,     /*!< 14 Cortex-M4 Pend SV Interrupt                                    */
  SysTick_IRQn                = -1,     /*!< 15 Cortex-M4 System Tick Interrupt                                */
/******  ft9001 specific Interrupt Numbers **********************************************************************/
  OTP_IRQn                    = 0,      /*!< EFM Interrupt                                         */
  PMU_IRQn                    = 1,      /*!< PMU Interrupt                         */
  TC_IRQn                     = 2,      /*!< TC Interrupt             */
  PIT1_IRQn                   = 3,      /*!< PIT1 interrupt                        */
  PIT2_IRQn                   = 4,      /*!< PIT2 Interrupt                                            */
  EDMA0_IRQn                  = 5,      /*!< EDMA0 Interrupt                                              */
  EDMA1_IRQn                  = 6,      /*!< EDMA1 Interrupt                                              */
  DMA1_IRQn                   = 7,      /*!< DMA Interrupt                                              */
  DMA2_IRQn                   = 8,      /*!< DMA2 Interrupt                                              */
  TRNG_IRQn                   = 10,     /*!< TRNG Interrupt                                              */
  SEC_PGD_LD_FD_IRQn          = 11,     /*!< SEC_DET/PGD/LD/FD Interrupt                                    */
  PCI_DET_IRQn                = 12,     /*!< PCI_DET Interrupt                                    */
  ASYNC_TIMER_IRQn            = 13,     /*!< ASYNC TIMER Interrupt                                    */
  PCI_IRQn                    = 14,     /*!< PCI Interrupt                                    */
  PMU_RTC_IRQn                = 15,     /*!< PMU RTC Interrupt                                    */
  RSA_IRQn                    = 16,     /*!< RSA Interrupt                                    */
  SHA_IRQn                    = 17,     /*!< SHA Interrupt                                    */
  AES_IRQn                    = 18,     /*!< AES Interrupts                             */
  SMS4_IRQn                   = 19,     /*!< SMS4 Interrupt                                                 */
  QADC_IRQn                   = 20,     /*!< QADC Interrupt                                                */
  DAC_IRQn                    = 21,     /*!< DAC Interrupt                                                */
  MCC_IRQn                    = 22,     /*!< MCC Interrupt                                                */
  USBC_IRQn                   = 24,     /*!< USBC interrupt                    */
  SPI1_IRQn                   = 26,     /*!< SPI1 interrupt */
  SPI2_IRQn                   = 27,     /*!< SPI2 Interrupt                                    */
  SPI3_IRQn                   = 28,     /*!< SPI3 Interrupt                                             */
  SPIM1_IRQn                  = 29,     /*!< SPIM1 Interrupt                                             */
  SPIM2_IRQn                  = 30,     /*!< SPIM2 global Interrupt                                             */
  SPIM3_EV_IRQn               = 31,     /*!< SPIM3 Event Interrupt                                              */
  UART1_IRQn                  = 32,     /*!< SCI1 Interrupt                                              */
  UART2_IRQn                  = 33,     /*!< SCI2 Interrupt                                              */
  USI2_IRQn                   = 34,     /*!< USI2 Error Interrupt                                              */
  USI3_IRQn                   = 35,     /*!< USI2 global Interrupt                                             */
  I2C1_IRQn                   = 36,     /*!< I2C global Interrupt                                             */
  PWM0_IRQn                   = 37,     /*!< PWM0 Interrupt                                           */
  PWM1_IRQn                   = 38,     /*!< PWM1 Interrupt                                           */
  PWM2_IRQn                   = 39,     /*!< PWM2 Interrupt                                           */
  PWM3_IRQn                   = 40,     /*!< PWM3 Interrupts                                   */
  EPORT0_0_IRQn               = 41,     /*!< EPORT0_0 Interrupt                   */
  EPORT0_1_IRQn               = 42,     /*!< EPORT0_1 interrupt                     */
  EPORT0_2_IRQn               = 43,     /*!< EPORT0_2 interrupt                   */
  EPORT0_3_IRQn               = 44,     /*!< EPORT0_3 interrupt                  */
  EPORT0_4_IRQn               = 45,     /*!< EPORT0_4 interrupt */
  EPORT0_5_IRQn               = 46,     /*!< EPORT0_5 Interrupt                                    */
  EPORT0_6_IRQn               = 47,     /*!< EPORT0_6 Interrupt                                            */
  EPORT0_7_IRQn               = 48,     /*!< EPORT0_7 Interrupt                                              */
  EPORT1_0_IRQn               = 49,     /*!< EPORT1_0 Interrupt                                             */
  EPORT1_1_IRQn               = 50,     /*!< EPORT1_1 Interrupt                                             */
  EPORT1_2_IRQn               = 51,     /*!< EPORT1_2 Interrupt                                             */
  EPORT1_3_IRQn               = 52,     /*!< EPORT1_3 Interrupt                                            */
  EPORT1_4_IRQn               = 53,     /*!< EPORT1_4 Interrupt                                            */
  EPORT1_5_IRQn               = 54,     /*!< EPORT1_5 interrupts                 */
  EPORT1_6_IRQn               = 55,     /*!< EPORT1_6 interrupt                                             */
  EPORT1_7_IRQn               = 56,     /*!< EPORT1_7 Interrupt                                    */
  SSISLV1_IRQn                = 57,     /*!< SSISLV1 Interrupt                                    */
  I2C2_IRQn                   = 59,     /*!< I2C2 Interrupt                                    */
  UART3_IRQn                   = 61,     /*!< SCI3 Interrupt                                    */
  USI1_IRQn                   = 63,     /*!< USI1 Interrupt                                    */
  CAN1_BUF00_03_IRQn          = 64,     /*!< CAN1_BUF00_03 Interrupt                                    */
  CAN1_BUF04_07_IRQn          = 65,
  CAN1_BUF08_11_IRQn          = 66,
  CAN1_BUF12_15_IRQn          = 67,
  CAN1_BUF16_31_IRQn          = 68,
  CAN1_BUF32_63_IRQn          = 69,
  BOFF_TRWARN_IRQn            = 70,
  CAN1_ERR_IRQn               = 71,
  CAN2_BUF00_03_IRQn          = 72,
  CAN2_BUF04_07_IRQn          = 73,
  CAN2_BUF08_11_IRQn          = 74,
  CAN2_BUF12_15_IRQn          = 75,
  CAN2_BUF16_31_IRQn          = 76,
  CAN2_BUF32_63_IRQn          = 77,
  BOFF2_TRWARN_IRQn           = 78,
  CAN2_ERR_IRQn               = 79,
  I2S1_IRQn                   = 80,
  I2S2_IRQn                   = 81,
  ACMP1_IRQn                  = 82,
  ACMP2_IRQn                  = 83,
  PWMT1_IRQn                  = 84,
  PWMT2_IRQn                  = 85,
  PWMT3_IRQn                  = 86,
  PWM4_IRQn                   = 87,
  PWM5_IRQn                   = 88,
  PWM6_IRQn                   = 89,
  PWM7_IRQn                   = 90,
  EPORT2_0_IRQn                  = 92,
  EPORT2_1_IRQn                  = 93,
  EPORT2_2_IRQn                  = 94,
  EPORT2_3_IRQn                  = 95,
  EPORT2_4_IRQn                  = 96,
  EPORT2_5_IRQn                  = 97,
  EPORT2_6_IRQn                  = 98,
  EPORT2_7_IRQn                  = 99,
  EPORT3_0_IRQn                  = 100,
  EPORT3_1_IRQn                  = 101,
  EPORT3_2_IRQn                  = 102,
  EPORT3_3_IRQn                  = 103,
  EPORT3_4_IRQn                  = 104,
  EPORT3_5_IRQn                  = 105,
  EPORT3_6_IRQn                  = 106,
  EPORT3_7_IRQn                  = 107,
  EPORT4_0_IRQn                  = 108,
  EPORT4_1_IRQn                  = 109,
  EPORT4_2_IRQn                  = 110,
  EPORT4_3_IRQn                  = 111,
  EPORT4_4_IRQn                  = 112,
  EPORT4_5_IRQn                  = 113,
  EPORT4_6_IRQn                  = 114,
  EPORT4_7_IRQn                  = 115,
  EPORT5_0_IRQn                  = 116,
  EPORT5_1_IRQn                  = 117,
  EPORT5_2_IRQn                  = 118,
  EPORT5_3_IRQn                  = 119,
  EPORT5_4_IRQn                  = 120,
  EPORT5_5_IRQn                  = 121,
  EPORT5_6_IRQn                  = 122,
  EPORT5_7_IRQn                  = 123,
  EPORT6_0_IRQn                  = 124,
  EPORT6_1_IRQn                  = 125,
  EPORT6_2_IRQn                  = 126,
  EPORT6_3_IRQn                  = 127,
  EPORT6_4_IRQn                  = 128,
  EPORT6_5_IRQn                  = 129,
  EPORT6_6_IRQn                  = 130,
  EPORT6_7_IRQn                  = 131,
  EPORT7_0_IRQn                  = 132,
  EPORT7_1_IRQn                  = 133,
  EPORT7_2_IRQn                  = 134,
  EPORT7_3_IRQn                  = 135,
  EPORT7_4_IRQn                  = 136,
  EPORT7_5_IRQn                  = 137,
  EPORT7_6_IRQn                  = 138,
  EPORT7_7_IRQn                  = 139,

} IRQn_Type;

/**
  * @}
  */

#include "core_cm4.h"             /* Cortex-M4 processor and core peripherals */
//#include "system_FT9001.h"
#include <stdint.h>


/**
  * @brief Debug MCU
  */

typedef struct
{
  __IO uint32_t IDCODE;  /*!< MCU device ID code,               Address offset: 0x00 */
  __IO uint32_t CR;      /*!< Debug MCU configuration register, Address offset: 0x04 */
  __IO uint32_t APB1FZ;  /*!< Debug MCU APB1 freeze register,   Address offset: 0x08 */
  __IO uint32_t APB2FZ;  /*!< Debug MCU APB2 freeze register,   Address offset: 0x0C */
}DBGMCU_TypeDef;

/**
  * @}
  */

/** @addtogroup Peripheral_memory_map
  * @{
  */
#define ROM_BASE              ((uint32_t)0x00000000) /*!< ROM(64K Bytes) base address in the alias region*/
#define FLASH_BASE            ((uint32_t)0x10000000) /*!< FLASH(2 MB) base address in the alias region*/
//#define SYS_SRAM_BASE         ((uint32_t)0x10000000) /*!< SYSTEM SRAM base address in the alias region*/
#define IN_SRAM_BASE          ((uint32_t)0x20000000) /*!< Internal SRAM in the alias region*/
#define SDRAM_BASE            ((uint32_t)0x60000000) /*!< SDRAM base address in the alias region*/
#define REG_BASE              ((uint32_t)0x40000000) /*!< peripherals registers base address*/
/**
  * @}
  */
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __FT9001_H */



/************************ (C) COPYRIGHT Focaltech *****END OF FILE****/
