/**
  **********************************************************************************
             Copyright(c) 2025 Focaltech Co. Ltd.
                      All Rights Reserved
  **********************************************************************************
  */
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// File Name    : lowpower_drv.c
// Version      : V0.1
// ~~
#include "lowpower_drv.h"
#include <soc.h>
#include <string.h>
#include <zephyr/irq.h>
#include <zephyr/linker/linker-defs.h>
#include <zephyr/sys/mem_manage.h>

void Lowpower_enter(void);

/**
 * @brief Enter system low power mode (Lowpower Mode).
 *
 * @param None
 *
 * The following configurations are performed:
 * - Configure system to enter Lowpower mode via slpcfgr register
 * - Set CPM_IPS_SLPEN
 * - Configure memory retention to prevent data loss
 * - Overwrite OSCL_STABLE, OSCH_STABLE, VCC, and CARDLDO_TRIM settings
 * - Enable wake-up digital filter and analog filter bypass
 * - Configure debounce functionality
 * - Set VCCCTMR before enabling analog filter bypass
 * - Enable wake_en and wake_intm wakeup source interrupts
 *
 * @return None
 */
void LP_LowpowerIn(void)
{
    int lock_key;

  #define SPRAM_SLP_EN (1<<28)

    DRV_DCACHE_Push(PUSHW0);
    DRV_DCACHE_Push(PUSHW1);

    uint32_t temp_value;
    lock_key = arch_irq_lock();
    temp_value = ((CPM->SLPCFGR) & (0x3FFFFFFF));

    //disable spram sleep
    temp_value &=~SPRAM_SLP_EN;

    CPM->SLPCFGR = temp_value;
    CPM->SLPCFGR2 &= ~((uint32_t)1 << 9);
    CPM->MPDSLPCR = 0x1faa000a;

    temp_value = ((CPM->VCCCTMR) & (0x3FFFFFFF));
    CPM->VCCCTMR = temp_value;
    CPM->VCCCTMR = (temp_value | 0x40000000);
    CPM->VCCCTMR = (temp_value | 0x80000000);
    CPM->VCCCTMR = (temp_value | 0xC0000000);
    CPM->VCCCTMR |= 0x01a60000;

    CPM->SLPCNTR = 0x00000080;
    CPM->WKPCNTR = 0x00000080;
    CPM->WKPFILTCNTR = 0x00000040;

    CPM->OSCLSTIMER = 0x20;
    CPM->OSCHSTIMER = 0x001001FF;

    CPM->CARDTRIMR |= ((uint32_t)1 << 30);
    CPM->CARDTRIMR |= ((uint32_t)1 << 28);

    CPM->CARDTRIMR |= ((uint32_t)1 << 15);
    CPM->CARDTRIMR &= ~((uint32_t)1 << 29);

    CPM->PADWKINTCR = 0x000202FF;
    irq_enable(14);

    SCB->SCR |= (1 << 2);
    CPM->PWRCR &= ~((uint32_t)1 << 3);
    temp_value = (CPM->SCDIVR & (0xFFFFFF00));
    CPM->SCDIVR = (temp_value | (0x13));
    CPM->CDIVUPDR |= ((uint32_t)1 << 1);
    arch_irq_unlock(lock_key);
}

/**
 * @brief Enter system low power mode (Hiber Mode).
 *
 * @param None
 *
 * The following configurations are performed:
 * - Configure system to enter Hiber mode via slpcfgr register
 * - Set CPM_IPS_SLPEN
 * - Configure memory retention to prevent data loss
 * - Overwrite OSCL_STABLE, OSCH_STABLE, VCC, and CARDLDO_TRIM settings
 * - Enable wake-up digital filter and analog filter bypass
 * - Configure debounce functionality
 * - Set VCCCTMR before enabling analog filter bypass
 * - Enable wake_en and wake_intm wakeup source interrupts
 * - Disable USBDET wake-up
 *
 * @return None
 */
void LP_HiberIn()
{
    uint32_t temp_value;
    int lock_key;

    lock_key = arch_irq_lock();
    temp_value = ((CPM->SLPCFGR) & (0x3FFFFFFF));
    CPM->SLPCFGR = (temp_value | ((uint32_t)2 << 30));
    CPM->SLPCFGR2 &= ~((uint32_t)1 << 9);
    // CPM->MPDSLPCR = 0x1faa000a;

    temp_value = ((CPM->VCCCTMR) & (0x3FFFFFFF));
    CPM->VCCCTMR = temp_value;
    CPM->VCCCTMR = (temp_value | 0x40000000);
    CPM->VCCCTMR = (temp_value | 0x80000000);
    CPM->VCCCTMR = (temp_value | 0xC0000000);
    CPM->VCCCTMR |= 0x01860000;

    CPM->SLPCNTR = 0x00000080;
    CPM->WKPCNTR = 0x00000080;
    CPM->WKPFILTCNTR = 0x00000040;

    CPM->OSCLSTIMER = 0x20;
    CPM->OSCHSTIMER = 0x001001FF;

    CPM->CARDTRIMR |= ((uint32_t)1 << 30);
    CPM->CARDTRIMR |= ((uint32_t)1 << 28);

    CPM->CARDTRIMR |= ((uint32_t)1 << 15);
    CPM->CARDTRIMR &= ~((uint32_t)1 << 29);

    CPM->PADWKINTCR = 0x000202FF;

    CPM->VCCVTRIMR &= ~((uint32_t)1 << 16);
    CPM->VCCVTRIMR |= ((uint32_t)1 << 9);

    CPM->VCCGTRIMR |= ((uint32_t)1 << 13);
    CPM->VCCGTRIMR |= ((uint32_t)1 << 12);

    irq_enable(14);
    SCB->SCR |= (1 << 2);
    CPM->PWRCR &= ~((uint32_t)1 << 3);
    temp_value = (CPM->SCDIVR & (0xFFFFFF00));
    CPM->SCDIVR = (temp_value | (0x13));
    CPM->CDIVUPDR |= ((uint32_t)1 << 1);
    arch_irq_unlock(lock_key);
}

/**
 * @brief Exit system Lowpower mode and restore normal operation.
 *
 * @param None
 *
 * The following configurations are performed:
 * - Restore system clock and configure clock dividers
 * - Enable 3.3V VCC level detection
 *
 * @return None
 */
void LP_LowpowerOut(void)
{
    uint32_t temp_value;
    int lock_key;
    lock_key = arch_irq_lock();
    // CPM->MPDSLPCR = 0x00000000;
    temp_value = (CPM->SCDIVR & (0xFFFFFF00));
    CPM->SCDIVR = (temp_value | (0x01));
    CPM->CDIVUPDR |= ((uint32_t)1 << 1);
    CPM->PWRCR |= ((uint32_t)1 << 3);
    SCB->SCR &= ~(SCB_SCR_SLEEPDEEP_Msk);
    arch_irq_unlock(lock_key);
}

/**
 * @brief Exit system Hiber mode and restore normal operation.
 *
 * @param None
 *
 * The following configurations are performed:
 * - Restore system clock and configure clock dividers
 * - Enable 3.3V VCC level detection
 *
 * @return None
 */
void LP_HiberOut(void)
{
    uint32_t temp_value;
    int lock_key;
    lock_key = arch_irq_lock();
    temp_value = (CPM->SCDIVR & (0xFFFFFF00));
    CPM->SCDIVR = (temp_value | (0x01));
    CPM->CDIVUPDR |= ((uint32_t)1 << 1);
    CPM->PWRCR |= ((uint32_t)1 << 3);
    SCB->SCR &= ~(SCB_SCR_SLEEPDEEP_Msk);
    arch_irq_unlock(lock_key);
}
