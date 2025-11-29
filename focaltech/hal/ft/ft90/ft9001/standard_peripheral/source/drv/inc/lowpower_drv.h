/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef    __LOWPOWER_DRV_H__
#define    __LOWPOWER_DRV_H__



/* Includes ------------------------------------------------------------------*/
#include "memmap.h"
#include "cpm_drv.h"


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
extern void LP_LowpowerIn(void);

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
extern void LP_HiberIn(void);

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
extern void LP_LowpowerOut(void);

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
extern void LP_HiberOut(void);


#endif