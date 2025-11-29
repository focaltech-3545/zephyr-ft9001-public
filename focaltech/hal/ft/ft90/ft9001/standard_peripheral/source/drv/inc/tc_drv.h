/**  
  **********************************************************************************
             Copyright(c) 2025 Focaltech Co. Ltd.
                      All Rights Reserved
  **********************************************************************************
  */

#ifndef TC_DRV_H_
#define TC_DRV_H_

#include "memmap.h"
#include "tc_reg.h"

typedef enum
{
    TC_WDP_2048 = 0x00,
    TC_WDP_1024 = 0x01,
    TC_WDP_512 = 0x02,
    TC_WDP_256 = 0x03,
    TC_WDP_128 = 0x04,
    TC_WDP_64 = 0x05,
    TC_WDP_32 = 0x06,
    TC_WDP_16 = 0x07
}TC_TIMER_PRESCALER;

/**
   * @brief Setting the hardware timer with specified mode configuration.
   *
   * @param tctd Pointer to the timer register structure (TC_TypeDef).
   * @param mode Timer operation mode:
   *             - 0: Normal mode (Timer runs in normal mode)
   *             - 1: Debug mode (Timer runs in debug mode)
   *             - 2: Stop mode (Timer runs in stop mode)
   *             - 3: Doze mode (Timer runs in doze mode)
   *             - 4: Wait mode (Timer runs in wait mode)
   *             - Other: Default to Normal mode
   *
   * Bit definitions:
   * - TC_WAIT:  Timer behavior in WAIT mode
   * - TC_DOZE:  Timer behavior in DOZE mode
   * - TC_STOP:  Timer behavior in STOP mode
   * - TC_DBG:   Timer behavior in DBG  mode
   *
   * @return None
   */
extern void Timer_SetMode(TC_TypeDef* tctd, uint8_t mode);


/**
  * @brief Get the current timer count value.
  *
  * @param[in] tctd Pointer to the timer control register structure.
  *
  * @return Current count value of the timer.
  */
extern uint32_t Timer_GetCount(TC_TypeDef* tctd);
 
/**
  * @brief Get the timer load value.
  *
  * @param[in] tctd Pointer to the timer control register structure.
  *
  * @return Current load value of the timer.
  */
extern uint32_t Timer_GetLoadvalue(TC_TypeDef* tctd);
 
/**
  * @brief TC: Set the load value and enable auto-reload on zero.
  *
  * @param[in] tctd Pointer to the timer control register structure.
  * @param[in] value The load value to set for the timer counter.
  *
  * @return None
  */
extern void Timer_SetLoadvalue(TC_TypeDef* tctd, uint32_t value);
 
/**
  * @brief TC: Enable interrupt on counter reaching zero.
  *
  * @param[in] tctd Pointer to the timer control register structure.
  *
  * @return None
  */
extern void Timer_IntEnable(TC_TypeDef* tctd);

/**
  * @brief TC: Clear the timer interrupt flag.
  *
  * @param[in] tctd Pointer to the timer control register structure.
  *
  * @return None
  */
extern void Timer_IntFlagClear(TC_TypeDef* tctd);

/**
  * @brief TC: Disable reset function on zero count.
  *
  * @param[in] tctd Pointer to the timer control register structure.
  *
  * @return None
  */
extern void Timer_RstDisable(TC_TypeDef* tctd);

/**
  * @brief TC: Disable timer interrupt.
  *
  * @param[in] tctd Pointer to the timer control register structure.
  *
  * @return None
  */
extern void Timer_IntDisable(TC_TypeDef* tctd);

/**
  * @brief TC: Set the timer prescaler value.
  *
  * @param[in] tctd Pointer to the timer control register structure.
  * @param[in] prescaler The prescaler value to set (typically 3 bits).
  *
  * @return None
  */
extern void Timer_SetPrescaler(TC_TypeDef* tctd, uint16_t prescaler);

/**
  * @brief TC: Set the timer alarm value (relative expiration).
  *
  *
  * @param[in] tctd   tctd Pointer to the timer control register structure.
  * @param[in] value  Alarm value .
  *
  * @return None
  */
extern void Timer_SetAlarmValue(TC_TypeDef* tctd, uint32_t value);



#endif