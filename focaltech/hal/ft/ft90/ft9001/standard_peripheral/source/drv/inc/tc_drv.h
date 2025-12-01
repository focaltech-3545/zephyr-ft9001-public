// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// File name    : tc_drv.h
// Version      : V0.1
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
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
  * @brief Start the hardware timer by clearing stop and low-power mode bits.
  *
  * @param tctd Pointer to the timer register structure (TC_TypeDef).
  *
  * The following bits are cleared:
  * - TC_WAIT: Timer stops in WAIT mode
  * - TC_DOZE: Timer halts in DOZE mode
  * - TC_STOP: Master stop bit for timer operation
  * - TC_DEBUG: Timer halts during debug halt
  *
  *@return NONE
  */
extern void Timer_Start(TC_TypeDef* tctd);

/**
  * @brief Stop the timer by setting the master stop bit.
  *
  * @param[in] tctd Pointer to the timer control register structure.
  *
  * @return NONE
  */
extern void Timer_Stop(TC_TypeDef* tctd);

/**
  * @brief Get the current timer count value.
  *
  * @param[in] tctd Pointer to the timer control register structure.
  *
  * @return Current count value of the timer.
  */
extern uint32_t Timer_Get_Count(TC_TypeDef* tctd);
 
/**
  * @brief Get the timer load value.
  *
  * @param[in] tctd Pointer to the timer control register structure.
  *
  * @return Current load value of the timer.
  */
extern uint32_t Timer_Get_Loadvalue(TC_TypeDef* tctd);
 
/**
  * @brief TC: Set the load value and enable auto-reload on zero.
  *
  * @param[in] tctd Pointer to the timer control register structure.
  * @param[in] value The load value to set for the timer counter.
  *
  * @return None
  */
extern void Timer_Set_Loadvalue(TC_TypeDef* tctd, uint32_t value);
 
/**
  * @brief TC: Enable interrupt on counter reaching zero.
  *
  * @param[in] tctd Pointer to the timer control register structure.
  *
  * @return None
  */
extern void Timer_Int_Enable(TC_TypeDef* tctd);

/**
  * @brief TC: Clear the timer interrupt flag.
  *
  * @param[in] tctd Pointer to the timer control register structure.
  *
  * @return None
  */
extern void Timer_IntFlag_Clear(TC_TypeDef* tctd);

/**
  * @brief TC: Disable reset function on zero count.
  *
  * @param[in] tctd Pointer to the timer control register structure.
  *
  * @return None
  */
extern void Timer_Rst_Disable(TC_TypeDef* tctd);

/**
  * @brief TC: Disable timer interrupt.
  *
  * @param[in] tctd Pointer to the timer control register structure.
  *
  * @return None
  */
extern void Timer_Int_Disable(TC_TypeDef* tctd);

/**
  * @brief TC: Set the timer prescaler value.
  *
  * @param[in] tctd Pointer to the timer control register structure.
  * @param[in] prescaler The prescaler value to set (typically 3 bits).
  *
  * @return None
  */
extern void Timer_Set_Prescaler(TC_TypeDef* tctd, uint16_t prescaler);

#endif