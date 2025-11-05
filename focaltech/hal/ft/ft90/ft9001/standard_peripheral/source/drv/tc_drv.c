#include "tc_drv.h"

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
void Timer_Start(TC_TypeDef *tctd)
{
    tctd->TCCR &= ~(TC_WAIT | TC_DOZE | TC_STOP | TC_DBG);
}

/**
  * @brief Stop the timer by setting the master stop bit.
  *
  * @param[in] tctd Pointer to the timer control register structure.
  * 
  * @return NONE
  */
void Timer_Stop(TC_TypeDef *tctd)
{
    tctd->TCCR |= TC_STOP;
}

/**
  * @brief Get the current timer count value.
  *
  * @param[in] tctd Pointer to the timer control register structure.
  * 
  * @return Current count value of the timer.
  */
uint32_t Timer_Get_Count(TC_TypeDef *tctd)
{
    return tctd->TCCNTR;
}

/**
  * @brief Get the timer load value.
  *
  * @param[in] tctd Pointer to the timer control register structure.
  * 
  * @return Current load value of the timer.
  */
uint32_t Timer_Get_Loadvalue(TC_TypeDef *tctd)
{
    return tctd->TCMR;
}

/**
  * @brief TC: Set the load value and enable auto-reload on zero.
  *
  * @param[in] tctd Pointer to the timer control register structure.
  * @param[in] value The load value to set for the timer counter.
  * 
  * @return None
  */
void Timer_Set_Loadvalue(TC_TypeDef *tctd, uint32_t value)
{
    tctd->TCMR = (uint16_t)value;
    tctd->TCCR |= TC_CU;
}

/**
  * @brief TC: Enable interrupt on counter reaching zero.
  *
  * @param[in] tctd Pointer to the timer control register structure.
  * 
  * @return None
  */
void Timer_Int_Enable(TC_TypeDef *tctd)
{
    tctd->TCCR |= TC_IE;
}

/**
  * @brief TC: Clear the timer interrupt flag.
  *
  * @param[in] tctd Pointer to the timer control register structure.
  * 
  * @return None
  */
void Timer_IntFlag_Clear(TC_TypeDef *tctd)
{
    tctd->TCCR |= TC_IF;
}

/**
  * @brief TC: Disable reset function on zero count.
  *
  * @param[in] tctd Pointer to the timer control register structure.
  * 
  * @return None
  */
void Timer_Rst_Disable(TC_TypeDef* tctd)
{
    tctd->TCCR &= ~TC_RN;
}

/**
  * @brief TC: Disable timer interrupt.
  *
  * @param[in] tctd Pointer to the timer control register structure.
  * 
  * @return None
  */
void Timer_Int_Disable(TC_TypeDef* tctd) 
{
    tctd->TCCR &= ~TC_IE;
}

/**
  * @brief TC: Set the timer prescaler value.
  *
  * @param[in] tctd Pointer to the timer control register structure.
  * @param[in] prescaler The prescaler value to set (typically 3 bits).
  * 
  * @return None
  */
void Timer_Set_Prescaler(TC_TypeDef* tctd, uint16_t prescaler)
{
    tctd->TCCR &= ~(0x7 << 4);
    tctd->TCCR |= (prescaler << 4);
}
