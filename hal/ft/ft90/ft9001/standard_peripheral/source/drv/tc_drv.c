/**
  **********************************************************************************
             Copyright(c) 2025 Focaltech Co. Ltd.
                      All Rights Reserved
  **********************************************************************************
  */
#include "tc_drv.h"

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
void Timer_SetMode(TC_TypeDef *tctd, uint8_t mode)
{
    switch (mode)
    {
    case 0:
        tctd->TCCR &= ~(TC_WAIT | TC_DOZE | TC_STOP | TC_DBG);
        break;
    case 1:
        tctd->TCCR &= ~(TC_WAIT | TC_DOZE | TC_STOP);
        tctd->TCCR |= TC_DBG;
        break;
    case 2:
        tctd->TCCR &= ~(TC_WAIT | TC_DOZE | TC_DBG);
        tctd->TCCR |= TC_STOP;
        break;
    case 3:
        tctd->TCCR &= ~(TC_WAIT | TC_DBG | TC_STOP);
        tctd->TCCR |= TC_DOZE;
        break;
    case 4:
        tctd->TCCR &= ~(TC_DOZE | TC_DBG | TC_STOP);
        tctd->TCCR |= TC_WAIT;
        break;
    default:
        tctd->TCCR &= ~(TC_WAIT | TC_DOZE | TC_STOP | TC_DBG);
        break;
    }
}

/**
 * @brief Get the current timer count value.
 *
 * @param[in] tctd Pointer to the timer control register structure.
 *
 * @return Current count value of the timer.
 */
uint32_t Timer_GetCount(TC_TypeDef *tctd)
{
    uint16_t tmp_data1;
    uint16_t tmp_data2;
    uint16_t tmp_data3;
    uint32_t data;

    tmp_data1 = tctd->TCCNTR;
    tmp_data2 = tctd->TCCNTR;
    tmp_data3 = tctd->TCCNTR;

    if ((tmp_data1 == tmp_data2) && (tmp_data1 == tmp_data3))
    {
        data = (uint32_t)tmp_data1;
    }
    else
    {
        tmp_data1 = tctd->TCCNTR;
        tmp_data1 = tctd->TCCNTR;
        data = (uint32_t)tmp_data1;
    }
    return data;
}

/**
 * @brief Get the timer load value.
 *
 * @param[in] tctd Pointer to the timer control register structure.
 *
 * @return Current load value of the timer.
 */
uint32_t Timer_GetLoadvalue(TC_TypeDef *tctd)
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
void Timer_SetLoadvalue(TC_TypeDef *tctd, uint32_t value)
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
void Timer_IntEnable(TC_TypeDef *tctd)
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
void Timer_IntFlagClear(TC_TypeDef *tctd)
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
void Timer_RstDisable(TC_TypeDef *tctd)
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
void Timer_IntDisable(TC_TypeDef *tctd)
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
void Timer_SetPrescaler(TC_TypeDef *tctd, uint16_t prescaler)
{
    tctd->TCCR &= 0xFF8F;
    tctd->TCCR |= (uint16_t)(prescaler << 4);
}

/**
 * @brief TC: Set the timer alarm value (relative expiration).
 *
 *
 * @param[in] tctd   tctd Pointer to the timer control register structure.
 * @param[in] value  Alarm value .
 *
 * @return None
 */
void Timer_SetAlarmValue(TC_TypeDef *tctd, uint32_t value)
{
    tctd->TCMR = (uint16_t)value;
    tctd->TCCR |= TC_CU;
}
