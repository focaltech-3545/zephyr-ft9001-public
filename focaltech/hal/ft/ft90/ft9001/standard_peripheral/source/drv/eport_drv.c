/**
  **********************************************************************************
             Copyright(c) 2025 Focaltech Co. Ltd.
                      All Rights Reserved
  **********************************************************************************
  */

// #include "iccemv.h"
#include "eport_drv.h"
#include "cpm_drv.h"
#include "eport_reg.h"
#include "ioctrl_reg.h"
#include "usi_reg.h"

/***************************************************************************
 * Function Name  : EPORT_ConfigGpio
 * Description    : Configures EPORT as GPIO.
 * Input          : - EPORT_TypeDef: Base address of EPORTx.
 *                  - EPORT_PINx: EPORT Pin; x can be 0~7 to select the EPORT peripheral.
 *                  - GpioDir: GPIO direction setting. GPIO_OUTPUT: Output, GPIO_INPUT: Input.
 * Output         : None
 * Return         : None
 ***************************************************************************/
uint32_t EPORT_ConfigGpio(EPORT_TypeDef *Eport, EPORT_PINx GpioNum, uint8_t GpioDir)
{
    uint32_t GpioNo;
    GpioNo = GPIO_NO(Eport, GpioNum);
    // Configure non-GINT default pins after reset
    if (GpioNo == EPORT_PIN6)
    {
        IOCTRL->SWAPCR3 |= BITS(0);
    }
    else if (GpioNo == EPORT_PIN7)
    {
        IOCTRL->SWAPCR3 |= BITS(1);
    }
    else if (GpioNo >= EPORT_PIN8 && GpioNo <= EPORT_PIN11)
    {
        IOCTRL->SWAPCR |= BITS(0);
    }
    else if (GpioNo >= EPORT_PIN14 && GpioNo <= EPORT_PIN15)
    {
        IOCTRL->SWAPCR &= ~BITS(1);
        IOCTRL->SWAPCR &= ~BITS(4 + GpioNo);
        IOCTRL->SWAPCR3 &= ~BITS(GpioNo - 12);
    }
    else if (GpioNo == EPORT_PIN22)
    { // for 16-21,default function:spi nor flash
        IOCTRL->SWAPCR4 |= BITS(20);
    }
    else if (GpioNo == EPORT_PIN23)
    {
        IOCTRL->SWAPCR4 |= BITS(21);
    }
    else if (GpioNo >= EPORT_PIN38 && GpioNo <= EPORT_PIN39)
    { // for 32-37,default function:psram
        IOCTRL->SWAPCR5 &= ~BITS(GpioNo - 28);
    }
    else if (GpioNo >= EPORT_PIN40 && GpioNo <= EPORT_PIN63)
    {
        if (GpioNo >= EPORT_PIN40 && GpioNo <= EPORT_PIN43)
        {
            IOCTRL->SWAPCR5 &= ~BITS(55 - GpioNo);
        }

        else if (GpioNo >= EPORT_PIN58 && GpioNo <= EPORT_PIN60)
        {
            CPM->PWRCR &= ~BITS(25);
            CPM->SLPCFGR = ((((CPM->SLPCFGR) & 0xFFFFFCFF) | (0x00000300)));
            CPM->SLPCFGR &= ~BITS(11);
            USI2->PCR &= 0x3F;
            USI2->PCR |= 0xC0;
        }
        IOCTRL->SWAPCR2 |= BITS(GpioNo - 32);
    }

    // Configure input/output mode
    if (GpioDir == EPORT_INPUT)
    {
        Eport->EPDDR &= ~(BITS(GpioNum));
    }
    else
    {
        Eport->EPDDR |= BITS(GpioNum);
    }
    return 0x0;
}

/***************************************************************************
 * Function Name  : EPORT_WriteGpioDatas
 * Description    : Sets the EPORT pin level.
 * Input          : - EPORT_TypeDef: Base address of EPORTx.
 *                  - Vals: Value to set, only lower 8 bits are effective.
 * Output         : None
 ***************************************************************************/
void EPORT_WriteGpioDatas(EPORT_TypeDef *Eport, uint8_t Vals)
{
    Eport->EPDR = Vals;
}

/***************************************************************************
 * Function Name  : EPORT_ReadGpioDatas
 * Description    : Reads the value from the EPORT port.
 * Input          : - EPORT_TypeDef: Base address of EPORTx.
 * Output         : None
 * Return         : EPORTx value
 ***************************************************************************/
uint8_t EPORT_ReadGpioDatas(EPORT_TypeDef *Eport)
{
    uint8_t get_value = Eport->EPPDR;
    return get_value;
}

/***************************************************************************
 * Function Name  : EPORT_ITTypeConfig
 * Description    : Configures the interrupt trigger mode and enables the interrupt.
 * Input          : - EPORT_TypeDef: Base address of EPORTx.
 *                  - EPORT_PINx: EPORT Pin; x can be 0~7 to select the EPORT peripheral.
 *                  - IntMode: Interrupt trigger mode.
 *                      LOW_LEVEL_INT: Low-level trigger
 *                      HIGH_LEVEL_INT: High-level trigger
 *                      RISING_EDGE_INT: Rising edge trigger
 *                      FALLING_EDGE_INT: Falling edge trigger
 *                      RISING_FALLING_EDGE_INT: Both rising and falling edge trigger
 * Output         : None
 * Return         : None
 ***************************************************************************/
void EPORT_ITTypeConfig(EPORT_TypeDef *Eport, EPORT_PINx GpioNum, EPORT_INT_MODE IntMode)
{
    // Disable interrupt
    Eport->EPIER &= ~BITS(GpioNum);

    // Set as input
    EPORT_ConfigGpio(Eport, GpioNum, EPORT_INPUT);

    switch (IntMode)
    {
    case LOW_LEVEL_INT:
        Eport->EPPAR &= ~(BITS(GpioNum * 2) | BITS(GpioNum * 2 + 1));
        Eport->EPLPR &= ~BITS(GpioNum);
        break;
    case HIGH_LEVEL_INT:
        Eport->EPPAR &= ~(BITS(GpioNum * 2) | BITS(GpioNum * 2 + 1));
        Eport->EPLPR |= BITS(GpioNum);
        break;
    case RISING_EDGE_INT:
        Eport->EPPAR &= ~(BITS(GpioNum * 2) | BITS(GpioNum * 2 + 1));
        Eport->EPPAR |= BITS(GpioNum * 2);
        break;
    case FALLING_EDGE_INT:
        Eport->EPPAR &= ~(BITS(GpioNum * 2) | BITS(GpioNum * 2 + 1));
        Eport->EPPAR |= BITS(GpioNum * 2 + 1);
        break;
    case RISING_FALLING_EDGE_INT:
        Eport->EPPAR |= (0x0003 << GpioNum * 2);
        break;
    default:
        break;
    }
    // Enable interrupt
    Eport->EPFR = BITS(GpioNum);
    Eport->EPIER |= BITS(GpioNum);
}

/*******************************************************************************
 * Function Name  : EPORT_ITConfig
 * Description    : Enables or disables EPORT interrupts.
 * Input          : - EPORT_TypeDef: Base address of EPORTx.
 *                  - EPORT_PINx: EPORT Pin; x can be 0~7 to select the EPORT peripheral.
 *                  - NewState: New state of the specified EPORT interrupts.
 *                              This parameter can be: ENABLE or DISABLE.
 *
 * Output         : None
 * Return         : None
 *******************************************************************************/
void EPORT_ITConfig(EPORT_TypeDef *Eport, EPORT_PINx GpioNum, FunctionalState NewState)
{
    if (NewState != DISABLE)
    {
        Eport->EPIER |= BITS(GpioNum); // Enable interrupt
    }
    else
    {
        Eport->EPIER &= ~(BITS(GpioNum)); // Disable interrupt
    }
}

/*******************************************************************************
 * Function Name  : EPORT_PullupConfig
 * Description    : Enables or disables EPORT pull-up resistor.
 * Input          : - EPORT_TypeDef: Base address of EPORTx.
 *                  - EPORT_PINx: EPORT Pin; x can be 0~7 to select the EPORT peripheral.
 *                  - NewState: New state of the specified EPORT pull-up.
 *                              This parameter can be: ENABLE or DISABLE.
 *
 * Output         : None
 * Return         : None
 *******************************************************************************/
void EPORT_PullupConfig(EPORT_TypeDef *Eport, EPORT_PINx GpioNum, FunctionalState NewState)
{
    if (NewState != DISABLE)
    {
        Eport->EPPUER |= BITS(GpioNum);
    }
    else
    {
        Eport->EPPUER &= ~(BITS(GpioNum));
    }
}

/**
 * Configure pull-up or pull-down resistor for EPORT pins.
 *
 * 1. Enable pull-up
 * 2. Enable pull-down (EPORT_PIN30~EPORT_PIN39 do not support pull-down)
 * 3. Disable pull-up or pull-down
 */
uint8_t EPORT_PullConfig(EPORT_TypeDef *Eport, const EPORT_PINx GpioNum, const EPORT_PULL_MODE eport_pull_mode)
{
    uint32_t GpioNo;
    // Calculate the pin number
    GpioNo = GPIO_NO(Eport, GpioNum);
    // Handle pull-down configuration
    if (eport_pull_mode == EPORT_PULLDOWN)
    {
        Eport->EPPUER |= BITS(GpioNum);

        if (GpioNo <= EPORT_PIN5) // Handle GINT 0~5
        {
            IOCTRL->GINTLCR &= ~BITS(24 + GpioNum);
        }
        else if (GpioNo >= EPORT_PIN6 && GpioNo <= EPORT_PIN7)
        {
            IOCTRL->USICR &= ~BITS(15 - GpioNo);
            USI1->PCR &= ~BITS(7 - GpioNo);
        }
        else if (GpioNo >= EPORT_PIN8 && GpioNo <= EPORT_PIN9)
        {
            IOCTRL->SPICR &= ~BITS(27 - GpioNum);
        }
        else if (GpioNo >= EPORT_PIN10 && GpioNo <= EPORT_PIN11)
        {
            IOCTRL->SPICR &= ~BITS(22 + GpioNum);
        }
        else if (GpioNo >= EPORT_PIN12 && GpioNo <= EPORT_PIN15) // Handle GINT12~15
        {
            IOCTRL->GINTHCR &= ~BITS(24 + GpioNum);
        }
        else if (GpioNo >= EPORT_PIN16 && GpioNo <= EPORT_PIN23) // Handle GINT16~23
        {
            IOCTRL->EPORT2CR &= ~BITS(24 + GpioNum);
        }
        else if (GpioNo >= EPORT_PIN24 && GpioNo <= EPORT_PIN31) // Handle GINT24~31
        {
            IOCTRL->EPORT3CR &= ~BITS(24 + GpioNum);
        }
        else if (GpioNo >= EPORT_PIN32 && GpioNo <= EPORT_PIN39) // Handle GINT32~39
        {
            IOCTRL->EPORT4CR &= ~BITS(24 + GpioNum);
        }
        else if (GpioNo >= EPORT_PIN40 && GpioNo <= EPORT_PIN47) // Handle GINT40~47
        {
            if (GpioNo >= EPORT_PIN40 && GpioNo <= EPORT_PIN41)
            {
                IOCTRL->SPICR &= ~BITS(11 - GpioNum);
            }
            else if (GpioNo >= EPORT_PIN42 && GpioNo <= EPORT_PIN43)
            {
                IOCTRL->SPICR &= ~BITS(6 + GpioNum);
            }
            else if (GpioNo >= EPORT_PIN44 && GpioNo <= EPORT_PIN45)
            {
                IOCTRL->SPICR &= ~BITS(23 - GpioNum);
            }
            else
            {
                IOCTRL->SPICR &= ~BITS(10 + GpioNum);
            }
            IOCTRL->EPORT5CR &= ~BITS(24 + GpioNum);
        }
        else if (GpioNo >= EPORT_PIN48 && GpioNo <= EPORT_PIN55) // Handle GINT48~55
        {
            if (GpioNo >= EPORT_PIN48 && GpioNo <= EPORT_PIN49)
            {
                IOCTRL->I2CCR &= ~BITS(16 + GpioNum);
            }
            else if (GpioNo >= EPORT_PIN50 && GpioNo <= EPORT_PIN51)
            {
                IOCTRL->I2CCR &= ~BITS(6 + GpioNum);
            }
            else if (GpioNo >= EPORT_PIN52 && GpioNo <= EPORT_PIN53)
            {
                IOCTRL->UARTCR &= ~BITS(4 + GpioNum);
            }
            else
            {
                IOCTRL->UARTCR &= ~BITS(10 + GpioNum);
            }
            IOCTRL->EPORT6CR &= ~BITS(24 + GpioNum);
        }
        else if (GpioNo >= EPORT_PIN56 && GpioNo <= EPORT_PIN63) // Handle GINT56~63
        {
            if (GpioNo == EPORT_PIN58)
            {
                IOCTRL->USICR &= ~BITS(17);
            }
            else if (GpioNo == EPORT_PIN59)
            {
                IOCTRL->USICR &= ~BITS(18);
            }
            else if (GpioNo == EPORT_PIN60)
            {
                IOCTRL->USICR &= ~BITS(16);
            }
            else if (GpioNo == EPORT_PIN61)
            {
                IOCTRL->USICR &= ~BITS(10);
            }
            IOCTRL->EPORT7CR &= ~BITS(24 + GpioNum);
        }
    }
    // Handle pull-up configuration
    else if (eport_pull_mode == EPORT_PULLUP)
    {
        Eport->EPPUER |= (1u << GpioNum);
        if (GpioNo <= EPORT_PIN5) // Handle GINT 0~5
        {
            IOCTRL->GINTLCR |= BITS(24 + GpioNum);
        }
        else if (GpioNo >= EPORT_PIN6 && GpioNo <= EPORT_PIN7)
        {
            IOCTRL->USICR |= BITS(15 - GpioNo);
            USI1->PCR |= BITS(7 - GpioNo);
        }
        else if (GpioNo >= EPORT_PIN8 && GpioNo <= EPORT_PIN9)
        {
            IOCTRL->SPICR |= BITS(27 - GpioNum);
        }
        else if (GpioNo >= EPORT_PIN10 && GpioNo <= EPORT_PIN11)
        {
            IOCTRL->SPICR |= BITS(22 + GpioNum);
        }
        else if (GpioNo >= EPORT_PIN12 && GpioNo <= EPORT_PIN15) // Handle GINT12~15
        {
            IOCTRL->GINTHCR |= BITS(24 + GpioNum);
        }
        else if (GpioNo >= EPORT_PIN16 && GpioNo <= EPORT_PIN23) // Handle GINT16~23
        {
            IOCTRL->EPORT2CR |= BITS(24 + GpioNum);
        }
        else if (GpioNo >= EPORT_PIN24 && GpioNo <= EPORT_PIN31) // Handle GINT24~31
        {
            IOCTRL->EPORT3CR |= BITS(24 + GpioNum);
        }
        else if (GpioNo >= EPORT_PIN32 && GpioNo <= EPORT_PIN39) // Handle GINT32~39
        {
            IOCTRL->EPORT4CR |= BITS(24 + GpioNum);
        }
        else if (GpioNo >= EPORT_PIN40 && GpioNo <= EPORT_PIN47) // Handle GINT40~47
        {

            if (GpioNo >= EPORT_PIN40 && GpioNo <= EPORT_PIN41)
            {
                IOCTRL->SPICR |= BITS(11 - GpioNum);
            }
            else if (GpioNo >= EPORT_PIN42 && GpioNo <= EPORT_PIN43)
            {
                IOCTRL->SPICR |= BITS(6 + GpioNum);
            }
            else if (GpioNo >= EPORT_PIN44 && GpioNo <= EPORT_PIN45)
            {
                IOCTRL->SPICR |= BITS(23 - GpioNum);
            }
            else
            {
                IOCTRL->SPICR |= BITS(10 + GpioNum);
            }
            IOCTRL->EPORT5CR |= BITS(24 + GpioNum);
        }
        else if (GpioNo >= EPORT_PIN48 && GpioNo <= EPORT_PIN55) // Handle GINT48~55
        {
            if (GpioNo >= EPORT_PIN48 && GpioNo <= EPORT_PIN49)
            {
                IOCTRL->I2CCR |= BITS(16 + GpioNum);
            }
            else if (GpioNo >= EPORT_PIN50 && GpioNo <= EPORT_PIN51)
            {
                IOCTRL->I2CCR |= BITS(6 + GpioNum);
            }
            else if (GpioNo >= EPORT_PIN52 && GpioNo <= EPORT_PIN53)
            {
                IOCTRL->UARTCR |= BITS(4 + GpioNum);
            }
            else
            {
                IOCTRL->UARTCR |= BITS(10 + GpioNum);
            }
            IOCTRL->EPORT6CR |= BITS(24 + GpioNum);
        }
        else if (GpioNo >= EPORT_PIN56 && GpioNo <= EPORT_PIN63) // Handle GINT56~63
        {

            if (GpioNo == EPORT_PIN58)
            {
                Eport->EPPUER &= ~(BITS(GpioNum));
                IOCTRL->USICR |= BITS(17);
            }
            else if (GpioNo == EPORT_PIN59)
            {
                Eport->EPPUER &= ~(BITS(GpioNum));
                IOCTRL->USICR |= BITS(18);
            }
            else if (GpioNo == EPORT_PIN60)
            {
                Eport->EPPUER &= ~(BITS(GpioNum));
                IOCTRL->USICR |= BITS(16);
            }
            else if (GpioNo == EPORT_PIN61)
            {
                IOCTRL->USICR |= BITS(9);
            }
            IOCTRL->EPORT7CR |= BITS(24 + GpioNum);
        }
    }
    else // Handle pull-up/pull-down disable
    {
        Eport->EPPUER &= ~(BITS(GpioNum));
    }

    return 0x0; // Return success
}

/*******************************************************************************
 * Function Name  : EPORT_OutputMode
 * Description    : Control the output mode of EPORT
 * Input          : - EPORT_TypeDef: Base address of EPORTx.
 *                  - EPORT_PINx: EPORT Pin; where x can be 0~7 to select the EPORT peripheral.
 *                  - omode: Output mode control. 1 for open-drain output, 0 for CMOS output.
 *
 * Output         : None
 * Return         : None
 ******************************************************************************/
uint8_t EPORT_OutputMode(EPORT_TypeDef *Eport, EPORT_PINx GpioNum, UINT8 omode)
{

    // Set the open-drain mode or CMOS mode based on the value of omode
    if (omode != 0)
    {
        Eport->EPODER |= BITS(GpioNum); // Open-drain mode
    }
    else
    {
        Eport->EPODER &= ~(BITS(GpioNum)); // CMOS mode
    }

    return 0x0; // Return success
}

/*******************************************************************************
 * Function Name  : EPORT_GetStatus
 * Description    : Get the interrupt status flag
 * Input          : - EPORT_TypeDef: Base address of EPORTx.
 *
 * Output         : None
 * Return         : status - Current interrupt status
 ******************************************************************************/
uint8_t EPORT_GetStatus(EPORT_TypeDef *Eport)
{
    uint8_t status;

    // Read the interrupt status from the EPFR register
    status = Eport->EPFR;

    return status; // Return the interrupt status
}

/*******************************************************************************
 * Function Name  : EPORT_ClrStatus
 * Description    : Clear the interrupt status flag
 * Input          : - EPORT_TypeDef: Base address of EPORTx.
 *                  - status: Interrupt status bit(s) to clear
 *
 * Output         : None
 * Return         : None
 ******************************************************************************/
void EPORT_ClrStatus(EPORT_TypeDef *Eport, uint8_t status)
{
    // Clear the interrupt status by writing to the EPFR register
    Eport->EPFR = status;
}
