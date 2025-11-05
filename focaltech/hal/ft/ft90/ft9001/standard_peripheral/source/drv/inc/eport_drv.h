// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// File name    : eport_drv.h
// Version      : V0.1
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#ifndef EPORT_DRV_H_
#define EPORT_DRV_H_

#include "memmap.h"
#include "eport_reg.h"

#define EPORT0   ((EPORT_TypeDef *)(EPORT0_BASE_ADDR))
#define EPORT1  ((EPORT_TypeDef *)(EPORT1_BASE_ADDR))
#define EPORT2  ((EPORT_TypeDef *)(EPORT2_BASE_ADDR))
#define EPORT3  ((EPORT_TypeDef *)(EPORT3_BASE_ADDR))
#define EPORT4  ((EPORT_TypeDef *)(EPORT4_BASE_ADDR))
#define EPORT5  ((EPORT_TypeDef *)(EPORT5_BASE_ADDR))
#define EPORT6  ((EPORT_TypeDef *)(EPORT6_BASE_ADDR))
#define EPORT7  ((EPORT_TypeDef *)(EPORT7_BASE_ADDR))



#define EPORT_GROUP(Eport)   \
    ((Eport) == EPORT0 ? 0 : \
    ((Eport) == EPORT1 ? 1 : \
    ((Eport) == EPORT2 ? 2 : \
    ((Eport) == EPORT3 ? 3 : \
    ((Eport) == EPORT4 ? 4 : \
    ((Eport) == EPORT5 ? 5 : \
    ((Eport) == EPORT6 ? 6 : \
    ((Eport) == EPORT7 ? 7 : -1))))))))

#define GPIO_NO(Eport, GpioNum)   ((EPORT_GROUP(Eport) != -1) ? (EPORT_GROUP(Eport) * 8 + (GpioNum)) : -1)

extern unsigned char eport_isr_mark;

typedef enum
{
	EPORT_PULLDOWN,
	EPORT_PULLUP,
	EPORT_DISPULL 
} EPORT_PULL_MODE;

typedef enum
{
	/*EPORT0*/
	EPORT_PIN0 = 0,
	EPORT_PIN1,
	EPORT_PIN2,
	EPORT_PIN3,
	EPORT_PIN4,
	EPORT_PIN5,
	EPORT_PIN6,
	EPORT_PIN7,

	/*EPORT1*/
	EPORT_PIN8,
	EPORT_PIN9,
	EPORT_PIN10,
	EPORT_PIN11,
	EPORT_PIN12,
	EPORT_PIN13,
	EPORT_PIN14,
	EPORT_PIN15,
	
		/*EPORT2*/
	EPORT_PIN16,
	EPORT_PIN17,
	EPORT_PIN18,
	EPORT_PIN19,
	EPORT_PIN20,
	EPORT_PIN21,
	EPORT_PIN22,
	EPORT_PIN23,
	
		/*EPORT3*/
	EPORT_PIN24,
	EPORT_PIN25,
	EPORT_PIN26,
	EPORT_PIN27,
	EPORT_PIN28,
	EPORT_PIN29,
	EPORT_PIN30,
	EPORT_PIN31,
	
		/*EPORT4*/
	EPORT_PIN32,
	EPORT_PIN33,
	EPORT_PIN34,
	EPORT_PIN35,
	EPORT_PIN36,
	EPORT_PIN37,
	EPORT_PIN38,
	EPORT_PIN39,

		/*EPORT5*/
	EPORT_PIN40,
	EPORT_PIN41,
	EPORT_PIN42,
	EPORT_PIN43,
	EPORT_PIN44,
	EPORT_PIN45,
	EPORT_PIN46,
	EPORT_PIN47,

		/*EPORT6*/
	EPORT_PIN48,
	EPORT_PIN49,
	EPORT_PIN50,
	EPORT_PIN51,
	EPORT_PIN52,
	EPORT_PIN53,
	EPORT_PIN54,
	EPORT_PIN55,

		/*EPORT7*/
	EPORT_PIN56,
	EPORT_PIN57,
	EPORT_PIN58,
	EPORT_PIN59,
	EPORT_PIN60,
	EPORT_PIN61,
	EPORT_PIN62,
	EPORT_PIN63,

}EPORT_PINx;

#define IS_EPORT_PINx(PIN) (((PIN) == EPORT_PIN0) || \
		                    ((PIN) == EPORT_PIN1) || \
		                    ((PIN) == EPORT_PIN2) || \
		                    ((PIN) == EPORT_PIN3) || \
		                    ((PIN) == EPORT_PIN4) || \
		                    ((PIN) == EPORT_PIN5) || \
		                    ((PIN) == EPORT_PIN6) || \
												((PIN) == EPORT_PIN7) || \
		                    ((PIN) == EPORT_PIN8) || \
		                    ((PIN) == EPORT_PIN9) || \
		                    ((PIN) == EPORT_PIN10)|| \
		                    ((PIN) == EPORT_PIN11)|| \
		                    ((PIN) == EPORT_PIN12)|| \
												((PIN) == EPORT_PIN13)|| \
		                    ((PIN) == EPORT_PIN14)|| \
		                    ((PIN) == EPORT_PIN15)|| \
												((PIN) == EPORT_PIN16)|| \
		                    ((PIN) == EPORT_PIN17)|| \
		                    ((PIN) == EPORT_PIN18)|| \
		                    ((PIN) == EPORT_PIN19)|| \
		                    ((PIN) == EPORT_PIN20)|| \
												((PIN) == EPORT_PIN21)|| \
		                    ((PIN) == EPORT_PIN22)|| \
		                    ((PIN) == EPORT_PIN23)|| \
												((PIN) == EPORT_PIN24)|| \
		                    ((PIN) == EPORT_PIN25)|| \
		                    ((PIN) == EPORT_PIN26)|| \
		                    ((PIN) == EPORT_PIN27)|| \
		                    ((PIN) == EPORT_PIN28)|| \
												((PIN) == EPORT_PIN29)|| \
		                    ((PIN) == EPORT_PIN30)|| \
		                    ((PIN) == EPORT_PIN31)|| \
												((PIN) == EPORT_PIN32)|| \
		                    ((PIN) == EPORT_PIN33)|| \
		                    ((PIN) == EPORT_PIN34)|| \
		                    ((PIN) == EPORT_PIN35)|| \
		                    ((PIN) == EPORT_PIN36)|| \
												((PIN) == EPORT_PIN37)|| \
		                    ((PIN) == EPORT_PIN38)|| \
		                    ((PIN) == EPORT_PIN39)   )


#define LEVEL_H        1
#define LEVEL_L        0

#define EPORT_INPUT    0
#define EPROT_OUTPUT   1

typedef enum
{
    LOW_LEVEL_INT = 0,
    HIGH_LEVEL_INT,
    RISING_EDGE_INT,
    FALLING_EDGE_INT,
    RISING_FALLING_EDGE_INT,

}EPORT_INT_MODE;

#define IS_EPORT_INT_MODE(MODE) (((MODE) == LOW_LEVEL_INT) || \
		                             ((MODE) == HIGH_LEVEL_INT) || \
		                             ((MODE) == RISING_EDGE_INT) || \
		                             ((MODE) == FALLING_EDGE_INT) || \
                                 ((MODE) == RISING_FALLING_EDGE_INT))


/***************************************************************************
 * Function Name  : EPORT_ConfigGpio
 * Description    : Configures EPORT as GPIO.
 * Input          : - EPORT_TypeDef: Base address of EPORTx.
 *                  - EPORT_PINx: EPORT Pin; x can be 0~7 to select the EPORT peripheral.
 *                  - GpioDir: GPIO direction setting. GPIO_OUTPUT: Output, GPIO_INPUT: Input.
 * Output         : None
 * Return         : None
 ***************************************************************************/
extern uint32_t  EPORT_ConfigGpio(EPORT_TypeDef* Eport,EPORT_PINx GpioNo, uint8_t GpioDir);

/***************************************************************************
 * Function Name  : EPORT_WriteGpioDatas
 * Description    : Sets the EPORT pin level.
 * Input          : - EPORT_TypeDef: Base address of EPORTx.
 *                  - Vals: Value to set, only lower 8 bits are effective.
 * Output         : None
 ***************************************************************************/
extern void  EPORT_WriteGpioDatas(EPORT_TypeDef* Eport, uint8_t Vals);

/***************************************************************************
 * Function Name  : EPORT_ReadGpioDatas
 * Description    : Reads the value from the EPORT port.
 * Input          : - EPORT_TypeDef: Base address of EPORTx.
 * Output         : None
 * Return         : EPORTx value
 ***************************************************************************/
extern uint8_t EPORT_ReadGpioDatas(EPORT_TypeDef* Eport);

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
extern void EPORT_ITTypeConfig(EPORT_TypeDef* Eport, EPORT_PINx IntNo, EPORT_INT_MODE IntMode);



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
extern void EPORT_ITConfig(EPORT_TypeDef* Eport,EPORT_PINx IntNo, FunctionalState NewState);


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
extern void EPORT_PullupConfig(EPORT_TypeDef* Eport,EPORT_PINx IntNo, FunctionalState NewState);

/**
 * Configure pull-up or pull-down resistor for EPORT pins.
 *
 * 1. Enable pull-up
 * 2. Enable pull-down (EPORT_PIN30~EPORT_PIN39 do not support pull-down)
 * 3. Disable pull-up or pull-down
 */
extern uint8_t EPORT_PullConfig(EPORT_TypeDef* Eport,const EPORT_PINx IntNo, const EPORT_PULL_MODE eport_pull_mode);

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
extern uint8_t EPORT_OutputMode(EPORT_TypeDef* Eport,EPORT_PINx GpioNo, UINT8 omode);


/*******************************************************************************
* Function Name  : EPORT_ClrStatus
* Description    : Clear the interrupt status flag
* Input          : - EPORT_TypeDef: Base address of EPORTx.
*                  - status: Interrupt status bit(s) to clear
*
* Output         : None
* Return         : None
******************************************************************************/
extern void EPORT_ClrStatus(EPORT_TypeDef* Eport, uint8_t status);

/*******************************************************************************
* Function Name  : EPORT_GetStatus
* Description    : Get the interrupt status flag
* Input          : - EPORT_TypeDef: Base address of EPORTx.
*
* Output         : None
* Return         : status - Current interrupt status
******************************************************************************/
extern uint8_t EPORT_GetStatus(EPORT_TypeDef* Eport);
#endif /* EPORT_DRV_H_ */
