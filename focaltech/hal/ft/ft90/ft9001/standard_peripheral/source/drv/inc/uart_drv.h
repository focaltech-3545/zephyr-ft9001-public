// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// File name    : uart_drv.h
// Version      : V0.1
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#ifndef UART_DRV_H_
#define UART_DRV_H_

#include "memmap.h"
#include "uart_reg.h"

////////////////////////////////////////////////////////////////////////////////////////
#define		UART_DATA_FRAME_LEN_10BIT	    0				//10bit frame:----StartBit:1bit DataBit:8bit StopBit:1bit
#define		UART_DATA_FRAME_LEN_11BIT	    SCICR1_M_MASK

#define IS_UART_FRAMW_LENGTH(LENGTH) (((LENGTH) == UART_DATA_FRAME_LEN_10BIT) || \
                                      ((LENGTH) == UART_DATA_FRAME_LEN_11BIT))



#define		ParityDIS	                 0			      	//Parity Disable
#define		ParityEN	                 SCICR1_PE_MASK	//Parity Enable

#define		UART_PARITY_EVE		         0			      	//Even Parity
#define     UART_PARITY_ODD		         SCICR1_PT_MASK	//Odd Parity
#define     UART_PARITY_NONE             2              //NONE Parity


#define IS_UART_PARITY(PARITY) (((PARITY) == UART_PARITY_EVE) || \
                                ((PARITY) == UART_PARITY_ODD) || \
                                ((PARITY) == UART_PARITY_NONE))

//#define UART_TIMEOUT_1S              20000000  // sys_clk=120MHz, delay 1MS ¡Ö sys_clk/6000
/*******************************************************************************
RXFLSEL                     UART_FIFO_LEN
SCIFCR_RXFLSEL_1_8          0x02
SCIFCR_RXFLSEL_1_4          0x04
SCIFCR_RXFLSEL_1_2          0x08
SCIFCR_RXFLSEL_3_4          0x0C
SCIFCR_RXFLSEL_7_8          0x0E
*******************************************************************************/
#define UART_FIFO_TRIGGER_LEVEL            0x02       //value decide by RXFLSEL
#define UART_RECV_MAX_LEN                  0x200

typedef enum
{
	UART_NORMAL_MODE = 0,
	UART_INT_MODE,
	UART_EDMA_MODE,
	UART_DMA_MODE

}UART_MODE;

//Choose a pattern
#define IS_UART_MODE(MODE) (((MODE) == UART_NORMAL_MODE) || \
                            ((MODE) == UART_INT_MODE) || \
                            ((MODE) == UART_EDMA_MODE) || \
                            ((MODE) == UART_DMA_MODE) )

/* UART Init Structure definition */
typedef struct
{
    unsigned int UART_BaudRate;
    unsigned char UART_FrameLength;
    unsigned char UART_StopBits;
    unsigned char UART_Parity;
    unsigned char UART_Mode;
    unsigned char UART_TimeoutCounter;
    unsigned char flow_ctrl;

} UART_InitTypeDef;

typedef enum
{
    UART_RX = 0,
    UART_TX,
} UART_PINx;

/*******************************************************************************
* Function Name  : UART_Init
* Description    : UART Initialize
* Input          : - UARTx: where x can be 0 to 1 to select the UART peripheral£»SCI0 or SCI1
*                  - UART_InitStruct£ºUARTx  initialization parameter structure Poniter
*
* Output         : None
* Return         : None
******************************************************************************/
extern void UART_Init(UART_TypeDef *UARTx, UART_InitTypeDef *UART_InitStruct, uint32_t sys_freq);

extern int UART_PollIn(UART_TypeDef *UARTx, unsigned char *rx_data);
extern void UART_PollOut(UART_TypeDef *UARTx, unsigned char outbyte);
extern int UART_FifoRead(UART_TypeDef *UARTx, unsigned char *rx_data, int size);
extern int UART_FifoFill(UART_TypeDef *UARTx, unsigned char outbyte);
extern void UART_IrqTxEnable(UART_TypeDef *UARTx);
extern void UART_IrqRxEnable(UART_TypeDef *UARTx);
extern void UART_IrqTxDisable(UART_TypeDef *UARTx);
extern void UART_IrqRxDisable(UART_TypeDef *UARTx);
extern int UART_IrqTxReady(UART_TypeDef *UARTx);
extern int UART_IrqRxReady(UART_TypeDef *UARTx);
extern int UART_IrqTxComplete(UART_TypeDef *UARTx);
extern int UART_IrqTxPending(UART_TypeDef *UARTx);
extern int UART_IrqIsPending(UART_TypeDef *UARTx);
extern void UART_IrqErrEnable(UART_TypeDef *UARTx);
extern void UART_IrqErrDisable(UART_TypeDef *UARTx);
extern int UART_IrqErrCheck(UART_TypeDef *UARTx);
extern void UART_Configure(UART_TypeDef *UARTx, UART_InitTypeDef *UART_InitStruct, uint32_t sys_freq);

#endif /* UART_DRV_H_ */
