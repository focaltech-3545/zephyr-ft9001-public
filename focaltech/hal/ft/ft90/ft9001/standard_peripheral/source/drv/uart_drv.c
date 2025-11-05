// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// File name    : uart_drv.c
// Version      : V0.1
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#include "uart_drv.h"
#include "uart_reg.h"
//#include "type.h"
//#include "dmac_drv.h"

#include "stddef.h"
#include "stdarg.h"
#include "stdlib.h"

/*******************************************************************************
* Function Name  : UART_Init
* Description    : UART Initialize
* Input          : - UARTx: SCI2 or SCI_BT
*                  - UART_InitStruct£ºUARTx  initialization parameter structure Poniter
*                  - sys_freq: system clock
* Output         : None
* Return         : None
******************************************************************************/
void UART_Init(UART_TypeDef *UARTx, UART_InitTypeDef *UART_InitStruct, uint32_t sys_freq)
{
	unsigned short int bandrate_I;
	unsigned char bandrate_F;
	unsigned char bandrate_h;
	unsigned char bandrate_l;

	//IP closk is 60MHz
	bandrate_I = ((unsigned short int)(sys_freq*4/UART_InitStruct->UART_BaudRate))>>6;
	bandrate_F = (((unsigned short int)(sys_freq*8/UART_InitStruct->UART_BaudRate)+1)/2)&0x003f;	
	bandrate_h = (unsigned char)((bandrate_I>>8)&0x00ff);
	bandrate_l = (unsigned char)(bandrate_I&0x00ff);

	//2, e, a, 1, 0, 3, 7(noOp),6

	UARTx->SCICR2 = 0;      //disable UART 
	UARTx->SCIFCR = 0;	
	UARTx->SCIFCR = SCIFCR_RFEN|SCIFCR_TFEN;  //FIFO mode

    //set bandrate
	UARTx->SCIBRDF = bandrate_F;		//Write float before Interger
	UARTx->SCIBDH = bandrate_h;
	UARTx->SCIBDL = bandrate_l;

	UARTx->SCICR1 = 0x00;
	UARTx->SCICR1 |= UART_InitStruct->UART_FrameLength; //11/10 bit frame
	if ((UART_InitStruct->UART_FrameLength == UART_DATA_FRAME_LEN_11BIT) &&
		(UART_InitStruct->UART_StopBits == 2))
	{
		UARTx->SCIDRH |= (1u << 6);
	}
	
	if (UART_InitStruct->UART_Parity == UART_PARITY_NONE)
	{
		UARTx->SCICR1 |= ParityDIS;	//parity disable
	}
	else
	{
		UARTx->SCICR1 |= ParityEN;	//parity enable
		UARTx->SCICR1 |= UART_InitStruct->UART_Parity;//odd parity
	}

	UARTx->SCIFCR  |= (SCIFCR_RXFLSEL_1_8|SCIFCR_TXFLSEL_1_8);    	//set FIFO waterline
	UARTx->SCIDRL = 0;                            //clear data

	//0x13, 0x15, 2
    UARTx->SCIFCR2 = 0;
	UARTx->SCIFCR2 |= SCIFCR2_RXFTOE;                  //enable RX FIFO timeout
	UARTx->SCIFCR2 |= (SCIFCR2_RXFCLR|SCIFCR2_TXFCLR); //clear TX FIFO and RX FIFO
	UARTx->SCIFCR2 = UARTx->SCIFCR2;                   //

	UARTx->SCIFSR2 = UARTx->SCIFSR2;

	UARTx->SCICR2 |= SCICR2_TE_MASK|SCICR2_RE_MASK; //enable tx and rx
	
    if(UART_InitStruct->UART_Mode == UART_DMA_MODE)
    { 
		UARTx->SCICR1  |=  SCICR1_ILT_MASK;
		UARTx->SCICR2  |=  SCICR2_ILIE_MASK;
    }
} 

void UART_Configure(UART_TypeDef *UARTx, UART_InitTypeDef *UART_InitStruct, uint32_t sys_freq)
{
	unsigned short int bandrate_I;
	unsigned char bandrate_F;
	unsigned char bandrate_h;
	unsigned char bandrate_l;

	//disable tx and rx
	UARTx->SCICR2 &= ~(SCICR2_TE_MASK|SCICR2_RE_MASK); 
	//set bandrate
	bandrate_I = ((unsigned short int)(sys_freq*4/UART_InitStruct->UART_BaudRate))>>6;
	bandrate_F = (((unsigned short int)(sys_freq*8/UART_InitStruct->UART_BaudRate)+1)/2)&0x003f;
	bandrate_h = (unsigned char)((bandrate_I>>8)&0x00ff);
	bandrate_l = (unsigned char)(bandrate_I&0x00ff);
	UARTx->SCIBRDF = bandrate_F;		//Write float before Interger
	UARTx->SCIBDH = bandrate_h;
	UARTx->SCIBDL = bandrate_l;

	//Set parity
	if (UART_InitStruct->UART_Parity == UART_PARITY_NONE)
	{
		UARTx->SCICR1 |= ParityDIS;	//parity disable
	}
	else
	{
		UARTx->SCICR1 |= ParityEN;	//parity enable
		UARTx->SCICR1 |= UART_InitStruct->UART_Parity;//odd parity
	}

	// no support flow ctrl

	//enable tx and rx
	UARTx->SCICR2 |= SCICR2_TE_MASK|SCICR2_RE_MASK; 
}

/*******************************************************************************
* Function Name  : UART_PollIn
* Description    : UART receive
* Input          : - UARTx: where x can be 1 to 3 to select the UART peripheral UART1/UART2/UART3
*
* Output         : None
* Return         : 0 or -1
******************************************************************************/
int UART_PollIn(UART_TypeDef *UARTx, unsigned char *rx_data)
{  
	int ret = -1;
	if(!(UARTx->SCIFSR & SCIFSR_REMPTY_MASK))
	{
		*rx_data= UARTx->SCIDRL;
		ret = 0;
	}
	return ret;	
}

/*******************************************************************************
* Function Name  : UART_PollOut
* Description    : UART send data
* Input          : - UARTx: where x can be 1 to 3 to select the UART peripheral£»UART1/UART2/UART3
*                  - SendByte£ºdata
*
* Output         : None
* Return         : STATUS
******************************************************************************/
void UART_PollOut(UART_TypeDef *UARTx, unsigned char outbyte)
{
	while(UARTx->SCIFSR & SCIFSR_TFULL_MASK);
	UARTx->SCIDRL = outbyte&0xff;
	while(1) 
	{
		if((UARTx->SCIFSR & SCIFSR_TEMPTY_MASK) && (UARTx->SCIFSR  & SCIFSR_FTC_MASK))
			break;
	}
	return;
}

int UART_FifoFill(UART_TypeDef *UARTx, unsigned char outbyte)
{
	int ret = -1;
	if(UARTx->SCIFSR & SCIFSR_TFULL_MASK)
    {
        ret = -1;
	}
	else
	{
		UARTx->SCIDRL = outbyte&0xff;
		ret = 0;
	}
	return ret;
}

int UART_FifoRead(UART_TypeDef *UARTx, unsigned char *rx_data, int size)
{
	int i = 0;
	while(!(UARTx->SCIFSR & SCIFSR_REMPTY_MASK) && (i<size))
	{
		rx_data[i] = UARTx->SCIDRL;
		i++;
	}
	return i;
}

void UART_IrqTxEnable(UART_TypeDef *UARTx)
{
	UARTx->SCIFCR2 = UARTx->SCIFCR2 | SCIFCR2_TXFIE;
	return;
}

void UART_IrqTxDisable(UART_TypeDef *UARTx)
{
	UARTx->SCIFCR2 = UARTx->SCIFCR2 & (~SCIFCR2_TXFIE);
	return;
}

void UART_IrqRxEnable(UART_TypeDef *UARTx)
{
	UARTx->SCIFCR2 = UARTx->SCIFCR2 | SCIFCR2_RXFIE | SCIFCR2_RXFTOIE;
	return;
}

void UART_IrqRxDisable(UART_TypeDef *UARTx)
{
	UARTx->SCIFCR2 = UARTx->SCIFCR2 & (~(SCIFCR2_RXFIE | SCIFCR2_RXFTOIE));
	return;
}

int UART_IrqTxReady(UART_TypeDef *UARTx)
{
	int ret = -1;
	if(!(UARTx->SCIFSR  & SCIFSR_TFULL_MASK))
		ret = 0;
	return ret;
}

int UART_IrqRxReady(UART_TypeDef *UARTx)
{
	int ret = -1;
	if((UARTx->SCIFSR & SCIFSR_RFTS_MASK) || !(UARTx->SCIFSR  & SCIFSR_REMPTY_MASK))
		ret = 0;
	return ret;
}

int UART_IrqTxComplete(UART_TypeDef *UARTx)
{
	int ret = -1;
	if((UARTx->SCIFSR & SCIFSR_TEMPTY_MASK) && (UARTx->SCIFSR  & SCIFSR_FTC_MASK))
		ret = 0;
	return ret;
}

int UART_IrqTxPending(UART_TypeDef *UARTx)
{
	int ret = -1;
	if((UARTx->SCIFSR & SCIFSR_TEMPTY_MASK) && (UARTx->SCIFSR  & SCIFSR_FTC_MASK) && (UARTx->SCIFSR & SCIFSR_REMPTY_MASK))
		ret = 0;
	return ret;
}

int UART_IrqIsPending(UART_TypeDef *UARTx)
{
	int ret = -1;
	if ((!(UARTx->SCIFSR  & SCIFSR_TFULL_MASK) &&  (UARTx->SCIFCR2 & SCIFCR2_TXFIE))  //tx_ready
	|| (((UARTx->SCIFSR & SCIFSR_RFTS_MASK) || !(UARTx->SCIFSR  & SCIFSR_REMPTY_MASK)) &&  (UARTx->SCIFCR2 & SCIFCR2_RXFIE))) //rx_ready
	{
		ret = 0;
	}
	return ret;
}

void UART_IrqErrEnable(UART_TypeDef *UARTx)
{
	UARTx->SCIFCR2 = UARTx->SCIFCR2 | SCIFCR2_RXORIE | SCIFCR2_RXFTOIE;
	return;
}

void UART_IrqErrDisable(UART_TypeDef *UARTx)
{
	UARTx->SCIFCR2 = UARTx->SCIFCR2 & (~ (SCIFCR2_RXORIE  | SCIFCR2_RXFTOIE));
	return;
}

int UART_IrqErrCheck(UART_TypeDef *UARTx)
{
	int z_err = 0;

	z_err = ((UARTx->SCIFSR2 & SCISR2_FOR_MASK) ? (1 << 0) : 0) |
			((UARTx->SCIFSR2 & SCISR2_FPF_MASK) ? (1 << 1) : 0) |
			((UARTx->SCIFSR2 & SCISR2_FNF_MASK) ? (1 << 5) : 0) |
			((UARTx->SCIFSR2 & SCISR2_FFE_MASK) ? (1 << 2) : 0);

	UARTx->SCIFSR2 = UARTx->SCIFSR2;
	return z_err;	
}
