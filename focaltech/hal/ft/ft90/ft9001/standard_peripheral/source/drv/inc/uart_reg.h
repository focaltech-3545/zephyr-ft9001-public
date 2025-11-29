/**  
  **********************************************************************************
             Copyright(c) 2025 Focaltech Co. Ltd.
                      All Rights Reserved
  **********************************************************************************
  */

#ifndef UART_REG_H_
#define UART_REG_H_

#include "type.h"

typedef struct
{
	__IO unsigned char SCIBDL;//0x00
	__IO unsigned char SCIBDH;//0x01
	__IO unsigned char SCICR2;//0x02
    __IO unsigned char SCICR1;//0x03
	__IO unsigned char SCISR2;//0x04
	__IO unsigned char SCISR1;//0x05
	__IO unsigned char SCIDRL;//0x06
	__IO unsigned char SCIDRH;//0x07
	__IO unsigned char SCIPORT;//0x08
	__IO unsigned char SCIPURD; //0x09
	__IO unsigned char SCIBRDF;//0x0a
	__IO unsigned char SCIDDR; //0x0b
	__IO unsigned char SCIIRCR;//0x0c
	__IO unsigned char SCITR; //0x0d
	__IO unsigned char SCIFCR;//0x0e
	__IO unsigned char SCIIRDR;//0x0f
	__IO unsigned char SCIDCR;//0x10
	__IO unsigned char SCIFSR;//0x11
	__IO unsigned char SCIRXTOCTR;//0x12
	__IO unsigned char SCIFCR2;//0x13
    __IO unsigned char SCIFCTRL;//0x14
  __IO unsigned char SCIFSR2;//0x15

}UART_TypeDef;


#define SCICR1_LOOPS_MASK		0x80
#define SCICR1_WOMS_MASK		0x40
#define SCICR1_RSRC_MASK		0x20
#define SCICR1_M_MASK			0x10
#define SCICR1_WAKE_MASK		0x08
#define SCICR1_ILT_MASK			0x04
#define SCICR1_PE_MASK			0x02
#define SCICR1_PT_MASK 			0x01

////////////////////////////////////////////
#define SCICR2_TIE_MASK			0x80
#define SCICR2_TCIE_MASK		0x40
#define SCICR2_RIE_MASK			0x20
#define SCICR2_ILIE_MASK		0x10
#define SCICR2_TE_MASK			0x08
#define SCICR2_RE_MASK			0x04
#define SCICR2_RWU_MASK			0x02
#define SCICR2_SBK_MASK			0x01

////////////////////////////////////////////
#define SCISR1_TDRE_MASK		0x80
#define SCISR1_TC_MASK		    0x40
#define SCISR1_RDRF_MASK		0x20
#define SCISR1_IDLE_MASK		0x10
#define SCISR1_OR_MASK			0x08
#define SCISR1_NF_MASK			0x04
#define SCISR1_FE_MASK			0x02
#define SCISR1_PF_MASK			0x01

////////////////////////////////////////////
//#define SCISR2_RAF_MASK			0x01
#define SCISR2_FPF_MASK			0x01
#define SCISR2_FFE_MASK			0x02
#define SCISR2_FNF_MASK			0x04
#define SCISR2_FOR_MASK			0x08

////////////////////////////////////////////
#define SCIDRH_R8_MASK		    	0x80
#define SCIDRH_T8_MASK		    	0x40

////////////////////////////////////////////
#define SCIPURD_SCISDOZ_MASK		0x80
#define SCIPURD_RDPSCI_MASK		    0x10
#define SCIPURD_PUPSCI_MASK		    0x01

////////////////////////////////////////////
#define SCIPORT_PORTSC1_MASK		0x02
#define SCIPORT_PORTSC0_MASK		0x01

////////////////////////////////////////////
#define SCIDDR_DDRSC1_MASK		0x02
#define SCIDDR_DDRSC0_MASK		0x01

////////////////////////////////////////////
#define SCIIRCR_IREN_MASK		0x01
#define SCIIRCR_IRSC_MASK		0x02
#define SCIIRCR_RINV_MASK		0x04
#define SCIIRCR_TINV_MASK		0x08

////////////////////////////////////////////
#define SCIFCR_RFEN			0x80
#define SCIFCR_TFEN			0x40

#define SCIFCR_RXFLSEL_1_8		0
#define SCIFCR_RXFLSEL_1_4		0x08
#define SCIFCR_RXFLSEL_1_2		0x10
#define SCIFCR_RXFLSEL_3_4		0x18
#define SCIFCR_RXFLSEL_7_8		0x20

#define SCIFCR_TXFLSEL_7_8		0
#define SCIFCR_TXFLSEL_3_4		0x01
#define SCIFCR_TXFLSEL_1_2		0x02
#define SCIFCR_TXFLSEL_1_4		0x03
#define SCIFCR_TXFLSEL_1_8		0x04

////////////////////////////////////////////
#define SCIFSR_TFTS_MASK		0x80
#define SCIFSR_FTC_MASK		    0x40
#define SCIFSR_RFTS_MASK		0x20
#define SCIFSR_RTOS_MASK		0x10
#define SCIFSR_TFULL_MASK		0x08
#define SCIFSR_TEMPTY_MASK		0x04
#define SCIFSR_RFULL_MASK		0x02
#define SCIFSR_REMPTY_MASK		0x01

////////////////////////////////////////////
#define SCIDCR_TXDMAE_MASK		0x02
#define SCIDCR_RXDMAE_MASK		0x01

#define SCIFCR2_TXFIE           0x80
#define SCIFCR2_TXFCIE          0x40
#define SCIFCR2_RXFIE           0x20
#define SCIFCR2_RXORIE          0x10
#define SCIFCR2_RXFTOIE         0x08
#define SCIFCR2_RXFTOE          0x04
#define SCIFCR2_TXFCLR          0x02
#define SCIFCR2_RXFCLR          0x01

#define SCIFCTRL_RTSE_MASK		0x08
#define SCIFCTRL_CTSE_MASK		0x04
#define SCIFCTRL_CTSIE_MASK		0x02
#define SCIFCTRL_CTSIS_MASK		0x01

#define SCI_ENABLE_TRANSMITTER(UARTx)  	(UARTx->SCICR2 |= SCICR2_TE_MASK)
#define SCI_DISABLE_TRANSMITTER(UARTx) 	(UARTx->SCICR2 &= ~SCICR2_TE_MASK)

#define SCI_ENABLE_RECEIVER(UARTx)		(UARTx->SCICR2 |= SCICR2_RE_MASK)
#define SCI_DISABLE_RECEIVER(UARTx)		(UARTx->SCICR2 &= ~SCICR2_RE_MASK)

#define SCI_ENABLE_DMA(UARTx)              (UARTx->SCIDCR |= (SCIDCR_TXDMAE_MASK|SCIDCR_RXDMAE_MASK))
#define SCI_DISABLE_DMA(UARTx)             (UARTx->SCIDCR &= ~(SCIDCR_TXDMAE_MASK|SCIDCR_RXDMAE_MASK))

#define SCI_RX_ENABLE_DMA(UARTx)           (UARTx->SCIDCR |= (SCIDCR_RXDMAE_MASK))
#define SCI_RX_DISABLE_DMA(UARTx)          (UARTx->SCIDCR &= ~(SCIDCR_RXDMAE_MASK))

#define SCI_FIFO_CLR(UARTx)                (UARTx->SCIFCR2 |= (SCIFCR2_TXFCLR|SCIFCR2_RXFCLR))

#endif /* UART_REG_H_ */
