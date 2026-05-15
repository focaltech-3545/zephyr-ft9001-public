// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// File name    : spi_reg.h
// Version      : V0.1
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#ifndef SPI_REG_H_
#define SPI_REG_H_

#include "type.h"

//#pragma anon_unions
typedef struct
{
	__IO unsigned char SPIBR;  //0x00
	__IO unsigned char SPIFR;  //0x01
	__IO unsigned char SPICR1; //0x02
	__IO unsigned char SPICR2; //0x03

	__IO unsigned char SPIRXFTOCTR; //0x04
	__IO unsigned char SPITXFTOCTR; //0x05
	__IO unsigned char SPIRXFCR;	//0x06
	__IO unsigned char SPITXFCR;	//0x07

	__IO unsigned char SPIASCDR; //0x08
	__IO unsigned char SPIBSCDR; //0x09
	__IO unsigned char SPIDDR;	 //0x0a
	__IO unsigned char SPIPURD;	 //0x0b

	__IO unsigned char SPITCNTM; //0x0c
	__IO unsigned char SPITCNTH; //0x0d
	__IO unsigned char SPIPORT;	 //0x0e
	__IO unsigned char SPITCNTL; //0x0f

	__IO unsigned char SPIIRSP; //0x010
	__IO unsigned char RESERVED1[1]; //0x11

union
	{
		__IO unsigned short SPIDRHW; //0x12
		struct
		{
			__IO unsigned char SPIDR;	//0x12
			__IO unsigned char SPIDRH;	//0x13
		};
	};

	__IO unsigned char SPIRXFSR; //0x14
	__IO unsigned char SPITXFSR; //0x15
	union
	{
		__IO unsigned short SPISRHW; //0x16
		struct
		{
			__IO unsigned char SPISR;  //0x16
			__IO unsigned char SPISRH; //0x17
		};
	};

	__IO unsigned char SPIFDCR;	  //0x18
	__IO unsigned char SPIICR;	  //0x19
	__IO unsigned char SPIDMACR;  //0x1a
	__IO unsigned char SPIDMATHR; //0x1b

	__IO unsigned char SPIRXFDBGR; //0x1c
	__IO unsigned char SPITXFDBGR; //0x1e

} SPI_TypeDef;

/////////////////////////////////////////////////////////////////////////////////

/*SPI FIFO size*/
#define SPI_FIFO_SIZE (8)

/*SPI default guard time*/
#define SPI_DFT_GT (0x03)

#define SPIDR_ADDR (0x12)
/*SPI error-related interrupt*/
/*-----------------------------------------------------------------------------------
BIT11 | BIT10 | BIT9 | BIT8 | BIT7 | BIT6 | BIT5 | BIT4 | BIT3 | BIT2 | BIT1 | BIT0
FLOST     *     MODF  RXFOVF RXFUDF RXFSER TXFOVF TXFUDF TXFSER  RXFTO  TXFTO  SPIE
-----------------------------------------------------------------------------------*/
#define SPI_ERROR_INT (0x0bb6)

/*macro to enable FIFO time-out*/
#define SPI_TIMEOUT_EN (0)

/*master/slave mode macros*/
#define SPI_MASTER (0x01)
#define SPI_SLAVE (0x00)

/*SPI interrupt macros*/
/*
BIT11 | BIT10 | BIT9 | BIT8 | BIT7 | BIT6 | BIT5 | BIT4 | BIT3 | BIT2 | BIT1 | BIT0
FLOST     *     MODF  RXFOVF RXFUDF RXFSER TXFOVF TXFUDF TXFSER  RXFTO TXFTO   SPIE
*/

#define SPIIE_SPIE_MASK (BIT0)
#define SPIIE_TXFTO_MASK (BIT1)
#define SPIIE_RXFTO_MASK (BIT2)
#define SPIIE_TXFSER_MASK (BIT3)
#define SPIIE_TXFUDF_MASK (BIT4)
#define SPIIE_TXFOVF_MASK (BIT5)
#define SPIIE_RXFSER_MASK (BIT6)
#define SPIIE_RXFUDF_MASK (BIT7)
#define SPIIE_RXFOVF_MASK (BIT8)
#define SPIIE_MODF_MASK (BIT9)
#define SPIIE_FLOST_MASK (BIT11)

/*SPIFR macros*/
#define SPIFR_CONT_MASK (0x80)
#define SPIFR_GTE_MASK (0x40)
#define SPIFR_LBM_MASK (0x20)
#define SPIFR_FFSEL_MASK (0x10)
#define SPIFR_FMSZ_MASK (0x0f)
#define FMSZ4_16(len) (len - 1)

/*Data regiter access mode macros*/
#define SPI_BYTE (0x01)
#define SPI_HALFWORD (0x00)

/*SPICR1 marcos*/
#define SPICR1_SPIE_MASK (0x80)
#define SPICR1_SPE_MASK  (0x40)
#define SPICR1_SWOM_MASK (0x20)
#define SPICR1_MSTR_MASK (0x10)
#define SPICR1_CPOL_MASK (0x08)
#define SPICR1_CPHA_MASK (0x04)
#define SPICR1_SSOE_MASK (0x02)
#define SPICR1_LSBFE_MASK (0x01)
#define SPICR1_CLEAR_MASK (0x00)

#define SPI_ENABLE (0x40)

/*SPICR2 marcos*/
#define SPICR2_GT_MASK (0xFC)
#define SPICR2_SPISDOZ_MASK (0x02)
#define SPICR2_SPC0_MASK (0x01)

/*SPITXFTOCTR macros*/
#define SPITXFTOCTR_TXFTOIE_MASK (0x80)
#define SPITXFTOCTR_TXFTOE_MASK (0x40)
#define SPITXFTOCTR_COUNTER_MASK (0x3F)

/*SPIRXFTOCTR macros*/
#define SPIRXFTOCTR_RXFTOIE_MASK (0x80)
#define SPIRXFTOCTR_RXFTOE_MASK (0x40)
#define SPIRXFTOCTR_COUNTER_MASK (0x3F)

/*SPITXFCR macros*/
#define SPITXFCR_TXFCLR_MASK (0x80)
#define SPITXFCR_TXFOVIE_MASK (0x40)
#define SPITXFCR_TXFUDIE_MASK (0x20)
#define SPITXFCR_TXFSTHIE_MASK (0x10)
#define SPITXFCR_TXFSTH_MASK (0x0f)

/*SPIRXFCR macros*/
#define SPIRXFCR_RXFCLR_MASK (0x80)
#define SPIRXFCR_RXFOVIE_MASK (0x40)
#define SPIRXFCR_RXFUDIE_MASK (0x20)
#define SPIRXFCR_RXFSTHIE_MASK (0x10)
#define SPIRXFCR_RXFSTH_MASK (0x0f)

/*SPIPURD macros*/
#define SPIPURD_HS_MASK (0x80)
#define SPIPURD_PSW_MASK (0x40)
#define SPIPURD_RDPSP_MASK (0x10)
#define SPIPURD_PDPSP_MASK (0x02)
#define SPIPURD_PUPSP_MASK (0x01)

/*SPIDDR macros*/
#define SPIDDR_SS_MASK (0x08)
#define SPIDDR_SCK_MASK (0x04)
#define SPIDDR_MOSI_MASK (0x02)
#define SPIDDR_MISO_MASK (0x01)

/*SPIPORT macros*/
#define SPIPORT_SS_MASK (0x08)
#define SPIPORT_SCK_MASK (0x04)
#define SPIPORT_MOSI_MASK (0x02)
#define SPIPORT_MISO_MASK (0x01)

/*SPITXFSR macros*/
#define SPITXFSR_TXNXTP_MASK (0xf0)
#define SPITXFSR_TXFCTR_MASK (0x0f)

/*SPISR macros*/
#define SPISR_TXFTO_MASK (0x8000)
#define SPISR_TXFOVF_MASK (0x4000)
#define SPISR_TXFUDF_MASK (0x2000)
#define SPISR_TXFSER_MASK (0x1000)
#define SPISR_RXFTO_MASK (0x0800)
#define SPISR_RXFOVF_MASK (0x0400)
#define SPISR_RXFUDF_MASK (0x0200)
#define SPISR_RXFSER_MASK (0x0100)
#define SPISR_SPIF_MASK (0x0080)
#define SPISR_FLOST_MASK (0x0040)
#define SPISR_EOTF_MASK (0x0020)
#define SPISR_MODF_MASK (0x0010)
#define SPISR_TXFFULL_MASK (0x0008)
#define SPISR_TXFEMP_MASK (0x0004)
#define SPISR_RXFFULL_MASK (0x0002)
#define SPISR_RXFEMP_MASK (0x0001)

/*SPIICR macros*/
#define SPIICR_FLOSTIE_MASK (0x40)
#define SPIICR_MODFIE_MASK (0x10)

/*SPIIRSP macros*/
#define SPIIRSP_SSIE_MASK (0x80)
#define SPIIRSP_SSF_MASK (0x10)
#define SPIIRSP_SSLPR_MASK (0x08)
#define SPIIRSP_SSD_MASK (0x04)
#define SPIIRSP_SSPA_LEVEL_MASK (0x00)
#define SPIIRSP_SSPA_R_EDGE_MASK (0x01)
#define SPIIRSP_SSPA_F_EDGE_MASK (0x02)
#define SPIIRSP_SSPA_R_F_EDGE_MASK (0x03)

/*SPIDMACR*/
#define SPISR_TXDMAE_MASK (0x02)
#define SPISR_RXDMAE_MASK (0x01)
#endif /* SPI_REG_H_ */
