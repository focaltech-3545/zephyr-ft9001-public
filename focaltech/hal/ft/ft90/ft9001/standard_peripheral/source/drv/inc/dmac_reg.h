/**  
  **********************************************************************************
             Copyright(c) 2025 Focaltech Co. Ltd.
                      All Rights Reserved
  **********************************************************************************
  */

#ifndef __DMAC_REG_H__
#define __DMAC_REG_H__

//------------------------------------------------------------------
//                        Headers
//------------------------------------------------------------------
#include "type.h"
//------------------------------------------------------------------

typedef struct _DMA_CHANNEL_REG
{
	__IO unsigned int DMA_SADDR;//0x0
	__IO unsigned int DMA_SADDR_H;// 0x04
	__IO unsigned int DMA_DADDR;//0x08
	__IO unsigned int DMA_DADDR_H;//0x0c
	__IO unsigned int DMA_LLP;//0x10
	__IO unsigned int DMA_LLP_H;//0x14
	__IO unsigned int DMA_CTRL;//0x18
	__IO unsigned int DMA_CTRL_HIGH;//0x1c
	__IO unsigned int RESERVED[8];//reserved[8];
	__IO unsigned int DMA_CFG;//0x40
	__IO unsigned int DMA_CFG_HIGH;//0x44

}DMA_CHANNEL_REG;


/****************DMA CONTROL Register define *************************/

typedef struct _DMA_CONTROL_REG
{
	__IO unsigned int DMA_RAWTFR;//0x00//0x2c0
	__IO unsigned int RESERVED1;//0x04//0x2c4
	__IO unsigned int DMA_RAWBLOCK;//0x08////0x2c8
	__IO unsigned int RESERVED2;//0x0c////0x2cc
	__IO unsigned int DMA_RAWSRCTRAN;//0x10//0x2d0
	__IO unsigned int RESERVED3;//0x14//0x2d4
	__IO unsigned int DMA_RAWDSTTRAN;//0x18//0x2d8
	__IO unsigned int RESERVED4;//0x1c//0x2dc
	__IO unsigned int DMA_RAWERR;//0x20//0x2e0
	__IO unsigned int RESERVED5;//0x24//0x2e4
	__IO unsigned int DMA_STATTFR;//0x28//0x2e8
	__IO unsigned int RESERVED6;//0x2c//0x2ec
	__IO unsigned int DMA_STATBLOCK;//0x30//0x2f0
	__IO unsigned int RESERVED7;//0x34//0x2f4
	__IO unsigned int DMA_STATSRC;//0x38//0x2f8
	__IO unsigned int RESERVED8;//0x3c //0x2fc

	__IO unsigned int DMA_STATDST;//0x40//0x300
	__IO unsigned int RESERVED9;//0x44//0x304
	__IO unsigned int DMA_STATERR;//0x48//0x308
	__IO unsigned int RESERVED10;//0x4c//0x30c

	__IO unsigned int DMA_MASKTFR;//0x50//0x310
	__IO unsigned int RESERVED11;//0x54//0x314
	__IO unsigned int DMA_MASKBLOCK;//0x58//0x318
	__IO unsigned int RESERVED12;//0x5c//0x31c

	__IO unsigned int DMA_MASKSRC;//0x60//0x320
	__IO unsigned int RESERVED13;//0x64//0x324
	__IO unsigned int DMA_MASKDST;//0x68//0x328
	__IO unsigned int RESERVED14;//0x6c//0x32c

	__IO unsigned int DMA_MASKERR;//0x70//0x330
	__IO unsigned int RESERVED15;//0x74//0x334
	__IO unsigned int DMA_CLRTFR;//0x78//0x338
	__IO unsigned int RESERVED16;//0x7c//0x33c

	__IO unsigned int DMA_CLRBLOCK;//0x80//0x340
	__IO unsigned int RESERVED17;//0x84//0x344
	__IO unsigned int DMA_CLRSRC;//0x88//0x348
	__IO unsigned int RESERVED18;//0x8c//0x34c

	__IO unsigned int DMA_CLRDST;//0x90//0x350
	__IO unsigned int RESERVED19;//0x94//0x354
	__IO unsigned int DMA_CLRERR;//0x98//0x358
	__IO unsigned int RESERVED20;//0x9c//0x35c

	__IO unsigned int DMA_STATUSINT;//0x100//0x360
	__IO unsigned int RESERVED21;//0x104//0x364
	__IO unsigned int DMA_SRCREQ;//0x108//0x368
	__IO unsigned int RESERVED22;//0x10c//0x36c

	__IO unsigned int DMA_DSTREQ;//0x110//0x370
	__IO unsigned int RESERVED23;//0x114//0x374
	__IO unsigned int DMA_SINGLESRC;//0x118//0x378
	__IO unsigned int RESERVED24;//0x11c//0x37c

	__IO unsigned int DMA_SINGLEDST;//0x120//0x380
	__IO unsigned int RESERVED25;//0x124//0x384
	__IO unsigned int DMA_LASTSRC;//0x128//0x388
	__IO unsigned int RESERVED26;//0x12c//0x38c

	__IO unsigned int DMA_LASTDST;//0x130//0x390
	__IO unsigned int RESERVED27;//0x134//0x394
	__IO unsigned int DMA_CONFIG;//0x138//0x398
	__IO unsigned int RESERVED28;//0x13c//0x39c

	__IO unsigned int DMA_CHEN;//0x140//0x3a0

}DMA_CONTROL_REG;

typedef struct _DMA_LLI {
	volatile unsigned int	src_addr;
	volatile unsigned int	dst_addr;
	volatile unsigned int	next_lli;
	volatile unsigned int	control;
	volatile unsigned int	len;
}DMA_LLI;

#define DMA_CONTROL_REG_BASE    0x2c0
#define DMA_CHANNEL_REG_LEN     0x58

#if 0
#define FT_DMA_SADDR(dma, ch) ((dma) + 0x58UL * (uint32_t)(ch))
#define FT_DMA_DADDR(dma, ch) ((dma) + 0x08UL) + 0x58UL * (uint32_t)(ch))
#define FT_DMA_LLP(dma, ch)   ((dma) + 0x10UL) + 0x58UL * (uint32_t)(ch))
#define FT_DMA_CTRL(dma, ch)  ((dma) + 0x18UL) + 0x58UL * (uint32_t)(ch))
#define FT_DMA_CTRLH(dma, ch) ((dma) + 0x1CUL) + 0x58UL * (uint32_t)(ch))
#define FT_DMA_CFG(dma, ch)   ((dma) + 0x40UL) + 0x58UL * (uint32_t)(ch))
#define FT_DMA_CFGH(dma, ch)  ((dma) + 0x44UL) + 0x58UL * (uint32_t)(ch))
#endif

//config data for DMACCxCTRL
#define DMA_IE    (0x01)
#define DMA_EN    (0x01)

#define DMA_CH0_MASK  0x01
#define DMA_CH2_MASK  0x02
#define DMA_CH3_MASK  0x04
#define DMA_CH4_MASK  0x08

#define DMA_INT_ERR_BIT   0x4
#define DMA_INT_DSTT_BIT  0x3
#define DMA_INT_SRCT_BIT  0x2
#define DMA_INT_BLOCK_BIT 0x1
#define DMA_INT_TRR_BIT   0x0

#define DI        (0x0)//increment
#define SI        (0x0)

#define SRC_HARD	~(0x1<<11)
#define SRC_SOFT	(0x1<<11)
#define DST_HARD	~(0x1<<10)
#define DST_SOFT	(0x1<<10)
#define DD (0x1<<7)
#define SD (0x1<<9)
#define DNC (0x2<<7)
#define SNC (0x2<<9)

#define CH0_MASK	0x0
#define CH0_UMASK	(0x1<<8 | 0x1)
#define CH0_MASK	0x0
#define CH1_UMASK	(0x2<<8 | 0x2)

#define DST_PER_DAC				(0x08<<11)

//config data for DMACCONFIG
#define DMACEN 0x1
//config data for CHENREG
#define CH0_EN 0x1<<8|0x1
#define CH1_EN 0x2<<8|0x2
#define CH2_EN 0x4<<8|0x4
#define CH3_EN 0x8<<8|0x8

//config data for MASKTfr
//#define CH0_MASK 0x1<<8|0x1
//#define CH1_MASK 0x2<<8|0x2
//#define CH2_MASK 0x4<<8|0x4
//#define CH3_MASK 0x8<<8|0x8



//config data for DMACCxCTRL
#define INTEN       0x1

#define DWIDTH_B    0x0<<1
#define DWIDTH_HW   0x1<<1
#define DWIDTH_W    0x2<<1
#define DWIDTH_DW   0x3<<1
#define DWIDTH_DW_MASK   0x3<<1
#define SWIDTH_B    0x0<<4
#define SWIDTH_HW   0x1<<4
#define SWIDTH_W    0x2<<4
#define SWIDTH_DW   0x3<<4
#define SWIDTH_DW_MASK   0x3<<4

#define DIEC        0x0<<7
#define DDEC        0x1<<7
#define DNOCHG      0x2<<7
#define DINC_MASK   0x3<<7

#define SIEC        0x0<<9
#define SDEC        0x1<<9
#define SNOCHG      0x2<<9
#define SINC_MASK   0x3<<9

#define DBSIZE_4 0x1<<11
#define DBSIZE_8 0x2<<11
#define DBSIZE_16 0x3<<11
#define DBSIZE_32 0x4<<11
#define DBSIZE_64 0x5<<11
#define DBSIZE_128 0x6<<11
#define DBSIZE_256 0x7<<11

#define SBSIZE_4 0x1<<14
#define SBSIZE_8 0x2<<14
#define SBSIZE_16 0x3<<14
#define SBSIZE_32 0x4<<14
#define SBSIZE_64 0x5<<14
#define SBSIZE_128 0x6<<14
#define SBSIZE_256 0x7<<14

#define SRC_GATHER_EN 0x1<<17
#define DST_SCATTER_EN 0x1<<18

#define M2M_DMA  0x0<<20
#define M2P_DMA  0x1<<20
#define P2M_DMA  0x2<<20
#define P2P_DMA  0x3<<20
#define P2M_PER  0x4<<20
#define P2P_SRC  0x5<<20
#define M2P_PER  0x6<<20
#define P2P_DST  0x7<<20
#define TT_FC_MASK  0x7<<20


#define LLP_SRC_EN 0x1<<28
#define LLP_DST_EN 0x1<<27

//config data for DMACCxCFG
#define CH_PRIOR0  0x0<<5   //0 is lowest, 3 is highest
#define CH_PRIOR1  0x1<<5
#define CH_PRIOR2  0x2<<5
#define CH_PRIOR3  0x3<<5
#define CH_PRIOR_MASK ~(0x7<<5)

#define CH_SUSP   0x1<<8

#define FIFO_EMPTY 0x1<<9

#define HS_SEL_SRC_HARD 0x0<<11
#define HS_SEL_SRC_SOFT 0x1<<11
#define HS_SEL_DST_HARD 0x0<<10
#define HS_SEL_DST_SOFT 0x1<<10

#define DST_HS_POL_HIGH 0x0<<18
#define DST_HS_POL_LOW  0x1<<18
#define SRC_HS_POL_HIGH 0x0<<19
#define SRC_HS_POL_LOW  0x1<<19

#define ABRST1           0x1<<20
#define ABRST4           0x4<<20
#define ABRST8           0x8<<20
#define ABRST16          0x10<<20
#define ABRST32          0x20<<20

#define RELOAD_SRC      0x1<<30
#define RELOAD_DST      0x1<<31
#define RELOAD_SRC_MASKC     ~(0x1<<30)
#define RELOAD_DST_MASK      ~(0x1<<31)

//config data for DMACCxCFG_HIGH
#define FCMODE0  0x0<<0
#define FCMODE1  0x1<<0

#define FIFOMODE0 0x0<<1
#define FIFOMODE1 0x1<<1

#define PROTCTL1   0x1<<3
#define PROTCTL2   0x2<<3
#define PROTCTL3   0x4<<3

#define SRC_PER0  0x0<<7
#define SRC_PER1  0x1<<7
#define SRC_PER2  0x2<<7
#define SRC_PER3  0x3<<7
#define SRC_PER4  0x4<<7
#define SRC_PER5  0x5<<7

#define DST_PER0  0x0<<11
#define DST_PER1  0x1<<11
#define DST_PER2  0x2<<11
#define DST_PER3  0x3<<11
#define DST_PER4  0x4<<11
#define DST_PER5  0x5<<11

#define SRC_PER_SPI_TX(n)   ((n)<<7)
#define SRC_PER_SPI_RX(n)   ((n)<<7)
#define DST_PER_SPI_TX(n)	((n)<<11)
#define DST_PER_SPI_RX(n)	((n)<<11)
//DMAC1
#define SRC_PER_RAM         (0x0<<7)
#define SRC_PER_SPI1_TX		(0x0<<7)
#define SRC_PER_SPI2_TX		(0x1<<7)
#define SRC_PER_SS2_TX		(0x2<<7)
#define SRC_PER_SPI1_RX		(0x3<<7)
#define SRC_PER_SPI2_RX		(0x4<<7)
#define SRC_PER_SS2_RX		(0x5<<7)
#define SRC_PER_QADC     	(0x6<<7)
#define SRC_PER_MCC1     	(0x7<<7)
#define SRC_PER_MCC2     	(0x8<<7)
#define SRC_PER_MCC3     	(0x9<<7)
#define SRC_PER_SCI1_TX		(0xa<<7)
#define SRC_PER_SCI1_RX		(0xb<<7)
#define SRC_PER_SCI3_TX		(0xc<<7)
#define SRC_PER_SCI3_RX		(0xd<<7)


#define DST_PER_RAM         (0x0<<11)
#define DST_PER_SPI1_TX		(0x0<<11)
#define DST_PER_SPI2_TX		(0x1<<11)
#define DST_PER_SS2_TX		(0x2<<11)
#define DST_PER_SPI1_RX		(0x3<<11)
#define DST_PER_SPI2_RX		(0x4<<11)
#define DST_PER_SS2_RX		(0x5<<11)
#define DST_PER_QADC     	(0x6<<11)
#define DST_PER_MCC1     	(0x7<<11)
#define DST_PER_MCC2     	(0x8<<11)
#define DST_PER_MCC3     	(0x9<<11)
#define DST_PER_SCI1_TX		(0xa<<11)
#define DST_PER_SCI1_RX		(0xb<<11)
#define DST_PER_SCI3_TX		(0xc<<11)
#define DST_PER_SCI3_RX		(0xd<<11)

//DMAC2
#define SRC_PER_SCI2_RX		(0x0<<7)
#define SRC_PER_SCI2_TX		(0x1<<7)
#define SRC_PER_SSI1_RX		(0x2<<7)
#define SRC_PER_SSI1_TX		(0x3<<7)
#define SRC_PER_SPI3_RX		(0x4<<7)
#define SRC_PER_SPI3_TX		(0x5<<7) 

#define DST_PER_SCI2_RX		(0x0<<11)
#define DST_PER_SCI2_TX		(0x1<<11)
#define DST_PER_SSI1_RX		(0x2<<11)
#define DST_PER_SSI1_TX		(0x3<<11)
#define DST_PER_SPI3_RX		(0x4<<11)
#define DST_PER_SPI3_TX		(0x5<<11)
#define DST_PER_DAC_DMA_REQUEST   (0x8<<11)
#define DST_PER_SDDC_DMA_REQ	  (0xc<<11)
////////////////////////////////////////////////////



#endif /* __DMAC_REG_H__ */
