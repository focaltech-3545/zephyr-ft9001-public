/**  
  **********************************************************************************
             Copyright(c) 2025 Focaltech Co. Ltd.
                      All Rights Reserved
  **********************************************************************************
  */

#ifndef __SSI_DRV_H
#define __SSI_DRV_H

#include "ssi_reg.h"

typedef struct
{
    uint8_t  SsiId;
    uint8_t  SysDiv;
    uint8_t  IsQpiMode;
    uint16_t StandBaudr;
    uint16_t QuadBaudr;
    uint32_t RxSampleDelay;

    uint8_t    Cmd;
    uint32_t   Value;
    uint8_t   IsMaskInterrupt;
//------------------------------------
    uint8_t    ProgramMode;
    uint32_t   Addr;
    uint16_t   Len;
    uint32_t   Buf;

//--------------------------------

    uint32_t  Delay;
    uint32_t  Timeout;
}SSI_ParaTypeDef;


//w25q128 CMD
#define DUMMY_BYTE				0xa5
#define READ_ID_CMD				0x90  
#define WRITE_EN_CMD			0x06
#define SECT_ERASE_CMD		    0x20
#define CHIP_ERASE_CMD		    0x60
#define GET_SAT1_CMD			0x05
#define READ_CMD				0x03
#define PAGE_PROG_CMD			0x02
#define GET_SAT2_CMD			0x35
#define PROG_STA2_CMD			0x31
#define DUAL_READ_CMD			0x3b
#define QUAD_PROG_CMD			0x32
#define QUAD_READ_CMD			0x6b
#define QPI_READ_CMD			0x0b
#define QPI_ENTER_CMD			0x38
#define QPI_EXIT_CMD			0xFF
#define SET_READ_PARA_CMD	    0xc0

#define     QUAD_PROGRAM           (0x04)
#define     QUAD_DMA_PROGRAM       (0x40)
#define     QUAD_DMA_CH0_PROGRAM   (0x40)
#define     QUAD_DMA_CH1_PROGRAM   (0x41)
#define     QUAD_DMA_CH2_PROGRAM   (0x42)
#define     QUAD_DMA_CH3_PROGRAM   (0x43)
#define     STD_PROGRAM            (0x01)
#define     STD_DMA_PROGRAM       (0x10)
#define     STD_DMA_CH0_PROGRAM   (0x10)
#define     STD_DMA_CH1_PROGRAM   (0x11)
#define     STD_DMA_CH2_PROGRAM   (0x12)
#define     STD_DMA_CH3_PROGRAM   (0x13)

#define     QPI_PROGRAM           (0x08)
#define     QPI_DMA_PROGRAM       (0x80)
#define     QPI_DMA_CH0_PROGRAM   (0x80)
#define     QPI_DMA_CH1_PROGRAM   (0x81)
#define     QPI_DMA_CH2_PROGRAM   (0x82)
#define     QPI_DMA_CH3_PROGRAM   (0x83)

typedef struct
{
    uint8_t (*xip_sys_clk_switch)(SSI_ParaTypeDef *p_ssi_para);

    void (*ssi_open_xip)(SSI_ParaTypeDef *p_ssi_para);
    void (*ssi_close_xip)(SSI_ParaTypeDef *p_ssi_para);

    uint8_t (*xip_enter_qpi)(SSI_ParaTypeDef *p_ssi_para);
    uint8_t (*xip_exit_qpi)(SSI_ParaTypeDef *p_ssi_para);

    uint8_t (*ssi_flash_erase)(SSI_ParaTypeDef *p_ssi_para);
    uint8_t (*ssi_flash_program)(SSI_ParaTypeDef *p_ssi_para);

    uint8_t (*xip_flash_erase)(SSI_ParaTypeDef *p_ssi_para);
    uint8_t (*xip_flash_program)(SSI_ParaTypeDef *p_ssi_para);
}ssi_ops_func_t;

#endif   //__SSI_DRV_H