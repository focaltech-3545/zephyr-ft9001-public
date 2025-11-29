/**  
  **********************************************************************************
             Copyright(c) 2025 Focaltech Co. Ltd.
                      All Rights Reserved
  **********************************************************************************
  */

#ifndef __DMAC_DRV_H__
#define __DMAC_DRV_H__

#include "def_ft.h"
#include "dmac_reg.h"
#include "memmap.h"

/* Maximum data sent in single transfer (Bytes) */
#define DMA_FT_MAX_DATA_ITEMS	0xfff

#define CHANNEL_UMASK(n)         (((1<<n)<<8) | (1<<n))
#define CHANNEL_WRITE_ENABLE(n)  ((1<<n)<<8)
#define CHANNEL_ENABLE(n)        (1<<n)
#define CHANNEL_STAT(n)          (1<<n)

extern void DMA_lliinit(DMA_CHANNEL_REG *DMAch, DMA_CONTROL_REG *DMAcfg, DMA_LLI *dma_lli);
#endif /* __DMAC_DRV_H__ */
