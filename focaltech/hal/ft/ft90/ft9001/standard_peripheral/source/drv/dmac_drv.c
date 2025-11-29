/**
  **********************************************************************************
             Copyright(c) 2025 Focaltech Co. Ltd.
                      All Rights Reserved
  **********************************************************************************
  */

#include "dmac_drv.h"
#include "memmap.h"

void DMA_lliinit(DMA_CHANNEL_REG *DMAch, DMA_CONTROL_REG *DMAcfg, DMA_LLI *dma_lli)
{

    DMAch->DMA_SADDR = dma_lli->src_addr;
    DMAch->DMA_DADDR = dma_lli->dst_addr;
    DMAch->DMA_LLP = (unsigned int)dma_lli;
    DMAch->DMA_CTRL = dma_lli->control;
    DMAch->DMA_CTRL_HIGH = dma_lli->len;
}
