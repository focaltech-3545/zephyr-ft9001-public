/**
  **********************************************************************************
             Copyright(c) 2025 Focaltech Co. Ltd.
                      All Rights Reserved
  **********************************************************************************
  */

#include "reset_drv.h"
#include "memmap.h"
#include "type.h"

uint32_t FT_GetResetStatus()
{
    RESET_TypeDef *RESETx = (RESET_TypeDef *)(RESET_BASE_ADDR);
    uint32_t flags = 0;
    uint8_t reset_status;

    reset_status = RESETx->RSR;

    if (reset_status & (0x01 << 1))
        flags |= FT_RSTCAUSE_HARDWARE;
    if (reset_status & (0x01 << 6))
        flags |= FT_RSTCAUSE_HARDWARE;
    if (reset_status & (0x01 << 7))
        flags |= FT_RSTCAUSE_HARDWARE;
    if (reset_status & (0x01 << 2))
        flags |= FT_RSTCAUSE_CLOCK;
    if (reset_status & (0x01 << 3))
        flags |= FT_RSTCAUSE_POR;
    if (reset_status & (0x01 << 4))
        flags |= FT_RSTCAUSE_WATCHDOG;
    if (reset_status & (0x01 << 5))
        flags |= FT_RSTCAUSE_SOFTWARE;

    return flags;
}
