/**  
  **********************************************************************************
             Copyright(c) 2025 Focaltech Co. Ltd.
                      All Rights Reserved
  **********************************************************************************
  */
  
#ifndef RESET_REG_H_
#define RESET_REG_H_

#include "type.h"

/**
* @brief  RESET
*/
typedef struct
{
    __IO uint32_t RCR;  //0x00
    __IO uint8_t LVDCR; //0x04
    __IO uint8_t HVDCR; //0x05
    __IO uint8_t RTR;   //0x06
    __IO uint8_t RSR;   //0x07
} RESET_TypeDef;



/*** RESET **********************************************/
/*** RCR ************/
#define RESET_RCR_HFDE                      (((uint32_t)1U << 0))
#define RESET_RCR_HFDRE                     (((uint32_t)1U << 1))
#define RESET_RCR_HFDIE                     (((uint32_t)1U << 2))
#define RESET_RCR_HFDF                      (((uint32_t)1U << 3))
#define RESET_RCR_LFDE                      (((uint32_t)1U << 8))
#define RESET_RCR_LFDRE                     (((uint32_t)1U << 9))
#define RESET_RCR_LFDIE                     (((uint32_t)1U << 10))
#define RESET_RCR_LFDF                      (((uint32_t)1U << 11))
#define RESET_RCR_HVDE                      (((uint32_t)1U << 16))
#define RESET_RCR_HVDRE                     (((uint32_t)1U << 17))
#define RESET_RCR_HVDIE                     (((uint32_t)1U << 18))
#define RESET_RCR_HVDF                      (((uint32_t)1U << 19))
#define RESET_RCR_CRWE                      (((uint32_t)1U << 22))
#define RESET_RCR_CRE                       (((uint32_t)1U << 23))
#define RESET_RCR_LVDE                      (((uint32_t)1U << 24))
#define RESET_RCR_LVDRE                     (((uint32_t)1U << 25))
#define RESET_RCR_LVDIE                     (((uint32_t)1U << 26))
#define RESET_RCR_LVDF                      (((uint32_t)1U << 27))
#define RESET_RCR_FRCR_STOUT                (((uint32_t)1U << 30))
#define RESET_RCR_SOFTRST                   (((uint32_t)1U << 31))


#define FT_RSTCAUSE_PIN                     (((uint32_t)1U << 0))
#define FT_RSTCAUSE_SOFTWARE                (((uint32_t)1U << 1))
#define FT_RSTCAUSE_POR                     (((uint32_t)1U << 3))
#define FT_RSTCAUSE_WATCHDOG                (((uint32_t)1U << 4))
#define FT_RSTCAUSE_CLOCK                   (((uint32_t)1U << 11))
#define FT_RSTCAUSE_HARDWARE                (((uint32_t)1U << 12))
#define FT_RSTCAUSE_USER                    (((uint32_t)1U << 16))


#endif /* RESET_REG_H_ */