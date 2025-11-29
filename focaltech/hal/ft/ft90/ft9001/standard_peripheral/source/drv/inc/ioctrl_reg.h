/**  
  **********************************************************************************
             Copyright(c) 2025 Focaltech Co. Ltd.
                      All Rights Reserved
  **********************************************************************************
  */
#ifndef __IOCTRL_REG_H__
#define __IOCTRL_REG_H__
#include "type.h"
#include "memmap.h"
typedef struct
{
    volatile uint32_t SPICR;                                                        /**< 0x00  */
    volatile uint32_t USICR;                                                        /**< 0x04  */
    volatile uint32_t I2CCR;                                                        /**< 0x08  */
    volatile uint32_t UARTCR;                                                       /**< 0x0C  */
    volatile uint32_t GINTLCR;                                                      /**< 0x10  */
    volatile uint32_t GINTHCR;                                                      /**< 0x14  */
    volatile uint32_t RESERVED;                                                     /**< 0x18  */
    volatile uint32_t SWAPCR;                                                       /**< 0x1C  */
    volatile uint32_t SPIM1CR;                                                      /**< 0x20  */
    volatile uint32_t SPIM2CR;                                                      /**< 0x24  */
    volatile uint32_t SPIM3CR;                                                      /**< 0x28  */
    volatile uint32_t RESERVED2;                                                    /**< 0x2C  */
    volatile uint32_t RESERVED3;                                                    /**< 0x30  */
    volatile uint32_t RESERVED4;                                                    /**< 0x34  */
    volatile uint32_t WKUPPADCR;                                                    /**< 0x38  */
    volatile uint32_t RESERVED5;                                                    /**< 0x3C  */
    volatile uint32_t RESERVED6;                                                    /**< 0x40  */
    volatile uint32_t RESERVED7;                                                    /**< 0x44  */
    volatile uint32_t PSRAMCR1;                                                     /**< 0x48  */
    volatile uint32_t PSRAMCR2;                                                     /**< 0x4C  */
    volatile uint32_t PSRAMCR3;                                                     /**< 0x50  */
    volatile uint32_t EPORT2CR;                                                     /**< 0x54  */
    volatile uint32_t EPORT3CR;                                                     /**< 0x58  */
    volatile uint32_t EPORT4CR;                                                     /**< 0x5C  */
    volatile uint32_t EPORT5CR;                                                     /**< 0x60  */
    volatile uint32_t EPORT6CR;                                                     /**< 0x64  */
    volatile uint32_t EPORT7CR;                                                     /**< 0x68  */
    volatile uint32_t SWAPCR2;                                                      /**< 0x6C  */
    volatile uint32_t SWAPCR3;                                                      /**< 0x70  */
    volatile uint32_t SWAPCR4;                                                      /**< 0x74  */
    volatile uint32_t SWAPCR5;                                                      /**< 0x78  */
    volatile uint32_t I2SIOCR;                                                      /**< 0x7C  */
    volatile uint32_t SSISLVCR;                                                     /**< 0x80  */
    volatile uint32_t PWMTCR;                                                       /**< 0x84  */
    volatile uint32_t CANCR;                                                        /**< 0x88  */
    volatile uint32_t SPI1CR;                                                       /**< 0x8C  */
    volatile uint32_t SPI2CR;                                                       /**< 0x90  */
    volatile uint32_t SPI3CR;                                                       /**< 0x94  */
} IOCTRL_TypeDef;

#define IOCTRL ((IOCTRL_TypeDef *)(IOCTRL_BASE_ADDR))
#endif


