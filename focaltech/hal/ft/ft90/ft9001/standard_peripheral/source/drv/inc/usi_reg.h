/**  
  **********************************************************************************
             Copyright(c) 2025 Focaltech Co. Ltd.
                      All Rights Reserved
  **********************************************************************************
  */
#ifndef __USI_REG_H__
#define __USI_REG_H__

#include "memmap.h"
/**
* @brief USI 模块寄存器定义
*/
typedef struct
{
    __IO uint8_t BDR;                                                               /**< 00*/
    __IO uint8_t CR1;                                                               /**< 01*/
    __IO uint8_t CR2;                                                               /**< 02*/
    __IO uint8_t SR;                                                                /**< 03*/
    __IO uint8_t IER;                                                               /**< 04*/
    __IO uint8_t DR;                                                                /**< 05*/
    __IO uint8_t WTRH;                                                              /**< 06*/
    __IO uint8_t WTRM;                                                              /**< 07*/
    __IO uint8_t WTRL;                                                              /**< 08*/
    __IO uint8_t GTRH;                                                              /**< 09*/
    __IO uint8_t GTRL;                                                              /**< 0A*/
    __IO uint8_t CSR;                                                               /**< 0B*/
    __IO uint8_t PCR;                                                               /**< 0C*/
    __IO uint8_t PDR;                                                               /**< 0D*/
    __IO uint8_t DDR;                                                               /**< 0E*/
    __IO uint8_t FIFOINTCON;                                                        /**< 0F*/
    __IO uint8_t CRCH;                                                              /**< 10*/
    __IO uint8_t CRCL;                                                              /**< 11*/
    __IO uint8_t CDCR;                                                              /**< 12*/
    __IO uint8_t EBLCR;                                                             /**< 13*/
} USI_TypeDef;

#define USI1                                ((USI_TypeDef *)USI1_BASE_ADDR)
#define USI2                                ((USI_TypeDef *)USI2_BASE_ADDR)

#endif //__USI_REG_H__