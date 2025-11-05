/**
  **********************************************************************************
             Copyright(c) 2020 Focaltech Co. Ltd.
                      All Rights Reserved
  **********************************************************************************
  * @file    GetSN.c
  * @author  Product application department
  * @version V1.0
  * @date    2021.09.13
  * @brief   GetSNº¯Êý.
  *
  * @version V1.1
  * @date    2021.12.21
  * @brief   
  */

#include "libGetSN.h"

void LIB_SN_Read(unsigned char * buff)
{
    unsigned char i=0;

    for(i=0; i<8; i++)
    {
        buff[i] = *(unsigned char * )(0x08200500+i);
    }
}
