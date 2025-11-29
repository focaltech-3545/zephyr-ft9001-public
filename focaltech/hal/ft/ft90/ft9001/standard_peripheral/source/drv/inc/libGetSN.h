/**
  **********************************************************************************
             Copyright(c) 2025 Focaltech Co. Ltd.
                      All Rights Reserved
  **********************************************************************************
  * @file    GetSN.h
  * @author  Product application department
  * @version V1.0
  * @date    2021.09.13
  * @brief   GetSN函数.
  *
  * @version V1.1
  * @date    2021.12.21
  * @brief   去掉了ahb3_clk参数.
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _GetSN_H_
#define _GetSN_H_
/**
 * @brief 获取芯片SN，8字节
 *
 * @param[in] *buff   :   SN缓存指针
 * @return
 */
extern void LIB_SN_Read(unsigned char * buff);

#endif /* _GetSN_H_ */

/************************ (C) COPYRIGHT Focaltech *****END OF FILE****/
