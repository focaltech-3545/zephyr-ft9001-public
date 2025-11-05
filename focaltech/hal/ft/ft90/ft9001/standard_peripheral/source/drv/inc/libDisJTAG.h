/**
  **********************************************************************************
             Copyright(c) 2020 Focaltech Systems CO.,Ltd
                      All Rights Reserved
  **********************************************************************************
  * @file    GetSN.h
  * @author  Product application department
  * @version V1.0
  * @date    2021.09.13
  * @brief   GetSN函数.
  *
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _DISABLESWD_H_
#define _DISABLESWD_H_

typedef enum
{
    JTAG_DIS_SUCCESS = 0,
    JTAG_AREA_ERR,
    JTAG_STATUS_ERR,
    JTAG_OPERATE_ERR
}JtagDisStatus;

/**
 * @brief 关闭SWD
 *
 * @param[in] ahb3_clk:   AHB3时钟
 * @return 0成功，其他失败
 */
extern unsigned char Lock_DisJtag_Opt(unsigned int ahb3_clk);

#endif /* _DISABLESWD_H_ */

/************************ (C) COPYRIGHT Focaltech *****END OF FILE****/
