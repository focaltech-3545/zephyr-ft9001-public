#ifndef _OTP_FT_H_
#define _OTP_FT_H_

#define __IO volatile

#include <stddef.h>
#include <stdio.h>

typedef struct {
  __IO uint32_t OTPCR;      /**< 00 */
  __IO uint32_t OTPAPR;     /**< 04 */
  __IO uint32_t OTPSTAT;    /**< 08 */
  __IO uint32_t OTPINTM;    /**< 0C */
  __IO uint32_t OTPCMD;     /**< 10 */
  __IO uint32_t OTPTIMBASE; /**< 14 */
  __IO uint32_t OTPTIMCFG;  /**< 18 */
  __IO uint32_t OTPPTIMER;  /**< 1C */
} OTP_TypeDef;

struct ft_config_page {
  uint8_t reserved_data[400];
  uint8_t reserved_data2[100];   /* otp data */
  uint8_t reserved_data3[12];
} __attribute__((packed));

#define OTP_CONFIG_OFFSET (0x100)
#define OTP_BASE_ADDR (DT_REG_ADDR(DT_NODELABEL(efm)))
#define OTP_ACTIVE_MAGIC 0x55aa55aa

#define OTP ((OTP_TypeDef *)OTP_BASE_ADDR)

uint32_t ft_get_ahb3_clk();

#endif