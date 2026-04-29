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

struct ft_otp_layout {
  uint8_t reserved_data[400];
  uint8_t user_data[100];   /* for user use */
  uint8_t reserved_data2[12];
} __attribute__((packed));

#define OTP_CONFIG_OFFSET (0x100)
#define OTP_ACTIVE_MAGIC 0x55aa55aa
#define OTP_WRITE_EN_KEY 0x9786AC03
#define OTP_WRITE_DIS_KEY 0x9786AC01
#define OTPCR_SWD_FEATURE_MASK 0x00B7A500
#define OTPCR_REDUNDANCY_BIT 0x08000000
#define OTP_CMD_WRITE_CONFIG 0x00008003
#define OTPSTAT_OP_BUSY_BIT 0x00008000

uint32_t ft_get_ahb3_clk();

#endif