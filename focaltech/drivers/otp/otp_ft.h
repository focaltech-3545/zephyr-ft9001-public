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
  uint32_t code_valid_control_word; /* 0x3AEC3721 for valid code */
  uint8_t reserved_04_18[0x14];
  uint32_t config_page_verification_enable; /* 0x904E3C21 for valid code */
  uint8_t reserved_data1[0x4];
  uint32_t code_start_address;
  uint32_t code_length;
  uint8_t code_reserved[16];
  uint8_t reserved_data2[8];
  uint8_t code_sig[256];        /* RSA2048 code signature */
  uint8_t config_page_hash[32]; /* SHA256 of the config page */
  uint8_t spi_config[48];
  uint8_t reserved_data3[100];   /* user data */
  uint32_t config_page_location; /* 0x301821EF for OTP */
  uint8_t reserved_data4[0x8];
} __attribute__((packed));

#define OTP_DATA_ADDR (0x08200000)
#define OTP_CONFIG_OFFSET (0x100)
#define OTP_BASE_ADDR (0x40003000)
#define OTP_ACTIVE_MAGIC 0x55aa55aa

#define OTP ((OTP_TypeDef *)OTP_BASE_ADDR)

uint32_t ft_get_ahb3_clk();

#endif