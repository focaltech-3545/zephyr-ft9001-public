/*
 * Copyright (c) 2026 Focaltech
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stddef.h>
#include <stdio.h>
#include <zephyr/device.h>
#include <zephyr/drivers/otp.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/toolchain.h>

#include "otp_ft.h"

#define OTP_USER_ADDR(str, num) str##num
#define DT_DRV_COMPAT ft_ft90_otp

LOG_MODULE_REGISTER(otp_ft);

#define BSEC_WORD_SIZE 4

static K_MUTEX_DEFINE(lock);

struct ft_otp_config {
  uint8_t*base;
};

static inline void otp_ft_lock(void) {
  if (!k_is_pre_kernel()) {
    (void)k_mutex_lock(&lock, K_FOREVER);
  }
}

static inline void otp_ft_unlock(void) {
  if (!k_is_pre_kernel()) {
    (void)k_mutex_unlock(&lock);
  }
}

#if defined(CONFIG_OTP_PROGRAM)

static int ft_opt_program(volatile uint32_t *otp_addr, const void *buf) {
  unsigned int reg_tmp, write_data;
  unsigned int ahb3_clk = ft_get_ahb3_clk();


  if (!buf) {
    LOG_ERR("buf is null");
    return -1;
  }

  write_data = *(uint32_t *)buf;

  // OPT clock
  // to calculate related parameter according to otp module clock
  OTP->OTPTIMBASE =
      ((ahb3_clk / 1000) + (1000 - 1)) / 1000; // OTP->OTPTIMBASE = clk_cnt_1us;

  if (0xFFFFFFFF != *(volatile unsigned int *)(otp_addr)) {
    LOG_ERR("otp is not empty");
    return -1;
  }

  // enable otp write
  OTP->OTPAPR = 0x9786ac03; // otp write enable.

  // write data to otp
  __asm("CPSID I");

  reg_tmp = OTP->OTPCR | 0x00B7A500;

  OTP->OTPCR = reg_tmp & ~0x08000000; // switch to main cell.

  OTP->OTPCMD = 0x00008003;
  (*(volatile unsigned int *)(otp_addr)) = write_data;
  while (!(OTP->OTPSTAT & 0x00008000))
    ;

  OTP->OTPCR = reg_tmp | 0x08000000; // switch to redundancy cell.

  OTP->OTPCMD = 0x00008003;
  (*(volatile unsigned int *)(otp_addr)) = write_data;
  while (!(OTP->OTPSTAT & 0x00008000))
    ;

  OTP->OTPCR = reg_tmp & ~0x08000000; // switch to main cell.

  __asm("CPSIE I");

  OTP->OTPAPR = 0x9786ac01; // otp write disable.

  // OPT data double check
  if (write_data == *(volatile unsigned int *)(otp_addr)) {
    return 0;
  } else {
    return -1;
  }
}

static int otp_ft_program(const struct device *dev, off_t offset,
                          const void *buf, size_t len) {

  int ret = 0, i;

  const struct ft_otp_config *config = dev->config;
  uint8_t* base = config->base;


  if (len % sizeof(uint32_t) || offset % sizeof(uint32_t)) {
    LOG_ERR("notice! otp addr algin 4");
    return -1;
  }

  if (!buf) {
    LOG_ERR("buf is null");
    return -1;
  }

  if (offset + len > sizeof((struct ft_otp_layout *)0)->user_data) {
    LOG_ERR("otp out range");
    return -2;
  }

  volatile uint32_t *otp_addr =
      (volatile unsigned int *)(base + OTP_CONFIG_OFFSET + offset +
                                offsetof(struct ft_otp_layout,
                                  user_data));
  printk("otp_addr=%p,offset=%ld\n", otp_addr, offset);

  otp_ft_lock();
  for (i = 0; i < len; i += sizeof(uint32_t)) {
    ret = ft_opt_program(otp_addr, (uint8_t *)buf + i);
    otp_addr++;
    if (ret) {
      LOG_ERR("otp program err");
      break;
    }
  }

  otp_ft_unlock();

  return ret;
}
#endif /* CONFIG_OTP_PROGRAM */



static int otp_ft_read(const struct device *dev, off_t offset, void *buf,
                       size_t len) {
  printk("%s enter,offset=%lx,len=%ld\n", __func__, offset, offset);

  const struct ft_otp_config *config = dev->config;
  uint8_t* base =(uint8_t*) config->base;

  printk("r_otp_addr=%p,OTP=%p\n", base,OTP);

  volatile uint32_t *otp_addr =
      (volatile unsigned int *)(base + OTP_CONFIG_OFFSET + offset +
                                offsetof(struct ft_otp_layout,
                                  user_data));

  uint32_t *out = (uint32_t *)buf;

  int i;

  if (len % sizeof(uint32_t) || offset % sizeof(uint32_t)) {
    LOG_ERR("notice! otp addr algin 4");
    return -1;
  }

  if (!buf) {
    LOG_ERR("buf is null");
    return -1;
  }

  if (offset + len > sizeof((struct ft_otp_layout *)0)->user_data) {
    LOG_ERR("otp out range");
    return -2;
  }

  memset(buf, 0, len);

  otp_ft_lock();
  unsigned int ahb3_clk = ft_get_ahb3_clk();

  // OPT clock  to calculate related parameter according to otp module clock
  OTP->OTPTIMBASE =
      ((ahb3_clk / 1000) + (1000 - 1)) / 1000; // OTP->OTPTIMBASE = clk_cnt_1us;

  for (i = 0; i < len / sizeof(uint32_t); i++) {

    out[i] = otp_addr[i];
  }

  otp_ft_unlock();

  return 0;
}

static DEVICE_API(otp, otp_ft_api) = {
#if defined(CONFIG_OTP_PROGRAM)
    .program = otp_ft_program,
#endif
    .read = otp_ft_read,

};

#define OTP_FT_DEVICE_DEFINE(n)                                                                                 \
  static const struct ft_otp_config ft_otp_config_##n = {                                                       \
        .base = (uint8_t *)DT_INST_REG_ADDR(n),                                                             \
                                                                            };                                  \
  DEVICE_DT_INST_DEFINE(n, NULL, NULL, NULL, &ft_otp_config_##n, PRE_KERNEL_1,                                  \
                        CONFIG_OTP_INIT_PRIORITY, &otp_ft_api);

DT_INST_FOREACH_STATUS_OKAY(OTP_FT_DEVICE_DEFINE)
