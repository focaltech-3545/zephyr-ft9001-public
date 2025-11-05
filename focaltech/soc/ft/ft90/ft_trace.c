/*
 * Copyright (c) 2025 Focaltech Systems CO.,Ltd
 Author :zhangpiaoxiang@focaltech-electronics.com
 * SPDX-License-Identifier: Apache-2.0
 */

/*
    how to use this func
    1.add this MICRO in you func
    2.find ft_trace_last_func_info addr in zephyr map,like this (0x20000494   ft_trace_last_func_info)
    3.use JLink cmd :connect->device FT9001 -->SWD
    4.read ft_trace_last_func_info from Jlink :mem addr 16
*/

#include "ft_trace.h"

#ifdef FT_TRACE_EN
#define LINE_MAGIC 0xF5


volatile ft_trace_t ft_trace_last_func_info;


void ft_set_func_info(unsigned int func_addr, unsigned int line)
{
    ft_trace_last_func_info.func_addr = func_addr;
    ft_trace_last_func_info.line = line | (LINE_MAGIC << 24);
}

#endif

#ifdef FT_TRACE_REG_EN

ft_trace_reg_t ft_trace_last_reg_info;

#define FT_TRACE_GET_REG_INFO                                                                                          \
    __asm__ volatile("mov %0,r0" : "=r"(ft_trace_last_reg_info.r0));                                                   \
    __asm__ volatile("mov %0,r1" : "=r"(ft_trace_last_reg_info.r1));                                                   \
    __asm__ volatile("mov %0,r2" : "=r"(ft_trace_last_reg_info.r2));                                                   \
    __asm__ volatile("mov %0,r3" : "=r"(ft_trace_last_reg_info.r3));                                                   \
    __asm__ volatile("mov %0,lr" : "=r"(ft_trace_last_reg_info.lr));                                                   \
    __asm__ volatile("mov %0,sp" : "=r"(ft_trace_last_reg_info.sp));

#endif
