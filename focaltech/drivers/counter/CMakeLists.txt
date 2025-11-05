# SPDX-License-Identifier: Apache-2.0
zephyr_syscall_header(${ZEPHYR_BASE}/include/zephyr/drivers/counter.h)

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_COUNTER_TIMER_FOCALTECH          counter_ft_timer.c)
