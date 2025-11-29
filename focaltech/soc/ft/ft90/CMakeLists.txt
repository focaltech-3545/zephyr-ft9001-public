# Copyright (c) 2025, Focaltech Systems CO.,Ltd
# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(${ZEPHYR_BASE}/drivers)
zephyr_sources(soc.c)

#zephyr_sources(ft_trace.c)
zephyr_sources(ft_sys.c)

zephyr_include_directories(.)
zephyr_library_sources_ifdef(CONFIG_PM power.c)

if(CONFIG_HAS_FT9001LIB)

zephyr_sources(xip_clock_switch.c)
zephyr_code_relocate(FILES xip_clock_switch.c LOCATION RAM)

endif()  #CONFIG_HAS_FT9001LIB


set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/include/zephyr/arch/arm/cortex_m/scripts/linker.ld CACHE INTERNAL "")
