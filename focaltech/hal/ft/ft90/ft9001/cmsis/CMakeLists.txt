# Makefile - FOCALTECH LIB SDK
#
# Copyright (c) 2025 Focaltech Systems CO.,Ltd
#
# SPDX-License-Identifier: Apache-2.0


if(CONFIG_HAS_FT9001LIB)
  zephyr_include_directories(include)

  if (CONFIG_HAS_FOCALTECH_EXT_LIB)
  else()
    zephyr_library_sources(source/system_ft9001.c)
  endif()#CONFIG_HAS_FOCALTECH_EXT_LIB

endif()  #CONFIG_HAS_FT9001LIB

