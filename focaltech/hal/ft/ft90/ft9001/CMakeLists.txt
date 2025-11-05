# Makefile - FOCALTECH LIB SDK
#
# Copyright (c) 2025 Focaltech Systems CO.,Ltd
#
# SPDX-License-Identifier: Apache-2.0


if(CONFIG_HAS_FT9001LIB)
  add_subdirectory(cmsis)
  add_subdirectory(standard_peripheral)
endif()  #CONFIG_HAS_FT9001LIB




