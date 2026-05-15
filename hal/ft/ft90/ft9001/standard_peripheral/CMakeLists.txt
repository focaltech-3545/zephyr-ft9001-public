# Makefile - FOCALTECH LIB SDK
#
# Copyright (c) 2025 Focaltech Systems CO.,Ltd
#
# SPDX-License-Identifier: Apache-2.0


if(CONFIG_HAS_FT9001LIB)
  zephyr_include_directories(include)
  add_subdirectory(source)

endif()  #CONFIG_HAS_FT9001LIB

