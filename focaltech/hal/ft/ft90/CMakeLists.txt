# Makefile - FOCALTECH LIB SDK
#
# Copyright (c) 2025 Focaltech Systems CO.,Ltd
#
# SPDX-License-Identifier: Apache-2.0


if(CONFIG_HAS_FT9002LIB)
  add_subdirectory(ft9002s)
endif()  #CONFIG_HAS_FT9002LIB

if(CONFIG_HAS_FT9001LIB)
  add_subdirectory(ft9001)
endif()  #CONFIG_HAS_FT9001LIB


