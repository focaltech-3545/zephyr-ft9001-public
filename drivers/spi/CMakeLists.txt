# Copyright (c) 2025 Focaltech Systems CO.,Ltd
# SPDX-License-Identifier: Apache-2.0

zephyr_library()


zephyr_library_sources_ifdef(CONFIG_SPI_FOCALTECH spi_ft.c)
