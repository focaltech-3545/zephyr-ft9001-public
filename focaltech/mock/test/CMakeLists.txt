# SPDX-License-Identifier: Apache-2.0

zephyr_library()


zephyr_library_sources_ifdef(CONFIG_SPI_FOCALTECH test_ft.c)
zephyr_library_sources_ifdef(CONFIG_SPI_FOCALTECH ft_flash_test.c)