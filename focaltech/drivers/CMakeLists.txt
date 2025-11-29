# Copyright (c) 2025 Focaltech Systems CO.,Ltd
# SPDX-License-Identifier: Apache-2.0

add_subdirectory(usb)

add_subdirectory_ifdef(CONFIG_CLOCK_CONTROL clock_control)
add_subdirectory_ifdef(CONFIG_RESET reset)
add_subdirectory_ifdef(CONFIG_GPIO gpio)
add_subdirectory_ifdef(CONFIG_PINCTRL pinctrl)
add_subdirectory_ifdef(CONFIG_SERIAL_HAS_DRIVER serial)
add_subdirectory_ifdef(CONFIG_SPI spi)
add_subdirectory_ifdef(CONFIG_FLASH_FT_FT90 flash)
add_subdirectory_ifdef(CONFIG_HWINFO hwinfo)
add_subdirectory_ifdef(CONFIG_ENTROPY_GENERATOR entropy)
#add_subdirectory_ifdef(CONFIG_WATCHDOG watchdog)
#add_subdirectory_ifdef(CONFIG_COUNTER counter)
#add_subdirectory_ifdef(CONFIG_DMA dma)
#add_subdirectory_ifdef(CONFIG_FLASH flash)
