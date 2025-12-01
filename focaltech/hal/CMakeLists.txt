# SPDX-License-Identifier: Apache-2.0

zephyr_library()

add_subdirectory_ifdef(CONFIG_HAS_FOCALTECH_HAL ft)
