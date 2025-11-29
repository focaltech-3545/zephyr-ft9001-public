# Copyright (c) 2025 Focaltech Systems CO.,Ltd
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_UDC_FOCALTECH udc_ft.c)
#zephyr_library_sources_ifdef(CONFIG_UDC_FOCALTECH udc_ft_hw_thread.c)
#zephyr_library_sources_ifdef(CONFIG_UDC_FOCALTECH udc_ft_hw_handler.c)
#zephyr_library_sources_ifdef(CONFIG_UDC_FOCALTECH udc_ft_handler_ep0.c)

