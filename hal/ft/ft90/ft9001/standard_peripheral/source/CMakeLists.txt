# Makefile - FOCALTECHLIB SDK
#
# Copyright (c) 2025 Neldtv
#
# SPDX-License-Identifier: Apache-2.0


if(CONFIG_HAS_FT9001LIB)
  zephyr_include_directories(drv/inc)
  zephyr_include_directories(drv)

  if(CONFIG_HAS_FOCALTECH_EXT_LIB)
    zephyr_library_sources(drv/GetSN.c)
    zephyr_library_sources(drv/libft9001.c)
	zephyr_library_sources(drv/trng_drv.c)
  else()
    zephyr_library_sources(drv/cache_drv.c)
    zephyr_library_sources(drv/cpm_drv.c)     
    zephyr_library_sources(drv/GetSN.c)
	zephyr_library_sources(drv/trng_drv.c)
    zephyr_library_sources(drv/libft9001.c)
    zephyr_library_sources_ifdef(CONFIG_HWINFO_FOCALTECH          drv/reset_drv.c)
    zephyr_library_sources_ifdef(CONFIG_USE_FOCALTECH_HAL_USB     drv/usb_drv.c)
    zephyr_library_sources_ifdef(CONFIG_USE_FOCALTECH_HAL_EPORT   drv/eport_drv.c)
    zephyr_library_sources_ifdef(CONFIG_USE_FOCALTECH_HAL_SPI     drv/spi_drv.c)
    zephyr_library_sources_ifdef(CONFIG_USE_FOCALTECH_HAL_UART    drv/uart_drv.c)
    zephyr_library_sources_ifdef(CONFIG_USE_FOCALTECH_HAL_WDT     drv/wdt_drv.c)
    zephyr_library_sources_ifdef(CONFIG_USE_FOCALTECH_HAL_TC      drv/tc_drv.c)	
    zephyr_library_sources_ifdef(CONFIG_PM                      drv/lowpower_drv.c)
  endif() #CONFIG_HAS_FOCALTECH_EXT_LIB


  # 连接生成的库到应用程序
  #find_library(HAL_Algorithm_LIBS Algorithm ${CMAKE_CURRENT_SOURCE_DIR}/lib NO_DEFALUT_PATH)
  #find_library(HAL_FT9001_LIBS  libft9001  ${CMAKE_CURRENT_SOURCE_DIR}/lib NO_DEFALUT_PATH)
  #find_library(HAL_DISJTAG_LIBS  libDisJTAG ${CMAKE_CURRENT_SOURCE_DIR}/lib NO_DEFALUT_PATH)
  #find_library(HAL_GETSN_LIBS  libGetSN_lib ${CMAKE_CURRENT_SOURCE_DIR}/lib NO_DEFALUT_PATH)
  if(CONFIG_HAS_FOCALTECH_EXT_LIB)
    find_library(HAL_FT9001_HAL_LIBS  hal_ft9001 ${CMAKE_CURRENT_SOURCE_DIR}/lib NO_DEFALUT_PATH)
  endif() #CONFIG_HAS_FOCALTECH_EXT_LIB

#  target_link_libraries(app PUBLIC ${HAL_Algorithm_LIBS}  ${HAL_FT9001_LIBS}  ${HAL_DISJTAG_LIBS}  ${HAL_GETSN_LIBS})
  if(CONFIG_HAS_FOCALTECH_EXT_LIB)
    target_link_libraries(app PUBLIC  ${HAL_FT9001_HAL_LIBS})
    # ${HAL_FT9001_LIBS}  ${HAL_DISJTAG_LIBS}  ${HAL_GETSN_LIBS})
  else()
    #target_link_libraries(app PUBLIC  ${HAL_FT9001_LIBS}  ${HAL_DISJTAG_LIBS}  ${HAL_GETSN_LIBS})
  endif() #NO CONFIG_HAS_FOCALTECH_EXT_LIB

endif()  #CONFIG_HAS_FT9001LIB

