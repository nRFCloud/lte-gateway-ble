#
# Copyright (c) 2018 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
#

cmake_minimum_required(VERSION 3.13.1)

set(mcuboot_CONF_FILE ${CMAKE_CURRENT_LIST_DIR}/mcuboot.conf)

set(mcuboot_DTC_OVERLAY_FILE ${CMAKE_CURRENT_LIST_DIR}/apricity_gateway_nrf52840.overlay)

set(BOARD_ROOT ${CMAKE_CURRENT_LIST_DIR})
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(lte-gateway-ble)

target_sources(app PRIVATE src/main.c)
target_sources(app PRIVATE src/usb_uart_bridge.c)
zephyr_include_directories(src)
