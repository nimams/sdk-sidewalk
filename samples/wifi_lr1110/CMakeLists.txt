#
# Copyright (c) 2022 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

cmake_minimum_required(VERSION 3.20.0)

include(../common/bootloader_version.cmake)

set(mcuboot_KCONFIG_ROOT $ENV{ZEPHYR_BASE}/../sidewalk/samples/template_subghz/child_image/mcuboot/Kconfig.root)
set(hci_rpmsg_KCONFIG_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/child_image/hci_rpmsg/Kconfig.root)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(sidewalk_template_subghz)

zephyr_include_directories(include)
	# ../../tests/validation/timer/src
# )

target_sources(app PRIVATE
  src/main.c
  src/application_thread.c
  src/board_events.c
  src/sidewalk_callbacks.c
  src/wifi.c
  src/fragment.c
)

if(DEFINED WIFI_DEBUG)
	zephyr_compile_definitions(WIFI_DEBUG)
endif()

if(DEFINED LR11xx)
	zephyr_compile_definitions(LR11xx)
endif()

