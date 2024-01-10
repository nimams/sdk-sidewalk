#
# Copyright (c) 2022 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

zephyr_include_directories(include)

zephyr_library_sources_ifdef(CONFIG_SIDEWALK src/app_ble_config.c)

if(DEFINED RADIO)
	if(${RADIO} STREQUAL "LR1110" OR ${RADIO} STREQUAL "LR1121" OR ${RADIO} STREQUAL "LR11xx" OR ${RADIO} STREQUAL "LR1110_SRC" OR ${RADIO} STREQUAL "LR1121_SRC")
        message("RADIO is LR1110 / LR1121")
        zephyr_library_sources_ifdef(CONFIG_SIDEWALK_SUBGHZ src/app_subGHz_config_lr11xx.c)
        zephyr_include_directories(../../../lib/lora_fsk/lr11xx/include/semtech)
	elseif(${RADIO} STREQUAL "SX126x" OR ${RADIO} STREQUAL "SX126x_SRC")
        zephyr_library_sources_ifdef(CONFIG_SIDEWALK_SUBGHZ src/app_subGHz_config.c)
    else()
        message(SEND_ERROR "RADIO ${RADIO}")
    endif()
else()
    zephyr_library_sources_ifdef(CONFIG_SIDEWALK_SUBGHZ src/app_subGHz_config.c)
endif()

