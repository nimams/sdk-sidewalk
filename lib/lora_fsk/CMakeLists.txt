#
# Copyright (c) 2022 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

set(SID_LIB_DIR ${ZEPHYR_BASE}/../sidewalk/lib/lora_fsk)

set(GCC_LIB_DIR ${ZEPHYR_SDK_INSTALL_DIR}/arm-zephyr-eabi/lib/gcc/arm-zephyr-eabi)
find_library(LIB_GCC NAMES gcc libgcc.a HINTS ${GCC_LIB_DIR}/* REQUIRED)

zephyr_library_link_libraries(${SID_LIB_DIR}/libsid_protocol_all.a
	${LIB_GCC}
)

if(DEFINED RADIO)
	if(${RADIO} STREQUAL "LR1110_SRC" OR ${RADIO} STREQUAL "LR1121_SRC")
		add_subdirectory(lr11xx_private)
		zephyr_compile_definitions(LR11xx_SRC)
	elseif(${RADIO} STREQUAL "SX126x_SRC")
		add_subdirectory(sx126x)
	elseif(${RADIO} STREQUAL "LR11xx" OR ${RADIO} STREQUAL "LR1110" OR ${RADIO} STREQUAL "LR1121" )
		set(SMTC_LIB_NAME libsmtc_lr11xx_swdr001_impl.a)
		zephyr_include_directories(lr11xx/include)
		if(DEFINED CONFIG_LOG)
			if(DEFINED CONFIG_LOG_RUNTIME_FILTERING)
				set(RADIO_LIB_NAME libsid_pal_radio_lr11xx_impl_lrf.a)
			else()
				set(RADIO_LIB_NAME libsid_pal_radio_lr11xx_impl.a)
			endif()
		else()
			set(RADIO_LIB_NAME libsid_pal_radio_lr11xx_impl_nolog.a)
		endif()
		zephyr_link_libraries_ifndef(CONFIG_SIDEWALK_PAL_RADIO_SOURCE
			${SID_LIB_DIR}/${SMTC_LIB_NAME}
			${SID_LIB_DIR}/${RADIO_LIB_NAME}
			${SID_LIB_DIR}/${SMTC_LIB_NAME}
		)
		if(${RADIO} STREQUAL "LR1110")
			zephyr_compile_definitions(LR1110 LR11xx)
		elseif(${RADIO} STREQUAL "LR1121")
			zephyr_compile_definitions(LR1121 LR11xx)
		endif()
	else()
		message(SEND_ERROR "RADIO isnt LR1110 or LR11210 or SX126x --> ${RADIO}")
	endif()
else()
	zephyr_link_libraries_ifndef(CONFIG_SIDEWALK_PAL_RADIO_SOURCE ${SID_LIB_DIR}/libsid_pal_radio_sx126x_impl.a)
endif()


