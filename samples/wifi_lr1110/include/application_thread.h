/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef APPLICATION_THREAD_H
#define APPLICATION_THREAD_H

#include <sid_api.h>
#include "fragment.h"

#ifdef CONFIG_SIDEWALK_LINK_MASK_BLE
#define BUILT_IN_LM             (uint32_t)(SID_LINK_TYPE_1)
#elif  CONFIG_SIDEWALK_LINK_MASK_FSK
#define BUILT_IN_LM             (uint32_t)(SID_LINK_TYPE_2)
#elif CONFIG_SIDEWALK_LINK_MASK_LORA
#define BUILT_IN_LM             (uint32_t)(SID_LINK_TYPE_3)
#else
#error "Not defined Sidewalk link mask!!"
#endif

#define NB_MAX_SV		8	/* GNSS */
#define DEFAULT_AUTO_SCAN_INTERVAL		30		/* seconds */

#define FRAGMENT_TYPE_GNSS         0	// to cloud port 198
#define FRAGMENT_TYPE_WIFI         1	// to cloud port 197
#define FRAGMENT_TYPE_MODEM_MSG    2	// to cloud port 199
#define FRAGMENT_TYPE_GNSS_NG      3	// to cloud port 192

typedef struct application_context {
	struct sid_event_callbacks event_callbacks;
	struct sid_config config;
	struct sid_handle *handle;
	bool connection_request;
	fragment_t frag;
} app_ctx_t;

typedef enum application_events {
	SIDEWALK_EVENT,

	BUTTON_EVENT_SEND_HELLO,
	BUTTON_EVENT_SET_BATTERY_LEVEL,
	BUTTON_EVENT_FACTORY_RESET,
	BUTTON_EVENT_FSK_MODE,
	BUTTON_EVENT_LORA_MODE,
	BUTTON_EVENT_GET_DEVICE_PROFILE,
	BUTTON_EVENT_SET_DEVICE_PROFILE,
	#if  defined(CONFIG_SIDEWALK_DFU)
	BUTTON_EVENT_NORDIC_DFU,
	#endif
	EVENT_WIFI_SCAN,
	EVENT_TO_FSK,
	EVENT_TO_LORA,
	EVENT_SCAN_RESULT_SEND,
	EVENT_SCAN_INTERVAL_INCR,
    EVENT_SID_INIT,
} app_event_t;

/**
 * @brief Send Event to application thread
 *
 * @param event event to send
 */
void app_event_send(app_event_t event);

/**
 * @brief Initialize and start application thread
 *
 * @param ctx application context
 *
 * @return SID_ERROR_NULL_POINTER in case of invalid ctx
 */
sid_error_t app_thread_init(app_ctx_t *ctx);

extern unsigned auto_scan_interval;
int gnss_scan_timer_set(unsigned sec);
unsigned gnss_scan_timer_get(void);
int scan_wifi(app_ctx_t *app_ctx);

#endif /* APPLICATION_THREAD_H */
