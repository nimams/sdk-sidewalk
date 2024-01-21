/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef BOARD_EVENTS_H
#define BOARD_EVENTS_H

#include <application_thread.h>

/**
 * @brief Application handler for sending hello message
 *
 * @param application_ctx
 */
void button_event_send_hello(app_ctx_t *application_ctx);

/**
 * @brief Application handler for sending battery level
 *
 * @param application_ctx
 */
void button_event_set_battery(app_ctx_t *application_ctx);

/**
 * @brief Application handler for entering in DFU mode
 *
 * @param application_ctx
 */
void button_event_DFU(app_ctx_t *application_ctx);

/**
 * @brief Applicaiton handler for requesting factory reset
 *
 * @param application_ctx
 */
void button_event_factory_reset(app_ctx_t *application_ctx);

/**
 * @brief Application handler for reading profile
 *
 * @param application_ctx
 */
void button_event_get_profile(app_ctx_t *application_ctx);

/**
 * @brief Application handler for setting profile
 *
 * @param application_ctx
 */
void button_event_set_ptofile(app_ctx_t *application_ctx);

/**
 * @brief Manual start and stop of sidewalk
 *
 * @note Custom function added
 *
 * @param application_ctx
 */
void button_stop_sidewalk_custom (app_ctx_t *application_ctx);
void button_start_sidewalk_custom (app_ctx_t *application_ctx);
int gnss_scan_timer_custom_set(unsigned sec);

#endif /* BOARD_EVENTS_H */
