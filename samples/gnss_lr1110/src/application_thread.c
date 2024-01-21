/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */
#include <sid_api.h>
#include <sid_error.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <board_events.h>
#include <pal_init.h>
#if defined(CONFIG_SIDEWALK_CLI)
#include <sid_shell.h>
#endif
#if defined(CONFIG_LR1110_CLI)
#include <lr1110_shell.h>
#endif

#include <application_thread.h>
#include <state_notifier.h>
#ifdef ALMANAC_UPDATE
#include <almanac_update.h>
#endif /* ALMANAC_UPDATE */

static void button_event_toggle_sid_custom(app_ctx_t *application_ctx);
static void button_event_toggle_gnss_scan_custom(app_ctx_t *application_ctx);

static struct k_thread application_thread;

K_THREAD_STACK_DEFINE(application_thread_stack, CONFIG_SIDEWALK_THREAD_STACK_SIZE);
K_MSGQ_DEFINE(application_thread_msgq, sizeof(app_event_t), CONFIG_SIDEWALK_THREAD_QUEUE_SIZE, 4);

LOG_MODULE_REGISTER(application, CONFIG_SIDEWALK_LOG_LEVEL);

static void sidewalk_app_entry(void *ctx, void *unused, void *unused2)
{
	ARG_UNUSED(unused);
	ARG_UNUSED(unused2);
	app_ctx_t *application_ctx = (app_ctx_t *)ctx;

	if (application_pal_init()) {
		LOG_ERR("Failed to initialze PAL layer for sidewalk applicaiton.");
		application_state_error(&global_state_notifier, true);
		return;
	}

	sid_error_t err = sid_init(&application_ctx->config, &application_ctx->handle);

	switch (err) {
	case SID_ERROR_NONE:
		break;
	case SID_ERROR_ALREADY_INITIALIZED:
		LOG_WRN("Sidewalk already initialized!");
		break;
	default:
		LOG_ERR("Unknown error (%d) during sidewalk initialization!", err);
		application_state_error(&global_state_notifier, true);
		return;
	}

#ifdef ALMANAC_UPDATE
	almanac_update();
#endif /* ALMANAC_UPDATE */

	err = sid_start(application_ctx->handle, BUILT_IN_LM);
	if (err) {
		LOG_ERR("Unknown error (%d) during sidewalk start!", err);
		application_state_error(&global_state_notifier, true);
		return;
	}
#if defined(CONFIG_SIDEWALK_CLI)
	CLI_init(application_ctx->handle, &application_ctx->config);
#endif
#if defined(CONFIG_LR1110_CLI)
	LR1110_CLI_init(application_ctx->handle);
#endif
	application_state_connected(&global_state_notifier, false);
	application_state_working(&global_state_notifier, true);
	while (true) {
		app_event_t event = SIDEWALK_EVENT;

		if (!k_msgq_get(&application_thread_msgq, &event, K_FOREVER)) {
			switch (event) {
			case SIDEWALK_EVENT:
				err = sid_process(application_ctx->handle);
				if (err) {
					LOG_WRN("sid_process returned %d", err);
				}
				break;
			case BUTTON_EVENT_SEND_HELLO:
				button_event_send_hello(application_ctx);
				break;
			case BUTTON_EVENT_SET_BATTERY_LEVEL:
				button_event_set_battery(application_ctx);
				break;
			case BUTTON_EVENT_FACTORY_RESET:
				button_event_factory_reset(application_ctx);
				break;
			case BUTTON_EVENT_GET_DEVICE_PROFILE:
				button_event_get_profile(application_ctx);
				break;
			case BUTTON_EVENT_SET_DEVICE_PROFILE:
				button_event_set_ptofile(application_ctx);
				break;
			case EVENT_GNSS_SCAN_START:
				start_gnss_scan(application_ctx);
				break;
			case EVENT_GNSS_SCAN_SEND:
				send_scan_result(application_ctx);
				break;
         case EVENT_WIFI_SCAN_START:
            scan_wifi(application_ctx);
            break;
         case EVENT_WIFI_SCAN_SEND:
            send_scan_result(application_ctx);
            break;
         case EVENT_TOGGLE_SID_CUSTOM:
            button_event_toggle_sid_custom(application_ctx);
            break;
         case EVENT_TOGGLE_GNSS_SCAN_CUSTOM:
            button_event_toggle_gnss_scan_custom(application_ctx);
            break;

#if defined(CONFIG_SIDEWALK_DFU_SERVICE_BLE)
			case BUTTON_EVENT_NORDIC_DFU:
				button_event_DFU(application_ctx);
				break;
#endif

			default:
				LOG_ERR("Invalid Event received!");
			}
		}
	}
	application_state_working(&global_state_notifier, false);
}

static void button_event_toggle_sid_custom(app_ctx_t *application_ctx)
{
   struct sid_status status = { .state = SID_STATE_NOT_READY };
	sid_error_t err;
   bool sidewalk_ready;

   err = sid_get_status(application_ctx->handle, &status);
	switch (err) {
      case SID_ERROR_NONE:
         break;
      case SID_ERROR_INVALID_ARGS:
         LOG_ERR("Sidewalk library is not initialzied!");
         return;
      default:
         LOG_ERR("Unknown error during sid_get_status() -> %d", err);
         return;
	}

	if (status.state != SID_STATE_READY &&
      status.state != SID_STATE_NOT_READY) {
		LOG_ERR("Sidewalk Status is invalid!, got %d",
			status.state);
		return;
	}

   if (status.state == SID_STATE_READY) {
      sidewalk_ready = true;
   } else {
      sidewalk_ready = false;
   }

   if (!sidewalk_ready) {
      LOG_INF("starting Sidewalk");
      button_start_sidewalk_custom(application_ctx);
   } else {
      LOG_INF("stopping Sidewalk");
      button_stop_sidewalk_custom(application_ctx);
   }
}

void button_event_toggle_gnss_scan_custom(app_ctx_t *application_ctx)
{
   struct sid_status status = { .state = SID_STATE_NOT_READY };
   sid_error_t err;
   static bool started = false;

   err = sid_get_status(application_ctx->handle, &status);
   switch (err) {
      case SID_ERROR_NONE:
         break;
      case SID_ERROR_INVALID_ARGS:
         LOG_ERR("Sidewalk library is not initialzied!");
         return;
      default:
         LOG_ERR("Unknown error during sid_get_status() -> %d", err);
         return;
   }

   if (status.state != SID_STATE_READY) {
      LOG_ERR("Sidewalk Status is not ready!, got %d",
         status.state);
      gnss_scan_timer_custom_set(0);
      started = false;
      return;
   }

   if (!started) {
      LOG_INF("starting GNSS scan");
      gnss_scan_timer_custom_set(30);
      started = true;
   } else {
      LOG_INF("stopping GNSS scan");
      gnss_scan_timer_custom_set(0);
      started = false;
   }
}

void app_event_send(app_event_t event)
{
	int ret = k_msgq_put(&application_thread_msgq, (void *)&event,
			     k_is_in_isr() ? K_NO_WAIT : K_FOREVER);

	if (ret) {
		LOG_ERR("Failed to send event to application thread. err: %d", ret);
	} else {
      LOG_INF("Sent event to application thread. event: %d", event);
   }
}

sid_error_t app_thread_init(app_ctx_t *ctx)
{
	if (!ctx) {
		return SID_ERROR_NULL_POINTER;
	}
	(void)k_thread_create(&application_thread, application_thread_stack,
			      K_THREAD_STACK_SIZEOF(application_thread_stack), sidewalk_app_entry,
			      ctx, NULL, NULL, CONFIG_SIDEWALK_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&application_thread, "sidewalk_thread");
	return SID_ERROR_NONE;
}

