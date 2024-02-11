/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <sid_api.h>
#include <sid_error.h>
#include <sid_900_cfg.h>

#include <state_notifier.h>
#include <board_events.h>

#if defined(CONFIG_SIDEWALK_DFU)
#include <nordic_dfu.h>
#endif

#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/kernel.h>

LOG_MODULE_REGISTER(board_events, CONFIG_SIDEWALK_LOG_LEVEL);

#if defined(CONFIG_SIDEWALK_LINK_MASK_FSK)
static struct sid_device_profile profile_light_fsk = {
	.unicast_params.device_profile_id = SID_LINK2_PROFILE_1,
	.unicast_params.rx_window_count = SID_RX_WINDOW_CNT_INFINITE,
	.unicast_params.unicast_window_interval.sync_rx_interval_ms =
		SID_LINK2_RX_WINDOW_SEPARATION_3,
	.unicast_params.wakeup_type = SID_TX_AND_RX_WAKEUP,
};

static struct sid_device_profile profile_fast_fsk = {
	.unicast_params.device_profile_id = SID_LINK2_PROFILE_2,
	.unicast_params.rx_window_count = SID_RX_WINDOW_CNT_INFINITE,
	.unicast_params.unicast_window_interval.sync_rx_interval_ms =
		SID_LINK2_RX_WINDOW_SEPARATION_3,
	.unicast_params.wakeup_type = SID_TX_AND_RX_WAKEUP,
};

static struct sid_device_profile profile_light_lora = {
	.unicast_params.device_profile_id = SID_LINK3_PROFILE_A,
	.unicast_params.rx_window_count = SID_RX_WINDOW_CNT_2,
	.unicast_params.unicast_window_interval.async_rx_interval_ms =
		SID_LINK3_RX_WINDOW_SEPARATION_3,
	.unicast_params.wakeup_type = SID_TX_AND_RX_WAKEUP,
};

static struct sid_device_profile profile_fast_lora = {
	.unicast_params.device_profile_id = SID_LINK3_PROFILE_B,
	.unicast_params.rx_window_count = SID_RX_WINDOW_CNT_INFINITE,
	.unicast_params.unicast_window_interval.async_rx_interval_ms =
		SID_LINK3_RX_WINDOW_SEPARATION_3,
	.unicast_params.wakeup_type = SID_TX_AND_RX_WAKEUP,
};

static struct sid_device_profile profile_from_dev_fsk = { .unicast_params.device_profile_id =
							      SID_LINK2_PROFILE_1 };
static struct sid_device_profile profile_from_dev_lora = { .unicast_params.device_profile_id =
							      SID_LINK3_PROFILE_A };
#elif defined(CONFIG_SIDEWALK_LINK_MASK_LORA)
#error "CONFIG_SIDEWALK_LINK_MASK_LORA is not supported"
static struct sid_device_profile profile_light = {
	.unicast_params.device_profile_id = SID_LINK3_PROFILE_A,
	.unicast_params.rx_window_count = SID_RX_WINDOW_CNT_2,
	.unicast_params.unicast_window_interval.async_rx_interval_ms =
		SID_LINK3_RX_WINDOW_SEPARATION_3,
	.unicast_params.wakeup_type = SID_TX_AND_RX_WAKEUP,
};

static struct sid_device_profile profile_fast = {
	.unicast_params.device_profile_id = SID_LINK3_PROFILE_B,
	.unicast_params.rx_window_count = SID_RX_WINDOW_CNT_INFINITE,
	.unicast_params.unicast_window_interval.async_rx_interval_ms =
		SID_LINK3_RX_WINDOW_SEPARATION_3,
	.unicast_params.wakeup_type = SID_TX_AND_RX_WAKEUP,
};

static struct sid_device_profile profile_from_dev = { .unicast_params.device_profile_id =
							      SID_LINK3_PROFILE_A };
#endif

#if defined(CUSTOM_CODE)

static bool parse_link_mask_opt(uint8_t arg, uint32_t *link_mask);
static sid_error_t change_protocol_custom(app_ctx_t *application_ctx);
static sid_error_t is_sidewalk_ready(app_ctx_t *application_ctx, bool* sidewalk_ready);
static sid_error_t set_profile(app_ctx_t *application_ctx);
#endif // CUSTOM_CODE

static void scan_timer_custom_cb(struct k_timer *);
K_TIMER_DEFINE(scan_timer_custom, scan_timer_custom_cb, NULL);

static void scan_timer_custom_cb(struct k_timer *timer_id)
{
   app_event_send(EVENT_GNSS_SCAN_START);
}

static void scan_timer_custom_wifi_gnss_cb(struct k_timer *);
K_TIMER_DEFINE(scan_timer_custom_wifi_gnss, scan_timer_custom_wifi_gnss_cb, NULL);

static void scan_timer_custom_wifi_gnss_cb(struct k_timer *timer_id)
{
   static bool alternate = false;
   if (!alternate) {
	   app_event_send(EVENT_GNSS_SCAN_START);
      alternate = true;
   } else {
      app_event_send(EVENT_WIFI_SCAN_START);
      alternate = false;
   }
}

unsigned gnss_scan_timer_custom_get()
{
	return k_ticks_to_ms_floor32(scan_timer_custom.period.ticks) / MSEC_PER_SEC;
}

int gnss_scan_timer_custom_set(unsigned sec)
{
	if (sec == 0) {
		k_timer_stop(&scan_timer_custom);
		LOG_INF("timer custom stopped");
	} else {
      LOG_INF("timer custom started");
		k_timer_start(&scan_timer_custom, Z_TIMEOUT_NO_WAIT, K_SECONDS(sec));
	}
	return 0;
}

int gnss_scan_timer_custom_wifi_gnss_set(unsigned sec)
{
	if (sec == 0) {
		k_timer_stop(&scan_timer_custom_wifi_gnss);
		LOG_INF("timer custom stopped Wifi + GNSS");
	} else {
      LOG_INF("timer custom started wifi + GNSS");
		k_timer_start(&scan_timer_custom_wifi_gnss, Z_TIMEOUT_NO_WAIT, K_SECONDS(sec));
	}
	return 0;
}

void button_event_send_hello(app_ctx_t *application_ctx)
{
	struct sid_status status = { .state = SID_STATE_NOT_READY };
	sid_error_t err;

	static uint8_t counter = 0;
	static struct sid_msg msg;
	static struct sid_msg_desc desc;

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

	if (status.state != SID_STATE_READY && status.state != SID_STATE_SECURE_CHANNEL_READY) {
		LOG_ERR("Sidewalk Status is invalid!, expected SID_STATE_READY or SID_STATE_SECURE_CHANNEL_READY, got %d",
			status.state);
		return;
	}

	msg = (struct sid_msg){ .data = (uint8_t *)&counter, .size = sizeof(uint8_t) };
	desc = (struct sid_msg_desc){
		.type = SID_MSG_TYPE_NOTIFY,
		.link_type = SID_LINK_TYPE_ANY,
		.link_mode = SID_LINK_MODE_CLOUD,
	};

	err = sid_put_msg(application_ctx->handle, &msg, &desc);
	switch (err) {
	case SID_ERROR_NONE: {
		application_state_sending(&global_state_notifier, true);
		counter++;
		LOG_INF("queued data message id:%d", desc.id);
		break;
	}
	case SID_ERROR_TRY_AGAIN: {
		LOG_ERR("there is no space in the transmit queue, Try again.");
		break;
	}
	default:
		LOG_ERR("Unknown error returned from sid_put_msg() -> %d", err);
	}
}


void button_event_set_battery(app_ctx_t *application_ctx)
{
	static uint8_t fake_bat_lev = 70;

	++fake_bat_lev;
	if (fake_bat_lev > 100) {
		fake_bat_lev = 0;
	}
	sid_error_t ret = sid_option(application_ctx->handle, SID_OPTION_BLE_BATTERY_LEVEL,
				     &fake_bat_lev, sizeof(fake_bat_lev));

	if (SID_ERROR_NONE != ret) {
		LOG_ERR("failed setting Sidewalk option!");
	} else {
		LOG_INF("set battery level to %d", fake_bat_lev);
	}
}

#if defined(CONFIG_SIDEWALK_DFU_SERVICE_BLE)
void button_event_DFU(app_ctx_t *application_ctx)
{
	bool DFU_mode = true;

	(void)settings_save_one(CONFIG_DFU_FLAG_SETTINGS_KEY, (const void *)&DFU_mode,
				sizeof(DFU_mode));

	sid_deinit(application_ctx->handle);
	k_sleep(K_SECONDS(1));

	sys_reboot(SYS_REBOOT_COLD);
}

#endif /* CONFIG_SIDEWALK_DFU_SERVICE_BLE */

void button_event_factory_reset(app_ctx_t *application_ctx)
{
	sid_error_t ret = sid_set_factory_reset(application_ctx->handle);

	if (SID_ERROR_NONE != ret) {
		LOG_ERR("Notification of factory reset to sid api failed!");
	} else {
		LOG_INF("Wait for Sid api to notify to proceed with factory reset!");
	}
}

void button_event_get_profile(app_ctx_t *application_ctx)
{
   uint32_t link_mask = application_ctx->config.link_mask;
   if (SID_LINK_TYPE_2 == link_mask) {
      sid_error_t ret = sid_option(application_ctx->handle, SID_OPTION_900MHZ_GET_DEVICE_PROFILE,
                  &profile_from_dev_fsk, sizeof(profile_from_dev_fsk));

      if (ret) {
         LOG_ERR("Profile get FSK failed (err %d)", ret);
         return;
      }

      LOG_INF("\n"
         "Profile FSK id 0x%x\n"
         "Profile FSK dl count %d\n"
         "Profile FSK dl interval %d\n"
         "Profile FSK wakeup %d\n",
         profile_from_dev_fsk.unicast_params.device_profile_id,
         profile_from_dev_fsk.unicast_params.rx_window_count,
         profile_from_dev_fsk.unicast_params.unicast_window_interval.async_rx_interval_ms,
         profile_from_dev_fsk.unicast_params.wakeup_type);
   } else if (SID_LINK_TYPE_3 == link_mask) {
      sid_error_t ret = sid_option(application_ctx->handle, SID_OPTION_900MHZ_GET_DEVICE_PROFILE,
                  &profile_from_dev_lora, sizeof(profile_from_dev_lora));

      if (ret) {
         LOG_ERR("Profile get LoRa failed (err %d)", ret);
         return;
      }

      LOG_INF("\n"
         "Profile LoRa id 0x%x\n"
         "Profile LoRa dl count %d\n"
         "Profile LoRa dl interval %d\n"
         "Profile LoRa wakeup %d\n",
         profile_from_dev_lora.unicast_params.device_profile_id,
         profile_from_dev_lora.unicast_params.rx_window_count,
         profile_from_dev_lora.unicast_params.unicast_window_interval.async_rx_interval_ms,
         profile_from_dev_lora.unicast_params.wakeup_type);
   } else {
      LOG_ERR("Invalid link mask option!");
   }
}

void button_stop_sidewalk_custom (app_ctx_t *application_ctx)
{
   sid_error_t ret = sid_stop(application_ctx->handle, application_ctx->config.link_mask);
   if (!ret) {
		LOG_INF("stop_sidewalk_custom set.");
	} else {
		LOG_ERR("stop_sidewalk_custom failed (err %d)", ret);
	}
}

void button_start_sidewalk_custom (app_ctx_t *application_ctx)
{
   sid_error_t ret = sid_start(application_ctx->handle, application_ctx->config.link_mask);
   if (!ret) {
		LOG_INF("start_sidewalk_custom set.");
	} else {
		LOG_ERR("start_sidewalk_custom failed (err %d)", ret);
	}
}

// @todo check this
void button_event_set_ptofile(app_ctx_t *application_ctx)
{
   uint32_t link_mask = application_ctx->config.link_mask;
   sid_error_t ret = set_profile(application_ctx);
   if (!ret) {
      if (SID_LINK_TYPE_2 == link_mask) {
         LOG_INF("Profile FSK set success.");
      } else if (SID_LINK_TYPE_3 == link_mask) {
         LOG_INF("Profile LORA set success.");
      } else {
         LOG_ERR("Invalid link mask option!");
      }
   } else {
      if (SID_LINK_TYPE_2 == link_mask) {
         LOG_ERR("Profile FSK set failed (err %d)", ret);
      } else if (SID_LINK_TYPE_3 == link_mask) {
         LOG_ERR("Profile LORA set failed (err %d)", ret);
      } else {
         LOG_ERR("Invalid link mask option!");
      }
   }
}

static sid_error_t set_profile(app_ctx_t *application_ctx) {
   uint32_t link_mask = application_ctx->config.link_mask;
   static struct sid_device_profile *new_profile_fsk = &profile_light_fsk;
   static struct sid_device_profile *new_profile_lora = &profile_light_lora;
   static uint8_t fake_bat_lev = 70;
   sid_error_t ret;
   if (SID_LINK_TYPE_2 == link_mask) {
      LOG_INF("Profile FSK set %s", (&profile_light_fsk == new_profile_fsk) ? "light_fsk" : "fast_fsk");

      ret = sid_option(application_ctx->handle, SID_OPTION_900MHZ_SET_DEVICE_PROFILE,
         new_profile_fsk, sizeof(*new_profile_fsk));

      if (!ret) {
         // @todo do not switch profiles
         // (void)profile_fast_fsk;
         new_profile_fsk = (&profile_light_fsk == new_profile_fsk) ? &profile_fast_fsk : &profile_light_fsk;
      }
   } else if (SID_LINK_TYPE_3 == link_mask) {
      LOG_INF("Profile LORA set %s", (&profile_light_lora == new_profile_lora) ? "light_lora" : "fast_lora");

      ret = sid_option(application_ctx->handle, SID_OPTION_900MHZ_SET_DEVICE_PROFILE,
         new_profile_lora, sizeof(*new_profile_lora));

      if (!ret) {
         // @todo do not switch profiles
         // (void)profile_fast_lora;
         new_profile_lora = (&profile_light_lora == new_profile_lora) ? &profile_fast_lora : &profile_light_lora;
      }
   } else if(SID_LINK_TYPE_1 == link_mask) {
      ++fake_bat_lev;
      if (fake_bat_lev > 100) {
         fake_bat_lev = 0;
      }
	   ret = sid_option(application_ctx->handle, SID_OPTION_BLE_BATTERY_LEVEL,
         &fake_bat_lev, sizeof(fake_bat_lev));
      if (!ret) {
         // empty
      }
   }else {
      LOG_ERR("Invalid link mask option!");
      ret = SID_ERROR_INVALID_ARGS;
   }
   return ret;
}

#if defined(CUSTOM_CODE)

static bool parse_link_mask_opt(uint8_t arg, uint32_t *link_mask)
{
	if (link_mask == NULL) {
		return false;
	}

	switch (arg) {
	case OPT_LINK_BLE:
		*link_mask = SID_LINK_TYPE_1;
		break;
	case OPT_LINK_FSK:
		*link_mask = SID_LINK_TYPE_2;
		break;
	case OPT_LINK_LORA:
		*link_mask = SID_LINK_TYPE_3;
		break;
	case OPT_LINK_LORA_BLE:
		*link_mask = SID_LINK_TYPE_1 | SID_LINK_TYPE_3;
		break;
	case OPT_LINK_FSK_BLE:
		*link_mask = SID_LINK_TYPE_1 | SID_LINK_TYPE_2;
		break;
	default:
		return false;
	}
	return true;
}

// here!!!!
static sid_error_t is_sidewalk_ready(app_ctx_t *application_ctx, bool* sidewalk_ready) {
   struct sid_status status = { .state = SID_STATE_NOT_READY };
	sid_error_t err;
   *sidewalk_ready = false;

   do {
      err = sid_get_status(application_ctx->handle, &status);
      if (SID_ERROR_NONE != err) {
         LOG_ERR("sid_get_status() failed with error %d", err);
         break;
      }

      if (status.state != SID_STATE_READY &&
         status.state != SID_STATE_NOT_READY) {
         LOG_ERR("Sidewalk Status is invalid!, got %d",
            status.state);
         err = SID_ERROR_GENERIC;
         break;
      }

      if (status.state == SID_STATE_READY) {
         *sidewalk_ready = true;
      }
   } while (false);

   return err;
}

static sid_error_t change_protocol_custom(app_ctx_t *application_ctx) {

   sid_error_t err = SID_ERROR_GENERIC;
   uint32_t link_mask;
   bool sidewalk_ready;
   static uint8_t link_mask_opt = OPT_LINK_FSK;

   do
   {
      err = is_sidewalk_ready(application_ctx, &sidewalk_ready);
      if (SID_ERROR_NONE != err) {
         LOG_ERR("is_sidewalk_ready() failed with error %d", err);
         break;
      }
      // @todo add logic to toggle
      if (OPT_LINK_FSK == link_mask_opt) {
         LOG_WRN("Change protocol to LORA");
         link_mask_opt = OPT_LINK_LORA;
      } else if (OPT_LINK_LORA == link_mask_opt) {
         LOG_WRN("Change protocol to BLE");
         link_mask_opt = OPT_LINK_BLE;
      } else if (OPT_LINK_BLE == link_mask_opt) {
         LOG_WRN("Change protocol to FSK");
         link_mask_opt = OPT_LINK_FSK;
      } else {
         LOG_ERR("Invalid link mask option! %d", link_mask_opt);
         break;
      }
      if (!parse_link_mask_opt(link_mask_opt, &link_mask)) {
         LOG_ERR("Invalid link mask option!");
         err = SID_ERROR_INVALID_ARGS;
         break;
      }
      if (true == sidewalk_ready) {
         LOG_ERR("Stop sidewalk before change protocol!");
         err = SID_ERROR_GENERIC;
         break;
      }
      err = sid_deinit(application_ctx->handle);
      if (SID_ERROR_NONE != err) {
         LOG_ERR("sid_deinit() failed with error %d", err);
         break;
      }
      LOG_WRN("Sidewalk deinited before. Sleep now!");
      k_sleep(K_SECONDS(1));
      application_ctx->config.link_mask = link_mask;
      err = sid_init(&application_ctx->config, &application_ctx->handle);
      if (SID_ERROR_NONE != err) {
         LOG_ERR("sid_init() failed with error %d", err);
         if (SID_ERROR_ALREADY_INITIALIZED == err) {
            LOG_WRN("Sidewalk already initialized!");
         }
         break;
      }
   // @todo figure out how to set profile here
#if 0
      err = set_profile(application_ctx);
      if (SID_ERROR_NONE != err) {
         LOG_ERR("set_profile() failed with error %d", err);
         break;
      }
#endif // 0
   } while (false);

   return err;
}

void change_protocol_sidewalk_custom (app_ctx_t *application_ctx) {
   sid_error_t err = change_protocol_custom(application_ctx);
   if (SID_ERROR_NONE != err) {
      LOG_ERR("change_protocol_custom() failed with error %d", err);
   } else {
      LOG_WRN("change protocol success");
   }
}
#endif // CUSTOM_CODE

void app_event_send_standby() { }
void app_event_send_sleep() { }
void app_event_send_sid_init() { }
void app_event_send_wake() { }

void app_event_wifi_scan()
{
	app_event_send(EVENT_WIFI_SCAN_START);
}