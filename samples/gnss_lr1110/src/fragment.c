/*!
 * @file      fragment.c
 *
 * @brief     packet fragmentation over sidewalk
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2022. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Semtech corporation nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
 * NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <application_thread.h>
#include <string.h>
#include <state_notifier.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>

LOG_MODULE_REGISTER(fragment, CONFIG_SIDEWALK_LOG_LEVEL);

void send_scan_result(void *context)
{
	app_ctx_t *app_ctx = (app_ctx_t *)context;
	sid_error_t err;
	static struct sid_msg msg;
	static struct sid_msg_desc desc;
	static uint8_t dummy[258];
	uint8_t remaining = app_ctx->frag.total_bytes - app_ctx->frag.index;
	uint8_t this_length = remaining;
	uint8_t byte_per_fragment = app_ctx->frag.mtu - 1;

#ifdef GNSS_DEBUG
	LOG_INF("send_scan_result index:%d total:%d cur_frag:%d mtu=%d", app_ctx->frag.index, app_ctx->frag.total_bytes, app_ctx->frag.current_fragment, app_ctx->frag.mtu);
#endif /* GNSS_DEBUG */

   if (SID_LINK_TYPE_1 == app_ctx->config.link_mask) {
      struct sid_status _status = { .state = SID_STATE_NOT_READY };
      sid_error_t _err;
      bool break_loop = false;
      bool next;
      sid_error_t _ret;
      uint8_t max_retry = 8;
      if (app_ctx->connection_request) {
         app_ctx->connection_request = false;
      }
      do {
         if (max_retry == 0) {
            LOG_ERR("Failed to try more");
            return;
         }
         max_retry--;
         _err = sid_get_status(app_ctx->handle, &_status);
         switch (_err) {
            case SID_ERROR_NONE:
               break;
            case SID_ERROR_INVALID_ARGS:
               LOG_ERR("Sidewalk library is not initialzied!");
               break_loop = true;
               break;
            default:
               LOG_ERR("Unknown error during sid_get_status() -> %d", _err);
               break_loop = true;
               break;
         }
         if (break_loop) {
            break;
         }
         if (_status.state != SID_STATE_READY) {
            next = !app_ctx->connection_request;
            LOG_INF("%s connection request", next ? "Set" : "Clear");
            if (next) {
               _ret = sid_ble_bcn_connection_request(app_ctx->handle, next);
               if (SID_ERROR_NONE == _ret) {
                  app_ctx->connection_request = next;
               } else {
                  LOG_ERR("Connection request failed %d", _ret);
               }
            }
            k_sleep(K_SECONDS(2));
         }
      } while (_status.state != SID_STATE_READY);
   }

	if (this_length > byte_per_fragment)
		this_length = byte_per_fragment;

	app_ctx->frag.nbytes_sent_this_fragment = this_length;

	dummy[0] = app_ctx->frag.current_fragment & 7;
	dummy[0] |= (app_ctx->frag.total_fragments & 7) << 3;
	dummy[0] |= (app_ctx->frag.frag_type & 3) << 6;
	memcpy(dummy+1, app_ctx->frag.buffer + app_ctx->frag.index, this_length);

#ifdef GNSS_DEBUG
	LOG_HEXDUMP_INF(dummy, this_length+1, "fragment");
#endif /* GNSS_DEBUG */
	msg = (struct sid_msg){ .data = dummy, .size = this_length+1};
	desc = (struct sid_msg_desc){
		.type = SID_MSG_TYPE_NOTIFY,
		.link_type = SID_LINK_TYPE_ANY,
		.link_mode = SID_LINK_MODE_CLOUD,
	};

	err = sid_put_msg(app_ctx->handle, &msg, &desc);
	switch (err) {
	case SID_ERROR_NONE: {
		application_state_sending(&global_state_notifier, true);
		LOG_INF("queued data message id:%d", desc.id);
		break;
	}
	case SID_ERROR_TRY_AGAIN: {
		LOG_ERR("there is no space in the transmit queue, Try again.");
		break;
	}
	default: LOG_ERR("Unknown error returned from sid_put_msg() -> %d", err);
	}
}

// @todo move this to proper location.
#define UPLINK_PIN DT_ALIAS(uplinkled)
static const struct gpio_dt_spec uplinkLED = GPIO_DT_SPEC_GET(UPLINK_PIN, gpios);
static bool _once_fragment_msg_sent = false;
void fragment_msg_sent(void *context)
{
	app_ctx_t *app_ctx = (app_ctx_t *)context;

   // remove this later.
   if (_once_fragment_msg_sent == false) {
      int err = 1;
      if (!device_is_ready(uplinkLED.port)) {
         LOG_ERR("Didn't find ant device referred by the UPLINK_PIN");
      }
      err = gpio_pin_configure_dt(&uplinkLED, GPIO_OUTPUT);
      if (err) {
         LOG_ERR("Couldn't configure the UPLINK_PIN");
      }
      err = gpio_pin_set_dt(&uplinkLED, 1);
      if (err) {
         LOG_ERR("Couldn't set the UPLINK_PIN");
      } else {
         _once_fragment_msg_sent = true;
      }
   }

	if (app_ctx->frag.total_fragments > 0) {
		/* done sending gnss fragment */
		app_ctx->frag.index += app_ctx->frag.nbytes_sent_this_fragment;
		if (++app_ctx->frag.current_fragment < app_ctx->frag.total_fragments) {
			/* send next fragment */
			app_event_send(EVENT_GNSS_SCAN_SEND);
		} else {
			if (app_ctx->frag.index != app_ctx->frag.total_bytes)
				LOG_ERR("incorrect send completion count (%d %d)", app_ctx->frag.index, app_ctx->frag.total_bytes);
			else {
				LOG_INF("done sending fragments (%u %u)", app_ctx->frag.index, app_ctx->frag.total_bytes);
            // remove this later.
            if (_once_fragment_msg_sent == true){
               int err = 1;
               err = gpio_pin_toggle_dt(&uplinkLED);
               if (err) {
                  LOG_ERR("Couldn't toggle UPLINK LED");
               }
            }
         }
			app_ctx->frag.total_fragments = 0;	// indicate done sending
			app_ctx->frag.current_fragment = 0;
		}
	}
}

void fragment_send_error(void *context)
{
	app_ctx_t *app_ctx = (app_ctx_t *)context;

	if (app_ctx->frag.total_fragments > 0) {
		/* TODO: reproduce this failure and then implement sending retry */
		LOG_ERR("aborting fragment-send");
		app_ctx->frag.total_fragments = 0;	// indicate done sending
		app_ctx->frag.current_fragment = 0;
	}
}
