/*!
 * @file      gnss.c
 *
 * @brief     GNSS LR11xx application layer, initiate scan & send
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

#include <math.h>
#include <stdbool.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>
#include <halo_lr11xx_radio.h>
#include <application_thread.h>



#ifdef LR11xx
lr11xx_gnss_result_t gnss_result;
#define ASSIST_LATITUDE		33.640
#define ASSIST_LONGITUDE	-117.748
lr11xx_gnss_solver_assistance_position_t assistance_position = { ASSIST_LATITUDE, ASSIST_LONGITUDE };
#endif /* LR11xx */



/* often one of the access points received is a mobile-AP, so get the max +1 */
#define WIFI_MAX_RESULTS ( 10 )

/**
 * @brief Minimal number of detected access point in a scan result to consider the scan valid
 */
#define WIFI_SCAN_NB_AP_MIN ( 1 )

/**
 * @brief Size in bytes of the payload tag to indicate frame format (as defined by LR1110 WiFi positioning protocol of
 * LoRaCloud)
 */

#define WIFI_TAG_SIZE ( 1 )
/**
 * @brief Size in bytes of a WiFi Access-Point address
 */
#define WIFI_AP_ADDRESS_SIZE ( 6 )

/**
 * @brief Size in bytes to store the RSSI of a detected WiFi Access-Point
 */
#define WIFI_AP_RSSI_SIZE ( 1 )

/**
 * @brief Wi-Fi payload format (as defined by LR1110 WiFi positioning protocol of LoRaCloud).
 */
typedef enum wifi_mw_payload_format_e
{
	WIFI_MW_PAYLOAD_MAC			= 0x00,  //!< Only the MAC addresses of the detected Access Points are sent
	WIFI_MW_PAYLOAD_MAC_RSSI	= 0x01,  //!< Both MAC address and RSSI of detected Access Points are sent
} wifi_mw_payload_format_t;

/*!
 * @brief Structure representing a single scan result
 */
typedef struct
{
	lr11xx_wifi_mac_address_t mac_address;	//!< MAC address of the Wi-Fi access point which has been detected
	lr11xx_wifi_channel_t channel;		//!< Channel on which the access point has been detected
	lr11xx_wifi_signal_type_result_t type;			//!< Type of Wi-Fi which has been detected
	int8_t rssi;			//!< Strength of the detected signal
} wifi_scan_single_result_t;

/*!
 * @brief Structure representing a collection of scan results
 */
typedef struct
{
	uint8_t nbr_results;					//!< Number of results
	uint32_t power_consumption_uah;		//!< Power consumption to acquire this set of results
	uint32_t timestamp;					//!< Timestamp at which the data set has been completed
	wifi_scan_single_result_t results[WIFI_MAX_RESULTS];	//!< Buffer containing the results
} wifi_scan_all_result_t;

wifi_configuration_scan_t wifi_configuration = {
	.signal_type			= LR11XX_WIFI_TYPE_SCAN_B,
	.base.channel_mask		= 0x0421,
	.scan_mode				= LR11XX_WIFI_SCAN_MODE_BEACON,
	.base.max_result		= WIFI_MAX_RESULTS,
	.nb_scan_per_channel	= 10,
	.timeout_per_scan		= 90,
   .abort_on_timeout		= true,
};

/*!
 * @brief The format of the Wi-Fi scan results to be used.
 */
static wifi_mw_payload_format_t payload_format = WIFI_MW_PAYLOAD_MAC;

extern lr11xx_gnss_result_t gnss_result;


static uint8_t wifi_result_buffer[WIFI_TAG_SIZE + ( ( WIFI_AP_RSSI_SIZE + WIFI_AP_ADDRESS_SIZE ) * WIFI_MAX_RESULTS )];

static lr11xx_wifi_basic_complete_result_t wifi_results_mac_addr[WIFI_MAX_RESULTS];
static wifi_scan_all_result_t wifi_results;

LOG_MODULE_REGISTER(app_gnss, CONFIG_SIDEWALK_LOG_LEVEL);

static int smtc_wifi_get_results(void *drv_ctx, wifi_scan_all_result_t* wifi_results);

// @todo move this to proper location.
#define LOCATION_PIN DT_ALIAS(locationreadyled)
static const struct gpio_dt_spec locationLED = GPIO_DT_SPEC_GET(LOCATION_PIN, gpios);
static bool _once_on_gnss_scan_done = false;
static bool tried_once = false;
void on_gnss_scan_done(void *arg)
{
	app_ctx_t *app_ctx = arg;
	uint8_t n_sv_detected = 0;
	void *drv_ctx = lr11xx_get_drv_ctx();


	if (app_ctx == NULL) {
		LOG_ERR("on_gnss_scan_done no context");
		return;
	}

	lr11xx_status_t status = lr11xx_gnss_get_result_size(drv_ctx, &gnss_result.length);
	if (status != LR11XX_STATUS_OK) {
		LOG_ERR("gnss_get_result_size fail");
		return;
	}
#ifdef GNSS_DEBUG
	LOG_INF("result size %d", gnss_result.length);
#endif /* GNSS_DEBUG */
	if (gnss_result.length >= GNSS_RESULT_SIZE) {
		LOG_ERR("result too big %d > %d", gnss_result.length, GNSS_RESULT_SIZE);
		return;
	}

   // remove this later.
   if (_once_on_gnss_scan_done == false) {
      int err = 1;
      if (!device_is_ready(locationLED.port)) {
         LOG_ERR("Didn't find ant device referred by the LOCATION_PIN");
      }
      err = gpio_pin_configure_dt(&locationLED, GPIO_OUTPUT);
      if (err) {
         LOG_ERR("Couldn't configure the LOCATION_PIN");
      }
      err = gpio_pin_set_dt(&locationLED, 1);
      if (err) {
         LOG_ERR("Couldn't set the LOCATION_PIN");
      } else {
         _once_on_gnss_scan_done = true;
      }
   }

	status = lr11xx_gnss_read_results(drv_ctx, gnss_result.buffer, gnss_result.length);
	if (status != LR11XX_STATUS_OK) {
		LOG_ERR("gnss_read_results fail");
		return;
	}

	lr11xx_gnss_get_nb_detected_satellites(drv_ctx, &n_sv_detected);
#ifdef GNSS_DEBUG
	lr11xx_gnss_detected_satellite_t sv_detected[NB_MAX_SV] = { 0 };
	LOG_INF("on_gnss_scan_done %d SV", n_sv_detected);
	lr11xx_gnss_get_detected_satellites( drv_ctx, n_sv_detected, sv_detected );
	for( uint8_t index_sv = 0; index_sv < n_sv_detected; index_sv++ )
	{
		const lr11xx_gnss_detected_satellite_t* local_sv = &sv_detected[index_sv];
		LOG_INF( "  - SV %u: CNR: %i, doppler: %i", local_sv->satellite_id, local_sv->cnr,
			local_sv->doppler );
	}
	LOG_HEXDUMP_INF(gnss_result.buffer, gnss_result.length, "nav");
#endif /* GNSS_DEBUG */

	if (app_ctx->frag.total_fragments > 0) {

		LOG_WRN("already sending scan-result");
      if (tried_once) {
         if (SID_LINK_TYPE_1 == app_ctx->config.link_mask) {
            LOG_WRN("cancel already sending result");
            app_ctx->frag.total_fragments = 0;
         }
      }
      if (!tried_once && SID_LINK_TYPE_1 == app_ctx->config.link_mask) {
         tried_once = true;
         return;
      }
      if (SID_LINK_TYPE_1 != app_ctx->config.link_mask) {
         return;
      }
	}
   tried_once = false;

	if (n_sv_detected > 4) {
		uint8_t length = gnss_result.length - 1;
		float total_fragments = length / (float)(app_ctx->frag.mtu-1);	// -1 space for header
		app_ctx->frag.total_fragments = ceil(total_fragments);
		LOG_INF("%p mtu %d, total fragments %u", app_ctx, app_ctx->frag.mtu, app_ctx->frag.total_fragments);
		app_ctx->frag.index = 1;
		app_ctx->frag.total_bytes = length + 1;
		app_ctx->frag.buffer = gnss_result.buffer;
		app_ctx->frag.frag_type = FRAGMENT_TYPE_GNSS;
		app_event_send(EVENT_GNSS_SCAN_SEND);
      // remove this later.
      if (_once_on_gnss_scan_done == true){
         int err = 1;
         err = gpio_pin_toggle_dt(&locationLED);
         if (err) {
            LOG_ERR("Couldn't toggle LOCATION LED");
         }
      }
	}
}

// @todo move this to proper location.
#define LOOP_PIN DT_ALIAS(looptimerled)
static const struct gpio_dt_spec loopLED = GPIO_DT_SPEC_GET(LOOP_PIN, gpios);
static bool _once_start_gnss_scan = false;
void start_gnss_scan(app_ctx_t *app_ctx)
{
   
	struct sid_timespec curr_time;
	void * drv_ctx = lr11xx_get_drv_ctx();
	if (lr11xx_system_wakeup(drv_ctx) != LR11XX_STATUS_OK) {
		LOG_ERR("scan_timer: wake-up fail");
		return;
	}

   // remove this later.
   if (_once_start_gnss_scan == false) {
      int err = 1;
      if (!device_is_ready(loopLED.port)) {
         LOG_ERR("Didn't find ant device referred by the ANT_PIN");
      }
      err = gpio_pin_configure_dt(&loopLED, GPIO_OUTPUT);
      if (err) {
         LOG_ERR("Couldn't configure the LOOP_PIN");
      }
      err = gpio_pin_set_dt(&loopLED, 1);
      if (err) {
         LOG_ERR("Couldn't set the LOOP_PIN");
      } else {
         _once_start_gnss_scan = true;
      }
   } else {
      int err = 1;
      err = gpio_pin_toggle_dt(&loopLED);
      if (err) {
         LOG_ERR("Couldn't toggle LOOP LED");
         return;
      }
   }

	sid_error_t ret = sid_get_time(app_ctx->handle, SID_GET_GPS_TIME, &curr_time);
	if (SID_ERROR_NONE != ret) {
		LOG_ERR("scan_timer: sid_get_time fail %d", ret);
		return;
	}
	lr11xx_status_t status;
	status = lr11xx_gnss_set_assistance_position(drv_ctx, &assistance_position);
	if (status == LR11XX_STATUS_ERROR) {
		LOG_ERR("scan_timer: set assist-pos fail");
		return;
	}
	status = lr11xx_gnss_scan_assisted(drv_ctx,
		curr_time.tv_sec,
		LR11XX_GNSS_OPTION_BEST_EFFORT,
		LR11XX_GNSS_RESULTS_DOPPLER_ENABLE_MASK | LR11XX_GNSS_RESULTS_DOPPLER_MASK | LR11XX_GNSS_RESULTS_BIT_CHANGE_MASK,
		NB_MAX_SV
	);
	if (status == LR11XX_STATUS_ERROR)
		LOG_ERR("scan_timer: assisted scan fail");
#ifdef GNSS_DEBUG
	else
		LOG_INF("scan_timer: assisted scan started %u, %f %f", curr_time.tv_sec, assistance_position.latitude, assistance_position.longitude);
#endif /* GNSS_DEBUG */
}

int scan_wifi(app_ctx_t *app_ctx)
{
	void *drv_ctx = lr11xx_get_drv_ctx();
	if (lr11xx_system_wakeup(drv_ctx) != LR11XX_STATUS_OK) {
		LOG_ERR("scan_wifi wake-up fail");
		return -1;
	}
#if defined(CONFIG_LR1110_CLI)
	lr11xx_status_t status = lr11xx_wifi_scan(drv_ctx,
		wifi_configuration.signal_type,
		wifi_configuration.base.channel_mask,
		wifi_configuration.scan_mode,
		wifi_configuration.base.max_result,
		wifi_configuration.nb_scan_per_channel,
		wifi_configuration.timeout_per_scan,
		wifi_configuration.abort_on_timeout
	);
#else
	lr11xx_status_t status = lr11xx_wifi_scan(drv_ctx,
		LR11XX_WIFI_TYPE_SCAN_B, /* const lr1110_wifi_signal_type_scan_t signal_type */
		0x0421, /* const lr1110_wifi_channel_mask_t channels */
		LR11XX_WIFI_SCAN_MODE_BEACON, /* const lr1110_wifi_mode_t scan_mode */
		6, /* const uint8_t max_results */
		10, /* const uint8_t nb_scan_per_channel */
		90, /* const uint16_t timeout_in_ms */
		true /* const bool abort_on_timeout */
	);
#endif
	if (status != LR11XX_STATUS_OK) {
		LOG_ERR("lr11xx_wifi_scan fail");
		return -1;
	} else
		return 0;
}


static int wifi_mw_send_results(app_ctx_t *app_ctx)
{
	uint8_t wifi_buffer_size = 0;
	/* Check if there are results to be sent */
	if (wifi_results.nbr_results < WIFI_SCAN_NB_AP_MIN) {
		LOG_WRN("only %u results, not sending", wifi_results.nbr_results);
		return -1;
	}
	LOG_INF("size wifi_result_buffer %u", sizeof(wifi_result_buffer));

	if (app_ctx->frag.total_fragments > 0) {

      LOG_WRN("already sending scan-result");
      if (tried_once) {
         if (SID_LINK_TYPE_1 == app_ctx->config.link_mask) {
            LOG_WRN("cancel already sending result");
            app_ctx->frag.total_fragments = 0;
         }
      }
      if (!tried_once && SID_LINK_TYPE_1 == app_ctx->config.link_mask) {
         tried_once = true;
         return -1;
      }
      if (SID_LINK_TYPE_1 != app_ctx->config.link_mask) {
         return -1;
      }
	}
   tried_once = false;

	/* Add the payload format tag */
	wifi_result_buffer[wifi_buffer_size] = payload_format;
	wifi_buffer_size += WIFI_TAG_SIZE;

	/* Concatenate all results in send buffer */
	for( uint8_t i = 0; i < wifi_results.nbr_results; i++ )
	{
		/* Copy Access Point RSSI address in result buffer (if requested) */
		if( payload_format == WIFI_MW_PAYLOAD_MAC_RSSI )
		{
			wifi_result_buffer[wifi_buffer_size] = wifi_results.results[i].rssi;
			wifi_buffer_size += WIFI_AP_RSSI_SIZE;
		}
		/* Copy Access Point MAC address in result buffer */
		memcpy( &wifi_result_buffer[wifi_buffer_size], wifi_results.results[i].mac_address, WIFI_AP_ADDRESS_SIZE );
		wifi_buffer_size += WIFI_AP_ADDRESS_SIZE;
	}
	LOG_HEXDUMP_INF(wifi_result_buffer, wifi_buffer_size, "wifi_result_buffer");

	float total_fragments = wifi_buffer_size / (float)(app_ctx->frag.mtu-1);	// -1 space for header
	app_ctx->frag.total_fragments = ceil(total_fragments);
	LOG_INF("mtu %d, total fragments %u", app_ctx->frag.mtu, app_ctx->frag.total_fragments);
	app_ctx->frag.index = 0;
	app_ctx->frag.total_bytes = wifi_buffer_size;
	app_ctx->frag.buffer = wifi_result_buffer;
	app_ctx->frag.frag_type = FRAGMENT_TYPE_WIFI;
	app_event_send(EVENT_WIFI_SCAN_SEND);

	return 0;
}

void on_wifi_scan_done(void *arg)
{
	void *drv_ctx = lr11xx_get_drv_ctx();
	memset( &wifi_results, 0, sizeof wifi_results );
	int ret = smtc_wifi_get_results(drv_ctx, &wifi_results);
	if (ret < 0) {
		LOG_ERR("smtc_wifi_get_results() fail");
		return;
	}
   LOG_WRN("wifi_results.nbr_results %u", wifi_results.nbr_results);

	wifi_mw_send_results(arg);
}

int smtc_wifi_get_results(void *drv_ctx, wifi_scan_all_result_t* wifi_results)
{
	lr11xx_status_t status;
	uint8_t nb_results;
	uint8_t max_nb_results;
	uint8_t result_index = 0;

	status = lr11xx_wifi_get_nb_results(drv_ctx, &nb_results);
	if (status != LR11XX_STATUS_OK) {
		LOG_ERR("lr11xx_wifi_get_nb_results() fail");
		return -1;
	}
	LOG_INF("nb_results %u", nb_results);

	/* check if the array is big enough to hold all results */
	max_nb_results = sizeof( wifi_results_mac_addr ) / sizeof( wifi_results_mac_addr[0] );
	if (nb_results > max_nb_results)
	{   
		LOG_ERR("Wi-Fi scan result size exceeds %u (%u)", max_nb_results, nb_results);
		return -1;
	}

	memset( wifi_results_mac_addr, 0, sizeof wifi_results_mac_addr );
	status = lr11xx_wifi_read_basic_complete_results(drv_ctx, 0, nb_results, wifi_results_mac_addr);
	if (status != LR11XX_STATUS_OK) {
		LOG_ERR("lr11xx_wifi_read_basic_complete_results() fail");
		return -1;
	}

	/* add scan to results */
	for( uint8_t index = 0; index < nb_results; index++ )
	{ 
		const lr11xx_wifi_basic_complete_result_t* local_basic_result = &wifi_results_mac_addr[index];
		lr11xx_wifi_channel_t		channel;
		bool						rssi_validity;
		lr11xx_wifi_mac_origin_t	mac_origin_estimation;

		lr11xx_wifi_parse_channel_info( local_basic_result->channel_info_byte, &channel, &rssi_validity,
							&mac_origin_estimation );

		if( mac_origin_estimation != LR11XX_WIFI_ORIGIN_BEACON_MOBILE_AP )
		{
			wifi_results->results[result_index].channel = channel;

			wifi_results->results[result_index].type =
				lr11xx_wifi_extract_signal_type_from_data_rate_info( local_basic_result->data_rate_info_byte );

			memcpy( wifi_results->results[result_index].mac_address, local_basic_result->mac_address,
					LR11XX_WIFI_MAC_ADDRESS_LENGTH );

			wifi_results->results[result_index].rssi = local_basic_result->rssi;
			{
				const uint8_t *mac = local_basic_result->mac_address;
				LOG_INF("%u) ch%u %02x %02x %02x %02x %02x %02x %ddBm", index, channel, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], local_basic_result->rssi);
			}
			wifi_results->nbr_results++;
			result_index++;
		} else
			LOG_INF("%u) mobile-AP", index);
	}

	return 0;
}