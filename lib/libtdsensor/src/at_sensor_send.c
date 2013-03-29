/***************************************************************************//**
 * @file at_sensor_send.c
 * @brief AT Sensor Send
 * @author Telecom Design S.A.
 * @version 1.0.0
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2013 Telecom Design S.A., http://www.telecom-design.com</b>
 ******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Telecom Design SA has no
 * obligation to support this Software. Telecom Design SA is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Telecom Design SA will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 ******************************************************************************/

#define USE_PRINTF
#include <at_parse.h>
#include <td_printf.h>
#include "sensor_data.h"
#include "sensor_service.h"
#include "sensor_register.h"
#include "sensor_keepalive.h"
#include "sensor_event.h"
#include "sensor_config.h"
#include "td_sensor.h"
#include "sensor_raw.h"
#include "td_measure.h"
#include "td_sensor_device.h"
#include "at_sensor_send.h"

/** RAW data buffer */
uint8_t raw_buffer[12];

/** RAW data len */
uint8_t raw_length;

// ****************************************************************************
// TYPES:
// ****************************************************************************

/** Manufacturing test AT command tokens */
typedef enum sensor_send_tokens_t {
	AT_EXTENSION_BASE = AT_BASE_LAST,

	// Below here, please try to keep the enum in alphabetical order!!!

	//DATA
	AT_SENSOR_SETPHONE,

	//EVENT
	AT_SENSOR_BATTERY,
	AT_SENSOR_CONNECTION,
	AT_SENSOR_RSSI,
	AT_SENSOR_TEMP,
	AT_SENSOR_BOOT,
	AT_SENSOR_SWITCH,

	//GEOLOC
	AT_SENSOR_GETGEOLOC,

	//SERVICE
	AT_SENSOR_SENDSMS,
	AT_SENSOR_SENDTWEET,

	//RAW
	AT_SENSOR_RAW,

	//KEEP-ALIVE
	AT_SENSOR_KEEPALIVE,

	//REGISTER
	AT_SENSOR_REGISTER,

} sensor_send_tokens;

// ****************************************************************************
// CONSTANTS:
// ****************************************************************************

/** AT command set */
static AT_command_t const sensor_send_commands[] = {

//DATA
		{ "AT$DP=", AT_SENSOR_SETPHONE },

		//EVENT
		{ "AT$EB", AT_SENSOR_BOOT },
		{ "AT$EC=", AT_SENSOR_CONNECTION },
		{ "AT$ER=", AT_SENSOR_RSSI },
		{ "AT$ES=", AT_SENSOR_SWITCH },
		{ "AT$ET=", AT_SENSOR_TEMP },
		{ "AT$EV=", AT_SENSOR_BATTERY },

		//GEOLOC
		{ "AT$GL", AT_SENSOR_GETGEOLOC },

		//KEEPALIVE
		{ "AT$KA", AT_SENSOR_KEEPALIVE },

		//SERVICE
		{ "AT$SSMS=", AT_SENSOR_SENDSMS },
		{ "AT$STWT=", AT_SENSOR_SENDTWEET },

		//RAW
		{ "AT$RAW=", AT_SENSOR_RAW },

		//REGISTER
		{ "AT$REG", AT_SENSOR_REGISTER },

		{ 0, 0 }

};

// ****************************************************************************
// LOCALS:
// ****************************************************************************

// ****************************************************************************
// CODE:
// ****************************************************************************

/**
 * @brief Manufacturing test AT command parser init.
 */
static void sensor_send_init(void) {

}

/**
 * @brief Manufacturing test AT help
 */
static void sensor_send_help(void) {
	tfp_printf("--SENSOR SEND Commands--\r\n"

	);
}
;

/***************************************************************************//**
 * @brief
 *   Parser AT extension function for Sensor.
 *
 * @param[in] token
 *   The token to parse.
 *
 * @return
 *   The parse result.
 ******************************************************************************/

static int8_t sensor_send_parse(uint8_t token) {
	int i, j;
	char *message = "";
	char hex[] = "0x00", c;

	int8_t result = AT_OK;

	switch (token) {

	/*****************************DATA****************************/
	case AT_SENSOR_SETPHONE:
		if (AT_argc == 2) {

			uint8_t index = AT_atoll(AT_argv[0]) - 1;
			if (index <= 3) {
				if (!TD_SENSOR_SetCellPhoneNumber((PhoneIndex) index, (uint8_t *) AT_argv[1])) {
					result = AT_ERROR;
				}
			} else {
				result = AT_ERROR;
			}

		} else {
			result = AT_ERROR;
		}
		break;

		/*****************************EVENT****************************/
	case AT_SENSOR_BATTERY:
		if (AT_argc == 1) {
			if (AT_argv[0][0] == '0') {
				if (!TD_SENSOR_SendEventBattery(false, 0)) {
					result = AT_ERROR;
				}
			} else if (AT_argv[0][0] == '1') {
				if (!TD_SENSOR_SendEventBattery(true, TD_MEASURE_Voltage())) {
					result = AT_ERROR;
				}
			} else {
				result = AT_ERROR;
			}
		} else {
			result = AT_ERROR;
		}

		break;

	case AT_SENSOR_CONNECTION:
		if (AT_argc == 2) {

			if (AT_atoll(AT_argv[0]) >= 1 && AT_atoll(AT_argv[0]) <= 15
					&& (AT_argv[1][0] == '0' || AT_argv[1][0] == '1')) {
				if (AT_argv[1][0] == '0') {
					if (!TD_SENSOR_SendEventConnection(false, AT_atoll(AT_argv[0]))) {
						result = AT_ERROR;
					}
				} else if (AT_argv[1][0] == '1') {
					if (!TD_SENSOR_SendEventConnection(true, AT_atoll(AT_argv[0]))) {
						result = AT_ERROR;
					}
				}
			} else {
				result = AT_ERROR;
			}

		} else {
			result = AT_ERROR;
		}
		break;

	case AT_SENSOR_RSSI:
		if (AT_argc == 2) {

			if (AT_atoll(AT_argv[0]) >= 1 && AT_atoll(AT_argv[0]) <= 15
					&& (AT_argv[1][0] == '0' || AT_argv[1][0] == '1')) {
				if (AT_argv[1][0] == '0') {
					if (!TD_SENSOR_SendEventRSSI(false, AT_atoll(AT_argv[0]))) {
						result = AT_ERROR;
					}
				} else if (AT_argv[1][0] == '1') {
					if (!TD_SENSOR_SendEventRSSI(true, AT_atoll(AT_argv[0]))) {
						result = AT_ERROR;
					}
				}
			} else {
				result = AT_ERROR;
			}

		} else {
			result = AT_ERROR;
		}
		break;

	case AT_SENSOR_TEMP:
		if (AT_argc == 1) {

			if (AT_argv[0][0] == '0') {
				if (!TD_SENSOR_SendEventTemperature(0)) {
					result = AT_ERROR;
				}
			} else if (AT_argv[0][0] == '1') {
				if (!TD_SENSOR_SendEventTemperature(1)) {
					result = AT_ERROR;
				}
			} else if (AT_argv[0][0] == '2') {
				if (!TD_SENSOR_SendEventTemperature(2)) {
					result = AT_ERROR;
				}
			} else {
				result = AT_ERROR;
			}

		} else {
			result = AT_ERROR;
		}
		break;

	case AT_SENSOR_BOOT:
		if (AT_argc == 0) {

			if (!TD_SENSOR_SendEventBoot()) {
				result = AT_ERROR;
			}

		} else {
			result = AT_ERROR;
		}
		break;

	case AT_SENSOR_SWITCH:
		if (AT_argc == 3) {

			if (AT_atoll(AT_argv[0]) >= 0 && AT_atoll(AT_argv[0]) <= 5
					&& AT_atoll(AT_argv[1]) >= 0 && AT_atoll(AT_argv[1]) <= 15
					&& AT_atoll(AT_argv[2]) >= 0 && AT_atoll(AT_argv[2]) <= 1) {
				if (!TD_SENSOR_SendEventSwitch(AT_atoll(AT_argv[0]), AT_atoll(AT_argv[1]), AT_atoll(AT_argv[2]))) {
					result = AT_ERROR;
				}
			} else {
				result = AT_ERROR;
			}

		} else {
			result = AT_ERROR;
		}
		break;

		/*****************************KEEPALIVE****************************/
	case AT_SENSOR_KEEPALIVE:
		if (AT_argc == 0) {

			if (!TD_SENSOR_SendKeepAlive()) {
				result = AT_ERROR;
			}

		} else {
			result = AT_ERROR;
		}
		break;

		/*****************************RAW****************************/

		/*case AT_SENSOR_RAW:
		 if (AT_argc == 2) {

		 TD_SENSOR_SendRaw((uint8_t *) AT_argv[0],AT_atoll(AT_argv[1]));

		 } else {
		 result = AT_ERROR;
		 }
		 break;*/

	case AT_SENSOR_RAW:

		if (AT_argc == 1) {

			message = AT_argv[0];
			for (i = 0; message[i]; i++) {
				if (message[i] == ' ' || message[i] == '\t') {
					for (j = i; message[j]; j++) {
						message[j] = message[j + 1];
					}
					if (i) {
						i--;
					}
				}
			}

			for (i = 0; i < 24 && message[i]; i++) {
				c = message[i];
				if ((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f')
						|| (c >= 'A' && c <= 'F')) {
					if (i & 1) {
						hex[3] = c;
						raw_buffer[i >> 1] = AT_atoll(hex);
					} else {
						hex[2] = c;
					}
				} else {
					break;
				}
			}
			if ((i & 1) || message[i] != '\0') {
				result = AT_ERROR;
			} else {
				raw_length = i >> 1;

				if (!TD_SENSOR_SendRaw(raw_buffer, raw_length)) {
					result = AT_ERROR;
				}
			}
		} else if (AT_argc == 0) {
			if (!TD_SENSOR_SendRaw(0, 0)) {
				result = AT_ERROR;
			}
		} else {
			result = AT_ERROR;
		}

		break;

		/*****************************REGISTER SENSOR****************************/

	case AT_SENSOR_REGISTER:
		if (AT_argc == 0) {

			if (!TD_SENSOR_SendRegister()) {
				result = AT_ERROR;
			}

		} else {
			result = AT_ERROR;
		}
		break;

		/*****************************SERVICE****************************/
	case AT_SENSOR_SENDSMS:
		if (AT_argc == 1) {

			if (!TD_SENSOR_SendSMS((uint8_t *) AT_argv[0])) {
				result = AT_ERROR;
			}

		} else {
			result = AT_ERROR;
		}
		break;

	case AT_SENSOR_SENDTWEET:
		if (AT_argc == 1) {

			if (!TD_SENSOR_SendTweet((uint8_t *) AT_argv[0])) {
				result = AT_ERROR;
			}

		} else {
			result = AT_ERROR;
		}
		break;

	default:
		result = AT_NOTHING;
		break;
	}

	return result;
}

/**
 * Manufacturing test AT extension
 */
AT_extension_t sensor_send_extension = {
		.commands = sensor_send_commands,
		.init = sensor_send_init,
		.help = sensor_send_help,
		.parse = sensor_send_parse
};

