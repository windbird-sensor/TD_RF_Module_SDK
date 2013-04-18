/***************************************************************************//**
 * @file at_sensor_lan.c
 * @brief AT Sensor LAN
 * @author Telecom Design S.A.
 * @version 1.1.0
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

#include <stdbool.h>
#include <stdint.h>

#include <at_parse.h>
#include <td_printf.h>

#include "sensor_config.h"
#include "td_sensor.h"
#include "td_sensor_lan.h"
#include "td_sensor_gateway.h"
#include "td_sensor_device.h"
#include "at_sensor_lan.h"

// ****************************************************************************
// TYPES:
// ****************************************************************************

/** Manufacturing test AT command tokens */
typedef enum sensor_lan_tokens_t {
	AT_EXTENSION_BASE = AT_BASE_LAST,

	//GENERAL
	AT_SENSOR_LAN_GET_ADDRESS,

	//GATEWAY
	AT_SENSOR_LAN_SET_REGISTRATION,
	AT_SENSOR_LAN_REMOVE_DEVICE,
	AT_SENSOR_LAN_REMOVE_ALL_DEVICES,

	//DEVICE
	AT_SENSOR_LAN_REGISTER,
	AT_SENSOR_LAN_DATA,
	AT_SENSOR_LAN_RESET_ADDRESS,

} sensor_lan_tokens;

// ****************************************************************************
// CONSTANTS:
// ****************************************************************************

/** AT command set */
static AT_command_t const sensor_lan_commands[] = {

	{ "AT$LA?", AT_SENSOR_LAN_GET_ADDRESS },
	{ "AT$LD=", AT_SENSOR_LAN_DATA },
	{ "AT$LRD=", AT_SENSOR_LAN_REMOVE_DEVICE },
	{ "AT$LRAD", AT_SENSOR_LAN_REMOVE_ALL_DEVICES },
	{ "AT$LR=",	AT_SENSOR_LAN_SET_REGISTRATION },
	{ "AT$LR", AT_SENSOR_LAN_REGISTER },
	{ "AT$LZ", AT_SENSOR_LAN_RESET_ADDRESS },
	{ 0, 0 }

};

// ****************************************************************************
// LOCALS:
// ****************************************************************************

/** Data buffer */
uint8_t data_buffer[16];

/** Data RX buffer */
uint8_t data_rx_buffer[16];

/** Data len */
uint8_t data_length;

// ****************************************************************************
// CODE:
// ****************************************************************************

/**
 * @brief Manufacturing test AT command parser init.
 */
static void sensor_lan_init(void)
{

}

/**
 * @brief Manufacturing test AT help
 */
static void sensor_lan_help(void)
{
	AT_printf(
		"AT$LA? => LAN address\r\n"
		"AT$LD= => Send LAN data\r\n"
		"AT$LR= => Gateway registration\r\n"
		"AT$LR => Device registration\r\n"
		"AT$LZ => Sensor LAN Reset\r\n"
	);
}
;

/***************************************************************************//**
 * @brief
 *   Parser AT extension function for TD Sensor LAN.
 *
 * @param[in] token
 *   The token to parse.
 *
 * @return
 *   The parse result.
 ******************************************************************************/
static int8_t sensor_lan_parse(uint8_t token)
{

	int i, j;
	char * message;
	char hex[] = "0x00", c;

	int8_t result = AT_OK;

	ModuleType type = TD_SENSOR_GetModuleType();
	LanAddress * lan_address = TD_SENSOR_LAN_GetAddress();

	switch (token) {
	/****************************GENERAL***************************/

	case AT_SENSOR_LAN_GET_ADDRESS:
		if (AT_argc == 0 && type != SENSOR_TRANSMITTER) {

			AT_printf("Address: %02x, Mask: %02x", lan_address->address,lan_address->mask);

		} else {
			result = AT_ERROR;
		}
		break;

		/*****************************GATEWAY****************************/
	case AT_SENSOR_LAN_SET_REGISTRATION:
		if (AT_argc == 1 && type == SENSOR_GATEWAY) {

			switch (AT_argv[0][0]) {
			case '1':
				TD_SENSOR_GATEWAY_StartRegistration(0);
				break;

			case '0':
				TD_SENSOR_GATEWAY_StopRegistration();
				break;

			default:
				result = AT_ERROR;
				break;

			}

		} else {
			result = AT_ERROR;
		}
		break;

	case AT_SENSOR_LAN_REMOVE_DEVICE:

		if (AT_argc == 1 && type == SENSOR_GATEWAY) {
			TD_SENSOR_GATEWAY_DeleteDevice(AT_atoll(AT_argv[0]));

		} else {
			result = AT_ERROR;
		}
		break;

	case AT_SENSOR_LAN_REMOVE_ALL_DEVICES:

		if (AT_argc == 0 && type == SENSOR_GATEWAY) {
			TD_SENSOR_GATEWAY_DeleteAllDevices();

		} else {
			result = AT_ERROR;
		}
		break;

	case AT_SENSOR_LAN_REGISTER:

		if (AT_argc == 0 && type == SENSOR_DEVICE) {
			if (TD_SENSOR_DEVICE_Register() != ACK_OK) {
				result = AT_ERROR;
			}

		} else {
			result = AT_ERROR;
		}

		break;

	case AT_SENSOR_LAN_DATA:

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

			for (i = 0; i < 32 && message[i]; i++) {
				c = message[i];
				if ((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f')
						|| (c >= 'A' && c <= 'F')) {
					if (i & 1) {
						hex[3] = c;
						data_buffer[i >> 1] = AT_atoll(hex);
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
				data_length = i >> 1;

				if (TD_SENSOR_DEVICE_Data(data_buffer, data_length,	data_rx_buffer) != ACK_OK) {
					result = AT_ERROR;
				} else {
					tfp_dump("RX: ", (unsigned char *) data_rx_buffer, 15);
				}

			}
		} else {
			result = AT_ERROR;
		}

		break;

	case AT_SENSOR_LAN_RESET_ADDRESS:

		if (AT_argc != 0 || type == SENSOR_TRANSMITTER) {
			result = AT_ERROR;
		} else if (AT_argc == 0 && type == SENSOR_DEVICE) {
			TD_SENSOR_DEVICE_Reset();
		} else if (AT_argc == 0 && type == SENSOR_GATEWAY) {
			TD_SENSOR_GATEWAY_DeleteAllDevices();
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
AT_extension_t sensor_lan_extension = {
		.commands = sensor_lan_commands,
		.init =	sensor_lan_init,
		.help = sensor_lan_help,
		.parse = sensor_lan_parse
};

