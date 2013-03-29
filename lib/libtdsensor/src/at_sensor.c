/***************************************************************************//**
 * @file at_sensor.c
 * @brief AT Sensor
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
#include <td_flash.h>
#include "td_sensor.h"
#include "sensor_raw.h"
#include "td_sensor_device.h"
#include "td_measure.h"
#include "sensor_config.h"
#include "at_sensor.h"

static ModuleType type;

// ****************************************************************************
// TYPES:
// ****************************************************************************

/** Manufacturing test AT command tokens */
typedef enum sensor_tokens_t {
	AT_EXTENSION_BASE = AT_BASE_LAST,

	AT_SENSOR_SET_TYPE,
	AT_SENSOR_QUERY_TYPE,
	AT_SENSOR_GET_TYPE,

	AT_SENSOR_QUERY_DEVICE_CLASS,
	AT_SENSOR_SET_DEVICE_CLASS,
	AT_SENSOR_GET_DEVICE_CLASS,

	AT_SENSOR_QUERY_BATTERY_MONITORING,
	AT_SENSOR_GET_BATTERY_MONITORING,
	AT_SENSOR_SET_BATTERY_MONITORING,

	AT_SENSOR_QUERY_BOOT_MONITORING,
	AT_SENSOR_GET_BOOT_MONITORING,
	AT_SENSOR_SET_BOOT_MONITORING,

	AT_SENSOR_QUERY_TEMPERATURE_MONITORING,
	AT_SENSOR_GET_TEMPERATURE_MONITORING,
	AT_SENSOR_SET_TEMPERATURE_MONITORING,

	AT_SENSOR_QUERY_RSSI_MONITORING,
	AT_SENSOR_GET_RSSI_MONITORING,
	AT_SENSOR_SET_RSSI_MONITORING,

	AT_SENSOR_QUERY_CONNECTION_MONITORING,
	AT_SENSOR_GET_CONNECTION_MONITORING,
	AT_SENSOR_SET_CONNECTION_MONITORING,

	AT_SENSOR_QUERY_SWITCH_MONITORING,
	AT_SENSOR_GET_SWITCH_MONITORING,
	AT_SENSOR_SET_SWITCH_MONITORING,

	AT_SENSOR_QUERY_KEEPALIVE_MONITORING,
	AT_SENSOR_GET_KEEPALIVE_MONITORING,
	AT_SENSOR_SET_KEEPALIVE_MONITORING,

	AT_SENSOR_SENSOR_RESET,

	AT_SENSOR_DELETE_FLASH_VARIABLES,

} sensor_tokens;

// ****************************************************************************
// CONSTANTS:
// ****************************************************************************

/** AT command set */
static AT_command_t const sensor_commands[] = {

	{ "ATS500=?", AT_SENSOR_QUERY_TYPE },
	{ "ATS500=", AT_SENSOR_SET_TYPE },
	{ "ATS500?", AT_SENSOR_GET_TYPE },

	{ "ATS501=?", AT_SENSOR_QUERY_DEVICE_CLASS },
	{ "ATS501=", AT_SENSOR_SET_DEVICE_CLASS },
	{ "ATS501?", AT_SENSOR_GET_DEVICE_CLASS },

	{ "ATS502=?", AT_SENSOR_QUERY_BATTERY_MONITORING },
	{ "ATS502=", AT_SENSOR_SET_BATTERY_MONITORING },
	{ "ATS502?", AT_SENSOR_GET_BATTERY_MONITORING },

	{ "ATS503=?", AT_SENSOR_QUERY_TEMPERATURE_MONITORING },
	{ "ATS503=", AT_SENSOR_SET_TEMPERATURE_MONITORING },
	{ "ATS503?", AT_SENSOR_GET_TEMPERATURE_MONITORING },

	{ "ATS504=?", AT_SENSOR_QUERY_RSSI_MONITORING },
	{ "ATS504=", AT_SENSOR_SET_RSSI_MONITORING },
	{ "ATS504?", AT_SENSOR_GET_RSSI_MONITORING },

	{ "ATS505=?", AT_SENSOR_QUERY_CONNECTION_MONITORING },
	{ "ATS505=", AT_SENSOR_SET_CONNECTION_MONITORING },
	{ "ATS505?", AT_SENSOR_GET_CONNECTION_MONITORING },

	{ "ATS506=?", AT_SENSOR_QUERY_SWITCH_MONITORING },
	{ "ATS506=", AT_SENSOR_SET_SWITCH_MONITORING },
	{ "ATS506?", AT_SENSOR_GET_SWITCH_MONITORING },

	{ "ATS507=?", AT_SENSOR_QUERY_BOOT_MONITORING },
	{ "ATS507=", AT_SENSOR_SET_BOOT_MONITORING },
	{ "ATS507?", AT_SENSOR_GET_BOOT_MONITORING },

	{ "ATS508=?", AT_SENSOR_QUERY_KEEPALIVE_MONITORING },
	{ "ATS508=", AT_SENSOR_SET_KEEPALIVE_MONITORING },
	{ "ATS508?", AT_SENSOR_GET_KEEPALIVE_MONITORING },

	{ "AT$SZ", AT_SENSOR_SENSOR_RESET },

	{ "AT$DF", AT_SENSOR_DELETE_FLASH_VARIABLES },

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
static void sensor_init(void) {
	type = SENSOR_TRANSMITTER;
}

/**
 * @brief Manufacturing test AT help
 */
static void sensor_help(void) {
	tfp_printf("--SENSOR Commands--\r\n"

	);
}
;

/***************************************************************************//**
 * @brief
 *   Persistence AT extension function for SIGFOX.
 *
 * @param[in] write
 *   Flag set to true for writing persistent data, false for reading persistent
 *   data.
 *
 * @param[out] buffer
 *   Pointer to the persistent data buffer to read/write.
 *
 * @param[in] count
 *   The length in bytes of the persistent data buffer.
 *
 * @return
 *   The number of bytes read from/written to the persistent data buffer.
 ******************************************************************************/
static uint8_t sensor_persist(bool write, uint8_t *buffer, uint8_t count) {

	if (buffer != 0 && count != 0) {
		if (write == true) {
			*buffer = type;
		} else {
			type = (ModuleType) *buffer;
		}
	}

	return 1;
}

/***************************************************************************//**
 * @brief
 *   Parser AT extension function for TD Sensor Monitor.
 *
 * @param[in] token
 *   The token to parse.
 *
 * @return
 *   The parse result.
 ******************************************************************************/
static int8_t sensor_parse(uint8_t token) {
	int8_t result = AT_OK;
	ModuleConfiguration * config = TD_SENSOR_GetModuleConfiguration();

	switch (token) {

	case AT_SENSOR_QUERY_TYPE:
		if (AT_argc != 0) {
			result = AT_ERROR;
		} else {
			AT_printf("Device: 0, Gateway: 1, Transmitter: 2\r\n");
		}
		break;

	case AT_SENSOR_GET_TYPE:
		if (AT_argc != 0) {
			result = AT_ERROR;
		} else {
			AT_printf("%02X\r\n", config->type);
		}
		break;

	case AT_SENSOR_SET_TYPE:
		if (AT_argc == 1) {
			if (AT_atoll(AT_argv[0]) >= 0 && AT_atoll(AT_argv[0]) <= 2) {
				type = (ModuleType) AT_atoll(AT_argv[0]);

				//to make sure the gateway address is reseted on reboot.
				if (AT_atoll(AT_argv[0]) == SENSOR_DEVICE) {
					TD_SENSOR_DEVICE_Reset();
				}
			} else {
				result = AT_ERROR;
			}
		} else {
			result = AT_ERROR;
		}
		break;

	case AT_SENSOR_QUERY_DEVICE_CLASS:
		if (AT_argc != 0) {
			result = AT_ERROR;
		} else {
			AT_printf("0..65535\r\n");
		}
		break;

	case AT_SENSOR_GET_DEVICE_CLASS:
		if (AT_argc != 0) {
			result = AT_ERROR;
		} else {
			AT_printf("%02X\r\n", config->class);
		}
		break;

	case AT_SENSOR_SET_DEVICE_CLASS:
		if (AT_argc == 1) {
			if (AT_atoll(AT_argv[0]) >= 0 && AT_atoll(AT_argv[0]) <= 65535) {
				TD_SENSOR_SetDeviceClass(AT_atoll(AT_argv[0]));
			} else {
				result = AT_ERROR;
			}
		} else {
			result = AT_ERROR;
		}
		break;

	case AT_SENSOR_QUERY_BATTERY_MONITORING:
		if (AT_argc != 0) {
			result = AT_ERROR;
		} else {
			AT_printf("0..1, 2100..3300, 2100..3300\r\n");
		}
		break;

	case AT_SENSOR_GET_BATTERY_MONITORING:
		if (AT_argc != 0) {
			result = AT_ERROR;
		} else {

			AT_printf("%d,%d,%d\r\n", config->battery.monitor,config->battery.level_low, config->battery.level_ok);
		}
		break;

	case AT_SENSOR_SET_BATTERY_MONITORING:
		if (AT_argc == 3) {

			if (AT_atoll(AT_argv[0]) == 1 && AT_atoll(AT_argv[1]) >= 1667
					&& AT_atoll(AT_argv[1]) <= 3300
					&& AT_atoll(AT_argv[2]) >= 2100
					&& AT_atoll(AT_argv[2]) <= 3300) {
				TD_SENSOR_MonitorBattery(true, AT_atoll(AT_argv[1]), AT_atoll(AT_argv[2]), 0);
			} else {
				result = AT_ERROR;
			}

		} else if (AT_argc == 1) {
			if (AT_atoll(AT_argv[0]) == 0) {
				TD_SENSOR_MonitorBattery(false, 0, 0, 0);
			}
		}

		else {
			result = AT_ERROR;
		}
		break;

	case AT_SENSOR_QUERY_BOOT_MONITORING:
		if (AT_argc != 0) {
			result = AT_ERROR;
		} else {
			AT_printf("0..1\r\n");
		}
		break;

	case AT_SENSOR_GET_BOOT_MONITORING:
		if (AT_argc != 0) {
			result = AT_ERROR;
		} else {
			AT_printf("%d\r\n", config->boot.monitor);
		}
		break;

	case AT_SENSOR_SET_BOOT_MONITORING:
		if (AT_argc == 1) {

			if (AT_atoll(AT_argv[0]) == 1) {
				TD_SENSOR_MonitorBoot(true, 0);
			} else if (AT_atoll(AT_argv[0]) == 0) {
				TD_SENSOR_MonitorBoot(false, 0);
			}
		} else {
			result = AT_ERROR;
		}

		break;

	case AT_SENSOR_QUERY_TEMPERATURE_MONITORING:
		if (AT_argc != 0) {
			result = AT_ERROR;
		} else {
			AT_printf("0..1, 0..4294967295, -30..85, -30..85\r\n");
		}
		break;

	case AT_SENSOR_GET_TEMPERATURE_MONITORING:
		if (AT_argc != 0) {
			result = AT_ERROR;
		} else {

			AT_printf("%d,%d,%d,%d\r\n", config->temperature.monitor,
					config->temperature.interval,
					config->temperature.level_low / 10,
					config->temperature.level_high / 10);
		}
		break;

	case AT_SENSOR_SET_TEMPERATURE_MONITORING:
		if (AT_argc == 4) {

			if (AT_atoll(AT_argv[0]) == 1 && AT_atoll(AT_argv[1]) >= 1
					&& AT_atoll(AT_argv[1]) <= 4294967295UL
					&& AT_atoll(AT_argv[2]) >= -30 && AT_atoll(AT_argv[2]) <= 85
					&& AT_atoll(AT_argv[3]) >= -30
					&& AT_atoll(AT_argv[3]) <= 85) {

				TD_SENSOR_MonitorTemperature(true, AT_atoll(AT_argv[1]),
						AT_atoll(AT_argv[2]) * 10, AT_atoll(AT_argv[3]) * 10,
						0);
			} else {
				result = AT_ERROR;
			}

		} else if (AT_argc == 1) {
			if (AT_atoll(AT_argv[0]) == 0) {
				TD_SENSOR_MonitorTemperature(false, 0, 0, 0, 0);
			} else {
				result = AT_ERROR;
			}
		}

		else {
			result = AT_ERROR;
		}
		break;

	case AT_SENSOR_QUERY_RSSI_MONITORING:
		if (AT_argc != 0) {
			result = AT_ERROR;
		} else {
			AT_printf("0..1, -122..14, -122..14\r\n");
		}
		break;

	case AT_SENSOR_GET_RSSI_MONITORING:
		if (AT_argc != 0) {
			result = AT_ERROR;
		} else {

			AT_printf("%d,%d,%d\r\n", config->rssi.monitor,
					config->rssi.level_low, config->rssi.level_ok);

		}
		break;

	case AT_SENSOR_SET_RSSI_MONITORING:
		if (AT_argc == 3) {
			if (AT_atoll(AT_argv[0]) == 1 && AT_atoll(AT_argv[1]) >= -122
					&& AT_atoll(AT_argv[1]) <= 14
					&& AT_atoll(AT_argv[2]) >= -122
					&& AT_atoll(AT_argv[2]) <= 14) {
				if (!TD_SENSOR_MonitorRSSI(true, AT_atoll(AT_argv[1]),
						AT_atoll(AT_argv[2]))) {
					result = AT_ERROR;
				}
			} else {
				result = AT_ERROR;
			}

		} else if (AT_argc == 1) {
			if (AT_atoll(AT_argv[0]) == 0) {
				TD_SENSOR_MonitorRSSI(false, 0, 0);
			} else {
				result = AT_ERROR;
			}
		}

		else {
			result = AT_ERROR;
		}
		break;

	case AT_SENSOR_QUERY_CONNECTION_MONITORING:
		if (AT_argc != 0) {
			result = AT_ERROR;
		} else {
			AT_printf("0..1, 10..4294967295\r\n");
		}
		break;

	case AT_SENSOR_GET_CONNECTION_MONITORING:
		if (AT_argc != 0) {
			result = AT_ERROR;
		} else {

			AT_printf("%d,%d\r\n", config->connection.monitor,
					config->connection.interval);
		}
		break;

	case AT_SENSOR_SET_CONNECTION_MONITORING:
		if (AT_argc == 2) {

			if (AT_atoll(AT_argv[0]) == 1 && AT_atoll(AT_argv[1]) >= 10
					&& AT_atoll(AT_argv[1]) <= 4294967295UL) {
				if (!TD_SENSOR_MonitorConnection(true, AT_atoll(AT_argv[1]))) {
					result = AT_ERROR;
				}
			} else {
				result = AT_ERROR;
			}

		} else if (AT_argc == 1) {
			if (AT_atoll(AT_argv[0]) == 0) {
				TD_SENSOR_MonitorConnection(false, 0);
			} else {
				result = AT_ERROR;
			}
		} else {
			result = AT_ERROR;
		}
		break;

	case AT_SENSOR_QUERY_KEEPALIVE_MONITORING:
		if (AT_argc != 0) {
			result = AT_ERROR;
		} else {
			AT_printf("0..1, 3600..4294967295\r\n");
		}
		break;

	case AT_SENSOR_GET_KEEPALIVE_MONITORING:
		if (AT_argc != 0) {
			result = AT_ERROR;
		} else {

			AT_printf("%d,%d\r\n", config->keepalive.monitor,
					config->keepalive.interval);
		}
		break;

	case AT_SENSOR_SET_KEEPALIVE_MONITORING:
		if (AT_argc == 2) {

			if (AT_atoll(AT_argv[0]) == 1 && AT_atoll(AT_argv[1]) >= 3600
					&& AT_atoll(AT_argv[1]) <= 4294967295UL) {
				if (!TD_SENSOR_MonitorKeepAlive(true, AT_atoll(AT_argv[1]))) {
					result = AT_ERROR;
				}
			} else {
				result = AT_ERROR;
			}

		} else if (AT_argc == 1) {
			if (AT_atoll(AT_argv[0]) == 0) {
				if (!TD_SENSOR_MonitorKeepAlive(false, 0)) {
					result = AT_ERROR;
				}
			} else {
				result = AT_ERROR;
			}
		} else {
			result = AT_ERROR;
		}
		break;

	case AT_SENSOR_QUERY_SWITCH_MONITORING:
		if (AT_argc != 0) {
			result = AT_ERROR;
		} else {
			AT_printf("0..1, 0..5, 0..15, 0..1, 0..1, 0..1, 0..1\r\n");
		}
		break;

	case AT_SENSOR_GET_SWITCH_MONITORING:
		if (AT_argc != 0) {
			result = AT_ERROR;
		} else {
			int i;

			for (i = 0; i < SENSOR_MAX_SWITCH; i++) {
				if (config->switches[i].monitor == true) {
					AT_printf("%d,%d", config->switches[i].port,
							config->switches[i].bit);
					AT_printf("%d,%d", config->switches[i].falling,
							config->switches[i].rising);
					AT_printf("%d,%d \r\n", config->switches[i].pull,
							config->switches[i].pull_state);

				}
			}
		}
		break;

	case AT_SENSOR_SET_SWITCH_MONITORING:
		if (AT_argc == 7) {

			if (AT_atoll(AT_argv[0]) == 1 && AT_atoll(AT_argv[1]) >= 0
					&& AT_atoll(AT_argv[1]) <= 5 && AT_atoll(AT_argv[2]) >= 0
					&& AT_atoll(AT_argv[2]) <= 15 && AT_atoll(AT_argv[3]) >= 0
					&& AT_atoll(AT_argv[3]) <= 1 && AT_atoll(AT_argv[4]) >= 0
					&& AT_atoll(AT_argv[4]) <= 1 && AT_atoll(AT_argv[5]) >= 0
					&& AT_atoll(AT_argv[5]) <= 1 && AT_atoll(AT_argv[6]) >= 0
					&& AT_atoll(AT_argv[6]) <= 1) {
				if (!TD_SENSOR_MonitorSwitch(true,
						(GPIO_Port_TypeDef) AT_atoll(AT_argv[1]),
						AT_atoll(AT_argv[2]), AT_atoll(AT_argv[3]),
						AT_atoll(AT_argv[4]), AT_atoll(AT_argv[5]),
						AT_atoll(AT_argv[6]), 0)) {
					result = AT_ERROR;
				}
			} else {
				result = AT_ERROR;
			}
		} else if (AT_argc == 3) {
			if (AT_atoll(AT_argv[0]) == 0 && AT_atoll(AT_argv[1]) >= 0
					&& AT_atoll(AT_argv[1]) <= 5 && AT_atoll(AT_argv[2]) >= 0
					&& AT_atoll(AT_argv[2]) <= 15) {
				TD_SENSOR_MonitorSwitch(false,
						(GPIO_Port_TypeDef) AT_atoll(AT_argv[1]),
						AT_atoll(AT_argv[2]), 0, 0, 0, 0, 0);
			} else {
				result = AT_ERROR;
			}
		}

		else {
			result = AT_ERROR;
		}
		break;

	case AT_SENSOR_DELETE_FLASH_VARIABLES:
		if (AT_argc == 0) {
			TD_FLASH_DeleteVariables();
		} else {
			result = AT_ERROR;
		}
		break;

	case AT_SENSOR_SENSOR_RESET:
		if (AT_argc == 0) {
			TD_SENSOR_Reset();
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
AT_extension_t sensor_extension = {
		.commands = sensor_commands,
		.init =	sensor_init,
		.help = sensor_help,
		.parse = sensor_parse,
		.persist = sensor_persist
};

/**
 * AT Sensor Module type
 */
ModuleType AT_SENSOR_GetModuleType() {
	return type;
}

