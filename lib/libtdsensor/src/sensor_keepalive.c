/***************************************************************************//**
 * @file
 * @brief API for sending KeepAlive frame type to Sensor
 * @author Telecom Design S.A.
 * @version 1.3.0
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2013-2014 Telecom Design S.A., http://www.telecomdesign.fr</b>
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

#include <stdbool.h>
#include <stdint.h>

#include <td_module.h>
#include <td_sigfox.h>
#include <td_measure.h>
#include <td_utils.h>

#include "td_sensor.h"
#include "sensor_send.h"
#include "sensor_data.h"
#include "sensor_keepalive.h"

/***************************************************************************//**
 * @addtogroup SENSOR_KEEPALIVE Sensor Keep-Alive
 * @brief
 *  Sensor API for sending a Keep-Alive Frame.
 *
 *  Temperature, battery level information and next expected keep-alive time are
 *  sent alongside with this frame.
 * @{
 ******************************************************************************/

/*******************************************************************************
 ************************   DEFINES   *****************************************
 ******************************************************************************/

/** @addtogroup SENSOR_KEEPALIVE_DEFINES Defines
 * @{ */

/** Keep-alive payload size in bytes */
#define MAX_KEEPALIVE_PAYLOAD_SIZE 		10

/** @} */

/*******************************************************************************
 **************************  PRIVATE FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup AT_SENSOR_SEND_LOCAL_FUNCTIONS Local Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Send a KEEPALIVE frame to Sensor.
 *
 * @param[in] type
 *   Sensor keep alive frame sub type.
 *
 * @param[in] interval
 *   Interval in seconds at which to send keepalives.
 *
 * @param[in] data
 *  Pointer to the buffer containing the custom data to send.
 *
 * @param[in] length
 *  Custom data size in bytes.
 *
 * @return
 *   True if the data has been sent over the SIGFOX Network.
 ******************************************************************************/
static bool SendKeepAlive(TD_SENSOR_KEEPALIVE_types_t type, uint8_t interval,
	uint8_t *data, uint8_t length)
{
	TD_SENSOR_Frame_t frame;
	uint8_t voltage, temperature;
	uint8_t *p, *start;
	uint8_t gps[6] = {1, 2, 3, 4, 5, 6};
	int i;

	TD_SENSOR_EncodeLocalVoltage(TD_SIGFOX_PowerVoltageExtended(), &voltage,
		NULL);
	TD_SENSOR_EncodeLocalTemperature(TD_MEASURE_TemperatureExtended(),
		&temperature, NULL);

	p = start = &frame.payload[0];
	*p++ = type;
	switch (type) {
	case KEEPALIVE_VOLTAGE_TEMP:
		*p++ = interval;
		*p++ = voltage;
		*p++ = temperature;
		break;

	case KEEPALIVE_VOLTAGE_TEMP_GEOLOC:
		*p++ = interval;
		*p++ = voltage;
		*p++ = temperature;

		// TODO : GPS simulation, call the right callback
		for (i = 0; i < sizeof (gps); i++) {
			*p++ = gps[i];
		}
		break;

	case KEEPALIVE_GEOLOC:
		*p++ = interval;

		// TODO : GPS simulation, call the right callback
		for (i = 0; i < sizeof (gps); i++) {
			*p++ = gps[i];
		}
		break;

	case KEEPALIVE_CUSTOM:
	default:
		*p++ = interval;
		break;
	}

	// If there is place to add custom data to payload
	if (((p - start) + length) <= MAX_KEEPALIVE_PAYLOAD_SIZE) {

		// Copy custom data if any
		for (i = 0; i < length; i++) {
			*p++ = data[i];
		}
	} else {
		return false;
	}
	return TD_SENSOR_SendUDM(0, SRV_FRM_KEEPALIVE, &frame, p - start);
}

/** @} */

/*******************************************************************************
 **************************  PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup SENSOR_KEEPALIVE_USER_FUNCTIONS User Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Send a KEEPALIVE frame to Sensor.
 *
 * @return
 *   Returns true if the data has been sent over the SIGFOX network, false
 *   otherwise.
 ******************************************************************************/
bool TD_SENSOR_SendKeepAlive(void)
{
	TD_SENSOR_Configuration_t *config;
	uint8_t interval;

	config = TD_SENSOR_GetModuleConfiguration();
	interval = config->keepalive.monitor ? config->keepalive.interval : 0;
	return SendKeepAlive(KEEPALIVE_VOLTAGE_TEMP, interval, 0, 0);
}

/***************************************************************************//**
 * @brief
 *   Send a KEEPALIVE frame to Sensor.
 *
 * @param[in] type
 *   Sensor keep alive frame sub type.
 *
 * @param[in] interval
 *   Interval in seconds at which to send keepalives.
 *
 * @param[in] data
 *  Pointer to the buffer containing the custom data to send.
 *
 * @param[in] length
 *  Custom data size in bytes.
 *
 * @return
 *   True if the data has been sent over the SIGFOX Network.
 ******************************************************************************/
bool TD_SENSOR_SendKeepAliveExt(TD_SENSOR_KEEPALIVE_types_t type,
	uint8_t interval, uint8_t *data, uint8_t length)
{
	return SendKeepAlive(type, interval, data, length);
}

/***************************************************************************//**
 * @brief
 *   Send a KEEPALIVE frame with voltage and temperature to Sensor.
 *
 * @param[in] id
 *   Device ID to use as keepalive origin.
 *
 * @param[in] interval
 *   Interval in seconds at which to send keepalives.
 *
 * @param[in] voltage
 *  Pointer to the buffer containing the custom data to send.
 *
 * @param[in] temperature
 *  Custom data size in bytes.
 *
 * @return
 *   True if the data has been sent over the SIGFOX Network.
 ******************************************************************************/
bool TD_SENSOR_SendKeepAliveVoltageTemperatureFor(uint8_t id,
	uint8_t interval, uint8_t voltage, uint8_t temperature)
{
	TD_SENSOR_Frame_t frame;
	frame.payload[0] = KEEPALIVE_VOLTAGE_TEMP;
	frame.payload[1] = interval;
	frame.payload[2] = voltage;
	frame.payload[3] = temperature;
	return TD_SENSOR_SendUDM(id, SRV_FRM_KEEPALIVE, &frame, 4);
}

/** @} */

/** @} */

/** @} (end addtogroup SENSOR_KEEPALIVE) */
