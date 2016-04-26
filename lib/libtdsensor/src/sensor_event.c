/***************************************************************************//**
 * @file
 * @brief API for sending Event frame type to Sensor
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
#include <string.h>

#include <em_assert.h>

#include "td_sensor_utils.h"
#include "sensor_send.h"
#include "sensor_data.h"
#include "sensor_event.h"

/***************************************************************************//**
 * @addtogroup SENSOR_EVENT Sensor Event
 * @brief Sensor API for sending an Event frame
 *
 * @details
 *
 *	Event Type values are:
 *
 *	- BATTERY_LOW & BATTERY_OK
 *
 *	Indicate that the device is running out of Battery. The BATTERY_LOW message
 *	is sent whenever the battery voltage goes below a specified threshold.
 *	The BATTERY_OK event is being sent after a BATTERY_LOW event has been
 *	emitted when batteries are changed or recharged. The Data field is used for
 *	BATTERY_OK event and indicated battery level.
 *
 *	- CONNECTION_LOST & CONNECTION_OK
 *
 *	Indicate whether the connection with a device has been lost or not. The
 *	CONNECTION_LOST event indicate that the device didn't send a keep-alive on
 *	time.
 *	CONNECTION_OK means the connection with the device has been recovered. The
 *	Data field is used for both messages and indicates which device is
 *	concerned.
 *
 *	- RSSI_LOW & RSSI_OK
 *
 *	Indicate whether the RSSI level is low or not. The RSSI level from all
 *	received message is being compared to a threshold and a RSSI_LOW event is
 *	emitted when the RSSI level is below it. When RSSI level comes back above
 *	the RSSI threshold, the RSSI_OK event is emitted. The Data field is used
 *	for both messages and indicates which device is concerned.
 *
 *	- TEMP_LOW & TEMP_HIGH & TEMP_OK
 *
 *	The on-board temperature sensor can provides accurate temperature
 *	measurement. A TEMP_LOW and TEMP_HIGH event can be emitted whenever the
 *	temperature
 *	falls below or rise above the corresponding threshold. When going back to
 *	normal temperature value a TEMP_OK event is emitted.
 *
 *	- BOOT
 *
 *	This message should be sent when the device is booting, ie on Init.
 *
 *	- SWITCH_ON & SWITCH_OFF
 *
 *	Indicate whether a switch has been turned ON or OFF. The Data field is used
 *	for both messages and contains port and bit information for the switch.
 * @{
 ******************************************************************************/

/*******************************************************************************
 *************************   DEFINES   *****************************************
 ******************************************************************************/

/** @addtogroup SENSOR_EVENT_DEFINES Defines
 * @{ */

/** Event payload size in bytes */
#define EVENT_PAYLOAD_SIZE			10

/** @} */

/*******************************************************************************
 **************************  PUBLIC FUNCTIONS   ********************************
 ******************************************************************************/

/** @addtogroup SENSOR_EVENT_USER_FUNCTIONS User Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Send an event frame to Sensor.
 *
 * @param[in] event_type
 *   Event Type.
 *
 * @param[in] data
 *   Pointer to the buffer containing the data to be sent.
 *
 * @param[in] length
 *   Length of the data to be sent in bytes.
 *
 * @return
 *   Returns true if the data has been sent over the SIGFOX network, false
 *   otherwise.
 ******************************************************************************/
bool TD_SENSOR_SendEvent(TD_SENSOR_EVENT_Types_t event_type, uint8_t *data,
	uint8_t length)
{
	TD_SENSOR_Frame_t frame;

	EFM_ASSERT(length < TD_SENSOR_PAYLOAD_SIZE - 1);
	frame.payload[0] = event_type;
	memcpy(&frame.payload[1], data, length);
	return TD_SENSOR_SendUDM(0, SRV_FRM_EVENT, &frame, 1 + length);
}


/***************************************************************************//**
 * @brief
 *   Send an event frame to Sensor for a specific entry id.
 *
 * @param[in] event_type
 *   Event Type.
 *
 *  @param[in] id
 *   ID of the device to send to.
 *
 * @param[in] data
 *   Pointer to the buffer containing the data to be sent.
 *
 * @param[in] length
 *   Length of the data to be sent in bytes.
 *
 * @return
 *   Returns true if the data has been sent over the SIGFOX network, false
 *   otherwise.
 ******************************************************************************/
bool TD_SENSOR_SendEventFor(TD_SENSOR_EVENT_Types_t event_type, uint8_t id,
	uint8_t *data,	uint8_t length)
{
	TD_SENSOR_Frame_t frame;

	EFM_ASSERT(length < TD_SENSOR_PAYLOAD_SIZE - 1);
	frame.payload[0] = event_type;
	memcpy(&frame.payload[1], data, length);
	return TD_SENSOR_SendUDM(id, SRV_FRM_EVENT, &frame, 1 + length);
}

/***************************************************************************//**
 * @brief
 *   Send a battery event to Sensor.
 *
 *  @param[in] state
 *  True to send EVENT_BATTERY_OK, false to send EVENT_BATTERY_LOW.
 *
 *  @param[in] battery_level
 *   battery_level to be sent alongside
 *
 * @return
 *   Returns true if the data has been sent over the SIGFOX network, false
 *   otherwise.
 ******************************************************************************/
bool TD_SENSOR_SendEventBattery(bool state, uint32_t battery_level)
{
	uint8_t data;

	if (!TD_SENSOR_EncodeLocalVoltage(battery_level, &data, NULL)) {
		data = 0;
	}
	return TD_SENSOR_SendEvent(
		state ? EVENT_BATTERY_OK : EVENT_BATTERY_LOW,
		&data,
		1);
}

/***************************************************************************//**
 * @brief
 *   Send an RSSI Low event to Sensor.
 *
 *  @param[in] state
 *   True to send EVENT_RSSI_OK, false to send EVENT_RSSI_LOW.
 *
 *  @param[in] id
 *   ID of the device for which the RSSI is low.
 *
 * @return
 *   Returns true if the data has been sent over the SIGFOX network, false
 *   otherwise.
 ******************************************************************************/
bool TD_SENSOR_SendEventRSSI(bool state, uint8_t id)
{
	return TD_SENSOR_SendEvent(
		state ? EVENT_RSSI_OK : EVENT_RSSI_LOW,
		&id,
		sizeof (id));
}

/***************************************************************************//**
 * @brief
 *   Send a Connection event to Sensor.
 *
 *  @param[in] state
 *  True to send EVENT_CONNECTION_OK, false to send EVENT_CONNECTION_LOST.
 *
 *  @param[in] id
 *   ID of the device for which the connection has been lost.
 *
 * @return
 *   Returns true if the data has been sent over the SIGFOX network, false
 *   otherwise.
 ******************************************************************************/
bool TD_SENSOR_SendEventConnection(bool state, uint8_t id)
{
	return TD_SENSOR_SendEvent(
		state ? EVENT_CONNECTION_OK : EVENT_CONNECTION_LOST,
		&id,
		sizeof (id));
}

/***************************************************************************//**
 * @brief
 *   Send a temperature event to Sensor.
 *
 *  @param[in] state
 *  0 is below min , 1 is ok, 2 is above max
 *
 * @return
 *   Returns true if the data has been sent over the SIGFOX network, false
 *   otherwise.
 ******************************************************************************/
bool TD_SENSOR_SendEventTemperature(uint8_t state)
{
	return TD_SENSOR_SendEventTemperatureExt(state, 0);
}

/***************************************************************************//**
 * @brief
 *   Send a temperature event with level to Sensor.
 *
 *  @param[in] state
 *  0 is below min , 1 is ok, 2 is above max
 *
 *  @param[in] temperature_level
 *  temperature_level to be sent alongside
 *
 * @return
 *   Returns true if the data has been sent over the SIGFOX network, false
 *   otherwise.
 ******************************************************************************/
bool TD_SENSOR_SendEventTemperatureExt(uint8_t state, int16_t temperature_level)
{
	TD_SENSOR_EVENT_Types_t event;
	uint8_t data;

	if (!TD_SENSOR_EncodeLocalTemperature(temperature_level, &data, NULL)) {
		data = 0;
	}
	switch (state) {
	case 0:
		event = EVENT_TEMP_LOW;
		break;

	case 1:
		event = EVENT_TEMP_OK;
		break;

	case 2:
		event = EVENT_TEMP_HIGH;
		break;

	default:
		return false;
	}
	return TD_SENSOR_SendEvent(event, &data, 1);
}

/***************************************************************************//**
 * @brief
 *   Send a boot event to Sensor with corresponding information.
 *
 * @param[in] reason
 *   The hardware boot reason.
 *
 * @param[in] cause
 *   The software boot cause.
 *
 * @param[in] data
 *   Pointer to the buffer containing data information bits saved across the
 *   boot.
 *
 * @param[in] length
 *   Length in bits of the data buffer.
 *
 * @return
 *   Returns true if the data has been sent over the SIGFOX network, false
 *   otherwise.
 ******************************************************************************/
bool TD_SENSOR_SendEventBootExt(uint8_t reason, uint8_t cause, uint8_t *data,
	uint8_t length)
{
	uint8_t frame[9];
	int i;

	EFM_ASSERT(length <= 7 * 8);
	i = length >> 3;
	if (length & 0x07) {

		// Count for trailing bits
		i++;
	}
	frame[0] = reason;
	frame[1] = cause;
	for (; i; i--) {
		frame[2 + i] = data[i];
	}
	return TD_SENSOR_SendEvent(EVENT_BOOT, frame, i + 2);
}

/***************************************************************************//**
 * @brief
 *   Send a switch event to Sensor.
 *
 * @param[in] port
 * 	Switch port.
 *
 * @param[in] bit
 * 	Switch bit.
 *
 * @param[in] state
 * 	Switch state.
 *
 * @return
 *   Returns true if the data has been sent over the SIGFOX network, false
 *   otherwise.
 ******************************************************************************/
bool TD_SENSOR_SendEventSwitch(uint8_t port, uint8_t bit, bool state)
{
	uint8_t data;

	data = ((port & 0xf) << 4) | (bit & 0xf);
	return TD_SENSOR_SendEvent(
		state ? EVENT_SWITCH_ON : EVENT_SWITCH_OFF,
		&data,
		sizeof (data));
}

/** @} */

/** @} (end addtogroup SENSOR_EVENT) */
