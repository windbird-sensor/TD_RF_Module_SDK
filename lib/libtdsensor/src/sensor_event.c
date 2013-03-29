/***************************************************************************//**
 * @file sensor_event.c
 * @brief API for sending Event frame type to Sensor
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
#include <stdbool.h>
#include <stdint.h>
#include "sensor_send.h"
#include "sensor_event.h"
#include "sensor_event_private.h"

/***************************************************************************//**
 * @addtogroup SENSOR_EVENT Sensor Event
 * @brief Sensor API for sending an Event frame
 *
 * @details
 *
 *
 *	Event Type values are:
 *
 *	- BATTERY_LOW & BATTERY_OK
 *
 *	Indicate that the device is running out of Battery. The BATTERY_LOW message is sent whenever the battery voltage goes below a specified threshold.
 *	The BATTERY_OK event is being sent after a BATTERY_LOW event has been emitted when batteries are changed or recharged. The Data field is used for
 *	BATTERY_OK event and indicated battery level.
 *
 *	- CONNECTION_LOST & CONNECTION_OK
 *
 *	Indicate whether the connection with a device has been lost or not. The CONNECTION_LOST event indicate that the device didn't send a keep-alive on time.
 *	CONNECTION_OK means the connection with the device has been recovered. The Data field is used for both messages and indicates which device is concerned.
 *
 *
 *
 *	- RSSI_LOW & RSSI_OK
 *
 *	Indicate whether the RSSI level is low or not. The RSSI level from all received message is being compared to a threshold and a RSSI_LOW event is
 *	emitted when the RSSI level is below it. When RSSI level comes back above the RSSI threshold, the RSSI_OK event is emitted. The Data field is used
 *	for both messages and indicates which device is concerned.
 *
 *	- TEMP_LOW & TEMP_HIGH & TEMP_OK
 *
 *	The on-board temperature sensor can provides accurate temperature measurement. A TEMP_LOW and TEMP_HIGH event can be emitted whenever the temperature
 *	falls below or rise above the corresponding threshold. When going back to normal temperature value a TEMP_OK event is emitted.
 *
 *	- BOOT
 *
 *	This message should be sent when the device is booting, ie on Init.
 *
 *	- SWITCH_ON & SWITCH_OFF
 *
 *	Indicate whether a switch has been turned ON or OFF. The Data field is used for both messages and contains port and bit information
 *	for the switch.
 * @{
 ******************************************************************************/

/*******************************************************************************
 *************************   DEFINES   *****************************************
 ******************************************************************************/

/** @addtogroup SENSOR_EVENT_DEFINES Defines
 * @{ */

#define EVENT_PAYLOAD_SIZE 1

#define EVENT_DEFAULT_REPETITON 3 	///<Default retransmission repetition
#define EVENT_DEFAULT_INTERVAL 120	///<Default retransmission interval in seconds

/** @} */

/*******************************************************************************
 *************************   PRIVATE VARIABLES   ****************************************
 ******************************************************************************/

/** @addtogroup SENSOR_EVENT_PRIVATE_VARIABLES Private Variables
 * @{ */

static TransmitProfile event_profile = { EVENT_DEFAULT_REPETITON,
		EVENT_DEFAULT_INTERVAL };
static uint8_t event_stamp = -1;

/** @} */

/** @cond TD_PRIVATE */

/*******************************************************************************
 **************************  PRIVATE FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup SENSOR_EVENT_PRIVATE_FUNCTIONS Private Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Send a EVENT frame to Sensor.
 *
 * @param[in] event
 *   Event Type
 *
 * @param[in] data
 *   Data to be sent
 *
 * @return
 *   True if the data has been sent over the Sigfox Network
 ******************************************************************************/

static bool TD_SENSOR_SendEvent(SensorEventType event, uint8_t data) {
	SRV_FRAME_EVENT frame;
	frame.type = event;

	event_stamp = (event_stamp & 0x07) + 1;

	if (event == EVENT_BATTERY_OK || event == EVENT_CONNECTION_LOST
			|| event == EVENT_CONNECTION_OK || event == EVENT_RSSI_LOW
			|| event == EVENT_RSSI_OK || event == EVENT_SWITCH_ON
			|| event == EVENT_SWITCH_OFF) {
		frame.data = data;
		return TD_SENSOR_Send(&event_profile, SRV_FRM_EVENT, event_stamp,
				(uint8_t *) &frame, 2);
	} else {
		return TD_SENSOR_Send(&event_profile, SRV_FRM_EVENT, event_stamp,
				(uint8_t *) &frame, 1);
	}

}

/** @} */

/*******************************************************************************
 **************************  PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup SENSOR_EVENT_PUBLIC_FUNCTIONS Public Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Send the Battery event to Sensor
 *
 *  @param[in] state
 *  True to send EVENT_BATTERY_OK, false to send EVENT_BATTERY_LOW
 *
 *  @param[in] battery_level
 *   If battery event is EVENT_BATTERY_OK, then battery level must also be provided.
 *
 * @return
 *   True if the data has been sent (ie. the gateway has acknowledged the request)
 *   False if the ack from the gateway was never received.
 ******************************************************************************/

bool TD_SENSOR_SendEventBattery(bool state, uint8_t battery_level) {
	if (state) {
		return TD_SENSOR_SendEvent(EVENT_BATTERY_OK, battery_level);
	} else {
		return TD_SENSOR_SendEvent(EVENT_BATTERY_LOW, 0);
	}
}

/***************************************************************************//**
 * @brief
 *   Send the event RSSI Low to Sensor
 *
 *  @param[in] state
 *   True to send EVENT_RSSI_OK, false to send EVENT_RSSI_LOW
 *
 *  @param[in] EntryID
 *   EntryID of which the RSSI is low.
 *
 * @return
 *   always true
 ******************************************************************************/
bool TD_SENSOR_SendEventRSSI(
		bool state, uint8_t EntryID) {
	if (state) {
		return TD_SENSOR_SendEvent(EVENT_RSSI_OK, EntryID);
	} else {
		return TD_SENSOR_SendEvent(EVENT_RSSI_LOW, EntryID);
	}
}

/***************************************************************************//**
 * @brief
 *   Send the Connection event to Sensor
 *
 *  @param[in] state
 *  True to send EVENT_CONNECTION_OK, false to send EVENT_CONNECTION_LOST
 *
 *  @param[in] EntryID
 *   EntryID of which the connection has been lost
 *
 * @return
 *   always true
 ******************************************************************************/
bool TD_SENSOR_SendEventConnection(bool state, uint8_t EntryID)
{
	if (state) {
		return TD_SENSOR_SendEvent(EVENT_CONNECTION_OK, EntryID);
	} else {
		return TD_SENSOR_SendEvent(EVENT_CONNECTION_LOST, EntryID);
	}
}

/***************************************************************************//**
 * @brief
 *   Send the Temperature event to Sensor
 *
 *  @param[in] state
 *  0 is below min , 1 is ok, 2 is above max
 *
 * @return
 *   True if the data has been sent (ie. the gateway has acknowledged the request)
 *   False if the ack from the gateway was never received.
 ******************************************************************************/
bool TD_SENSOR_SendEventTemperature(uint8_t state)
{
	switch (state) {
	case 0:
		return TD_SENSOR_SendEvent(EVENT_TEMP_LOW, 0);
		break;

	case 1:
		return TD_SENSOR_SendEvent(EVENT_TEMP_OK, 0);
		break;

	case 2:
		return TD_SENSOR_SendEvent(EVENT_TEMP_HIGH, 0);
		break;
	}
	return false;
}

/***************************************************************************//**
 * @brief
 *   Send the Boot event to Sensor
 *
 * @return
 *   True if the data has been sent (ie. the gateway has acknowledged the request)
 *   False if the ack from the gateway was never received.
 ******************************************************************************/
bool TD_SENSOR_SendEventBoot()
{
	return TD_SENSOR_SendEvent(EVENT_BOOT, 0);
}

/***************************************************************************//**
 * @brief
 *   Send the Boot event to Sensor
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
 *   True if the data has been sent (ie. the gateway has acknowledged the request)
 *   False if the ack from the gateway was never received.
 ******************************************************************************/
bool TD_SENSOR_SendEventSwitch(	uint8_t port, uint8_t bit, bool state)
{
	uint8_t data = 0;
	data = ((port & 0xF) << 4) | (bit & 0xF);

	if (!state) {
		return TD_SENSOR_SendEvent(EVENT_SWITCH_ON, data);
	} else {
		return TD_SENSOR_SendEvent(EVENT_SWITCH_OFF, data);
	}
}

/***************************************************************************//**
 * @brief
 *   Set a transmission profile to a given frame type
 *
 * @param[in] repetition
 *	Number of repetition
 *
 * @param[in] interval
 *	Interval between two repetitions in seconds.
 *
 ******************************************************************************/

void TD_SENSOR_SetEventTransmissionProfile(uint8_t repetition,uint32_t interval)
{
	event_profile.repetition = repetition;
	event_profile.interval = interval;
}

/** @} */

/** @endcond */

/** @} */
