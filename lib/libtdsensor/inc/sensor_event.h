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

#ifndef __SENSOR_EVENT_H
#define __SENSOR_EVENT_H

#include <stdint.h>
#include <stdbool.h>

#include "sensor_send.h"

#ifdef __cplusplus
extern "C" {
#endif

	/***********************************************************************//**
	 * @addtogroup SENSOR_EVENT Sensor Event
	 * @{
	 **************************************************************************/

	/***************************************************************************
	*************************   DEFINES   **************************************
	***************************************************************************/

	/** @addtogroup SENSOR_EVENT_DEFINES Defines
	 * @{ */

	/** Maximum Sensor custom event data size */
#define TD_SENSOR_EVENT_CUSTOM_MAX_SIZE	9

	/** Maximum Sensor extended boot event custom data size */
#define TD_SENSOR_EVENT_BOOT_MAX_SIZE	7

	/** @} */

	/***************************************************************************
	 ***********************   ENUMERATIONS   **********************************
	 **************************************************************************/

	/** @addtogroup SENSOR_EVENT_ENUMERATIONS Enumerations
	 * @{ */

	/** Event Type*/
	typedef enum {
		EVENT_BATTERY_LOW = 0,
		EVENT_BATTERY_OK = 1,
		EVENT_CONNECTION_LOST = 2,
		EVENT_CONNECTION_OK = 3,
		EVENT_RSSI_LOW = 4,
		EVENT_RSSI_OK = 5,
		EVENT_TEMP_LOW = 6,
		EVENT_TEMP_HIGH = 7,
		EVENT_TEMP_OK = 8,
		EVENT_BOOT = 9,
		EVENT_SWITCH_ON = 10,
		EVENT_SWITCH_OFF = 11,
		EVENT_ACCELERO_MOVE = 12,
		EVENT_CUSTOM_1 = 128,
		EVENT_CUSTOM_2 = 129,
		EVENT_CUSTOM_3 = 130,
		EVENT_CUSTOM_4 = 131,
		EVENT_CUSTOM_5 = 132,
		EVENT_CUSTOM_6 = 133,
		EVENT_CUSTOM_7 = 134,
		EVENT_CUSTOM_8 = 135,
		EVENT_CUSTOM_9 = 136,
		EVENT_CUSTOM_10 = 137,
		EVENT_CUSTOM_11 = 138,
		EVENT_CUSTOM_12 = 139,
		EVENT_CUSTOM_13 = 140,
		EVENT_CUSTOM_14 = 141,
		EVENT_CUSTOM_15 = 142,
		EVENT_CUSTOM_16 = 143,
		EVENT_CUSTOM_17 = 144,
		EVENT_CUSTOM_18 = 145,
		EVENT_CUSTOM_19 = 146,
		EVENT_CUSTOM_20 = 147,
		EVENT_CUSTOM_21 = 148,
		EVENT_CUSTOM_22 = 149,
		EVENT_CUSTOM_23 = 150,
		EVENT_CUSTOM_24 = 151,
		EVENT_CUSTOM_25 = 152,
	}
	TD_SENSOR_EVENT_Types_t;

	/** Boot reason for Event boot */
	typedef enum {
		BOOT_EVENT_POR = 0,
		BOOT_EVENT_WDOG = 1,
		BOOT_EVENT_BROR = 2
	} TD_SENSOR_EVENT_BootReason_t;

	/** @} */

	/***************************************************************************
	 *************************   PROTOTYPES   **********************************
	 **************************************************************************/

	/** @addtogroup SENSOR_EVENT_USER_FUNCTIONS User Functions
	 * @{ */

	bool TD_SENSOR_SendEvent(TD_SENSOR_EVENT_Types_t event_type, uint8_t *data,
		uint8_t length);
	bool TD_SENSOR_SendEventBattery(bool state, uint32_t battery_level);
	bool TD_SENSOR_SendEventRSSI(bool state, uint8_t id);
	bool TD_SENSOR_SendEventConnection(bool state, uint8_t id);
	bool TD_SENSOR_SendEventTemperature(uint8_t state);
	bool TD_SENSOR_SendEventTemperatureExt(uint8_t state,
		int16_t temperature_level);
	bool TD_SENSOR_SendEventBootExt(uint8_t cause, uint8_t custom_cause,
		uint8_t *data, uint8_t length);
	bool TD_SENSOR_SendEventSwitch(uint8_t port, uint8_t bit, bool state);
	bool TD_SENSOR_SendEventFor(TD_SENSOR_EVENT_Types_t event_type, uint8_t id,
		uint8_t *data,	uint8_t length);

	/***********************************************************************//**
	 * @brief
	 *   Send a boot event to Sensor.
	 *
	 * @return
	 *   Returns true if the data has been sent over the SIGFOX network, false
	 *   otherwise.
	 **************************************************************************/
	static bool __INLINE TD_SENSOR_SendEventBoot(void)
	{
		return TD_SENSOR_SendEvent(EVENT_BOOT, 0, 0);
	}

	/***********************************************************************//**
	 * @brief
	 *   Set a transmission profile for an event frame type.
	 *
	 * @param[in] repetition
	 *	Number of repetitions.
	 *
	 * @param[in] interval
	 *	Interval between two repetitions in seconds.
	 **************************************************************************/
	static void __INLINE TD_SENSOR_SetEventTransmissionProfile(
		uint8_t repetition,
		uint32_t interval)
	{
		TD_SENSOR_SetTransmissionProfile(SRV_FRM_EVENT, repetition, interval);
	}

	/** @} */

	/** @} (end addtogroup SENSOR_EVENT) */

#ifdef __cplusplus
}
#endif

#endif // __SENSOR_EVENT_H
