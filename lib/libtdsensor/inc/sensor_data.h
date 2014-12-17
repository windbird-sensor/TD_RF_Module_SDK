/***************************************************************************//**
 * @file
 * @brief API for sending Data frame type to Sensor
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

#ifndef __SENSOR_DATA_H
#define __SENSOR_DATA_H

#include <stdint.h>
#include <stdbool.h>

#include "sensor_send.h"

#ifdef __cplusplus
extern "C" {
#endif

	/***********************************************************************//**
	 * @addtogroup SENSOR_DATA Sensor Data
	 * @{
	 **************************************************************************/

	/***************************************************************************
	*************************   DEFINES   **************************************
	***************************************************************************/

	/** @addtogroup SENSOR_DATA_DEFINES Defines
	 * @{ */

	/** Maximum Sensor data size */
#define TD_SENSOR_DATA_MAX_SIZE					9

	/** @} */

	/***************************************************************************
	 ***********************   ENUMERATIONS   **********************************
	 **************************************************************************/

	/** @addtogroup SENSOR_DATA_ENUMERATIONS Enumerations
	 * @{ */

	/** Phone number index */
	typedef enum {
		PHONE_1 = 0,
		PHONE_2 = 1,
		PHONE_3 = 2,
		PHONE_4 = 3
	} TD_SENSOR_DATA_PhoneIndex_t;

	/** Data Types */
	typedef enum {
		DATA_PHONE = 0,
		DATA_GPS = 1,
		DATA_CUSTOM = 2,
		DATA_PIN = 3
	} TD_SENSOR_DATA_Types_t;

	/** @} */

	/***************************************************************************
	 *************************   PROTOTYPES   **********************************
	 **************************************************************************/

	/** @addtogroup SENSOR_DATA_USER_FUNCTIONS User Functions
	 * @{ */

	bool TD_SENSOR_SendData(TD_SENSOR_DATA_Types_t data_type, uint8_t *data,
		uint8_t length);
	void TD_SENSOR_EncodeCellPhoneNumber(uint8_t index,	uint8_t *phone_number,
		uint8_t *data, uint8_t *length);
	bool TD_SENSOR_SendDataCellPhoneNumber(uint8_t index,
		uint8_t *phone_number);
	bool TD_SENSOR_EncodeLocalVoltage(uint32_t voltage, uint8_t *data,
		uint8_t *length);
	bool TD_SENSOR_EncodeLocalTemperature(int32_t temperature, uint8_t *data,
		uint8_t *length);
	bool TD_SENSOR_SendDataPIN(uint8_t index, uint8_t *code_number);

	/***********************************************************************//**
	 * @brief
	 *   Encode a PIN number on Sensor.
	 *
	 * @param[in] index
	 *  Code number index. Up to 256 PIN code numbers can be registered
	 *
	 * @param[in] pin
	 *   PIN number i.e. 1234.
	 *
	 * @param[out] data
	 *   Pointer to the buffer that will receive the encoded code number.
	 *
	 * @param[out] length
	 *   Pointer to an unsigned character that will receive the encoded length.
	 **************************************************************************/
	static __INLINE void TD_SENSOR_EncodePIN(uint8_t index,	uint8_t *pin,
		uint8_t *data, uint8_t *length)
	{
		TD_SENSOR_EncodeCellPhoneNumber(index, pin, data, length);
	}

	/***************************************************************************
	 * @brief
	 *   Set a transmission profile for data frame type.
	 *
	 * @param[in] repetition
	 *	Number of repetitions.
	 *
	 * @param[in] interval
	 *	Interval between two repetitions in seconds.
	 **************************************************************************/
	static void __INLINE TD_SENSOR_SetDataTransmissionProfile(
		uint8_t repetition,
		uint32_t interval)
	{
		TD_SENSOR_SetTransmissionProfile(SRV_FRM_DATA, repetition, interval);
	}

	/***********************************************************************//**
	 * @brief
	 *   Register a new cell phone number for sending SMS on Sensor.
	 *
	 * @param[in] index
	 *	Phone number index. Up to 4 cell phone number can be registered.
	 *
	 * @param[in] phone_number
	 *   Pointer to a buffer containing the phone number. ie 0601020304.
	 *
	 * @return
	 *   Returns true if the data has been sent (i.e. the gateway has
	 *   acknowledged the request) False if the acknowledgment from the gateway
	 *   was never received.
	 *
	 * @note
	 *   This function is depreciated, use TD_SENSOR_SendDataCellPhoneNumber
	 *   instead.
	 **************************************************************************/
	static bool __INLINE TD_SENSOR_SetCellPhoneNumber(
		TD_SENSOR_DATA_PhoneIndex_t index,
		uint8_t *phone_number)
	{
		return TD_SENSOR_SendDataCellPhoneNumber(index, phone_number);
	}

	/** @} */

	/** @} (end addtogroup SENSOR_DATA) */

#ifdef __cplusplus
}
#endif

#endif // __SENSOR_DATA_H
