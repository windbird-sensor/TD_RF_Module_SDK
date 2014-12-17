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

#include <stdbool.h>
#include <stdint.h>

#include <em_assert.h>

#include <td_utils.h>

#include "td_sensor_utils.h"
#include "sensor_send.h"
#include "sensor_data.h"

/***************************************************************************//**
 * @addtogroup SENSOR_DATA Sensor Data
 * @brief Sensor API for sending Data frame
 *
 * @details
 * 	 Data frames allows you to send data to Sensor. Right now, only phone
 * 	 numbers are handled.
 * @{
 ******************************************************************************/

/*******************************************************************************
*************************   DEFINES   ******************************************
*******************************************************************************/

/** @addtogroup SENSOR_DATA_DEFINES Defines
 * @{ */

/* Payload size to be able to transport a phone number */
#define DATA_PAYLOAD_SIZE		10

/** @} */

/*******************************************************************************
 **************************  PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup SENSOR_DATA_USER_FUNCTIONS User Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Send a DATA frame to Sensor.
 *
 * @param[in] data_type
 *   Type of Data being sent. Used by Sensor for decoding. Refer to the
 *   SensorDataType enumeration for allowed values.
 *
 * @param[in] data
 *   Pointer to the data to be sent. Maximum allowed length for data is 16
 *   bytes.
 *
 * @param[in] length
 *   The length in bytes of the data to be sent.
 *
 * @return
 *   Returns true if the data has been sent over the SIGFOX network, false
 *   otherwise.
 ******************************************************************************/
bool TD_SENSOR_SendData(TD_SENSOR_DATA_Types_t data_type, uint8_t *data,
	uint8_t length)
{
	TD_SENSOR_Frame_t frame;

	EFM_ASSERT(length < TD_SENSOR_PAYLOAD_SIZE - 1);
	frame.payload[0] = data_type;
	memcpy(&frame.payload[1], data, length);
	return TD_SENSOR_SendUDM(0, SRV_FRM_DATA, &frame, 1 + length);
}

/***************************************************************************//**
 * @brief
 *   Register a new cell phone number for sending SMS on Sensor.
 *
 * @param[in] index
 *	Phone number index. Up to 256 phone numbers can be registered.
 *
 * @param[in] phone_number
 *   Pointer to a buffer containing the phone number. ie 0601020304.
 *
 * @return
 *   Returns true if the data has been sent over the SIGFOX network, false
 *   otherwise.
 ******************************************************************************/
bool TD_SENSOR_SendDataCellPhoneNumber(uint8_t index, uint8_t *phone_number)
{
	uint8_t frame[TD_SENSOR_PAYLOAD_SIZE - 1];
	uint8_t length;

	TD_SENSOR_EncodeCellPhoneNumber(index, phone_number, frame, &length);
	return TD_SENSOR_SendData(DATA_PHONE, frame, length);
}

/***************************************************************************//**
 * @brief
 *   Encode a cell phone number for sending SMS on Sensor.
 *
 * @param[in] index
 *   Phone number index. Up to 256 phone numbers can be registered.
 *
 * @param[in] phone_number
 *   Pointer to a null-terminated buffer containing the phone number, i.e.
 *   0601020304.
 *
 * @param[in] data
 *   Pointer to the buffer that will receive the encoded phone number.
 *
 * @param[in] length
 *   Pointer to a byte that will receive the encoded phone number length.
 ******************************************************************************/
void TD_SENSOR_EncodeCellPhoneNumber(uint8_t index, uint8_t *phone_number,
	uint8_t *data, uint8_t *length)
{
	int i;
	uint8_t msb = 0, lsb;

	EFM_ASSERT(phone_number != 0 && data != 0 && length != 0);
	*data++ = index;
	for (i = 0;
		*phone_number && i < (TD_SENSOR_PAYLOAD_SIZE - 1) * 2;
		i++, phone_number++) {
		lsb = *phone_number;
		switch (lsb) {
		case '0': case '1': case '2': case '3': case '4':
		case '5': case '6': case '7': case '8': case '9':
			lsb -= '0';
			break;
		case ' ':
			lsb = 0x0a;
			break;
		case '(':
			lsb = 0x0b;
			break;
		case ')':
			lsb = 0x0c;
			break;
		case '+':
			lsb = 0x0d;
			break;
		case '-':
			lsb = 0x0e;
			break;
		case '.':
			lsb = 0x0f;
			break;
		default:
			break;
		}
		if (i & 1) {

			// 2 nibbles, store them
			*data++ = msb | lsb;
		} else {

			// Collect first nibble
			msb = lsb << 4;
		}
	}
	if (i & 1) {

		// If length is odd, pad data with a final 0x0f
		*data++ = msb | 0x0f;
		i++;
	}
	*length = (i >> 1) + 1;
}

/***************************************************************************//**
 * @brief
 *   Encode a voltage for Sensor.
 *
 * @param[in] voltage
 *   The voltage value in mV, range is from 1.85 V to 4.40 V.
 *
 * @param[in] data
 *   Pointer to the buffer that will receive the encoded voltage in 10 mV
 *   resolution.
 *
 * @param[in] length
 *   Pointer to a byte that will receive the encoded voltage length in bits.
 *
 * @note
 *   The encoded value range is in 10 mV units, min 0 is 1.85 V, max 255 is
 *   4.40 V.
 ******************************************************************************/
bool TD_SENSOR_EncodeLocalVoltage(uint32_t voltage, uint8_t *data,
	uint8_t *length)
{
	if (voltage == 0) {
		voltage = 1850;
	}
	voltage /= 10;
	if (voltage >= 185 && voltage <= 440) {
		voltage -= 185;
		data[0] = voltage;
		if (length != NULL) {
			*length = 8;
		}
		return true;
	} else {
		return false;
	}
}

/***************************************************************************//**
 * @brief
 *   Encode a temperature for Sensor.
 *
 * @param[in] temperature
 *   The temperature value in 1/10°C, range is from -30°C to 97.5°C.
 *
 * @param[in] data
 *   Pointer to the buffer that will receive the encoded temperature in 0.5°C
 *   resolution.
 *
 * @param[in] length
 *   Pointer to a byte that will receive the encoded temperature length.
 *
 * @note
 *   The encoded value range is in 0.5°C units, min 0 is -30°C, max 255 is
 *   97.5°C.
 ******************************************************************************/
bool TD_SENSOR_EncodeLocalTemperature(int32_t temperature, uint8_t *data,
	uint8_t *length)
{
	// Round to half degrees
	int modulo = temperature % 5;

	switch (modulo) {
	case 1:
	case 2:
		temperature -= modulo;
		break;

	case 3:
	case 4:
		temperature += modulo;
		break;
	}
	if (temperature >= -300 && temperature <= 975) {
		*data = (temperature + 300) / 5;
		if (length != NULL) {
			*length = 8;
		}
		return true;
	}
	return false;
}

/***************************************************************************//**
 * @brief
 *   Register a new PIN number on Sensor.
 *
 * @param[in] index
 *  Code number index. Up to 256 PIN code numbers can be registered.
 *
 * @param[in] pin
 *   PIN value i.e. 1234.
 *
 * @return
 *   Returns true if the data has been sent over the SIGFOX network, false
 *   otherwise.
 ******************************************************************************/
bool TD_SENSOR_SendDataPIN(uint8_t index, uint8_t *pin)
{
	uint8_t data[3];
	uint8_t length;

	TD_SENSOR_EncodePIN(index, pin, data, &length);
	return TD_SENSOR_SendData(DATA_PIN, data, length);
}

/** @} */

/** @} (end addtogroup SENSOR_DATA) */
