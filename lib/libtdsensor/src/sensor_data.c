/***************************************************************************//**
 * @file sensor_data.c
 * @brief API for sending Data frame type to Sensor
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
#include <td_utils.h>
#include "sensor_send.h"
#include "sensor_data.h"
#include "sensor_event.h"
#include "sensor_data_private.h"

/***************************************************************************//**
 * @addtogroup SENSOR_DATA Sensor Data
 * @brief Sensor API for sending Data frame
 *
 * @details
 *
 * 	Data frames allows you to send data to Sensor. Only a phone number can be send for now.
 *
 * @{
 ******************************************************************************/

/*******************************************************************************
 *************************   DEFINES   *****************************************
 ******************************************************************************/

/** @addtogroup SENSOR_DATA_DEFINES Defines
 * @{ */
#define DATA_PAYLOAD_SIZE 9
#define MAX_PHONE_ZEROS 4

#define DATA_DEFAULT_REPETITON 3	///<Default retransmission repetitions
#define DATA_DEFAULT_INTERVAL 120	///<Default retransmission interval in seconds
/** @} */

/*******************************************************************************
 *************************   PRIVATE VARIABLES   ****************************************
 ******************************************************************************/

/** @addtogroup SENSOR_DATA_PRIVATE_VARIABLES Private Variables
 * @{ */

static TransmitProfile data_profile = { DATA_DEFAULT_REPETITON, DATA_DEFAULT_INTERVAL };
static uint8_t data_stamp = -1;

/** @} */

/*******************************************************************************
 **************************  PRIVATE FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup SENSOR_DATA_INFO__PRIVATE_FUNCTIONS Private Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Send a DATA frame to Sensor.
 *
 * @param[in] data_type
 *   Type of Data being sent. Used by Sensor for decoding. Refer to SensorDataType enum
 *   for allowed values.
 *
 * @param[in] data
 *   Pointer to the data to be sent. Max allowed length for data is 16 bytes.
 *
 * @param[in] count
 *   The length in bytes of the data to be sent.
 *
 * @return
 *   True if the data has been sent over the Sigfox Network
 ******************************************************************************/
static bool TD_SENSOR_SendData(SensorDataType data_type, uint8_t * data, uint8_t count)
{

	SRV_FRAME_DATA frame;
	unsigned int i;
	frame.type = data_type;

	for (i = 0; i < count; i++) {
		frame.data[i] = data[i];
	}

	data_stamp = (data_stamp & 0x07) + 1;

	return TD_SENSOR_Send(&data_profile, SRV_FRM_DATA, data_stamp, (uint8_t*) &frame, count + 1);

}

/** @} */

/*******************************************************************************
 **************************  PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup SENSOR_DATA_INFO__PUBLIC_FUNCTIONS Public Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Register a new cell phone number on Sensor for SMS reception.
 *
 * @param[in] index
 *	Phone number index. Up to 4 cell phone number can be registered
 *
 * @param[in] phone_number
 *   Phone number. ie 0601020304
 *
 * @return
 *   True if the data has been sent (ie. the gateway has acknowledged the request)
 *   False if the ack from the gateway was never received.
 ******************************************************************************/

bool TD_SENSOR_SetCellPhoneNumber(PhoneIndex index, uint8_t * phone_number)
{
	uint8_t count = 0;
	uint8_t msg[7];
	uint64_t encoded;

	uint8_t zeros = 0;
	while (phone_number[zeros] == '0') {
		zeros++;
		//more than 6 '0' to start with is weird
		if (zeros > MAX_PHONE_ZEROS)
			return false;
	}

	encoded = atoll((char *) &phone_number[zeros]);

	msg[0] = (zeros & 0x03) << 6 | (index & 0x3F);

	while (encoded != 0 && count < 6) {
		//LSB first
		msg[1 + count] = encoded & 0xFF;
		encoded = encoded >> 8;
		count++;
	}

	return TD_SENSOR_SendData(DATA_PHONE, msg, count + 1);
}

/*****************************************************************************
 * @brief
 *   Set a transmission profile to a given frame type
 *
 * @param[in] repetition
 *	Number of repetition
 *
 * @param[in] interval
 *	Interval between two repetitions in seconds
 *
 ******************************************************************************/

void TD_SENSOR_SetDataTransmissionProfile(uint8_t repetition, uint32_t interval)
{
	data_profile.repetition = repetition;
	data_profile.interval = interval;
}

/** @} */

/** @} */
