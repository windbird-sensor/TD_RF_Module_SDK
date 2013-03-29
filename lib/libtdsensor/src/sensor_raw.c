/***************************************************************************//**
 * @file sensor_raw.c
 * @brief API for sending Raw frame type to Sensor
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

#include "sensor_send.h"
#include "sensor_raw.h"

/***************************************************************************//**
 * @addtogroup SENSOR_RAW Sensor Raw
 * @brief Sensor API for sending a Raw Frame.
 *
 * @details
 *	The raw frames allows to send up to 10 bytes to Sensor. Processing of these frames is NOT enabled by default
 *	and they will be delivered such as.
 *
 * @{
 ******************************************************************************/

/** @addtogroup SENSOR_RAW_DEFINES Defines
 * @{ */

#define RAW_DEFAULT_REPETITON 3 	///<Default retransmission repetition
#define RAW_DEFAULT_INTERVAL 120		///<Default retransmission interval in seconds
/** @} */

/*******************************************************************************
 *************************   PRIVATE VARIABLES   ****************************************
 ******************************************************************************/

/** @addtogroup SENSOR_RAW_PRIVATE_VARIABLES Private Variables
 * @{ */

static TransmitProfile raw_profile = { RAW_DEFAULT_REPETITON, RAW_DEFAULT_INTERVAL };
static uint8_t raw_stamp = -1;

/** @} */

/*******************************************************************************
 **************************  PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup SENSOR_RAW_PUBLIC_FUNCTIONS Public Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Send a RAW frame to Sensor.
 *
 *
 * @param[in] msg
 *   Pointer to message data. Message length must be <= 10
 *
 * @param[in] count
 *  Message length.
 *
 * @return
 *   True if the data has been sent over the Sigfox Network
 ******************************************************************************/

bool TD_SENSOR_SendRaw(uint8_t * msg, uint8_t count)
{
	if (count > 10)
		return false;

	raw_stamp = (raw_stamp & 0x07) + 1;
	return TD_SENSOR_Send(&raw_profile, SRV_FRM_RAW, raw_stamp, msg, count);

}

/***************************************************************************//**
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

void TD_SENSOR_SetRawTransmissionProfile(uint8_t repetition, uint32_t interval)
{
	raw_profile.repetition = repetition;
	raw_profile.interval = interval;
}

/** @} */

/** @} */
