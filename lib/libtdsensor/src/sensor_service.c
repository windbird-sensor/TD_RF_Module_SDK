/***************************************************************************//**
 * @file sensor_service.c
 * @brief API for sending Service frame type to Sensor
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

#include "sensor_send.h"
#include "sensor_service.h"
#include "sensor_service_private.h"

/***************************************************************************//**
 * @addtogroup SENSOR_SERVICE Sensor Service
 * @brief Sensor API for sending a Service Frame
 *
 * @details
 *	There are two service frames :
 *
 *	- SERVICE_SMS:
 *		Allows sending an SMS to a previously registered cell phone number.
 *	- SERVICE_TWEET
 *		Allows sending a tweet on a previously registered twitter account.
 *
 * @{
 ******************************************************************************/

/*******************************************************************************
 ************************   DEFINES   ********************************
 ******************************************************************************/

/** @addtogroup SENSOR_SERVICE_DEFINES Defines
 * @{ */

#define SERVICE_PAYLOAD_SIZE 10
#define MAX_SMS_LEN 9
#define MAX_TWEET_LEN 9

#define SERVICE_DEFAULT_REPETITON 3		///<Default retransmission repetitions
#define SERVICE_DEFAULT_INTERVAL 120	///<Default retransmission interval in seconds

/** @} */

/*******************************************************************************
 *************************   PRIVATE VARIABLES   ****************************************
 ******************************************************************************/

/** @addtogroup SENSOR_SERVICE_PRIVATE_VARIABLES Private Variables
 * @{ */

static TransmitProfile service_profile = { SERVICE_DEFAULT_REPETITON, SERVICE_DEFAULT_INTERVAL };
static uint8_t service_stamp = -1;

/** @} */

/*******************************************************************************
 **************************  PRIVATE FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup SENSOR_SERVICE_PRIVATE_FUNCTIONS Private Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Send a Service frame to Sensor.
 *
 * @param[in] service
 *   Service Type
 *
 * @param[in] data
 *   Data to be sent.
 *
 * @param[in] count
 *   Data length
 *
 * @return
 *   True if the data has been sent (ie. the gateway has acknowledged the request)
 *   False if count>Data size or if the ack from the gateway was never received.
 ******************************************************************************/
static bool TD_SENSOR_SendService(SensorServiceType service, uint8_t * data, uint8_t count)
{
	unsigned int i;
	SRV_FRAME_SERVICE frame;

	//think smaller but not too small!
	if (count > 10 || count == 0)
		return false;

	//First byte is Service type
	frame.type = service;

	for (i = 0; i < count; i++) {
		frame.data[i] = data[i];
	}

	service_stamp = (service_stamp & 0x07) + 1;
	return TD_SENSOR_Send(&service_profile, SRV_FRM_SERVICE, service_stamp, (uint8_t*) &frame, count + 1);

}

/** @} */

/*******************************************************************************
 **************************  PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup SENSOR_SERVICE_PUBLIC_FUNCTIONS Public Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Send a SMS
 *
 * @details
 * 	 At most 9 bytes can be sent.
 *
 * @return
 *   True if the data has been sent (ie. the gateway has acknowledged the request)
 *   False if the ack from the gateway was never received.
 ******************************************************************************/
bool TD_SENSOR_SendSMS(uint8_t * SMS)
{
	uint8_t count = 0;
	while (count <= MAX_SMS_LEN) {
		if (SMS[count] == 0) {
			return TD_SENSOR_SendService(SERVICE_SMS, SMS, count);
		} else {
			count++;
		}
	}

	return false;
}

/***************************************************************************//**
 * @brief
 *   Send a tweet
 *
 * @details
 * 	 At most 9 bytes can be sent.
 *
 * @return
 *   True if the data has been sent (ie. the gateway has acknowledged the request)
 *   False if the ack from the gateway was never received.
 ******************************************************************************/
bool TD_SENSOR_SendTweet(uint8_t * Tweet)
{
	uint8_t count = 0;

	while (count <= MAX_TWEET_LEN) {
		if (Tweet[count] == 0) {
			return TD_SENSOR_SendService(SERVICE_TWEET, Tweet, count);
		} else {
			count++;
		}
	}

	return false;
}

/***************************************************************************//**
 * @brief
 *   Set a transmission profile to a given frame type
 *
 * @param[in] repetition
 *	Number of repetition.
 *
 * @param[in] interval
 *	Interval between two repetitions in seconds.
 *
 ******************************************************************************/
void TD_SENSOR_SetServiceTransmissionProfile(uint8_t repetition, uint32_t interval)
{
	service_profile.repetition = repetition;
	service_profile.interval = interval;
}

/** @} */

/** @} */
