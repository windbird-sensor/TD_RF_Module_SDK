/***************************************************************************//**
 * @file sensor_send.c
 * @brief API for sending frames to Sensor
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
#include "td_lan.h"
#include "td_sigfox.h"
#include "td_printf.h"
#include "sensor_config.h"
#include "td_sensor.h"
#include "td_sensor_device.h"
#include "td_sensor_gateway.h"
#include "td_sensor_transmitter.h"
#include "sensor_send_private.h"
#include "sensor_send.h"

/***************************************************************************//**
 * @addtogroup SENSOR_SEND Sensor Send
 * @brief Sensor protocol Send function
 * @details
 *
 *
 *	Remember that the SRV_FRM_REGISTER must be sent before any other frame. Sensor frame types are:
 *
 *		- SRV_FRM_EVENT
 *
 *		Send an Event Information to Sensor. An Event information is usually correlated to a status information derived from a bias value.
 *		ie. Battery Low, Temperature High, Connection Lost etc...
 *
 *
 *		- SRV_FRM_DATA
 *
 *		This Frame is emitted to transmit Data to Sensor. this can be measured data from a sensor or other kind of data like
 *		a phone number or a measured value.
 *
 *
 *		- SRV_FRM_REGISTER
 *
 *		Allows registering a new device on Sensor so that any other kind of frame type can be processed properly.
 *
 *
 *		- SRV_FRM_KEEPALIVE
 *
 *		To let Sensor know the device is still alive.
 *
 *
 *		- SRV_FRM_RAW
 *
 *		Provided for custom user purposes. This frame allows a 10 bytes user data payload which won't be processed by Sensor unless
 *		configured to do so.
 *
 *
 *		- SRV_FRM_SERVICE
 *
 *		Allows to trigger a service like sending an SMS or a Tweet. Remember that to send a SMS or a Rweet you must previously register
 *		a phone number or a twitter account by sending the corresponding SRV_FRM_DATA frame or by using Sensor Web Interface.
 *
 *
 *
 * @{
 ******************************************************************************/

/*******************************************************************************
 **************************  PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup SENSOR_PUBLIC_FUNCTIONS Public Functions
 * @{ */

/** @cond TD_PRIVATE */

/***************************************************************************//**
 * @brief
 *   Send Sensor Frame according to Sensor Protocol. Depending on module type the
 *   frame can be forwarded or not to a gateway.
 *
 * @param[in] profile
 * 	Retransmission profile
 *
 * @param[in] frame_type
 * 	Sensor frame type
 *
 * @param[in] stamp
 * 	Stamp
 *
 * @param[in] payload
 * 	Frame payload
 *
 * @param[in] count
 * 	Frame payload count
 *
 * @return
 *  True if the data has been sent over the Sigfox Network
 ******************************************************************************/

bool TD_SENSOR_Send(TransmitProfile * profile, SensorFrameType frame_type, uint8_t stamp, uint8_t * payload, uint8_t count)
{
	SensorFrame frame;
	unsigned int i = 0;

	//think smaller!
	if (count > 10) {
		return false;
	}

	frame.header.retry = 0;
	frame.header.stamp = stamp & 0x7;
	frame.header.cpt = 0;
	frame.header.entry_id = 0;
	frame.header.type = frame_type;

	for (i = 0; i < count; i++) {
		frame.payload[i] = payload[i];
	}

	//if gateway, send sigfox, else ask gateway to forward
	if (TD_SENSOR_GetModuleType() != SENSOR_DEVICE) {
		return TD_SENSOR_TRANSMITTER_SendSigfox(&frame, count, 0, profile);
	} else {
		if (TD_SENSOR_DEVICE_Forward((uint8_t*) &frame, count + sizeof(SensorFrameHeader), profile->repetition, profile->interval) != NOT_ACKED) {
			return true;
		} else {
			return false;
		}
	}
}

/** @endcond */

/** @} */

/** @} */
