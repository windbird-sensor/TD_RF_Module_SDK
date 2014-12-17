/***************************************************************************//**
 * @file
 * @brief API for sending frames to Sensor
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

#include <em_assert.h>

#include "td_sensor.h"
#include "td_sensor_device.h"
#include "td_sensor_transmitter.h"
#include "td_sensor_utils.h"
#include "sensor_send.h"
#include "sensor_raw.h"
#include "sensor_event.h"
#include "sensor_data.h"
#include "sensor_register.h"
#include "sensor_service.h"
#include "sensor_keepalive.h"

/***************************************************************************//**
 * @addtogroup SENSOR_SEND Sensor Send
 * @brief Sensor protocol Send function
 *
 * @details
 *	 Sensor frame types are:
 *
 *		- SRV_FRM_EVENT
 *
 *		This frame type allows to send an event information to Sensor. An event
 *		information is usually completed by a status information derived from a
 *		bias value, i.e. Battery Low, Temperature High, Connection Lost, etc.
 *
 *		- SRV_FRM_DATA
 *
 *		This frame type is used to transmit data to Sensor. This can any kind of
 *		data like a phone number or a measurement value.
 *
 *		- SRV_FRM_REGISTER
 *
 *		This frame type allows registering a new device on Sensor so that any
 *		other kind of frame type can be processed properly.
 *
 *		- SRV_FRM_KEEPALIVE
 *
 *		This periodic frame lets Sensor know the device is still alive.
 *
 *		- SRV_FRM_RAW
 *
 *		This frame type is provided for custom user purposes. This frame allows
 *		a 10 bytes user data payload that will not be processed by Sensor,
 *		unless configured to do so.
 *
 *		- SRV_FRM_SERVICE
 *
 *		This frame type triggers a service, like sending an SMS or a Tweet.
 *		Remember that to send a SMS or a tweet you must previously register
 *		a phone number or a twitter account by sending the corresponding
 *		SRV_FRM_DATA frame or by using the Sensor Web Interface.
 * @{
 ******************************************************************************/

/*******************************************************************************
 **************************  PRIVATE VARIABLES   *******************************
 ******************************************************************************/

/** @addtogroup SENSOR_SEND_PRIVATE_VARIABLES Private Variables
 * @{ */


/** User callback to execute before and after lan transmission */
static bool (*UserCallback)(uint8_t *payload, uint8_t count, uint8_t repetition, uint32_t interval) = 0;

/** @} */

/*******************************************************************************
 *************************   PUBLIC VARIABLES   ********************************
 ******************************************************************************/

/** @addtogroup SENSOR_SEND_GLOBAL_VARIABLES Global Variables
 * @{ */

/** Transmit configurations */
TD_SENSOR_TransmitConfig_t TD_SENSOR_TransmitConfig[SRV_FRM_MAX] = {
	{{0, 0}, 0},
	{{0, 0}, 0},
	{{0, 0}, 0},
	{{0, 0}, 0},
	{{0, 0}, 0},
	{{0, 0}, 0},
	{{0, 0}, 0}
};

/** @} */

/*******************************************************************************
 **************************  PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/


/** @addtogroup SENSOR_SEND_GLOBAL_FUNCTIONS Global Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Send a Sensor Frame according to UDM protocol. Depending on module type,
 *   the frame can be forwarded or not to a gateway.
 *
 * @param[in] id
 *   The ID to use for transmission.
 *
 * @param[in] frame_type
 * 	Sensor frame type.
 *
 * @param[in] frame
 * 	Pointer to the buffer containing the UDM TD_SENSOR_Frame_t frame.
 *
 * @param[in] count
 * 	Number of bytes of frame payload data.
 *
 * @return
 *   Returns true if the data has been sent over the SIGFOX network, false
 *   otherwise.
 ******************************************************************************/
bool TD_SENSOR_SendUDM(uint8_t id, TD_SENSOR_FrameType_t frame_type,
	TD_SENSOR_Frame_t *frame, uint8_t count)
{
	TD_SENSOR_LAN_AckCode_t code;
	bool ok;

	if (count > TD_SENSOR_PAYLOAD_SIZE) {
		return false;
	}
	EFM_ASSERT(frame_type < SRV_FRM_MAX);
	frame->header[0] = ((TD_SENSOR_TransmitConfig[frame_type].stamp & 0x07)
		<< 4);
	frame->header[1] = ((id & 0x0f) << 4) | (frame_type & 0x0f);
	if (TD_SENSOR_GetModuleType() != SENSOR_DEVICE ||
		TD_SENSOR_DEVICE_GetTxSkipLan()) {
		ok = TD_SENSOR_TRANSMITTER_SendSigfox((uint8_t *) frame,
			2 + count,
        	id,
			&TD_SENSOR_TransmitConfig[frame_type].profile);

		// Check battery status
		TD_SENSOR_BatteryCallBack(false);
	} else {

		// Let us use our own lan management
		if (UserCallback != 0) {
			ok = (*UserCallback)((uint8_t *) frame,
				2 + count,
				TD_SENSOR_TransmitConfig[frame_type].profile.repetition,
				TD_SENSOR_TransmitConfig[frame_type].profile.interval);
		} else {

		// Try to send via LAN in synchronous or asynchronous mode
		code = TD_SENSOR_DEVICE_Forward((uint8_t *) frame,
			2 + count,
			TD_SENSOR_TransmitConfig[frame_type].profile.repetition,
			TD_SENSOR_TransmitConfig[frame_type].profile.interval);
		if (code == ACK_OK || code == SENSOR_LAN_QUEUED) {
			ok = true;
		} else {
			ok = false;
		}
	}
	}
	return ok;
}

/***************************************************************************//**
 * @brief
 *   Send a Sensor Frame according to Sensor Protocol. Depending on module type,
 *   the frame can be forwarded or not to a gateway.
 *
 * @param[in] profile
 * 	Retransmission profile.
 *
 * @param[in] frame_type
 * 	Sensor frame type.
 *
 * @param[in] stamp
 * 	Redundancy counter for the given frame type.
 *
 * @param[in] payload
 * 	Pointer to the buffer containing the frame payload data.
 *
 * @param[in] count
 * 	Number of bytes of frame payload data.
 *
 * @return
 *   Returns true if the data has been sent over the SIGFOX network, false
 *   otherwise.
 ******************************************************************************/
bool TD_SENSOR_Send(TD_SENSOR_TransmitProfile_t *profile,
	TD_SENSOR_FrameType_t frame_type, uint8_t stamp, uint8_t *payload,
	uint8_t count)
{
	TD_SENSOR_Frame_t frame;
	TD_SENSOR_LAN_AckCode_t code;
	unsigned int i = 0;

	// Think smaller!
	if (count > 10) {
		return false;
	}
    frame.header[0] = frame_type & 0x0f;
    frame.header[1] = (stamp & 0x07) << 4;
	for (i = 0; i < count; i++) {
		frame.payload[i] = payload[i];
	}
	if (TD_SENSOR_GetModuleType() != SENSOR_DEVICE ||
		TD_SENSOR_DEVICE_GetTxSkipLan()) {
		return TD_SENSOR_TRANSMITTER_SendSigfox((uint8_t *) &frame,
			2 + count,
			0,
			profile);
	} else {

		// Try to send via LAN in synchronous or asynchronous mode
		code = TD_SENSOR_DEVICE_Forward((uint8_t *) &frame,
			2 + count,
			profile->repetition,
			profile->interval);
		if (code == ACK_OK || code == SENSOR_LAN_QUEUED) {
			return true;
		} else {
			return false;
		}
	}
}

/***************************************************************************//**
 * @brief
 *   Callback to call before and after lan transmission
 ******************************************************************************/
void TD_SENSOR_SEND_SetUserCallback(bool (*user_callback)(uint8_t *payload,
	uint8_t count, uint8_t repetition, uint32_t interval))
{
	UserCallback = user_callback;
}

/** @} */

/** @} (end addtogroup SENSOR_SEND) */
