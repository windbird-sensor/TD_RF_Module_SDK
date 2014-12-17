/***************************************************************************//**
 * @file
 * @brief API for sending Ping frame type to Sensor
 * @author Telecom Design S.A.
 * @version 1.0.0
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

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <td_sigfox.h>

#include "td_sensor.h"
#include "sensor_send.h"
#include "sensor_ping.h"
#include "td_sensor_utils.h"

/***************************************************************************//**
 * @addtogroup SENSOR_PING Sensor Ping
 * @brief Sensor API for sending a Ping Frame
 * @{
 ******************************************************************************/

/*******************************************************************************
 ************************   DEFINES   *****************************************
 ******************************************************************************/

/** @addtogroup SENSOR_PING_DEFINES Defines
 * @{ */

/** Ping payload size in bytes */
#define PING_PAYLOAD_SIZE		9

/** @} */

/*******************************************************************************
 **************************  PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup SENSOR_PING_USER_FUNCTIONS User Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Send a Ping frame to Sensor.
 *
 * @param custom
 *   True to send additional user value
 *
 * @param custom_value
 *   User custom parameter in ping frame
 *
 * @details
 * 	 Can be used to test sigfox coverage
 *
 * @return
 *   Returns true if the data has been sent over the SIGFOX network, false
 *   otherwise.
 ******************************************************************************/
bool TD_SENSOR_SendPing(bool custom, uint8_t custom_value)
{
	bool ret = false;
	TD_SENSOR_Frame_t frame;
	uint32_t sigfox_id;
	TD_SENSOR_Configuration_t *config;
	const TD_SIGFOX_Proxy_t *pProxy;
	pProxy = TD_SIGFOX_PROXY_Get();
	TD_SIGFOX_Transport_t sigfoxTransport;

	sigfox_id = TD_SENSOR_GetSigfoxID();
	config = TD_SENSOR_GetModuleConfiguration();
	frame.payload[0] = ((RELEASE & 0x0f) << 4) | (SUB_RELEASE & 0x0f);
	frame.payload[1] = config->soft_release >> 8;
	frame.payload[2] = config->soft_release & 0xFF;
	frame.payload[3] = config->class >> 8;
	frame.payload[4] = config->class & 0xFF;
	frame.payload[5] = sigfox_id >> 24;
	frame.payload[6] = (sigfox_id >> 16) & 0xFF;
	frame.payload[7] = (sigfox_id >> 8) & 0xFF;
	frame.payload[8] = sigfox_id & 0xFF;
	if (custom) {
		frame.payload[9] = custom_value;
	}

	// Memorise transport layer
	sigfoxTransport = pProxy->config.transport;
	TD_SIGFOX_PROXY_SetTransportLayer(TD_SIGFOX_USE_DIRECT);
	ret = TD_SENSOR_SendUDM(0, SRV_FRM_PING, &frame,
		custom ? PING_PAYLOAD_SIZE + 1 : PING_PAYLOAD_SIZE);

	// Restore transport layer
	TD_SIGFOX_PROXY_SetTransportLayer(sigfoxTransport);
	return ret;
}

/** @} */

/** @} (end addtogroup SENSOR_PING) */
