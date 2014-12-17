/***************************************************************************//**
 * @file
 * @brief API for sending Register frame type to Sensor
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

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <td_sigfox.h>

#include "td_sensor.h"
#include "sensor_send.h"
#include "sensor_register.h"
#include "td_sensor_utils.h"
#include "td_sensor_gateway.h"

/***************************************************************************//**
 * @addtogroup SENSOR_REGISTER Sensor Register
 * @brief Sensor API for sending a Register Frame
 *
 * The Register frame must be sent to register a new device on Sensor.
 * @{
 ******************************************************************************/

/*******************************************************************************
 ************************   DEFINES   *****************************************
 ******************************************************************************/

/** @addtogroup SENSOR_REGISTER_DEFINES Defines
 * @{ */

/** Register payload size in bytes */
#define REGISTER_PAYLOAD_SIZE		9

/** @} */

/*******************************************************************************
 **************************  PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup SENSOR_REGISTER_USER_FUNCTIONS User Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Send a Register frame to Sensor.
 *
 * @details
 * 	 Only a registered device can be seen on Sensor. This frame must be sent
 * 	 before any other.
 *
 * @return
 *   Returns true if the data has been sent over the SIGFOX network, false
 *   otherwise.
 ******************************************************************************/
bool TD_SENSOR_SendRegister(void)
{
	TD_SENSOR_Configuration_t *config;

	config = TD_SENSOR_GetModuleConfiguration();
	return TD_SENSOR_SendRegisterExtended(0, TD_SENSOR_GetSigfoxID(),
		((RELEASE & 0x0f) << 4) | (SUB_RELEASE & 0x0f),
		config->soft_release,
		config->class,
		false, 0);
}

/***************************************************************************//**
 * @brief
 *   Send a Register frame to Sensor with extended parameters.
 *
 * @param[in] id
 *   Device ID to use.
 *
 * @param[in] sigfox_id
 *   SIGFOX ID.
 *
 * @param[in] release
 *   UDM release number.
 *
 * @param[in] soft
 *   Software release number.
 *
 * @param[in] class
 *   Device class.
 *
 * @param custom
 *   True to send additional user value
 *
 * @param custom_value
 *   User custom parameter in register frame
 *
 * @details
 * 	 Only a registered device can be seen on Sensor. This frame must be sent
 * 	 before any other.
 *
 * @return
 *   Returns true if the data has been sent over the SIGFOX network, false
 *   otherwise.
 ******************************************************************************/
bool TD_SENSOR_SendRegisterExtended(uint8_t id, uint32_t sigfox_id,
	uint8_t release, uint16_t soft, uint16_t class, bool custom,
	uint8_t custom_value)
{
	TD_SENSOR_Frame_t frame;

	frame.payload[0] = release;
	frame.payload[1] = soft >> 8;
	frame.payload[2] = soft & 0xFF;
	frame.payload[3] = class >> 8;
	frame.payload[4] = class & 0xFF;
	frame.payload[5] = sigfox_id >> 24;
	frame.payload[6] = (sigfox_id >> 16) & 0xFF;
	frame.payload[7] = (sigfox_id >> 8) & 0xFF;
	frame.payload[8] = sigfox_id & 0xFF;
	if(custom) {
		frame.payload[9] = custom_value;
	}

	return TD_SENSOR_SendUDM(id, SRV_FRM_REGISTER, &frame,
		custom ? REGISTER_PAYLOAD_SIZE+1 : REGISTER_PAYLOAD_SIZE);
}

/***************************************************************************//**
 * @brief
 *   Send a Register frame to Sensor for a paired and available booster.
 *   This register shall not be sent first to permit update proxy "available"
 *   parameter
 *
 * @details
 * 	 Only a registered device can be seen on Sensor. This frame must be sent
 * 	 before any other.
 *
 * @return
 *   Returns true if the data has been sent over the SIGFOX network, false
 *   otherwise.
 ******************************************************************************/
bool TD_SENSOR_SendRegisterForBooster(void)
{
	bool ret = false;
	uint8_t entry_id = 0, device_nb = 0;
	uint8_t IndexList[5] = {0};
	const TD_SIGFOX_Proxy_t *pProxy;
	const TD_SIGFOX_ProxyStatus_t *pProxyStatus;
	const TD_SIGFOX_ProxyConfig_t *pProxyConfig;

	pProxy = TD_SIGFOX_PROXY_Get();
	pProxyStatus = &pProxy->status;
	pProxyConfig= &pProxy->config;

	// Check if a Booster is paired and is available at start up
	if ((pProxyStatus->paired == true)
			&& (pProxyStatus->available == true)) {

		// Look for booster entry_id
		device_nb = TD_SENSOR_GATEWAY_GetDeviceIndex(pProxyConfig->class,
			IndexList);
		entry_id = IndexList[device_nb - 1];

		// Send Booster register frame
		ret = TD_SENSOR_SendRegisterExtended(entry_id,
			pProxyStatus->id,
			((pProxyStatus->udm_release & 0x0f) << 4) |
				(pProxyStatus->udm_subrelease & 0x0f),
			pProxyStatus->soft_version,
			pProxyConfig->class,
			true,
			0xFF);	// TODO : get booster boot cause and send it
	}
	return ret;
}

/** @} */

/** @} (end addtogroup SENSOR_REGISTER) */
