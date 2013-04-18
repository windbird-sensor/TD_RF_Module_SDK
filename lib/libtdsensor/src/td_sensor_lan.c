/***************************************************************************//**
 * @file td_sensor_lan.c
 * @brief Sensor LAN
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

#include <stdbool.h>
#include <stdint.h>

#include <td_lan.h>
#include <td_flash.h>

#include "sensor_config.h"
#include "sensor_private.h"

#include "td_sensor_gateway.h"
#include "td_sensor_device.h"
#include "td_sensor_lan.h"


/***************************************************************************//**
 * @addtogroup TD_SENSOR_LAN Sensor LAN
 * @brief LAN protocol management
 *
 * @{
 ******************************************************************************/


/*******************************************************************************
 **************************  PRIVATE VARIABLES   *******************************
 ******************************************************************************/

/** @addtogroup TD_SENSOR_LAN_PRIVATE_VARIABLES Private Variables
 * @{ */

/**LAN address and mask		*/
static LanAddress LanConfig;

/** @} */

/*******************************************************************************
 **************************  PRIVATE FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup TD_SENSOR_LAN_PRIVATE_FUNCTIONS Private Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *  Send a local LAN frame
 *
 * @param[in] data
 *  Pointer to data to be sent.
 *
 * @param[in] count
 *  Length of data to be sent.
 *
 * @param[out] data_rx
 *  Data received
 *
 * @return
 *  Ack error code
 ******************************************************************************/
static AckCode TD_SENSOR_LAN_SendFramePrivate(uint8_t * data, uint8_t count, uint8_t * data_rx)
{
	TD_LAN_frame_t TX, RX;
	int i;

	TX.header = 0;
	SET_ADDRESS(TX.header, LanConfig.address);

	for (i = 0; i < count; i++) {
		TX.payload[i] = data[i];
	}

	//fill up remaining bytes with zeros
	for (; i < TD_LAN_PAYLOAD_SIZE; i++) {
		TX.payload[i] = 0;
	}

	if (!TD_LAN_SendReceive(-1, 1, &TX, &RX)) {
		return NOT_ACKED;
	} else {

		if (data_rx != 0) {
			//don't copy ack code
			for (i = 0; i < TD_SENSOR_LAN_PAYLOAD_SIZE; i++) {
				data_rx[i] = RX.payload[i + 1];
			}
		}

		return (AckCode) RX.payload[0];
	}
}

/***************************************************************************//**
 * @brief
 *  Define current module as being a gateway
 *
 ******************************************************************************/
static void TD_SENSOR_LAN_DefineAsGateway()
{
	TD_SENSOR_GATEWAY_Init();
	TD_SENSOR_GATEWAY_StartReception();
}

/***************************************************************************//**
 * @brief
 *  Define current module as being a device
 *
 ******************************************************************************/
static void TD_SENSOR_LAN_DefineAsDevice()
{
	TD_SENSOR_LAN_setLanAddress(LanConfig.address, LanConfig.mask);
}

/** @} */

/*******************************************************************************
 **************************  PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup TD_SENSOR_LAN_PUBLIC_FUNCTIONS Public Functions
 * @{ */


/***************************************************************************//**
 * @brief
 *  Check if the module is a Gateway or a device.
 *
 * @param[in] address
 *  LAN addres to be set.
 *
 * @param[in] mask
 *  LAN mask to be set.
 *
 * @return
 *  Return true if the module is a gateway. False is device.
 ******************************************************************************/
bool TD_SENSOR_LAN_setLanAddress(uint32_t address, uint32_t mask)
{
	LanConfig.address = address;
	LanConfig.mask = mask;

	TD_LAN_Stop(RESULT_ABORT);

	if (TD_LAN_Init(false, LanConfig.address, LanConfig.mask) == false) {
		return false;
	}

	TD_LAN_Restart();

	return true;
}

/***************************************************************************//**
 * @brief
 *  Returns current LAN address
 *
 * @return
 *  Pointer to LanAdress structure
 ******************************************************************************/
LanAddress * TD_SENSOR_LAN_GetAddress()
{
	return &LanConfig;
}

/***************************************************************************//**
 * @brief
 *   Send a Frame on the local network according to the TD protocol
 *
 * @param[in] type
 *   Type of frame to be sent
 *
 * @param[in] payload
 *   Pointer to data to be sent
 *
 * @param[in] count
 *   Data length
 *
 * @param[in] data_rx
 *   Received data
 *
 * @return
 *   AckCode.
 *
 ******************************************************************************/
AckCode TD_SENSOR_LAN_SendFrame(LocalFrameType type, uint8_t * payload, uint8_t count, uint8_t * data_rx)
{
	LocalSensorFrame frame;
	unsigned int i;

	frame.type = type;
	frame.count = count;

	if (count + 1 > (TD_LAN_PAYLOAD_SIZE)) {
		return ACK_ERROR;
	}

	for (i = 0; i < count; i++) {
		frame.data[i] = payload[i];
	}

	return TD_SENSOR_LAN_SendFramePrivate((uint8_t*) &frame, count + 1, data_rx);

}

/***************************************************************************//**
 * @brief
 *  Init Sensor LAN.
 *
 * @param[in] gateway
 *  If set to true, init LAN for a gateway. If false init LAN for a Device.
 *
 * @param[in] lan_frequency
 *	LAN frequency in Hz. Can be within 868000000..869700000 range.
 *
 * @param[in] lan_power_level
 *	LAN power level in dBm. Max is 14.
 ******************************************************************************/
bool TD_SENSOR_LAN_Init(bool gateway, uint32_t lan_frequency, int16_t lan_power_level)
{
	if (!TD_FLASH_DeclareVariable((uint8_t *) &LanConfig, sizeof(LanAddress), 0)) {
		LanConfig.address = BROADCAST_ADDRESS;
		LanConfig.mask = BROADCAST_MASK;
	}

	if (lan_frequency >= 868000000 && lan_frequency <= 869700000) {
		if (TD_LAN_SetFrequencyLevel(lan_frequency, lan_power_level) == false) {
			return false;
		}
	}

	if (gateway) {
		TD_SENSOR_LAN_DefineAsGateway();
	} else {
		TD_SENSOR_LAN_DefineAsDevice();
	}

	return true;

}

/** @} */

/** @} */
