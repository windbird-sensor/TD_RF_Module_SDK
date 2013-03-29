/***************************************************************************//**
 * @file td_sensor_device.c
 * @brief Sensor Device
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

#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#define USE_PRINTF
#include <td_printf.h>
#include <td_rtc.h>
#include <td_lan.h>
#include <td_flash.h>
#include <sensor_send.h>
#include "td_sensor.h"
#include "td_sensor_lan.h"
#include "td_sensor_device.h"

extern uint32_t SigfoxID;

/***************************************************************************//**
 * @addtogroup TD_SENSOR_DEVICE Sensor LAN Device
 * @{
 ******************************************************************************/

/*******************************************************************************
 **************************  PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup TD_SENSOR_DEVICE_PUBLIC_FUNCTIONS Public Functions
 * @{ */

/** @cond TD_PRIVATE */

/***************************************************************************//**
 * @brief
 *  Send a Forward frame on local network
 *
 * @param[in] payload
 *   Data to be forwarded by the gateway
 *
 * @param[in] count
 *   Data length
 *
 * @param[in] repetition
 *  Retransmissions count.
 *
 * @param[in] interval
 *   Retransmission interval in seconds
 *
 * @return
 *  AckCode
 *
 ******************************************************************************/

AckCode TD_SENSOR_DEVICE_Forward(uint8_t * payload, uint8_t count, uint8_t repetition, uint32_t interval)
{
	LocalForwardFrame frame;
	AckCode code;

	unsigned int i;

	if (count > TD_LAN_PAYLOAD_SIZE - sizeof(TransmitProfile))
		return false;

	frame.profile.repetition = repetition;
	frame.profile.interval = interval;

	for (i = 0; i < count; i++) {
		frame.sigfox[i] = payload[i];
	}

	code = TD_SENSOR_LAN_SendFrame(LOCAL_FORWARD, (uint8_t *) &frame, count + sizeof(TransmitProfile), 0);

	return code;
}

/***************************************************************************//**
 * @brief
 *  Send a Keep alive frame on local network
 *
 * @param[in] keepalive
 *   Set to true to activate connection monitoring on gateway side. Set to false to desactivate
 *   connection monitoring.
 *
 * @param[in] interval
 *   Keepalive interval monitoring if keepalive = true.
 *
 * @param[in] rssi
 *   Set to true to activate rssi monitoring on gateway side. Set to false to desactivate
 *   rssi monitoring.
 *
 * @param[in] level_low
 * 	RSSI level low monitoring if rssi = true.
 *
 * @param[in] level_ok
 * 	RSSI level ok monitoring if rssi = true.
 *
 * @return
 *  Return AckCode
 *
 ******************************************************************************/

AckCode TD_SENSOR_DEVICE_KeepAlive(bool keepalive, uint32_t interval, bool rssi, int8_t level_low, int8_t level_ok)
{
	LocalKeepAliveFrame frame;

	frame.keepalive = keepalive;
	frame.interval = interval;
	frame.rssi = rssi;
	frame.level_low = level_low;
	frame.level_ok = level_ok;

	AckCode code = TD_SENSOR_LAN_SendFrame(LOCAL_KEEPALIVE, (uint8_t *) &frame, sizeof(LocalKeepAliveFrame), 0);

	return code;
}

/** @endcond */

/***************************************************************************//**
 * @brief
 *  Send a Data frame on local network
 *
 * @param[in] data
 *   Data to be sent to the gateway
 *
 * @param[in] count
 *   Data length
 *
 * @param[in] data_rx
 *   Pointer to buffer which will be filled with Gateway's reply
 *
 * @return
 *  Return AckCode
 *
 ******************************************************************************/

AckCode TD_SENSOR_DEVICE_Data(uint8_t * data, uint8_t count, uint8_t data_rx[TD_SENSOR_LAN_PAYLOAD_SIZE])
{

	if (count > 16)
		return ACK_ERROR;

	AckCode code = TD_SENSOR_LAN_SendFrame(LOCAL_DATA, (uint8_t *) data, count, data_rx);

	return code;

}

/***************************************************************************//**
 * @brief
 *  Send a Register frame on LAN
 *
 * @return
 *  Return AckCode
 *
 ******************************************************************************/

AckCode TD_SENSOR_DEVICE_Register()
{
	LocalRegisterFrame frame;
	LanAddress lan_address;
	AckCode code;
	uint8_t rx[TD_SENSOR_LAN_PAYLOAD_SIZE];

	frame.checker = LOCAL_REGISTER_CHECK;
	frame.SigfoxID = SigfoxID;

	code = TD_SENSOR_LAN_SendFrame(LOCAL_REGISTER, (uint8_t *) &frame, sizeof(LocalRegisterFrame), (uint8_t *) rx);
	switch (code) {
	case ACK_OK:

		lan_address.address = rx[0] | rx[1] << 8 | rx[2] << 16 | rx[3] << 24;
		TD_SENSOR_LAN_setLanAddress(lan_address.address, BROADCAST_MASK);
		TD_FLASH_WriteVariables();

		break;

	default:
		break;
	}

	return code;
}

/***************************************************************************//**
 * @brief
 *  Reset Device Address
 *
 ******************************************************************************/

void TD_SENSOR_DEVICE_Reset()
{
	TD_SENSOR_LAN_setLanAddress(BROADCAST_ADDRESS, BROADCAST_MASK);
}

/** @} */

/** @} */

