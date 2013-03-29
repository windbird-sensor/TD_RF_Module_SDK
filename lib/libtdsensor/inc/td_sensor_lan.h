/***************************************************************************//**
 * @file td_sensor_lan.h
 * @brief Sensor LAN
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

#ifndef TD_SENSOR_LAN_H_
#define TD_SENSOR_LAN_H_

#include <stdbool.h>
#include <stdint.h>
#include <td_lan.h>
#include "sensor_private.h"
#include "sensor_send.h"

/***************************************************************************//**
 * @addtogroup TD_SENSOR_LAN Sensor LAN
 *
 * @{
 ******************************************************************************/

/*******************************************************************************
 **************************  DEFINES   ****************************************
 ******************************************************************************/

/** @addtogroup TD_SENSORLAN_DEFINES Defines
 * @{ */

#define TD_SENSOR_LAN_PAYLOAD_SIZE TD_LAN_PAYLOAD_SIZE-1

#define BROADCAST_ADDRESS 0x000000
#define BROADCAST_MASK 0xFFFFFF
#define NETWORK_MASK 0xFFFF00
/** @} */

/** @cond TD_PRIVATE */

/*******************************************************************************
 **************************  TYPEDEFS   ****************************************
 ******************************************************************************/

/** @addtogroup TD_SENSOR_LAN_TYPEDEFS Typedefs
 * @{ */

typedef enum {
	LOCAL_REGISTER = 1, LOCAL_FORWARD = 2, LOCAL_KEEPALIVE = 3, LOCAL_DATA = 4

} LocalFrameType;

typedef enum {
	NOT_ACKED,
	ACK_OK,
	ACK_ERROR,
	ACK_REGISTRATION_DISABLED,
	ACK_ALREADY_REGISTERED

} AckCode;

typedef struct {
	uint32_t address;
	uint32_t mask;

}__PACKED LanAddress;

typedef struct {
	uint8_t count :4; //count can be anything depending on LocalFrameType
	LocalFrameType type :4;
	uint8_t data[16];

}__PACKED LocalSensorFrame;

typedef struct {
	TransmitProfile profile;
	uint8_t sigfox[12];

} LocalForwardFrame;

typedef struct {
	uint32_t interval :32;
	int8_t level_low :8;
	int8_t level_ok :8;
	bool keepalive :1;
	bool rssi :1;

}__PACKED LocalKeepAliveFrame;

typedef struct {
	uint32_t checker;
	uint32_t SigfoxID;

} LocalRegisterFrame;

/** @} */

/*******************************************************************************
 *************************   PROTOTYPES   **************************************
 ******************************************************************************/

/** @addtogroup TD_SENSOR_LAN_PUBLIC_FUNCTIONS Public Functions
 * @{ */
/** @addtogroup TD_SENSOR_LAN_PROTOTYPES Prototypes
 * @{ */

bool TD_SENSOR_LAN_Init(bool gateway, uint32_t lan_frequency, int16_t lan_power_level);
AckCode TD_SENSOR_LAN_SendFrame(LocalFrameType type, uint8_t * payload,	uint8_t count, uint8_t * data_rx);
LanAddress * TD_SENSOR_LAN_GetAddress();
bool TD_SENSOR_LAN_setLanAddress(uint32_t adress, uint32_t mask);

/** @} */
/** @} */

/** @endcond */

/** @} (end addtogroup TD_SENSOR_LAN) */

#endif /* TD_SENSOR_LAN_H_ */
