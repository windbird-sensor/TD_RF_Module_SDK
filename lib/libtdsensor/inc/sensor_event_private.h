/***************************************************************************//**
 * @file sensor_event_private.h
 * @brief Event Frame Private
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
#ifndef SENSOR_EVENT_PRIVATE_H_
#define SENSOR_EVENT_PRIVATE_H_

#include "sensor_private.h"

/***************************************************************************//**
 * @addtogroup SENSOR_EVENT Sensor Event
 * @{
 ******************************************************************************/

/***************************************************************************//**
 * @addtogroup SENSOR_EVENT_TYPEDEFS Sensor Event Typedefs
 * @{ */

/** Event Type*/
typedef enum {
	EVENT_BATTERY_LOW,
	EVENT_BATTERY_OK,
	EVENT_CONNECTION_LOST,
	EVENT_CONNECTION_OK,
	EVENT_RSSI_LOW,
	EVENT_RSSI_OK,
	EVENT_TEMP_LOW,
	EVENT_TEMP_HIGH,
	EVENT_TEMP_OK,
	EVENT_BOOT,
	EVENT_SWITCH_ON,
	EVENT_SWITCH_OFF

} SensorEventType;

/**Event Frame Structure*/
typedef struct {
	SensorEventType type :8;
	uint8_t data :8;

}__PACKED SRV_FRAME_EVENT;

/** @} */

/** @} (end addtogroup SENSOR_EVENT) */

#endif /* SENSOR_EVENT_PRIVATE_H_ */
