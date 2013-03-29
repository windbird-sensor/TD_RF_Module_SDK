/***************************************************************************//**
 * @file sensor_event.h
 * @brief API for sending Event frame type to Sensor
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

#ifndef __SENSOR_EVENT_H
#define __SENSOR_EVENT_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************//**
 * @addtogroup SENSOR_EVENT Sensor Event
 *
 * @{
 ******************************************************************************/

/*******************************************************************************
 *************************   PROTOTYPES   **************************************
 ******************************************************************************/

/** @addtogroup SENSOR_EVENT_PUBLIC_FUNCTIONS Public Functions
 * @{ */
/** @addtogroup SENSOR_EVENT_PROTOTYPES Prototypes
 * @{ */

bool TD_SENSOR_SendEventBattery(bool state, uint8_t battery_level);
bool TD_SENSOR_SendEventRSSI(bool state, uint8_t EntryID);
bool TD_SENSOR_SendEventConnection(bool state, uint8_t entryID);
bool TD_SENSOR_SendEventTemperature(uint8_t state);
bool TD_SENSOR_SendEventBoot();
bool TD_SENSOR_SendEventSwitch(uint8_t port, uint8_t bit, bool state);

void TD_SENSOR_SetEventTransmissionProfile(uint8_t repetition,uint32_t interval);

/** @} */

/** @} */
/** @} (end addtogroup SENSOR_EVENT) */

#ifdef __cplusplus
}
#endif

#endif // __SENSOR_EVENT_H
