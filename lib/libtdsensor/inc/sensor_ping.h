/***************************************************************************//**
 * @file
 * @brief API for sending Ping frame type to Sensor
 * @author Telecom Design S.A.
 * @version 1.1.0
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2013-2015 Telecom Design S.A., http://www.telecomdesign.fr</b>
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

#ifndef __SENSOR_PING_H
#define __SENSOR_PING_H

#include <stdint.h>
#include <stdbool.h>

#include "sensor_send.h"

#ifdef __cplusplus
extern "C" {
#endif

	/***********************************************************************//**
	 * @addtogroup SENSOR_PING Sensor Ping
	 * @{
	 **************************************************************************/

	/***************************************************************************
	 *************************   PROTOTYPES   **********************************
	 **************************************************************************/

	/** @addtogroup SENSOR_PING_USER_FUNCTIONS User Functions
	 * @{ */

	/***********************************************************************//**
	 * @brief
	 *   Set a transmission profile for a ping frame type.
	 *
	 * @param[in] repetition
	 *	Number of repetitions.
	 *
	 * @param[in] interval
	 *	Interval between two repetitions in seconds.
	 **************************************************************************/
	static void __INLINE TD_SENSOR_SetPingTransmissionProfile(
		uint8_t repetition,
		uint32_t interval)
	{
		TD_SENSOR_SetTransmissionProfile(SRV_FRM_PING, repetition, interval);
	}

	bool TD_SENSOR_SendPing(bool custom, uint8_t custom_value);
	bool TD_SENSOR_SendPingExtended(uint8_t id, uint32_t sigfox_id,
		uint8_t release, uint16_t soft, uint16_t class, bool custom,
		uint8_t custom_value);

	/** @} */

	/** @} (end addtogroup SENSOR_PING) */

#ifdef __cplusplus
}
#endif

#endif // __SENSOR_PING_H
