/***************************************************************************//**
 * @file
 * @brief API for sending KeepAlive frame type to Sensor
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

#ifndef __SENSOR_KEEPALIVE_H
#define __SENSOR_KEEPALIVE_H

#include <stdint.h>
#include <td_core.h>

#include "sensor_send.h"

#ifdef __cplusplus
extern "C" {
#endif

	/***********************************************************************//**
	 * @addtogroup SENSOR_KEEPALIVE Sensor Keep-Alive
	 * @{
	 **************************************************************************/

	/***************************************************************************
	 ************************   DEFINES   **************************************
	 **************************************************************************/

	/** @addtogroup SENSOR_KEEPALIVE_DEFINES Defines
	 * @{ */

	/** Maximum Sensor keep-alive custom size */
#define TD_SENSOR_KEEPALIVE_MAX_CUSTOM_SIZE	8

	/** @} */

	/***************************************************************************
	 *************************   ENUMERATIONS   ********************************
	 **************************************************************************/

	/** @addtogroup SENSOR_KEEPALIVE_ENUMERATIONS Enumerations
	 * @{ */

	/** Sensor extended keep-alive type */
	typedef enum {
		KEEPALIVE_VOLTAGE_TEMP = 0,
		KEEPALIVE_VOLTAGE_TEMP_GEOLOC = 1,
		KEEPALIVE_GEOLOC = 2,
		KEEPALIVE_CUSTOM = 3
	} TD_SENSOR_KEEPALIVE_types_t;

	/** @} */

	/***************************************************************************
	 *************************   PROTOTYPES   **********************************
	 **************************************************************************/

	/** @addtogroup SENSOR_KEEPALIVE_USER_FUNCTIONS User Functions
   * @{ */

	bool TD_SENSOR_SendKeepAlive(void);
	bool TD_SENSOR_SendKeepAliveExt(TD_SENSOR_KEEPALIVE_types_t type,
		uint8_t interval, uint8_t *data, uint8_t length);
	bool TD_SENSOR_SendKeepAliveVoltageTemperatureFor(uint8_t id,
		uint8_t interval, uint8_t voltage, uint8_t temperature);

	/***********************************************************************//**
	 * @brief
	 *   Set a transmission profile for a keep-alive frame type.
	 *
	 * @param[in] repetition
	 *	Number of repetitions.
	 *
	 * @param[in] interval
	 *	Interval between two repetitions in seconds.
	 **************************************************************************/
	static void __INLINE TD_SENSOR_SetKeepAliveTransmissionProfile(
		uint8_t repetition,
		uint32_t interval)
	{
		TD_SENSOR_SetTransmissionProfile(SRV_FRM_KEEPALIVE, repetition,
			interval);
	}

	/** @} */

/** @} (end addtogroup SENSOR_KEEPALIVE) */

#ifdef __cplusplus
}
#endif

#endif // __SENSOR_KEEPALIVE_H
