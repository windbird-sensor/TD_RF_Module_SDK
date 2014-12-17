/***************************************************************************//**
 * @file
 * @brief API for sending Raw frame type to Sensor
 * @author Telecom Design S.A.
 * @version 1.2.0
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

#ifndef __SENSOR_RAW_H
#define __SENSOR_RAW_H

#include <stdint.h>
#include <stdbool.h>

#include "sensor_send.h"

#ifdef __cplusplus
extern "C" {
#endif

	/***********************************************************************//**
	 * @addtogroup SENSOR_RAW Sensor Raw
	 * @{
	 **************************************************************************/

	/***************************************************************************
	*************************   DEFINES   **************************************
	***************************************************************************/

	/** @addtogroup SENSOR_RAW_DEFINES Defines
	 * @{ */

	/** Maximum raw data size */
#define TD_SENSOR_RAW_MAX_SIZE	10

	/** @} */

	/***************************************************************************
	 *************************   PROTOTYPES   **********************************
	 **************************************************************************/

	/** @addtogroup SENSOR_RAW_USER_FUNCTIONS User Functions
	 * @{ */

	/***********************************************************************//**
	 * @brief
	 *   Send a RAW frame to Sensor.
	 *
	 * @param[in] data
	 *   Pointer to the raw message data.
	 *
	 * @param[in] length
	 *  Message length in bytes, must be <= 10 bytes.
	 *
	 * @return
	 *   Returns true if the data has been sent over the SIGFOX network, false
	 *   otherwise.
	 **************************************************************************/
	static bool __INLINE TD_SENSOR_SendRaw(uint8_t *data, uint8_t length)
	{
		TD_SENSOR_Frame_t frame;

		memcpy(&frame.payload[0], data, length);
		return TD_SENSOR_SendUDM(0, SRV_FRM_RAW, &frame, length);
	}

	/***********************************************************************//**
	 * @brief
	 *   Set a transmission profile for a raw frame type.
	 *
	 * @param[in] repetition
	 *	Number of repetitions.
	 *
	 * @param[in] interval
	 *	Interval between two repetitions in seconds.
	 **************************************************************************/
	static void __INLINE TD_SENSOR_SetRawTransmissionProfile(uint8_t repetition,
		uint32_t interval)
	{
		TD_SENSOR_SetTransmissionProfile(SRV_FRM_RAW, repetition, interval);
	}

	/** @} */
	/** @} (end addtogroup SENSOR_RAW) */

#ifdef __cplusplus
}
#endif

#endif // __SENSOR_RAW_H
