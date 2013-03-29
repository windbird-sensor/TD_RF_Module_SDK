/***************************************************************************//**
 * @file sensor_keepalive.c
 * @brief API for sending KeepAlive frame type to Sensor
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

#include <stdbool.h>
#include <stdint.h>
#include "sensor_send.h"
#include "sensor_keepalive.h"

/***************************************************************************//**
 * @addtogroup SENSOR_KEEPALIVE Sensor Keep-Alive
 * @brief Sensor API for sending a Keep-Alive Frame
 *
 *
 * @{
 ******************************************************************************/

/** @addtogroup SENSOR_KEEPALIVE_DEFINES Defines
 * @{ */

#define KEEPALIVE_DEFAULT_REPETITON 0
#define KEEPALIVE_DEFAULT_INTERVAL 0
/** @} */

/*******************************************************************************
 *************************   PRIVATE VARIABLES   ****************************************
 ******************************************************************************/

/** @addtogroup SENSOR_KEEPALIVE_PRIVATE_VARIABLES Private Variables
 * @{ */

static TransmitProfile keepalive_profile = { KEEPALIVE_DEFAULT_REPETITON, KEEPALIVE_DEFAULT_INTERVAL };
static uint8_t keepalive_stamp = -1;

/** @} */

/*******************************************************************************
 **************************  PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup SENSOR_KEEPALIVE_PUBLIC_FUNCTIONS Public Functions
 * @{ */

/** @cond TD_PRIVATE */

/***************************************************************************//**
 * @brief
 *   Send a KEEPALIVE frame to Sensor
 *
 * @return
 *   True if the data has been sent over the Sigfox Network
 ******************************************************************************/

bool TD_SENSOR_SendKeepAlive()
{
	keepalive_stamp = (keepalive_stamp & 0x07) + 1;

	return TD_SENSOR_Send(&keepalive_profile, SRV_FRM_KEEPALIVE, keepalive_stamp, 0, 0);

}

/** @endcond */

/***************************************************************************//**
 * @brief
 *   Set a transmission profile to a given frame type
 *
 * Sensor Frame type to which the profile will be changed.
 *
 * @param[in] repetition
 *	Number of repetition.
 *
 * @param[in] interval
 *	Interval between two repetitions in seconds.
 *
 ******************************************************************************/

void TD_SENSOR_SetKeepAliveTransmissionProfile(uint8_t repetition, uint32_t interval)
{
	keepalive_profile.repetition = repetition;
	keepalive_profile.interval = interval;
}

/** @} */

/** @} */
