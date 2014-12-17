/***************************************************************************//**
 * @file
 * @brief API for sending Service frame type to Sensor
 * @author Telecom Design S.A.
 * @version 1.1.0
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

#include <td_utils.h>

#include "td_sensor_utils.h"
#include "sensor_send.h"
#include "sensor_service.h"

/***************************************************************************//**
 * @addtogroup SENSOR_SERVICE Sensor Service
 * @brief Sensor API for sending a Service Frame
 *
 * @details
 *	There are two service frames:
 *
 *	- SERVICE_SMS:
 *		Allows sending an SMS to a previously registered cell phone number.
 *
 *	- SERVICE_TWEET
 *		Allows sending a tweet on a previously registered twitter account.
 * @{
 ******************************************************************************/

/*******************************************************************************
 **************************  DEFINES   ****************************************
 ******************************************************************************/

/** @addtogroup SENSOR_SERVICE_DEFINES Defines
 * @{ */

/** Service payload size in bytes */
#define SERVICE_PAYLOAD_SIZE				10

/** @} */

/*******************************************************************************
 **************************  PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup SENSOR_SERVICE_USER_FUNCTIONS User Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Send a Service frame to Sensor.
 *
 * @param[in] service
 *   Service Type
 *
 * @param[in] data
 *   Data to be sent, it should be null-terminated.
 *
 * @return
 *   Returns true if the data has been sent over the SIGFOX network, false
 *   otherwise.
 ******************************************************************************/
bool TD_SENSOR_SendService(TD_SENSOR_SERVICE_Types_t service, uint8_t *data)
{
	TD_SENSOR_Frame_t frame;
	int i;

    frame.payload[0] = service;
    for (i = 0; i <= TD_SENSOR_SERVICE_MAX_DATA_LENGTH; i++) {
        if (data[i] == 0 || i == TD_SENSOR_SERVICE_MAX_DATA_LENGTH) {
        	return TD_SENSOR_SendUDM(0, SRV_FRM_SERVICE, &frame, 1 + i);
        } else {
        	frame.payload[1 + i] = data[i];
        }
    }
    return false;
}

/** @} */

/** @} (end addtogroup SENSOR_SERVICE) */
