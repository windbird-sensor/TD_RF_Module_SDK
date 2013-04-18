/***************************************************************************//**
 * @file sensor_send.h
 * @brief Service Send
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

#ifndef SENSOR_SEND_H_
#define SENSOR_SEND_H_

#include <stdbool.h>
#include <stdint.h>
#include "sensor_private.h"

/***************************************************************************//**
 * @addtogroup SENSOR_SEND Sensor Send
 * @{
 ******************************************************************************/

/***************************************************************************//**
 * @addtogroup SENSOR_SEND_TYPEDEFS Typedefs
 * @{ */

/** Sensor Frame Type */
typedef enum {
	SRV_FRM_EVENT,
	SRV_FRM_DATA,
	SRV_FRM_REGISTER,
	SRV_FRM_KEEPALIVE,
	SRV_FRM_RAW,
	SRV_FRM_SERVICE,
	SRV_FRM_GEOLOC,

} SensorFrameType;

/**Transmit Profile*/
typedef struct {
	uint8_t repetition :4;
	uint32_t interval :28;

}__PACKED TransmitProfile;
//NB: don't change order here!!

/** @} */

/*******************************************************************************
 *************************   PROTOTYPES   **************************************
 ******************************************************************************/

/** @addtogroup SENSOR_SEND_PUBLIC_FUNCTIONS Public Functions
 * @{ */
/** @addtogroup SENSOR_SEND_PROTOTYPES Prototypes
 * @{ */

bool TD_SENSOR_Send(TransmitProfile * profile, SensorFrameType frame_type, uint8_t stamp, uint8_t * payload, uint8_t count);
/** @} */

/** @} */
/** @} (end addtogroup SENSOR_SEND) */

#endif /* SENSOR_SEND_H_ */
