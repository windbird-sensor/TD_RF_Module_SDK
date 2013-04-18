/***************************************************************************//**
 * @file sensor_data_private.h
 * @brief Data Frame Private
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
#ifndef SENSOR_DATA_PRIVATE_H_
#define SENSOR_DATA_PRIVATE_H_

#include <stdint.h>
#include "sensor_private.h"

/***************************************************************************//**
 * @addtogroup SENSOR_DATA Sensor Data
 * @{
 ******************************************************************************/

/*******************************************************************************
 *************************   TYPEDEFS   **************************************
 ******************************************************************************/

/** @addtogroup SENSOR_DATA_TYPEDEFS Typedefs
 * @{ */

/** Data Types*/
typedef enum {

	DATA_PHONE

} SensorDataType;

/**Data Frame Structure*/
typedef struct {
	SensorDataType type;
	uint8_t data[8];

} SRV_FRAME_DATA;

/** @} */

/** @} (end addtogroup SENSOR_DATA) */

#endif /* SENSOR_DATA_PRIVATE_H_ */
