/***************************************************************************//**
 * @file sensor_data.h
 * @brief API for sending Data frame type to Sensor
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

#ifndef SENSOR_DATA_H_
#define SENSOR_DATA_H_

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************//**
 * @addtogroup SENSOR_DATA Sensor Data
 *
 * @{
 ******************************************************************************/

/*******************************************************************************
 *************************   TYPEDEFS   **************************************
 ******************************************************************************/

/** @addtogroup SENSOR_DATA_TYPEDEFS Typedefs
 * @{ */

/** Data value index */
typedef enum {

	PHONE_1 = 0, PHONE_2 = 1, PHONE_3 = 2, PHONE_4 = 3

} PhoneIndex;

/** @} */

/*******************************************************************************
 *************************   PROTOTYPES   **************************************
 ******************************************************************************/

/** @addtogroup SENSOR_EVENT_PUBLIC_FUNCTIONS Public Functions
 * @{ */
/** @addtogroup SENSOR_EVENT_PROTOTYPES Prototypes
 * @{ */

bool TD_SENSOR_SetCellPhoneNumber(PhoneIndex index, uint8_t * phone_number);
void TD_SENSOR_SetDataTransmissionProfile(uint8_t repetition, uint32_t interval);

/** @} */

/** @} */

/** @} (end addtogroup SENSOR_DATA) */

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_DATA_H_ */
