/***************************************************************************//**
 * @file sensor_register.c
 * @brief API for sending Register frame type to Sensor
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

#ifndef SENSOR_REGISTER_H_
#define SENSOR_REGISTER_H_

#ifdef __cplusplus
extern "C" {
#endif


/** @defgroup SENSOR_REGISTER_USER_FUNCTIONS User Functions
 *  @ingroup SENSOR_REGISTER
 *  @nosubgrouping
 */

/***************************************************************************//**
 * @addtogroup SENSOR_REGISTER Sensor Register
 * @{
 ******************************************************************************/

/*******************************************************************************
 *************************   PROTOTYPES   **************************************
 ******************************************************************************/

/** @addtogroup SENSOR_REGISTER_PUBLIC_FUNCTIONS Public Functions
 * @{ */
/** @addtogroup SENSOR_REGISTER_PROTOTYPES Prototypes
 * @{ */

/** @ingroup SENSOR_REGISTER_USER_FUNCTIONS
 * @{ */

bool TD_SENSOR_SendRegister();

void TD_SENSOR_SetRegisterTransmissionProfile(uint8_t repetition, uint32_t interval);

/** @} */

/** @} */

/** @} */
/** @} (end addtogroup SENSOR_REGISTER) */

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_REGISTER_H_ */
