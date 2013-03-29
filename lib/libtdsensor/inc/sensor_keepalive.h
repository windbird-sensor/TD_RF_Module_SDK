/***************************************************************************//**
 * @file sensor_keepalive.h
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

#ifndef SENSOR_KEEPALIVE_H_
#define SENSOR_KEEPALIVE_H_

#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************//**
 * @addtogroup SENSOR_KEEPALIVE Sensor Keep-Alive
 *
 * @{
 ******************************************************************************/

/*******************************************************************************
 *************************   PROTOTYPES   **************************************
 ******************************************************************************/

/** @addtogroup SENSOR_KEEPALIVE_PUBLIC_FUNCTIONS Public Functions
 * @{ */
/** @addtogroup SENSOR_KEEPALIVE_PROTOTYPES Prototypes
 * @{ */
bool TD_SENSOR_SendKeepAlive();

void TD_SENSOR_SetKeepAliveTransmissionProfile(uint8_t repetition,uint32_t interval);

/** @} */

/** @} */
/** @} (end addtogroup SENSOR_KEEPALIVE) */

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_KEEPALIVE_H_ */
