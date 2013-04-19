/***************************************************************************//**
 * @file td_sensor_gateway.h
 * @brief Sensor Gateway
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

#ifndef TD_SENSOR_GATEWAY_H_
#define TD_SENSOR_GATEWAY_H_

#include "td_lan.h"
#include "sensor_send_private.h"


/** @defgroup TD_SENSOR_GATEWAY_USER_FUNCTIONS User Functions
 *  @ingroup TD_SENSOR_GATEWAY
 *  @nosubgrouping
 */

/***************************************************************************//**
 * @addtogroup TD_SENSOR_GATEWAY Sensor LAN Gateway
 * @brief
 *
 * @{
 ******************************************************************************/

/*******************************************************************************
 *************************   PROTOTYPES   **************************************
 ******************************************************************************/

/** @addtogroup TD_SENSOR_GATEWAY_PUBLIC_FUNCTIONS Public Functions
 * @{ */
/** @addtogroup TD_SENSOR_GATEWAY_PROTOTYPES Prototypes
 * @{ */

void TD_SENSOR_GATEWAY_Init();
bool TD_SENSOR_GATEWAY_SendSigfox(SensorFrame * frame, uint8_t count,TransmitProfile * profile);

/** @ingroup TD_SENSOR_GATEWAY_USER_FUNCTIONS
 * @{ */

void TD_SENSOR_GATEWAY_StartRegistration(void (*callback)(uint32_t lan_address, uint32_t sigfox_id));
void TD_SENSOR_GATEWAY_StopRegistration();

void TD_SENSOR_GATEWAY_StartReception();
void TD_SENSOR_GATEWAY_StopReception();

void TD_SENSOR_GATEWAY_DeleteAllDevices();
void TD_SENSOR_GATEWAY_DeleteDevice(uint32_t lan_address);

/** @} */


/** @} */

/** @} */

/** @} (end addtogroup TD_SENSOR_GATEWAY) */

#endif /* TD_SENSOR_GATEWAY_H_ */