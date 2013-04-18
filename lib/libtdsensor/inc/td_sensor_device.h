/***************************************************************************//**
 * @file td_sensor_device.h
 * @brief Sensor LAN Device
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

#ifndef TD_SENSOR_DEVICE_H_
#define TD_SENSOR_DEVICE_H_

#include "sensor_private.h"
#include "td_sensor_lan.h"
#include "sensor_send.h"


/** @defgroup TD_SENSOR_DEVICE_USER_FUNCTIONS User Functions
 *  @ingroup TD_SENSOR_DEVICE
 *  @nosubgrouping
 */


/***************************************************************************//**
 * @addtogroup TD_SENSOR_DEVICE Sensor LAN Device
 * @brief
 *	Devices LAN functions to Register, Forward Sensor Frames, send custom Data
 *	and send Keep-alive to the gateway.
 *
 * @{
 ******************************************************************************/

/*******************************************************************************
 **************************  DEFINES   ****************************************
 ******************************************************************************/

/** @addtogroup TD_SENSOR_DEVICE_DEFINES Defines
 * @{ */
#define LOCAL_REGISTER_CHECK 0x37
/** @} */

/*******************************************************************************
 *************************   PROTOTYPES   **************************************
 ******************************************************************************/

/** @addtogroup TD_SENSOR_DEVICE_PUBLIC_FUNCTIONS Public Functions
 * @{ */
/** @addtogroup TD_SENSOR_DEVICE_PROTOTYPES Prototypes
 * @{ */


AckCode TD_SENSOR_DEVICE_Forward(uint8_t * payload, uint8_t count, uint8_t repetition, uint32_t interval);
AckCode TD_SENSOR_DEVICE_KeepAlive(bool keepalive, uint32_t interval, bool rssi, int8_t level_low, int8_t level_ok);


/** @ingroup TD_SENSOR_DEVICE_USER_FUNCTIONS
 * @{ */

AckCode TD_SENSOR_DEVICE_Data(uint8_t * data, uint8_t count, uint8_t data_rx[TD_SENSOR_LAN_PAYLOAD_SIZE]);
AckCode TD_SENSOR_DEVICE_Register();
void TD_SENSOR_DEVICE_Reset();

/** @} */


/** @} */

/** @} */

/** @} (end addtogroup TD_SENSOR_DEVICE) */

#endif /* TD_SENSOR_DEVICE_H_ */
