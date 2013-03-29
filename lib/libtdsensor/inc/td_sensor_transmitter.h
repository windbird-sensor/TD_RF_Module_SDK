/***************************************************************************//**
 * @file td_sensor_transmitter.h
 * @brief Sensor Transmitter
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

#ifndef TD_SENSOR_TRANSMITTER_H_
#define TD_SENSOR_TRANSMITTER_H_

/***************************************************************************//**
 * @addtogroup TD_SENSOR_TRANSMITTER Sensor Transmitter
 *
 *  * @{
 ******************************************************************************/

/*******************************************************************************
 *************************   PROTOTYPES   **************************************
 ******************************************************************************/

/** @addtogroup TD_SENSOR_TRANSMITTER_PUBLIC_FUNCTIONS Public Functions
 * @{ */
/** @addtogroup TD_SENSOR_TRANSMITTER_PROTOTYPES Prototypes
 * @{ */

bool TD_SENSOR_TRANSMITTER_SendSigfox(SensorFrame * frame, uint8_t count, uint8_t entry_id, TransmitProfile * profile);
void TD_SENSOR_TRANSMITTER_Process();
void TD_SENSOR_TRANSMITTER_Init();

/** @} */

/** @} */

/** @} (end addtogroup TD_SENSOR_TRANSMITTER) */

#endif /* TD_SENSOR_TRANSMITTER_H_ */
