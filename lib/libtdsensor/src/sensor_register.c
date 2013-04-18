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

#include <stdint.h>
#include <stdbool.h>

#include "sensor_config.h"
#include "td_sensor.h"
#include "sensor_send.h"
#include "sensor_register.h"
#include "sensor_register_private.h"

/***************************************************************************//**
 * @addtogroup SENSOR_REGISTER Sensor Register
 * @brief Sensor API for sending a Register Frame
 *
 * The Register frame must be sent to register a new device on Sensor.
 * @{
 ******************************************************************************/

/** @addtogroup SENSOR_REGISTER_DEFINES Defines
 * @{ */

#define REGISTER_DEFAULT_REPETITON 3	///<Default retransmission repetitions
#define REGISTER_DEFAULT_INTERVAL 120	///<Default retransmission interval in seconds
#define REGISTER_PAYLOAD_SIZE 7

/** @} */



/*******************************************************************************
 ************************   PRIVATE VARIABLES   ********************************
 ******************************************************************************/

/** @addtogroup SENSOR_REGISTER_PRIVATE_VARIABLES Private Variables
 * @{ */

static TransmitProfile register_profile = { REGISTER_DEFAULT_REPETITON, REGISTER_DEFAULT_INTERVAL };

/** @} */


/*******************************************************************************
 **************************  PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup SENSOR_REGISTER_PUBLIC_FUNCTIONS Public Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Send a Register frame to Sensor.
 *
 * @details
 * 	 Only a registered device can get be seen on Sensor. This frame must be sent before any other.
 *
 * @return
 *   True if the data has been sent over the Sigfox Network
 ******************************************************************************/
bool TD_SENSOR_SendRegister()
{
	SRV_FRAME_REGISTER frame;
	ModuleConfiguration * config;
	uint32_t sigfox_id;

	sigfox_id = TD_SENSOR_GetSigfoxID();
	config = TD_SENSOR_GetModuleConfiguration();

	frame.release = RELEASE & 0x07;
	frame.sub_release = SUB_RELEASE & 0x07;

	frame.device_class_byte1 = config->class >> 8;
	frame.device_class_byte2 = config->class & 0xFF;

	frame.sigfox_id_byte1 = sigfox_id >> 24;
	frame.sigfox_id_byte2 = (sigfox_id >> 16) & 0xFF;
	frame.sigfox_id_byte3 = (sigfox_id >> 8) & 0xFF;
	frame.sigfox_id_byte4 = sigfox_id & 0xFF;

	return TD_SENSOR_Send(&register_profile, SRV_FRM_REGISTER, 0, (uint8_t *) &frame, REGISTER_PAYLOAD_SIZE);
}

/***************************************************************************//**
 * @brief
 *   Set a transmission profile to a given frame type
 *
 * @param[in] repetition
 *	Number of repetition
 *
 * @param[in] interval
 *	Interval between two repetitions in seconds
 *
 ******************************************************************************/
void TD_SENSOR_SetRegisterTransmissionProfile(uint8_t repetition, uint32_t interval)
{
	register_profile.repetition = repetition;
	register_profile.interval = interval;
}

/** @} */

/** @} */
