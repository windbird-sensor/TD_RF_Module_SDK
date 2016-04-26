/***************************************************************************//**
 * @file
 * @brief Hall effect functions for the TD1205P RF modules.
 * @author Telecom Design S.A.
 * @version 1.0.0
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2016 Telecom Design S.A., http://www.telecomdesign.fr</b>
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
#include <string.h>

#include "td_core.h"
#include "td_printf.h"
#include "td_scheduler.h"
#include "td_rtc.h"
#include "td_halleffect.h"
#include "td_gpio.h"

/***************************************************************************//**
 * @addtogroup HALLEFFECT
 * @brief Hall effect functions for the TD1205P RF modules.
 * @{
 ******************************************************************************/

/*******************************************************************************
 *************************   DEFINES   *****************************************
 ******************************************************************************/

/** @addtogroup HALLEFFECT_DEFINES Defines
 * @{ */

/** Duration for sensor on */
#define TIME_ON 			(T160MS)

/** Poll period */
#define PERIOD				(T1S)

/** When magnet is detected, turn on fast polling */
#define PERIOD_DETECTION	(T500MS)

/** Duration for sensor off */
#define TIME_OFF			(PERIOD-TIME_ON)

/** Duration for sensor off when polling fast */
#define TIME_OFF_DETECTION	(PERIOD_DETECTION-TIME_ON)

/** @} */

/*******************************************************************************
 *************************   ENUMERATION****************************************
 ******************************************************************************/

/** Hall state sensor states */
typedef enum
{
	DISABLE,
	IDLE,
	POWERED,
} TD_HALL_state;

/** Hall effect sensor configuration */
typedef struct{
	TD_GPIO_Port_TypeDef CommandPort;
	unsigned int CommandBit;
	TD_GPIO_Port_TypeDef OutPort;
	unsigned int OutBit;
} TD_HALL_config;

/*******************************************************************************
 *************************   PRIVATE VARIABLES   *******************************
 ******************************************************************************/

/** @addtogroup HALLEFFECT_LOCAL_VARIABLES Local Variables
 * @{ */

static uint8_t HallTimer = 0xFF;
static TD_HALL_config Config = {.CommandPort = TD_GPIO_PortNull};
static TD_HALL_state State = DISABLE;
static uint16_t MagnetCount = 0;

static TD_HALL_EFFECT_Events_t Event = TD_HALLEFFECT_NO_EVENT;
static uint16_t MagnetCountEvent = 0;

/** @} */

/*******************************************************************************
 *************************   PRIVATE FUNCTIONS   *******************************
 ******************************************************************************/
/** @addtogroup HALLEFFECT_PRIVATE_FUNCTIONS Private Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Hall effect sensor callback function.
 *
 * @param[in] arg
 *   Not used.
 *
 * @param[in] repetition
 *   Not used.
 ******************************************************************************/
static void HallEffectTimer(uint32_t arg, uint8_t repetition)
{
	bool magnet_present = false;

	switch (State) {
	case DISABLE:

		//trap
		break;

	case IDLE:
		GPIO_PinModeSet(Config.CommandPort, Config.CommandBit, gpioModePushPull, 1);
		TD_SCHEDULER_SetInterval(HallTimer,
			TIME_ON/32768,
			TIME_ON % 32768,
			0);
		State = POWERED;
		break;

	case POWERED:

		// Read if magnet is here
		magnet_present = !GPIO_PinInGet(Config.OutPort, Config.OutBit);
		GPIO_PinModeSet(Config.CommandPort, Config.CommandBit, gpioModePushPull, 0);
		if (magnet_present) {
			MagnetCount++;
			if (Event != TD_HALLEFFECT_MAGNET_RELEASED) {
				Event = TD_HALLEFFECT_MAGNET_PRESENT;
				MagnetCountEvent = MagnetCount;
			}
			TD_SCHEDULER_SetInterval(HallTimer,
				TIME_OFF_DETECTION/32768,
				TIME_OFF_DETECTION % 32768,
				0);
		} else {
			if (MagnetCount != 0) {

				//Magnet RELEASED
				Event = TD_HALLEFFECT_MAGNET_RELEASED;
				MagnetCountEvent = MagnetCount;
				MagnetCount = 0;
			}
			TD_SCHEDULER_SetInterval(HallTimer,
				TIME_OFF/32768,
				TIME_OFF % 32768,
				0);
		}
		State = IDLE;
		break;
	}
}

/** @} */

/*******************************************************************************
 **************************   PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup HALLEFFECT_USER_FUNCTIONS User Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Initialize Hall effect sensor.
 *
 * @param[in] command_port
 *   The port to use for powering the sensor.
 *
 * @param[in] command_bit
 *   The bit to use for powering the sensor.
 *
 * @param[in] out_port
 *   The port used to read the sensor value.
 *
 * @param[in] out_bit
 *   The bit used to read the sensor value.
 *
 * @return
 *   Returns true if everything is OK, false otherwise.
 ******************************************************************************/
bool TD_HALLEFFECT_Init(TD_GPIO_Port_TypeDef command_port, uint8_t command_bit,
		TD_GPIO_Port_TypeDef out_port, uint8_t out_bit)
{
	Config.CommandPort = command_port;
	Config.CommandBit = command_bit;
	Config.OutPort = out_port;
	Config.OutBit = out_bit;

	//Configure as input and output the correct pin
	GPIO_PinModeSet(Config.CommandPort, Config.CommandBit, gpioModePushPull, 0);

	// Define the pin to read the output of the halleffect
	GPIO_PinModeSet(Config.OutPort, Config.OutBit, gpioModeInput, 0);
	State = DISABLE;
	return true;
}

/***************************************************************************//**
 * @brief
 *  Activate / disable the Hall effect sensor polling.
 *
 * @param[in] on
 *   Set to true to active, false to disable.
 *
 * @return
 *   Returns true if everything is OK, false otherwise.
 ******************************************************************************/
bool TD_HALLEFFECT_PollingEnable(bool on)
{
	if (on) {
		if (Config.CommandPort == TD_GPIO_PortNull) {

			//No initialised
			TD_Trap(TRAP_USAGE_FAULT, 1);
			return false;
		}
		HallTimer = TD_SCHEDULER_AppendIrq(TIME_ON / 32768,
			TIME_ON % 32768,
			0,
			TD_SCHEDULER_INFINITE,
			HallEffectTimer,
			0);
		if (HallTimer == 0xFF) {
			TD_Trap(TRAP_USAGE_FAULT, 2);
			return false;
		}
		GPIO_PinModeSet(Config.CommandPort, Config.CommandBit, gpioModePushPull, 1);
		State = POWERED;
	} else {
		if (HallTimer != 0xFF) {
			TD_SCHEDULER_Remove(HallTimer);
			HallTimer = 0xFF;
		}
		GPIO_PinModeSet(Config.CommandPort, Config.CommandBit, gpioModePushPull, 0);
		State = DISABLE;
	}
	return true;
}

/***************************************************************************//**
 * @brief
 *   Compute the next pseudo-random PN value.
 *
 * @param[out] DetectionCount
 *   Will be filled in with the number of detection count.
 *
 * @return
 *   Returns the Hall sensor effect event structure.
 ******************************************************************************/
TD_HALL_EFFECT_Events_t TD_HALLEFFECT_Process(uint16_t *DetectionCount)
{
	TD_HALL_EFFECT_Events_t local_event;
	*DetectionCount = MagnetCountEvent;

	local_event = Event;
	if (Event == TD_HALLEFFECT_MAGNET_RELEASED) {
		Event = TD_HALLEFFECT_NO_EVENT;
	}
	return local_event;
}

/** @} */

/** @} (end addtogroup HALLEFFECT) */
