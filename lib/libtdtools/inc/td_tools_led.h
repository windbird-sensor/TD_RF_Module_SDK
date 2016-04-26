/***************************************************************************//**
 * @file
 * @brief LED API for the TDxxxx RF modules.
 * @author Telecom Design S.A.
 * @version 1.1.0
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2014-2015 Telecom Design S.A., http://www.telecomdesign.fr</b>
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

#ifndef __TD_TOOLS_LED_H
#define __TD_TOOLS_LED_H

#include <stdint.h>
#include <stdbool.h>
#include <td_tools.h>

#ifdef __cplusplus
extern "C" {
#endif

	/***********************************************************************//**
	 * @addtogroup LED
	 * @brief LED API for the TDxxxx RF modules
	 * @{
	 **************************************************************************/

	/***************************************************************************
	 *************************   DEFINES   *************************************
	 **************************************************************************/

	/** @addtogroup LED_DEFINES Defines
	 * @{ */

	/** Use LETIMER0 for LEDs */
#define LED_LETIMER0	0

	/** Use LETIMER0 for LEDs */
#define LED_LETIMER1	1

	/** LED is active low */
#define LED_OUTPUT0		0

	/** LED is active high */
#define LED_OUTPUT1		1

#define LED_LOCATION0	0		///< Use location 0 for LED
#define LED_LOCATION1	1		///< Use location 1 for LED
#define LED_LOCATION2	2		///< Use location 2 for LED
#define LED_LOCATION3	3		///< Use location 3 for LED
#define LED_LOCATION4	4		///< Use location 4 for LED
#define LED_LOCATION5	5		///< Use location 5 for LED

	/** Macro to encode LED information */
#define LTO(timer, output, location)		((timer) | ((output) << 2) | ((location) << 3))

	/** @} */

/***************************************************************************
	 *************************   PROTOTYPES   **********************************
	 **************************************************************************/

	/** @addtogroup LED_USER_FUNCTIONS User Functions
	 * @{ */

	int TD_TOOLS_LED_Init(TD_GPIO_Port_TypeDef port, uint8_t bit,
		GPIO_DriveMode_TypeDef drive, bool polarity);
	bool TD_TOOLS_LED_InitInverted(uint8_t id1, uint8_t id2);
	bool TD_TOOLS_LED_Set(uint8_t id, bool enable);
	bool TD_TOOLS_PUSHPULL_Set(uint8_t id, bool state);
	bool TD_TOOLS_LED_Toggle(uint8_t id);
	bool TD_TOOLS_LED_StartBlink(uint8_t id, uint8_t seconds, uint16_t ticks,
		uint8_t repeats);
	bool TD_TOOLS_LED_StopBlink(uint8_t id, bool nice_stop);
	bool TD_TOOLS_LED_StartFaintBlink(uint8_t id, uint8_t seconds);
	void TD_TOOLS_LED_ReinitPWM (uint8_t id, uint8_t period, uint8_t value);
	int TD_TOOLS_LED_SetPWM(uint8_t id,uint8_t period, uint8_t value);
	int TD_TOOLS_LED_InitPWM_Pump(TD_GPIO_Port_TypeDef port_pump,
		uint8_t bit_pump, GPIO_DriveMode_TypeDef drive_pump,
		bool polarity,uint8_t letimerId, uint8_t period, uint8_t value,
		uint8_t vcomp, TD_GPIO_Port_TypeDef port, uint8_t bit,
		GPIO_DriveMode_TypeDef drive);

	/** @} */

	/** @} (end addtogroup LED) */

#ifdef __cplusplus
}
#endif

#endif // __TD_TOOLS_LED_H
