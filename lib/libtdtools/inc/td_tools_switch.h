/***************************************************************************//**
 * @file
 * @brief Switch API for the TDxxxx RF modules.
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

#ifndef __TD_TOOLS_SWITCH_H
#define __TD_TOOLS_SWITCH_H

#include <stdint.h>
#include <stdbool.h>
#include <td_tools.h>

#ifdef __cplusplus
extern "C" {
#endif

	/***********************************************************************//**
	 * @addtogroup SWITCH
	 * @brief Switch API for the TDxxxx RF modules
	 * @{
	 **************************************************************************/

	/***************************************************************************
	 ***********************   ENUMERATIONS   **********************************
	 **************************************************************************/

	/** @addtogroup SWITCH_ENUMERATIONS Enumerations
	 * @{ */

	/** Switch events */
	typedef enum {
		TD_TOOLS_SWITCH_NO_EVENT,
		TD_TOOLS_SWITCH_STARTUP_PUSHED,
		TD_TOOLS_SWITCH_STARTUP_LONG_PUSHED,
		TD_TOOLS_SWITCH_STARTUP_LONG_RELEASED,
		TD_TOOLS_SWITCH_STARTUP_RELEASED,
		TD_TOOLS_SWITCH_SHORT_PUSHED,
		TD_TOOLS_SWITCH_SHORT_RELEASED,
		TD_TOOLS_SWITCH_LONG_PUSHED,
		TD_TOOLS_SWITCH_LONG_RELEASED,
		TD_TOOLS_SWITCH_VERY_LONG_PUSHED,
		TD_TOOLS_SWITCH_VERY_LONG_RELEASED,
		TD_TOOLS_SWITCH_EXTRA_LONG_PUSHED,
		TD_TOOLS_SWITCH_EXTRA_LONG_RELEASED,
		TD_TOOLS_SWITCH_DOUBLE_CLICK,
		TD_TOOLS_SWITCH_TRIPLE_CLICK,
		TD_TOOLS_SWITCH_FIVE_CLICK,
		TD_TOOLS_SWITCH_FIVE_CLICK_PUSH,
		TD_TOOLS_SWITCH_MULTIPLE_CLICK_PENDING,
		TD_TOOLS_SWITCH_MULTIPLE_CLICK_DONE,
		TD_TOOLS_SWITCH_ON,
		TD_TOOLS_SWITCH_OFF
	} TD_TOOLS_SWITCH_Events_t;

	/** @} */

	/***************************************************************************
	 *************************   PROTOTYPES   **********************************
	 **************************************************************************/

	/** @addtogroup SWITCH_USER_FUNCTIONS User Functions
	 * @{ */

	int TD_TOOLS_SWITCH_Init(GPIO_Port_TypeDef port, uint8_t pin,
		GPIO_Mode_TypeDef mode, bool pull, bool push, int8_t LongTime,
		int8_t LongLongTime, int8_t LongLongLongTime);
	TD_TOOLS_SWITCH_Events_t TD_TOOLS_SWITCH_Process(uint8_t id);
	bool TD_TOOLS_SWITCH_Start(uint8_t id);
	bool TD_TOOLS_SWITCH_StartPolling(uint8_t id, uint32_t interval);
	bool TD_TOOLS_SWITCH_Stop(uint8_t id);
	bool TD_TOOLS_SWITCH_StopPolling(uint8_t id);
	bool TD_TOOLS_SWITCH_ConfigureMultipleClick(uint8_t id, uint8_t interval);
	uint8_t TD_TOOLS_SWITCH_GetMultipleClickCount(uint8_t id);
	bool TD_TOOLS_SWITCH_SetPushTime(uint8_t id, int8_t long_time,
		int8_t very_long_time, int8_t extra_long_time);
	uint32_t TD_TOOLS_SWITCH_GetPushTime(uint8_t id);
	bool TD_TOOLS_SWITCH_Simulate(uint8_t i, TD_TOOLS_SWITCH_Events_t event);
	bool TD_TOOLS_SWITCH_IsOn(uint8_t id);
	uint8_t TD_TOOLS_SWITCH_CountIrq(void);

	/** @} */

	/** @} (end addtogroup SWITCH) */

#ifdef __cplusplus
}
#endif

#endif // __TD_TOOLS_SWITCH_H
