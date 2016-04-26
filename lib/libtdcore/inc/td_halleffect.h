/***************************************************************************//**
 * @file
 * @brief Hall effect functions for the TD1205P RF modules.
 * @author Telecom Design S.A.
 * @version 1.0.0
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2012-2016 Telecom Design S.A., http://www.telecomdesign.fr</b>
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

#ifndef __TD_HALLEFFECT_H
#define __TD_HALLEFFECT_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

	/***********************************************************************//**
	 * @addtogroup HALLEFFECT Hall Effect fonction
	 * @brief Hall effect functions for the TD1205P RF modules
	 * @{
	 **************************************************************************/

	/***************************************************************************
	 *************************   DEFINES   *************************************
	 **************************************************************************/

	/** @addtogroup HALLEFFECT_DEFINES Defines
	 * @{ */

#ifndef NULL

/** Define NULL if not already defined by someone else */
#define NULL	(void *) 0
#endif

	/** @} */


/***************************************************************************
	 ***********************   ENUMERATIONS   **********************************
	 **************************************************************************/

	/** @addtogroup HALLEFFECT_ENUMERATIONS Enumerations
	 * @{ */

	/** Halleffect events */
	typedef enum {
		TD_HALLEFFECT_DISABLED,
		TD_HALLEFFECT_NO_EVENT,
		TD_HALLEFFECT_MAGNET_PRESENT,	//Magnet detected at least 2 times
		TD_HALLEFFECT_MAGNET_RELEASED,	//End of detection
	} TD_HALL_EFFECT_Events_t;
	/** @} */

	/***************************************************************************
	 *************************   PROTOTYPES   **********************************
	 **************************************************************************/

	/** @addtogroup HALLEFFECT_USER_FUNCTIONS User Functions
	 * @{ */
	bool TD_HALLEFFECT_Init(TD_GPIO_Port_TypeDef command_port, uint8_t command_bit,
			TD_GPIO_Port_TypeDef out_port, uint8_t out_bit);
	bool TD_HALLEFFECT_PollingEnable(bool on);
	TD_HALL_EFFECT_Events_t TD_HALLEFFECT_Process(uint16_t * DetectionCount);

	/** @} */

	/** @} (end addtogroup HALLEFFECT) */

#ifdef __cplusplus
}
#endif

#endif // __TD_HALLEFFECT_H
