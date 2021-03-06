/**************************************************************************//**
 * @file
 * @brief Capacitive sense peripheral API for the TDxxxx RF modules.
 * @author Telecom Design S.A.
 * @version 1.1.0
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
 *****************************************************************************/
#ifndef __TD_CAPSENSE_H
#define __TD_CAPSENSE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

	/***********************************************************************//**
	 * @addtogroup CAPSENSE
	 * @brief CAPSENSE Peripheral API for the TDxxxx RF modules
	 * @{
	 **************************************************************************/

	/***************************************************************************
	 *************************   PROTOTYPES   **********************************
	 **************************************************************************/

	/** @addtogroup CAPSENSE_USER_FUNCTIONS User Functions
	 * @{ */

	bool TD_CAPSENSE_Sense(void);
	void TD_CAPSENSE_Init(void);
	uint32_t TD_CAPSENSE_GetValue(void);

	/** @} */

	/** @} (end group CAPSENSE) */

#ifdef __cplusplus
}
#endif

#endif // __TD_CAPSENSE_H
