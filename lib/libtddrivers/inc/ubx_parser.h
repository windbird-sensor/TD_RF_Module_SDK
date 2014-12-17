/***************************************************************************//**
 * @file
 * @brief Driver definition for the LIS3DH accelerometer used in TD12xx RF modules.
 * @author Telecom Design S.A.
 * @version 1.0.0
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2013-2014 Telecom Design S.A., http://www.telecomdesign.fr</b>
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
#ifndef __UBX_PARSER_H
#define __UBX_PARSER_H

#include <stdint.h>
#include <stdbool.h>

#include "ubx7.h"

#ifdef __cplusplus
extern "C" {
#endif

	/***********************************************************************//**
	 * @addtogroup TD_UBX_PARSER UBX_PARSER
	 * @brief TD_UBX_PARSER
	 * @{
	 **************************************************************************/

	/***************************************************************************
	 **************************  ENUM   ****************************************
	 **************************************************************************/

	/** @addtogroup TD_UBX_PARSER_ENUM Enumerations
	 * @{ */

	/** Ublox parser result codes */
	typedef enum {
		TD_UBX_PARSER_PENDING,
		TD_UBX_PARSER_DONE,
		TD_UBX_PARSER_FAILED
	} TD_UBX_PARSER_LookForResult_t;

	/** @} */

	/***************************************************************************
	 *************************   PROTOTYPES   **********************************
	 **************************************************************************/

	/** @addtogroup TD_UBX_PARSER_GLOBAL_FUNCTIONS Global Functions
	 * @{ */

	void TD_UBX_PARSER_StopLookFor(void);
	TD_UBX_PARSER_LookForResult_t TD_UBX_PARSER_LookFor(TD_UBX7_Class_t class,
		uint8_t id, bool ack, uint8_t *data, uint16_t *len);
	void * TD_UBX_PARSER_SetDataPointer(TD_UBX7_Class_t class, uint8_t id,
		void *ptr, uint32_t param);
	bool TD_UBX_PARSER_IsMessageUpdated(TD_UBX7_Class_t class, uint8_t id);
	bool TD_UBX_PARSER_Parse(char data);
	void TD_UBX_PARSER_Init(void);
	void TD_UBX_PARSER_Reset(void);
	bool TD_UBX_PARSER_Process(void);

	/** @} */

	/** @} (end addtogroup TD_UBX_PARSER) */

#ifdef __cplusplus
}
#endif

#endif // __UBX_PARSER_H
