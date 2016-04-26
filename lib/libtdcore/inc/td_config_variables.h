/** @cond TD_CONFIG */
/***************************************************************************//**
 * @file
 * @brief Configuration file for the TDxxxx RF modules Flash variables.
 * @author Telecom Design S.A.
 * @version 1.0.0
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2013-2015 Telecom Design S.A., http://www.telecomdesign.fr</b>
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
#ifndef __TD_CONFIG_VARIABLES_H
#define __TD_CONFIG_VARIABLES_H

#include <stdint.h>
#include <stdbool.h>

#include <td_config_ext.h>

#include <td_flash.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef FLASH_VARIABLES_VERSION
#define FLASH_VARIABLES_VERSION 0
#endif

/** Maximum number of data pointers */
#ifdef TD_FLASH_MAX_DATA_POINTER

#define MAX_FLASH_DATA_POINTER TD_FLASH_MAX_DATA_POINTER

#else // !TD_FLASH_MAX_DATA_POINTER

#ifdef EFM32TG210F32
#define MAX_FLASH_DATA_POINTER	10
#else
#define MAX_FLASH_DATA_POINTER	25
#endif

#endif // TD_FLASH_MAX_DATA_POINTER

#ifndef	TD_FLASH_USER_PAGE
#define TD_FLASH_USER_PAGE		1
#endif

	uint32_t const CONFIG_TD_FLASH_VARIABLES_VERSION = FLASH_VARIABLES_VERSION;
	uint8_t const CONFIG_TD_FLASH_MAX_DATA_POINTER = MAX_FLASH_DATA_POINTER;
	static TD_FLASH_variable_t FlashDataList[MAX_FLASH_DATA_POINTER];
	TD_FLASH_variable_t *TD_FLASH_DataList = FlashDataList;
	uint8_t const CONFIG_TD_FLASH_USER_PAGE = TD_FLASH_USER_PAGE;

#ifdef __cplusplus
}
#endif

#endif // __TD_CONFIG_VARIABLES_H
/** @endcond */
