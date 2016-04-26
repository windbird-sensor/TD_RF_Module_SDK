/** @cond TD_CONFIG */
/***************************************************************************//**
 * @file
 * @brief Configuration file for the TDxxxx RF modules geoloc.
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
#ifndef __TD_CONFIG_GEOLOC_H
#define __TD_CONFIG_GEOLOC_H

#include <stdint.h>
#include <stdbool.h>

#include <td_config_ext.h>

#ifdef __cplusplus
extern "C" {
#endif

#if IS_EMPTY(TD_GEOLOC_USE_CODE)
#error("TD_GEOLOC_USE_CODE must be set to 0 or 1, or removed")
#endif

#if TD_GEOLOC_USE_CODE
/* If there is a compilation error at this point
 * -if you don't want to use libtdgeoloc : add #define TD_GEOLOC_USE_CODE 0
 * - if you want to use libtdgeoloc : add libtdgeoloc/inc include path and the
 *    path to the libtdgeoloc library corresponding to your build configuration
 *    in your project settings
 */

#include <td_accelero.h>
#include <td_geoloc.h>
#include <ubx7.h>

#ifndef TD_GEOLOC_RAW_OUTPUT
#define TD_GEOLOC_RAW_OUTPUT false
#endif

#ifdef TD_GEOLOC_LOGGER_REMOVE_CODE
TRAP_DYNAMIC(TD_FLASH_InitLogger);
TRAP_DYNAMIC(TD_GEOLOC_Log);
#else
INIT_DYNAMIC(TD_FLASH_InitLogger);
INIT_DYNAMIC(TD_GEOLOC_Log);
#endif

	bool const CONFIG_TD_GEOLOC_RAW_OUTPUT = TD_GEOLOC_RAW_OUTPUT;

#endif // TD_GEOLOC_USE_CODE

#ifdef __cplusplus
}
#endif

#endif // __TD_CONFIG_GEOLOC_H
/** @endcond */
