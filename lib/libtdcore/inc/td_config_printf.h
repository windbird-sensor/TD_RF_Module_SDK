/** @cond TD_CONFIG */
/***************************************************************************//**
 * @file
 * @brief Configuration file for the TDxxxx RF modules printf.
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
#ifndef __TD_CONFIG_PRINTF_H
#define __TD_CONFIG_PRINTF_H

#include <stdint.h>
#include <stdbool.h>

#include <td_config_ext.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef PRINTF_INT64_SUPPORT
#define PRINTF_ULONG_FNC ulli2a
#define PRINTF_LONG_FNC lli2a
#define PRINTF_INT64_SUPPORT_ON 1
#else
#define PRINTF_ULONG_FNC uli2a
#define PRINTF_LONG_FNC li2a
#define PRINTF_INT64_SUPPORT_ON 0
#endif

	const void * CONFIG_PRINTF_ULONG = (void *) PRINTF_ULONG_FNC;
	const void * CONFIG_PRINTF_LONG = (void *) PRINTF_LONG_FNC;
	const uint8_t CONFIG_PRINTF_INT64_SUPPORT_ON = PRINTF_INT64_SUPPORT_ON;

#ifdef __cplusplus
}
#endif

#endif // __TD_CONFIG_PRINTF_H
/** @endcond */
