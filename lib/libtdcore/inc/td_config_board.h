/** @cond TD_CONFIG */
/***************************************************************************//**
 * @file
 * @brief Configuration file for the TDxxxx RF standard boards.
 * @author Telecom Design S.A.
 * @version 1.1.0
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2013-2016 Telecom Design S.A., http://www.telecomdesign.fr</b>
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
#ifndef __TD_CONFIG_BOARD_H
#define __TD_CONFIG_BOARD_H

#include <stdint.h>
#include <stdbool.h>

#include <td_config_ext.h>

#include <td_gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Check MANDATORY variables and set others to default values */
#ifdef MODULE_REVISION	/* Do we have a standard board? */

#ifndef PRODUCT_TYPE	/* Do we want to automatically define the chip? */

#if MODULE_REVISION == REVISION_TD1202
#define PRODUCT_TYPE			0
#define CHIP_TD1202
#endif

#if MODULE_REVISION == REVISION_TD1204
#define PRODUCT_TYPE			8
#define CHIP_TD1204
#endif

#if MODULE_REVISION == REVISION_TD1205
#define PRODUCT_TYPE			9
#define PRODUCT_LED_POLARITY 	1
#define CHIP_TD1205
#endif

#if MODULE_REVISION == REVISION_TD1208
#define PRODUCT_TYPE			10
#define CHIP_TD1208
#endif

#if MODULE_REVISION == REVISION_TD1508
#define PRODUCT_TYPE			31
#define CHIP_TD1508
#endif

#if MODULE_REVISION == REVISION_REF_DESIGN_FCC
#define PRODUCT_TYPE			34
#define CHIP_REF_DESIGN_FCC
#endif

#if MODULE_REVISION == REVISION_REF_DESIGN_ETSI
#define PRODUCT_TYPE			35
#define PRODUCT_LED_POLARITY 	1
#define CHIP_REF_DESIGN_ETSI
#endif

#if MODULE_REVISION == REVISION_TD1205P
#define PRODUCT_TYPE			36
#define PRODUCT_LED_POLARITY 	1
#define CHIP_TD1205P
#endif

#endif // PRODUCT_TYPE

#endif // MODULE_REVISION

#ifdef __cplusplus
}
#endif

#endif // __TD_CONFIG_BOARD_H
/** @endcond */
