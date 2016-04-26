/** @cond TD_CONFIG */
/***************************************************************************//**
 * @file
 * @brief Configuration file for the TDxxxx RF modules RF LAN.
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
#ifndef __TD_CONFIG_LAN_H
#define __TD_CONFIG_LAN_H

#include <stdint.h>
#include <stdbool.h>

#include <td_config_ext.h>

#ifdef __cplusplus
extern "C" {
#endif
/** LAN sampling period */
#ifndef LAN_PERIOD
#define LAN_PERIOD 				T1S
#endif

/** LAN threshold level (LBT standard TX) */
#ifndef LAN_THRESHOLD
#define LAN_THRESHOLD 			40
#endif

/** LAN threshold level (Windowed reception) */
#ifndef LAN_THRESHOLD_RX
#define LAN_THRESHOLD_RX		LAN_THRESHOLD
#endif

/** LAN LBT count max */
#ifndef LAN_LBT_COUNT_MAX
#define LAN_LBT_COUNT_MAX 		1
#endif

/** LAN address size */
#ifndef LAN_ADDRESS_SIZE
#define LAN_ADDRESS_SIZE 		24
#endif

/** LAN time check callback */
#ifndef LAN_CHECK_CALLBACK_TIME
#define LAN_CHECK_CALLBACK_TIME	(T1MS * 175) //doesn't work if removed...
#endif

#if ((LAN_PERIOD / T26_6MS) >= (1 << (31 - LAN_ADDRESS_SIZE)))
#error("LAN_PERIOD/LAN_ADDRESS_SIZE error. LAN_PERIOD or LAN_ADDRESS_SIZE is too large ")
#endif

#if ((LAN_PERIOD / T26_6MS) >= 253)
#error("LAN_PERIOD too high. For the time no more than 253 packets can be send. This limitation is to be removed")
#endif

#if (LAN_CHECK_CALLBACK_TIME > 0x00ffffff)
#error("LAN_CHECK_CALLBACK_TIME error. LAN_CHECK_CALLBACK_TIME is too large ")
#endif

	uint32_t const CONFIG_LAN_PERIOD = LAN_PERIOD;
	uint8_t const CONFIG_LAN_THRESHOLD = LAN_THRESHOLD;
	uint8_t const CONFIG_LAN_THRESHOLD_RX = LAN_THRESHOLD_RX;
	uint8_t const CONFIG_LAN_LBT_COUNT_MAX = LAN_LBT_COUNT_MAX;
	uint8_t const CONFIG_LAN_ADDRESS_SIZE = LAN_ADDRESS_SIZE;
	uint32_t const CONFIG_LAN_CHECK_CALLBACK_TIME = LAN_CHECK_CALLBACK_TIME;

#ifdef __cplusplus
}
#endif

#endif // __TD_CONFIG_LAN_H
/** @endcond */
