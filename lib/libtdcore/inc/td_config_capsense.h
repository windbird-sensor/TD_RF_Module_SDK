/***************************************************************************//**
 * @file
 * @brief Configuration file for the TDxxxx RF modules accelerometer.
 * @author Telecom Design S.A.
 * @version 1.0.0
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2016 Telecom Design S.A., http://www.telecomdesign.fr</b>
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
#ifndef __TD_CONFIG__CAPSENSE_H
#define __TD_CONFIG__CAPSENSE_H

#include <stdint.h>
#include <stdbool.h>

#include <td_config_ext.h>

#include <td_gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __TD_CAPSENSE_H

#ifndef ACMP_CAPSENSE
#error("ACMP_CAPSENSE is undefined")
#endif

#ifndef ACMP_CAPSENSE_CHANNEL
#error("ACMP_CAPSENSE_CHANNEL is undefined")
#endif

#ifndef ACMP_CAPSENSE_CLKEN
#error("ACMP_CAPSENSE_CLKEN is undefined")
#endif

#ifndef PRS_CH_CTRL_SOURCESEL_ACMP_CAPSENSE
#error("PRS_CH_CTRL_SOURCESEL_ACMP_CAPSENSE is undefined")
#endif

#ifndef PRS_CH_CTRL_SIGSEL_ACMPOUT_CAMPSENSE
#error("PRS_CH_CTRL_SIGSEL_ACMPOUT_CAMPSENSE is undefined")
#endif

#ifndef CAPSENSE_DELTA_CHANGE_THRESHOLD
#error("CAPSENSE_DELTA_CHANGE_THRESHOLD is undefined")
#endif

#ifndef CAPSENSE_HISTORY_DEPTH
#error("CAPSENSE_HISTORY_DEPTH is undefined")
#endif

	ACMP_TypeDef * const CONFIG_ACMP_CAPSENSE = ACMP_CAPSENSE;
	uint8_t const CONFIG_ACMP_CAPSENSE_CHANNEL = ACMP_CAPSENSE_CHANNEL;
	uint32_t const CONFIG_ACMP_CAPSENSE_CLKEN = ACMP_CAPSENSE_CLKEN;
	uint32_t const CONFIG_PRS_CH_CTRL_SOURCESEL_ACMP_CAPSENSE = PRS_CH_CTRL_SOURCESEL_ACMP_CAPSENSE;
	uint32_t const CONFIG_PRS_CH_CTRL_SIGSEL_ACMPOUT_CAMPSENSE = PRS_CH_CTRL_SIGSEL_ACMPOUT_CAMPSENSE;
	int32_t const CONFIG_CAPSENSE_DELTA_CHANGE_THRESHOLD = CAPSENSE_DELTA_CHANGE_THRESHOLD;
	uint8_t const CONFIG_CAPSENSE_HISTORY_DEPTH = CAPSENSE_HISTORY_DEPTH;
	uint32_t CapsenseHistory[CAPSENSE_HISTORY_DEPTH];

#endif // __TD_CAPSENSE_H

#ifdef __cplusplus
}
#endif

#endif // __TD_CONFIG__CAPSENSE_H

