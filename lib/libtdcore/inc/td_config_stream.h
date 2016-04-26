/** @cond TD_CONFIG */
/***************************************************************************//**
 * @file
 * @brief Configuration file for the TDxxxx RF modules stream.
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
#ifndef __TD_CONFIG_STREAM_H
#define __TD_CONFIG_STREAM_H

#include <stdint.h>
#include <stdbool.h>

#include <td_config_ext.h>

#include <td_stream.h>

#ifdef __cplusplus
extern "C" {
#endif
#ifndef TD_STREAM_COUNT
#define TD_STREAM_COUNT			1
#endif

#ifndef TD_STREAM_FIFOSIZE

/** Stream receive FIFO size in characters */
#define TD_STREAM_FIFOSIZE		61
#endif

	uint8_t const CONFIG_TD_STREAM_COUNT = TD_STREAM_COUNT;
	TD_STREAM_cast_t TD_STREAM[TD_STREAM_COUNT];
	int const CONFIG_TD_STREAM_FIFOSIZE = TD_STREAM_FIFOSIZE;
	char TD_STREAM_FIFO[TD_STREAM_COUNT * TD_STREAM_FIFOSIZE];

#ifdef __cplusplus
}
#endif

#endif // __TD_CONFIG_STREAM_H
/** @endcond */
