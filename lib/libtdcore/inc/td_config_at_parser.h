/** @cond TD_CONFIG */
/***************************************************************************//**
 * @file
 * @brief Configuration file for the TDxxxx RF modules AT parser.
 * @author Telecom Design S.A.
 * @version 1.0.2
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
#ifndef __TD_CONFIG_AT_PARSER_H
#define __TD_CONFIG_AT_PARSER_H

#include <stdint.h>
#include <stdbool.h>

#include <td_config_ext.h>

#ifdef __cplusplus
extern "C" {
#endif

/* If MANUFACTURER is not defined, we suppose no AT command is included */
#ifdef MANUFACTURER
#ifndef PRODUCT
#error("To use AT command, PRODUCT must be defined")
#endif

#ifndef HARDWARE_VERSION
#error("To use AT command, HARDWARE_VERSION must be defined")
#endif

#ifndef SOFTWARE_VERSION
#error("To use AT command, SOFTWARE_VERSION must be defined")
#endif

#ifndef RELEASE_DATE
#error("To use AT command, RELEASE_DATE must be defined")
#endif

#ifndef SERIAL_NUMBER
#error("To use AT command, SERIAL_NUMBER must be defined")
#endif

#ifndef AT_ARG_CASE_SENSITIVE
#define AT_ARG_CASE_SENSITIVE	0
#endif

#ifndef AT_LEUART_PERSIST
#define AT_LEUART_PERSIST		0
#endif

#ifndef	AT_PERSIST_SIZE

#ifdef EFM32TG210F32
/** AT persist buffer size */
#define AT_PERSIST_SIZE		36
#else
/** AT persist buffer size */
#define AT_PERSIST_SIZE		220
#endif

#endif // AT_PERSIST_SIZE

    /** AT device Manufacturer */
    char const *CONFIG_MANUFACTURER = MANUFACTURER" "PRODUCT;

    /** AT hardware revision */
    char const *CONFIG_HARDWARE_VERSION = HARDWARE_VERSION;

    /** AT software revision */
    char const *CONFIG_SOFTWARE_VERSION = SOFTWARE_VERSION;

    /** AT firmware release date */
    char const *CONFIG_RELEASE_DATE = RELEASE_DATE;

    /** AT device serial number */
	char *CONFIG_SERIAL_NUMBER = SERIAL_NUMBER;
	bool const CONFIG_AT_LEUART_PERSIST = AT_LEUART_PERSIST;
	bool const CONFIG_AT_ARG_CASE_SENSITIVE = AT_ARG_CASE_SENSITIVE;
	uint16_t const CONFIG_AT_PERSIST_SIZE = AT_PERSIST_SIZE;

	/** AT persist buffer */
	uint8_t AT_persist_buffer_buffer[AT_PERSIST_SIZE];
	uint8_t *AT_persist_buffer = AT_persist_buffer_buffer;

#endif /*MANUFACTURER*/

#ifdef __cplusplus
}
#endif

#endif // __TD_CONFIG_AT_PARSER_H
/** @endcond */
