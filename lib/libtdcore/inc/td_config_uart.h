/** @cond TD_CONFIG */
/***************************************************************************//**
 * @file
 * @brief Configuration file for the TDxxxx RF modules UART.
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
#ifndef __TD_CONFIG_UART_H
#define __TD_CONFIG_UART_H

#include <stdint.h>
#include <stdbool.h>

#include <td_config_ext.h>

#include <td_uart.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(SYSTEM_UART) && (defined(LEUART_DEVICE) || defined(LEUART_LOCATION))
#error You should remove old LEUART_DEVICE and LEUART_LOCATION definitions
#endif

#ifdef SYSTEM_UART
#define LEUART_DEVICE		SYSTEM_UART
#endif

#ifdef SYSTEM_UART_LOCATION
#define LEUART_LOCATION		SYSTEM_UART_LOCATIION
#endif

#ifdef SYSTEM_UART_SPEED
#define LEUART_SPEED		SYSTEM_UART_SPEED
#endif

/* LEUART_LOCATION : location of system UART pin
 * default value : LEUART_ROUTE_LOCATION_LOC0
 */
#ifndef LEUART_LOCATION
#define LEUART_LOCATION LEUART_ROUTE_LOCATION_LOC0
#endif

/* LEUART_SPEED : UART baudrate
 * default value : 9600 bps
 */
#ifndef LEUART_SPEED
#define LEUART_SPEED 9600
#endif

/* LEUART_DEVICE : device used for system UART pin
 * default value : LEUART0
 */
#ifndef LEUART_DEVICE
#define LEUART_DEVICE LEUART0
#endif

/* TD_UART_COUNT : number of UARTs
 * default value : 1
 */
#ifndef TD_UART_COUNT
#define TD_UART_COUNT			1
#endif

#include <td_uart.h>

/* ALLOW_LEUART_LOW_BAUDRATE_ON_HFRCO : allow LEUART low baudrates on internal
 * HF RC Oscillator
 * default value : 0
 */
#ifndef ALLOW_LEUART_LOW_BAUDRATE_ON_HFRCO
#define ALLOW_LEUART_LOW_BAUDRATE_ON_HFRCO	0
#endif

	uint8_t const CONFIG_TD_UART_COUNT = TD_UART_COUNT;
	TD_UART_port_t TD_UART[TD_UART_COUNT];
	void * const CONFIG_LEUART_DEVICE = LEUART_DEVICE;
	uint32_t const CONFIG_LEUART_LOCATION = LEUART_LOCATION;
	uint32_t const CONFIG_LEUART_SPEED = LEUART_SPEED;
	bool const CONFIG_ALLOW_LEUART_LOW_BAUDRATE_ON_HFRCO = ALLOW_LEUART_LOW_BAUDRATE_ON_HFRCO;

#ifdef __cplusplus
}
#endif

#endif // __TD_CONFIG_UART_H
/** @endcond */
