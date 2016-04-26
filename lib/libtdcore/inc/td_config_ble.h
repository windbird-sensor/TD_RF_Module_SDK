/** @cond TD_CONFIG */
/***************************************************************************//**
 * @file
 * @brief Configuration file for the TDxxxx RF modules for BLE.
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
#ifndef __TD_CONFIG_BLE_H
#define __TD_CONFIG_BLE_H

#include <stdint.h>
#include <stdbool.h>

#include <td_config_ext.h>

#include <td_gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef BLE_UART

#if !defined(BLE_UART_CONF)
#define BLE_UART_CONF 0
#endif

#if !defined(BLE_POWER_PORT)
#define BLE_POWER_PORT TD_GPIO_PortNull
#endif

#if !defined(BLE_POWER_BIT)
#define BLE_POWER_BIT 0
#endif

#if !defined(BLE_RESET_PORT)
#define BLE_RESET_PORT TD_GPIO_PortNull
#endif

#if !defined(BLE_RESET_BIT)
#define BLE_RESET_BIT 0
#endif

//speed for preload synchronizing
#if !defined(BLE_UART_PRELOAD_SPEED)
#define BLE_UART_PRELOAD_SPEED BLE_UART_SPEED
#endif

//speed for loading the firmware -> application speed is always 9600 bauds for low conso!
#if !defined(BLE_UART_SPEED)
#error BLE_UART_SPEED (in bauds) must be defined!
#endif

	void const *CONFIG_BLE_UART = BLE_UART;
	uint8_t const CONFIG_BLE_UART_CONF = BLE_UART_CONF;
	TD_GPIO_Port_TypeDef const CONFIG_BLE_POWER_PORT = (TD_GPIO_Port_TypeDef) BLE_POWER_PORT;
	uint8_t const CONFIG_BLE_POWER_BIT = BLE_POWER_BIT;
	TD_GPIO_Port_TypeDef const CONFIG_BLE_RESET_PORT = (TD_GPIO_Port_TypeDef) BLE_RESET_PORT;
	uint8_t const CONFIG_BLE_RESET_BIT = BLE_RESET_BIT;
	uint32_t const CONFIG_BLE_UART_PRELOAD_SPEED = BLE_UART_PRELOAD_SPEED;
	uint32_t const CONFIG_BLE_UART_SPEED = BLE_UART_SPEED;

#endif // BLE_UART

#ifdef __cplusplus
}
#endif

#endif // __TD_CONFIG_BLE_H
/** @endcond */
