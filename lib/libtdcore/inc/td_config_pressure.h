/** @cond TD_CONFIG */
/***************************************************************************//**
 * @file
 * @brief Configuration file for the TDxxxx RF modules pressure sensor.
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
#ifndef __TD_CONFIG_PRESSURE_H
#define __TD_CONFIG_PRESSURE_H

#include <stdint.h>
#include <stdbool.h>

#include <td_config_ext.h>

#include <td_gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef PRESSURE_CS_PORT

#if !defined(PRESSURE_POWER_PORT)
#define PRESSURE_POWER_PORT TD_GPIO_PortNull
#endif

#if !defined(PRESSURE_POWER_BIT)
#define PRESSURE_POWER_BIT 0
#endif

#ifndef PRESSURE_SPI_MODE
#define PRESSURE_SPI_MODE usartClockMode3
#endif

#ifndef PRESSURE_SPI_BUS
#define PRESSURE_SPI_BUS 		0
#endif

#ifndef PRESSURE_SPI_SPEED
#define PRESSURE_SPI_SPEED 10000000
#endif

	TD_GPIO_Port_TypeDef const CONFIG_PRESSURE_CS_PORT = (TD_GPIO_Port_TypeDef)
	PRESSURE_CS_PORT;
	uint8_t const CONFIG_PRESSURE_CS_BIT = PRESSURE_CS_BIT;
    TD_GPIO_Port_TypeDef const CONFIG_PRESSURE_POWER_PORT = (TD_GPIO_Port_TypeDef) PRESSURE_POWER_PORT;
    uint8_t const CONFIG_PRESSURE_POWER_BIT = PRESSURE_POWER_BIT;
    uint16_t const CONFIG_PRESSURE_SPI_MODE = PRESSURE_SPI_MODE;
    uint8_t const CONFIG_PRESSURE_SPI_BUS = PRESSURE_SPI_BUS;
    uint32_t const CONFIG_PRESSURE_SPI_SPEED = PRESSURE_SPI_SPEED;

#endif // PRESSURE_CS_PORT

#ifdef __cplusplus
}
#endif

#endif // __TD_CONFIG_PRESSURE_H
/** @endcond */
