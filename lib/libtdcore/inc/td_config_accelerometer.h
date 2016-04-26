/** @cond TD_CONFIG */
/***************************************************************************//**
 * @file
 * @brief Configuration file for the TDxxxx RF modules accelerometer.
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
#ifndef __TD_CONFIG__ACCELEROMETER_H
#define __TD_CONFIG__ACCELEROMETER_H

#include <stdint.h>
#include <stdbool.h>

#include <td_config_ext.h>

#include <td_gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef TD_ACCELERO_H_

#ifdef ACCELERO_CS_PORT

#ifndef ACCELERO_CS_BIT
#error("ACCELERO_CS_BIT is undefined")
#endif

#ifndef ACCELERO_IRQ_PORT
#error("ACCELERO_IRQ_PORT is undefined")
#endif

#ifndef ACCELERO_IRQ_BIT
#error("ACCELERO_IRQ_BIT is undefined")
#endif

#if !defined(ACCELERO_POWER_PORT)
#define ACCELERO_POWER_PORT TD_GPIO_PortNull
#endif

#if !defined(ACCELERO_POWER_BIT)
#define ACCELERO_POWER_BIT 0
#endif

#if !defined(ACCELERO_SKIP_INIT)
#define ACCELERO_SKIP_INIT 0
#endif

#if !defined(ACCELERO_POLLING_MODE)
#define ACCELERO_POLLING_MODE 0
#endif

#if ACCELERO_POLLING_MODE
INIT_DYNAMIC_ALT(TD_ACCELERO_MonitorData, 	TD_ACCELERO_MonitorDataPolling);
INIT_DYNAMIC_ALT(TD_ACCELERO_MonitorEvent, 	TD_ACCELERO_MonitorEventPolling);
#else
INIT_DYNAMIC(TD_ACCELERO_MonitorData);
INIT_DYNAMIC(TD_ACCELERO_MonitorEvent);
#endif

#ifndef ACCELERO_SPI_SPEED
#define ACCELERO_SPI_SPEED 10000000
#endif

#ifndef ACCELERO_SPI_MODE
#define ACCELERO_SPI_MODE usartClockMode3
#endif

#ifndef ACCELERO_SPI_BUS
#define ACCELERO_SPI_BUS 		0
#endif

    TD_GPIO_Port_TypeDef const CONFIG_ACCELERO_CS_PORT = (TD_GPIO_Port_TypeDef)
    ACCELERO_CS_PORT;
    uint8_t const CONFIG_ACCELERO_CS_BIT = ACCELERO_CS_BIT;
    TD_GPIO_Port_TypeDef const CONFIG_ACCELERO_POWER_PORT = (TD_GPIO_Port_TypeDef) ACCELERO_POWER_PORT;
    uint8_t const CONFIG_ACCELERO_POWER_BIT = ACCELERO_POWER_BIT;
    GPIO_Port_TypeDef const CONFIG_ACCELERO_IRQ_PORT = ACCELERO_IRQ_PORT;
    uint8_t const CONFIG_ACCELERO_IRQ_BIT = ACCELERO_IRQ_BIT;
    bool const CONFIG_ACCELERO_SKIP_INIT = ACCELERO_SKIP_INIT;
    bool const CONFIG_ACCELERO_POLLING_MODE = ACCELERO_POLLING_MODE;
    uint32_t const CONFIG_ACCELERO_SPI_SPEED = ACCELERO_SPI_SPEED;
    uint16_t const CONFIG_ACCELERO_SPI_MODE = ACCELERO_SPI_MODE;
    uint8_t const CONFIG_ACCELERO_SPI_BUS = ACCELERO_SPI_BUS;

#endif // ACCELERO_CS_PORT

#endif // TD_ACCELERO_H_

#ifdef __cplusplus
}
#endif

#endif // __TD_CONFIG__ACCELEROMETER_H
/** @endcond */

