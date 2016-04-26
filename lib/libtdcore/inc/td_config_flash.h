/** @cond TD_CONFIG */
/***************************************************************************//**
 * @file
 * @brief Configuration file for the TDxxxx RF modules external Flash.
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
#ifndef __TD_CONFIG_FLASH_H
#define __TD_CONFIG_FLASH_H

#include <stdint.h>
#include <stdbool.h>

#include <td_config_ext.h>

#include <td_gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef FLASH_CS_PORT

#ifndef FLASH_CS_BIT
#error("FLASH_CS_BIT is undefined")
#endif

#ifndef FLASH_IRQ_PORT
#error("FLASH_IRQ_PORT is undefined")
#endif

#ifndef FLASH_IRQ_BIT
#error("FLASH_IRQ_BIT is undefined")
#endif

#if !defined(FLASH_POWER_PORT)
#define FLASH_POWER_PORT TD_GPIO_PortNull
#endif

#if !defined(FLASH_POWER_BIT)
#define FLASH_POWER_BIT 0
#endif

#ifndef FLASH_SPI_MODE
#define FLASH_SPI_MODE usartClockMode3
#endif

#ifndef FLASH_SPI_BUS
#define FLASH_SPI_BUS 		0
#endif

#else

#define FLASH_POWER_PORT TD_GPIO_PortNull
#define FLASH_POWER_BIT		0
#define FLASH_SPI_MODE		0
#define FLASH_SPI_BUS 		0
#define FLASH_CS_PORT		TD_GPIO_PortNull
#define FLASH_CS_BIT		0
#define FLASH_IRQ_PORT		TD_GPIO_PortNull
#define FLASH_IRQ_BIT		0

#endif // FLASH_CS_PORT

    TD_GPIO_Port_TypeDef const CONFIG_FLASH_CS_PORT = (TD_GPIO_Port_TypeDef)
    FLASH_CS_PORT;
    uint8_t const CONFIG_FLASH_CS_BIT = FLASH_CS_BIT;
    TD_GPIO_Port_TypeDef const CONFIG_FLASH_POWER_PORT = (TD_GPIO_Port_TypeDef) FLASH_POWER_PORT;
    uint8_t const CONFIG_FLASH_POWER_BIT = FLASH_POWER_BIT;
    GPIO_Port_TypeDef const CONFIG_FLASH_IRQ_PORT = FLASH_IRQ_PORT;
    uint8_t const CONFIG_FLASH_IRQ_BIT = FLASH_IRQ_BIT;
    uint16_t const CONFIG_FLASH_SPI_MODE = FLASH_SPI_MODE;
    uint8_t const CONFIG_FLASH_SPI_BUS = FLASH_SPI_BUS;

#ifdef __cplusplus
}
#endif

#endif // __TD_CONFIG_FLASH_H
/** @endcond */
