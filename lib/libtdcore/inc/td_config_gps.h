/** @cond TD_CONFIG */
/***************************************************************************//**
 * @file
 * @brief Configuration file for the TDxxxx RF modules GPS.
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
#ifndef __TD_CONFIG_GPS_H
#define __TD_CONFIG_GPS_H

#include <stdint.h>
#include <stdbool.h>

#include <td_config_ext.h>

#include <td_gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef GPS_CS_PORT

#ifndef GPS_CS_BIT
#error("GPS_CS_BIT is undefined")
#endif

#ifndef GPS_IRQ_PORT
#error("GPS_IRQ_PORT is undefined")
#endif

#ifndef GPS_IRQ_BIT
#error("GPS_IRQ_BIT is undefined")
#endif

#ifndef GPS_VIO_PORT
#error("GPS_VIO_PORT is undefined")
#endif

#ifndef GPS_RESET_PORT
#error("GPS_RESET_PORT is undefined")
#endif

#ifndef GPS_RESET_BIT
#error("GPS_RESET_BIT is undefined")
#endif

#ifndef GPS_VBCKP_PORT
#error("GPS_VBCKP_PORT is undefined")
#endif

#ifndef GPS_VBCKP_BIT
#error("GPS_VBCKP_BIT is undefined")
#endif

#ifndef GPS_SPI_BUS
#define GPS_SPI_BUS 0
#endif

#ifndef GPS_INTERFACE
#define GPS_INTERFACE USART1
#endif

    TD_GPIO_Port_TypeDef const CONFIG_GPS_CS_PORT = GPS_CS_PORT;
    uint8_t const CONFIG_GPS_CS_BIT = GPS_CS_BIT;
    GPIO_Port_TypeDef const CONFIG_GPS_IRQ_PORT = GPS_IRQ_PORT;
    uint8_t const CONFIG_GPS_IRQ_BIT = GPS_IRQ_BIT;
    TD_GPIO_Port_TypeDef const CONFIG_GPS_RESET_PORT = GPS_RESET_PORT;
    uint8_t const CONFIG_GPS_RESET_BIT = GPS_RESET_BIT;
    TD_GPIO_Port_TypeDef const CONFIG_GPS_VBCKP_PORT = GPS_VBCKP_PORT;
    uint8_t const CONFIG_GPS_VBCKP_BIT = GPS_VBCKP_BIT;
    TD_GPIO_Port_TypeDef const CONFIG_GPS_VIO_PORT = GPS_VIO_PORT;
    uint8_t const CONFIG_GPS_VIO_BIT = GPS_VIO_BIT;
    uint8_t const CONFIG_GPS_SPI_BUS = GPS_SPI_BUS;
    void * const CONFIG_GPS_INTERFACE = GPS_INTERFACE;

#endif // GPS_CS_PORT

#ifdef __cplusplus
}
#endif

#endif // __TD_CONFIG_GPS_H
/** @endcond */
