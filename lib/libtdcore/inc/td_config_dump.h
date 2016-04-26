/** @cond TD_CONFIG */
/***************************************************************************//**
 * @file
 * @brief Configuration file for the TDxxxx RF modules dump system.
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
#ifndef __TD_CONFIG_DUMP_H
#define __TD_CONFIG_DUMP_H

#include <stdint.h>
#include <stdbool.h>

#include <td_config_ext.h>

#include <td_gpio.h>
#include <td_scheduler.h>
#include <td_spi.h>
#include <td_boot.h>
#include <td_trap.h>

#if TD_SENSOR_USE_CODE
#include <td_sensor.h>
#endif

#if TD_GEOLOC_USE_CODE
#include <td_geoloc.h>
#include <td_accelero.h>
#endif


#ifdef __cplusplus
extern "C" {
#endif

#ifdef 	TD_ALL_DUMP_REMOVE_CODE
NULL_DYNAMIC(TD_SystemDump);
#define TD_GPIO_DUMP_REMOVE_CODE
#define TD_RF_DUMP_REMOVE_CODE
#define TD_IRQ_DUMP_REMOVE_CODE
#define TD_SCHEDULER_DUMP_REMOVE_CODE
#define TD_GEOLOC_DUMP_REMOVE_CODE
#define TD_SPILOCK_DUMP_REMOVE_CODE
#define TD_ACCELERO_DUMP_REMOVE_CODE
#define TD_SENSOR_DUMP_REMOVE_CODE
#else
INIT_DYNAMIC(TD_SystemDump);
#endif

void TD_RF_Dump(void);

TD_Dump_Func_t const TD_TRAP_SystemDumpFunc [] = {

#ifdef TD_GPIO_DUMP_REMOVE_CODE
    NULL,
#else
    TD_GPIO_Dump,
#endif

#if defined(TD_RF_DUMP_REMOVE_CODE)
    NULL,
#else
    TD_RF_Dump,
#endif

#ifdef TD_IRQ_DUMP_REMOVE_CODE
    NULL,
#else
    TD_IRQ_Dump,

#endif

#ifdef TD_SCHEDULER_DUMP_REMOVE_CODE
    NULL,
#else
    TD_SCHEDULER_Dump,
#endif

#if defined(TD_GEOLOC_DUMP_REMOVE_CODE) || TD_GEOLOC_USE_CODE == 0
    NULL,
#else
    TD_GEOLOC_Dump,
#endif

#ifdef TD_SPILOCK_DUMP_REMOVE_CODE
    NULL,
#else
    TD_SPI_LockDump,
#endif

#if defined(TD_ACCELERO_DUMP_REMOVE_CODE) || TD_GEOLOC_USE_CODE == 0
    NULL,
#else
    TD_ACCELERO_Dump,
#endif

#if defined(TD_ACCELERO_DUMP_REMOVE_CODE) || TD_SENSOR_USE_CODE == 0
    NULL,
#else
    TD_SENSOR_Dump
#endif

};

uint8_t const TD_TRAP_MaxSystemDump = sizeof (TD_TRAP_SystemDumpFunc) /
    sizeof (TD_Dump_Func_t);

#ifdef __cplusplus
}
#endif

#endif // __TD_CONFIG_DUMP_H
/** @endcond */
