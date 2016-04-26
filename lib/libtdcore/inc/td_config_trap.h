/** @cond TD_CONFIG */
/***************************************************************************//**
 * @file
 * @brief Configuration file for the TDxxxx RF modules trap system.
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
#ifndef __TD_CONFIG_TRAP_H
#define __TD_CONFIG_TRAP_H

#include <stdint.h>
#include <stdbool.h>

#include <td_config_ext.h>

#include <td_trap.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(TD_TRAP_MINI_CODE)
    TD_TRAP_callback_t TD_Trap_Callback = (TD_TRAP_callback_t)
        TD_TRAP_Mini_Callback;
    TD_TRAP_callback_t const TD_Trap_Callback2 =
    (TD_TRAP_callback_t)TD_TRAP_Mini_Callback;
#elif defined(TD_TRAP_FLASH_CODE)
    TD_TRAP_callback_t TD_Trap_Callback = (TD_TRAP_callback_t)
        TD_TRAP_Flash_Callback;
    TD_TRAP_callback_t const TD_Trap_Callback2 =
        (TD_TRAP_callback_t)TD_TRAP_Mini_Callback;
#elif defined(TD_TRAP_FLASH_PRINTF_CODE)
    TD_TRAP_callback_t TD_Trap_Callback = (TD_TRAP_callback_t)
        TD_TRAP_Mini_Callback;
    TD_TRAP_callback_t const TD_Trap_Callback2 = (TD_TRAP_callback_t)
        TD_TRAP_Mini_Callback;
#elif defined(TD_TRAP_PRINTF_CODE)
    TD_TRAP_callback_t TD_Trap_Callback = TD_TRAP_Printf_Callback;
    TD_TRAP_callback_t const TD_Trap_Callback2 = TD_TRAP_Printf_Callback;
#elif defined(TD_TRAP_RESET_CODE)
    TD_TRAP_callback_t TD_Trap_Callback = TD_TRAP_Reset_Callback;
    TD_TRAP_callback_t const TD_Trap_Callback2 = TD_TRAP_Reset_Callback;
#else
    TD_TRAP_callback_t TD_Trap_Callback = TD_TRAP_Printf_Callback;
    TD_TRAP_callback_t const TD_Trap_Callback2 = TD_TRAP_Printf_Callback;
#endif

#ifdef __cplusplus
}
#endif

#endif // __TD_CONFIG_TRAP_H
/** @endcond */
