/** @cond TD_CONFIG */
/***************************************************************************//**
 * @file
 * @brief Configuration file for the TDxxxx RF modules stack.
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
#ifndef __TD_CONFIG_STACK_H
#define __TD_CONFIG_STACK_H

#include <stdint.h>
#include <stdbool.h>

#include <td_config_ext.h>
#include <td_trap.h>
#include <td_rtc.h>
#include <td_scheduler.h>
#include <td_spi.h>
#include <td_gpio.h>
#include <td_utils.h>
#include <td_boot.h>

#ifdef __cplusplus
extern "C" {
#endif

#if TD_STACK_PROTECT == 1
TD_StackProtect_t const TD_StackProtect = TD_TRAP_StackProtect;
#else
TD_StackProtect_t const TD_StackProtect = NULL;
#endif

#if defined(__ICCARM__)

#ifdef STACK_SIZE
//#error("Stack size must be modified directly in .ICF file with IAR. If you want TD1202/1208 source code compatibility use, #if defined(__GNUC__) around #define STACK_SIZE")
#endif

#endif // defined(__ICCARM__)

#if defined(__GNUC__)

#ifndef STACK_SIZE
#define STACK_SIZE				0x800
#endif

/* Create a custom stack with name SYMBOL, aligned to ALIGNMENT bytes, sized
 * by SIZE bytes, but possibly shortened such that the initial stack pointer
 * (symbol __cs3_stack) that points to the block's last extent is aligned to
 * ALIGNMENT bytes, too.
 * HACK : we must add 'used' attributed to not have stack removed by gentle
 * GCC compiler in its size optimization process ...
 */
#define TD_CS3_STACK_SYMBOL(symbol, size, alignment) \
    static char __attribute__ ((section(".stack"),aligned (alignment),used)) \
    symbol[(size - ((size) % (alignment)))]; \
    asm (".global __cs3_stack"); \
    asm ("__cs3_stack = " #symbol " + (" #size ") - (" #alignment ")" \
        " - ((" #size ") % (" #alignment "))")
#define TD_STACK(size)\
    TD_CS3_STACK_SYMBOL(__cs3_stack_block, (size), 32);
TD_STACK(STACK_SIZE)

#endif // defined(__GNUC__)

/* Real stack size can be lowered for alignment issue. Here we take it in account, not optimized way
 * by removing alignment size on stack size
 */
#ifndef STACK_SIZE
#define STACK_SIZE			0x400
#endif

	uint16_t const CONFIG_STACK_SIZE = STACK_SIZE - 0x20;

#ifdef __cplusplus
}
#endif

#endif // __TD_CONFIG_STACK_H
/** @endcond */
