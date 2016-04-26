/***************************************************************************//**
 * @file
 * @brief Configuration file for the TDxxxx RF modules Flash layout.
 * @author Telecom Design S.A.
 * @version 1.1.1
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
#ifndef __TD_CONFIG_FLASH_LAYOUT_H
#define __TD_CONFIG_FLASH_LAYOUT_H

#include <stdint.h>
#include <stdbool.h>

#include <td_config_ext.h>

#ifdef __cplusplus
extern "C" {
#endif
#if defined(__GNUC__)

/***************************************************************************//**
 * @addtogroup FLASH_LAYOUT FLASH_LAYOUT
 * @brief Dynamic declaration of USER flash sections
 * @{
 * @details
 *
 *   # History
 *	Theses sections were declared statically in map file (and used in code)
 *
 *   # Now
 *
 * Theses sections are stored descending from top of flash
 * +------------+ -- Top of flash
 * | Section1   |
 * -------------+
 * | Section2   |
 * -------------+
 * | Section3   |
 * -------------+
 * | Section4   |
 * -------------+
 * | -- Gap --  |
 * -------------+
 * | Code,const |
 * -------------+ -- 0
 *  FLASH_LAYOUTx
 *  FLASH_LAYOUTx_SIZE
 *
 *  Define a section name and size of section. Sections must be sequential (1,2,3,4).
 *  Unused sections can be omitted. If not sections are defined, automatic legacy
 *  sections are defined (compatible with previous SDK).
 *  Some name are reserved for system :
 *  EEPROM : storage of FlashVariable
 *  SEQUENCE : Sigfox sequence
 ******************************************************************************/
/** @cond TD_CONFIG */
#define FLASH_ARRAY					uint32_t const CONFIG_FLASH_LAYOUT[] = {
#define FLASH_ARRAY_END				0};
#define FLASH_STRING1(x)			#x
#define FLASH_STRING(x)				FLASH_STRING1(x)
#define FLASH_NAME_ARRAY			char * const CONFIG_FLASH_NAME_LAYOUT[] = {
#define FLASH_NAME_ARRAY_END		NULL};
#define DECLARE_FL_(x,x1)\
    uint8_t const __attribute__ ((section (".flash_layout"#x))) __attribute__ ((__used__)) flash_layout##x[FLASH_LAYOUT##x##_SIZE] = {0};\
    uint8_t const __attribute__ ((section (".flash_layout"#x"i"))) __attribute__ ((__used__)) flash_layout##x##i[FLASH_LAYOUT##x##_SIZE] = {0};\
    uint8_t const x1 = x;
#define DECLARE_FL_EVAL(x)			FLASH_LAYOUT##x
#define DECLARE_FL_EVAL2(x,y)		x##y
#define DECLARE_FL_EVAL4(x,y)		DECLARE_FL_EVAL2(x, y)
#define DECLARE_FL(x)				DECLARE_FL_(x, DECLARE_FL_EVAL4(CONFIG_FLASH_LAYOUT_, DECLARE_FL_EVAL(x)))

// Define default sections
#ifndef	FLASH_LAYOUT1
#define FLASH_LAYOUT1				SEQUENCE
#define FLASH_LAYOUT1_SIZE			FLASH_PAGE_SIZE
#define FLASH_LAYOUT2				EEPROM
#define FLASH_LAYOUT2_SIZE			(FLASH_PAGE_SIZE * TD_FLASH_USER_PAGE)
#endif

// Define padding section for flash size less than maximum
#ifdef LIMIT_FLASH_SIZE
#define PROTECTED_LOADER_FLASH_SIZE	1
#else
#define LIMIT_FLASH_SIZE                                        FLASH_SIZE
#endif

#if ((LIMIT_FLASH_SIZE) % FLASH_PAGE_SIZE) > 0
#error LIMIT_FLASH_SIZE must be an integral count of FLASH_PAGE_SIZE
#endif

#if LIMIT_FLASH_SIZE > FLASH_SIZE
#error You can reduce flash used by your code, but you cant generate flash from Void
#endif

#ifndef PROTECTED_LOADER_FLASH_SIZE
#define PROTECTED_LOADER_FLASH_SIZE 0
#endif

/** Size of the protected Flash memory area for the bootloader */
bool const CONFIG_PROTECTED_LOADER_FLASH_SIZE = PROTECTED_LOADER_FLASH_SIZE;

/** Flash size actually used by firmware, may be < physical Flash size */
uint32_t const CONFIG_LIMIT_FLASH_SIZE = LIMIT_FLASH_SIZE;

// Padding section (from 256K max flash size)
#define FLASH_LAYOUT0 				PADDING
#define FLASH_LAYOUT0_SIZE 			(256 * 1024 - LIMIT_FLASH_SIZE)

// Declare flash layout size array
FLASH_ARRAY

#ifdef FLASH_LAYOUT1_SIZE
FLASH_LAYOUT1_SIZE,
#endif

#ifdef FLASH_LAYOUT2_SIZE
FLASH_LAYOUT2_SIZE,
#endif

#ifdef FLASH_LAYOUT3_SIZE
FLASH_LAYOUT3_SIZE,
#endif

#ifdef FLASH_LAYOUT4_SIZE
FLASH_LAYOUT4_SIZE,
#endif

FLASH_ARRAY_END

// Declare flash layout name array
FLASH_NAME_ARRAY

#ifdef FLASH_LAYOUT1
FLASH_STRING(FLASH_LAYOUT1),
#endif

#ifdef FLASH_LAYOUT2
FLASH_STRING(FLASH_LAYOUT2),
#endif

#ifdef FLASH_LAYOUT3
FLASH_STRING(FLASH_LAYOUT3),
#endif

#ifdef FLASH_LAYOUT4
FLASH_STRING(FLASH_LAYOUT4),
#endif

FLASH_NAME_ARRAY_END

// Declare flash layout sections
#ifdef FLASH_LAYOUT0	/* used for different flash configuration size */

#if FLASH_LAYOUT0_SIZE > 0
DECLARE_FL(0)
#endif

#endif // FLASH_LAYOUT0

#ifdef FLASH_LAYOUT1
DECLARE_FL(1)
#endif

#ifdef FLASH_LAYOUT2
DECLARE_FL(2)
#endif

#ifdef FLASH_LAYOUT3
DECLARE_FL(3)
#endif

#ifdef FLASH_LAYOUT4
DECLARE_FL(4)
#endif

// Test sizes
#ifdef FLASH_LAYOUT0

#if ((FLASH_LAYOUT0_SIZE) % FLASH_PAGE_SIZE) > 0
#error FLASH_LAYOUT0_SIZE must be an integral count of FLASH_PAGE_SIZE
#endif

#endif // FLASH_LAYOUT0

#ifdef FLASH_LAYOUT1

#if ((FLASH_LAYOUT1_SIZE) % FLASH_PAGE_SIZE) > 0
#error FLASH_LAYOUT1_SIZE must be an integral count of FLASH_PAGE_SIZE
#endif

#endif // FLASH_LAYOUT1

#ifdef FLASH_LAYOUT2

#if ((FLASH_LAYOUT2_SIZE) % FLASH_PAGE_SIZE) > 0
#error FLASH_LAYOUT2_SIZE must be an integral count of FLASH_PAGE_SIZE
#endif

#endif // FLASH_LAYOUT2

#ifdef FLASH_LAYOUT3

#if ((FLASH_LAYOUT3_SIZE) % FLASH_PAGE_SIZE) > 0
#error FLASH_LAYOUT3_SIZE must be an integral count of FLASH_PAGE_SIZE
#endif

#endif // FLASH_LAYOUT3

#ifdef FLASH_LAYOUT4

#if ((FLASH_LAYOUT4_SIZE) % FLASH_PAGE_SIZE) != 0
#error FLASH_LAYOUT4_SIZE must be an integral count of FLASH_PAGE_SIZE
#endif

#endif // FLASH_LAYOUT4
/** @endcond */

#else // !defined(__GNUC__)

/** Flash size actually used by firmware, may be < physical Flash size */
uint32_t const CONFIG_LIMIT_FLASH_SIZE = 262144;

/** Protected Flash size used by bootloader */
bool const CONFIG_PROTECTED_LOADER_FLASH_SIZE = 0;

#endif // defined(__GNUC__)

#ifdef __cplusplus
}
#endif

#endif // __TD_CONFIG_FLASH_LAYOUT_H
/** @endcond */
