/** @cond TD_CONFIG */
/***************************************************************************//**
 * @file
 * @brief Configuration file for the TDxxxx RF modules bootloader.
 * @author Telecom Design S.A.
 * @version 1.3.0
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
#ifndef __TD_CONFIG_BOOTLOADER_H
#define __TD_CONFIG_BOOTLOADER_H

#include <stdint.h>
#include <stdbool.h>

#include <td_config_ext.h>

#include <td_gpio.h>
#include <td_boot.h>

#ifdef __cplusplus
extern "C" {
#endif

/* PRODUCT_TYPE is mandatory, see error message for standard value or ask Telecom Design for custom design
 * This type is used by the integrated RF loader to prevent bricking products with wrong firmware updates.
 */
#ifndef PRODUCT_TYPE
#error("You must specify a product type : PRODUCT_TYPE=x. TD1202_EVB:0 TD1204_EVB:8 TD1205_EVB:9 TD1208_EVB:10 TD1508_EVB:31 REF DESIGN FCC:34 REF DESIGN ETSI:35 TD1205P:36")
#endif

/* PRODUCT_INIT_DATA : CPU I/Os port initialized in bootloader
 * default value : 0x00F0. This value is a "do nothing" value
 */
#ifndef PRODUCT_INIT_DATA
#define PRODUCT_INIT_DATA {PINULL}
#endif

/* PRODUCT_LED_POLARITY : value to set on PIN TIM2(or
 * PRODUCT_LED_PORT/PRODUCT_LED_BIT) to turn OFF information led (used by the
 * integrated bootloader)
 * default value : led active high
 */
#ifndef PRODUCT_LED_POLARITY
#define PRODUCT_LED_POLARITY 	0
#endif

#if IS_EMPTY(PRODUCT_LED_POLARITY)
#error("PRODUCT_LED_POLARITY is empty, must be set to 0 or 1")
#else

#if ((PRODUCT_LED_POLARITY != 0) && (PRODUCT_LED_POLARITY != 1))
#error("PRODUCT_LED_POLARITY must be set to 0 or 1")
#endif

#endif // PRODUCT_LED_POLARITY

/* PRODUCT_LED_PORT : port used to turn on information led (used by the
 * integrated bootloader)
 * default value : TIM2_PORT (TDxxxx modules)
 */
#ifndef PRODUCT_LED_PORT
#define PRODUCT_LED_PORT TIM2_PORT
#endif

/* PRODUCT_LED_BIT : bit used to turn on information led (used by the
 * integrated bootloader)
 * default value : TIM2_BIT (TDxxxx modules)
 */
#ifndef PRODUCT_LED_BIT
#define PRODUCT_LED_BIT TIM2_BIT
#endif

/* PRODUCT_LED_BLINK : set to 1, blink led during each flash sector write
 * default value : false
 * Note : remember if you want to change this default value for TDxxxx module
 * to update ALL user bootloader documentations ...
 */
#ifndef PRODUCT_LED_BLINK
#define PRODUCT_LED_BLINK 		0
#endif

#if (IS_EMPTY(PRODUCT_LED_BLINK))
#error("PRODUCT_LED_BLINK is empty, must be set to 0 or 1")
#else

#if ((PRODUCT_LED_BLINK != 0) && (PRODUCT_LED_BLINK != 1))
#error("PRODUCT_LED_BLINK must be set to 0 or 1")
#endif

#endif // (IS_EMPTY(PRODUCT_LED_BLINK))

/* PRODUCT_BOOTLOADER_CHANNEL : channel to use for bootloader feature
 * default value : 255
 * Note :
 * 	channel 25 (868.4MHz) and 26 (869.80MHz) are allowed in France
 * 	channel 254 is on 856.798185
 * 	channel 255 is on 855.173185
 */
#ifndef PRODUCT_BOOTLOADER_CHANNEL
#define PRODUCT_BOOTLOADER_CHANNEL	255
#endif

/* PRODUCT_BOOTLOADER_SKIP : set to true, do not execute bootloader
 * default value : 0
 */
#ifndef PRODUCT_BOOTLOADER_SKIP
#define PRODUCT_BOOTLOADER_SKIP	0
#endif


#if IS_EMPTY(PRODUCT_BOOTLOADER_SKIP)
#error("PRODUCT_BOOTLOADER_SKIP is empty, must be set to 0 or 1")
#endif

/* PRODUCT_BOOTLOADER_FULL_SKIP : set to true, do not execute bootloader
 * default value : 0
 */
#ifndef PRODUCT_BOOTLOADER_FULL_SKIP
#define PRODUCT_BOOTLOADER_FULL_SKIP	0
#endif


#if IS_EMPTY(PRODUCT_BOOTLOADER_FULL_SKIP)
#error("PRODUCT_BOOTLOADER_FULL_SKIP is empty, must be set to 0 or 1")
#endif

/* PRODUCT_BOOTLOADER_SKIP_UART : set to true, do not execute UART
 * bootloader
 * default value : 0
 */
#ifndef PRODUCT_UART_BOOTLOADER_SKIP
#define PRODUCT_UART_BOOTLOADER_SKIP	0
#endif

#if IS_EMPTY(PRODUCT_UART_BOOTLOADER_SKIP)
#error("PRODUCT_UART_BOOTLOADER_SKIP is empty, must be set to 0 or 1")
#endif

#if PRODUCT_BOOTLOADER_FULL_SKIP
TD_BOOT_Handler_t const TD_BOOT_Handler = NULL;
#else

#if PRODUCT_BOOTLOADER_SKIP
TD_BOOT_Handler_t const TD_BOOT_Handler = TD_BOOT_Init_Skip;
#endif

#endif // PRODUCT_BOOTLOADER_FULL_SKIP

uint8_t const CONFIG_PRODUCT_UART_BOOTLOADER_SKIP = PRODUCT_UART_BOOTLOADER_SKIP;

#ifdef PRODUCT_EARLY_DEBUG
const TD_MAIN_Init_t CONFIG_EARLY_DEBUG = main_init_debug;
bool const CONFIG_PRODUCT_EARLY_DEBUG = true;
#else
const TD_MAIN_Init_t CONFIG_EARLY_DEBUG = main_init;
bool const CONFIG_PRODUCT_EARLY_DEBUG = false;
#endif

#ifdef PRODUCT_EARLY_DEBUG_CUSTOM
const TD_MAIN_Init_t CONFIG_EARLY_DEBUG_CUSTOM = PRODUCT_EARLY_DEBUG_CUSTOM;
#else
const TD_MAIN_Init_t CONFIG_EARLY_DEBUG_CUSTOM = (TD_MAIN_Init_t)TD_NopHere;
#endif

#define	STRINGI(a,b)	a##b
#define	PRODUCT_BOOTLOADER_CHANNEL_VALUE_INT(x)	STRINGI(BOOT_FREQ_CHAN,x)
#define	PRODUCT_BOOTLOADER_CHANNEL_VALUE		PRODUCT_BOOTLOADER_CHANNEL_VALUE_INT(PRODUCT_BOOTLOADER_CHANNEL)

#if PRODUCT_BOOTLOADER_CHANNEL_VALUE + 0 == 0
#error("PRODUCT_BOOTLOADER_CHANNEL is set to an invalid value")
#endif

#ifndef PRODUCT_LED_DRIVE
#define PRODUCT_LED_DRIVE	gpioDriveModeStandard
#endif

#ifdef TD_REMOVE_UART_LOADER
NOP_DYNAMIC(TD_BOOT_UartLoader);
#else
INIT_DYNAMIC(TD_BOOT_UartLoader);
#endif

#define _PRODUCT_TYPE PRODUCT_TYPE

	int8_t const CONFIG_PRODUCT_TYPE = PRODUCT_TYPE;
    uint16_t const CONFIG_PRODUCT_INIT_DATA[] = PRODUCT_INIT_DATA;
	int8_t const CONFIG_PRODUCT_LED_POLARITY = PRODUCT_LED_POLARITY;
	GPIO_Port_TypeDef const CONFIG_PRODUCT_LED_PORT = (GPIO_Port_TypeDef) PRODUCT_LED_PORT;
	int8_t const CONFIG_PRODUCT_LED_BIT = PRODUCT_LED_BIT;
	int8_t const CONFIG_PRODUCT_LED_BLINK = PRODUCT_LED_BLINK;
    GPIO_DriveMode_TypeDef const CONFIG_PRODUCT_LED_DRIVE = PRODUCT_LED_DRIVE;
    uint8_t const CONFIG_PRODUCT_BOOTLOADER_CHANNEL = PRODUCT_BOOTLOADER_CHANNEL;
    uint8_t const ProductBootloaderP1[] = {
    	8, 0x11, 0x40, 4, 0x00,
    	PRODUCT_BOOTLOADER_CHANNEL_VALUE & 0xFF,
    	(PRODUCT_BOOTLOADER_CHANNEL_VALUE >> 24) & 0xFF,
    	(PRODUCT_BOOTLOADER_CHANNEL_VALUE >> 16) & 0xFF,
    	(PRODUCT_BOOTLOADER_CHANNEL_VALUE >> 8) & 0xFF,
    	0
    };
    uint8_t const CONFIG_PRODUCT_INIT_DATA_SIZE =
    	sizeof (CONFIG_PRODUCT_INIT_DATA) / sizeof (uint16_t);

#ifdef __cplusplus
}
#endif

#endif // __TD_CONFIG_BOOTLOADER_H
/** @endcond */
