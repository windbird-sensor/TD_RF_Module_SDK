/** @cond TD_CONFIG */
/***************************************************************************//**
 * @file
 * @brief Configuration file for the TDxxxx RF modules radio.
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
#ifndef __TD_CONFIG_RADIO_H
#define __TD_CONFIG_RADIO_H

#include <stdint.h>
#include <stdbool.h>

#include <td_config_ext.h>

#include <td_gpio.h>
#include <td_sigfox.h>

#ifdef __cplusplus
extern "C" {
#endif
#ifndef RADIO_CS_PORT
#define RADIO_CS_PORT			NSEL_RF_PORT
#endif

#ifndef RADIO_CS_BIT
#define RADIO_CS_BIT			NSEL_RF_BIT
#endif

#ifndef RADIO_IRQ_PORT
#define RADIO_IRQ_PORT			NIRQ_RF_PORT
#endif

#ifndef RADIO_IRQ_BIT
#define RADIO_IRQ_BIT			NIRQ_RF_BIT
#endif

/* RADIO_INFO_PIN : RF IO port link to EFM32 CPU
 * default value : empty
 */
#ifndef RADIO_INFO_PIN
#error("RADIO_INFO_PIN must be defined! (if PRODUCT_TYPE is externally defined all SYSTEM define must also occurs)")
#endif

/* RADIO_INFO_PORT : EFM32 CPU port link to RF I/O
 * default value : empty
 */
#ifndef RADIO_INFO_PORT
#error("RADIO_INFO_PORT must be defined! (if PRODUCT_TYPE is externally defined all SYSTEM define must also occurs)")
#endif

/* RADIO_INFO_BIT : EFM32 CPU bit link to RF I/O
 * default value : empty
 */
#ifndef RADIO_INFO_BIT
#error("RADIO_INFO_BIT must be defined! (if PRODUCT_TYPE is externally defined all SYSTEM define must also occurs)")
#endif

#if !defined(POWER_CRYSTAL_PORT) && IS_EMPTY(POWER_CRYSTAL_PORT)
#error("POWE_CRYSTAL_PORT must be set (if PRODUCT_TYPE is externally defined all SYSTEM define must also occurs)")
#endif

#if !defined(POWER_CRYSTAL_BIT) && IS_EMPTY(POWER_CRYSTAL_BIT)
#error("POWE_CRYSTAL_BIT must be set (if PRODUCT_TYPE is externally defined all SYSTEM define must also occurs)")
#endif

#if !defined(SHTD_PORT) && IS_EMPTY(SHTD_PORT)
#error("SHTD_PORT must be set (if PRODUCT_TYPE is externally defined all SYSTEM define must also occurs)")
#endif

#if !defined(SHTD_BIT) && IS_EMPTY(SHTD_BIT)
#error("SHTD_BIT must be set (if PRODUCT_TYPE is externally defined all SYSTEM define must also occurs)")
#endif

#ifndef RADIO_POWER_PORT
#define RADIO_POWER_PORT		TD_GPIO_PortNull
#define RADIO_POWER_BIT			0
#define RADIO_POWER_TIMER		NULL
#define RADIO_POWER_TIMER_CC	0
#endif

#ifndef RADIO_INT_PA_TX_PORT
#define RADIO_INT_PA_TX_PORT	TD_GPIO_PortNull
#define RADIO_INT_PA_TX_BIT		0
#endif

#ifndef RADIO_PA_TX_PORT
#define RADIO_PA_TX_PORT		TD_GPIO_PortNull
#define RADIO_PA_TX_BIT			0
#endif

#ifndef RADIO_PA_RX_LNA_PORT
#define RADIO_PA_RX_LNA_PORT	TD_GPIO_PortNull
#define RADIO_PA_RX_LNA_BIT		0
#endif

#ifndef RADIO_PA_ON_PORT
#define RADIO_PA_ON_PORT		TD_GPIO_PortNull
#define RADIO_PA_ON_BIT			0
#ifdef  CHIP_EZR_LG
INIT_DYNAMIC_ALT(TD_SIGFOX_InitPA,TD_SIGFOX_InitWithoutPA_EZR);
#else
INIT_DYNAMIC_ALT(TD_SIGFOX_InitPA,TD_SIGFOX_InitWithoutPA);
#endif
#else
INIT_DYNAMIC(TD_SIGFOX_InitPA);
#endif

#if defined(RADIO_BOOT_RF_SHUTDOWN) && IS_EMPTY(RADIO_BOOT_RF_SHUTDOWN)
#error("RADIO_BOOT_RF_SHUTDOWN must be set to 0 or 1 (or ommited)")
#endif

/* RADIO_BOOT_RF_SHUTDOWN : shutdown RF at boot
 * default value : 1
 */
#ifndef RADIO_BOOT_RF_SHUTDOWN
#define RADIO_BOOT_RF_SHUTDOWN	1
#endif

/* FORCE_RADIO_RESET : force RF reset at each reset
 * default value : 0
 */
#ifndef FORCE_RADIO_RESET
#define FORCE_RADIO_RESET		0
#endif

/* RADIO_USE_TCXO : RF chip uses a TCXO
 * default value : empty
 */
#ifndef RADIO_USE_TCXO
#error("RADIO_USE_TCXO must be defined! (if PRODUCT_TYPE is externally defined all SYSTEM define must also occurs)")
#endif

#ifndef USE_FREQUENCY_CAL
#define USE_FREQUENCY_CAL 0
#else

#if IS_EMPTY(USE_FREQUENCY_CAL)
#error("USE_FREQUENCY_CAL is empty, must be set to 0 or 1")
#endif

#endif // USE_FREQUENCY_CAL

/* RADIO_PA_POWER : default RF PA power
 * default value : 14
 */
#ifndef RADIO_PA_POWER
#define RADIO_PA_POWER			14
#endif

/* RADIO_PA_MAX : default Maximum RF PA raw value
 * default value : 127
 */
#ifndef RADIO_PA_MAX
#define RADIO_PA_MAX			127
#endif

/* RADIO_XO_TUNE : default XO_TUNE for crystal
 * default value : 47
 */
#ifndef RADIO_XO_TUNE
#define RADIO_XO_TUNE			47
#endif

/* Si4461 PA mode - switch current if 0 or not defined, square wave if 1 */
#ifndef TD_RF_SI4461_CLASS_E
#define TD_RF_SI4461_CLASS_E	0
#endif

#ifndef TD_RF_PA_CUSTOM
#define TD_RF_PA_CUSTOM			0
#endif

#ifndef ITU_ISM_REGION
#define ITU_ISM_REGION			1
#endif

/* RF Power calculation defines, define to use user callback */
#ifndef TD_RF_POWER_CALCULATION
extern uint8_t TD_RF_PowerdBmToRegStd(int level, uint32_t voltage);
#define TD_RF_POWER_CALCULATION	TD_RF_PowerdBmToRegStd
#endif

/* RF Power calculation callback */
RFPowerCalculation_t const CONFIG_TD_RF_POWER_CALCULATION =
    TD_RF_POWER_CALCULATION;

/* RF Power calculation coef, [0] is dB Value, [1] is max_voltage_mv / 100;
 * [2] is power for max_voltage;
 * [3] is power for max_voltage - 100mV; etc
 * indice = 127 means max for all next values
 * indice = -1 means end of table
 * indice = -2 means end
 *
 * ex :
 *
 * TD_RF_POWER_CALCULATION_COEF {14,36,100,101,102,103,104,106,108,112,120,
 * 127,-1,10,36,127,-1,-2}
 *
 * -> 14dB, start level 3.6V
 *
 * 3.6 -> 100
 * 3.5 -> 101
 * 3.4 -> 102
 * 3.3 -> 103
 * 3.2 -> 104
 * 3.1 -> 106
 * 3.0 -> 108
 * 2.9 -> 112
 * 2.8 -> 120
 * <=2.7 -> 127
 *
 * -> 10dB, start level 3.6V
 * <=3.6 -> 127
 *
 * intermediate voltage values will use linear approximation
 *
 *    */
#ifndef TD_RF_POWER_CALCULATION_COEF
#define TD_RF_POWER_CALCULATION_COEF	{-2}
#endif

/* RADIO_INIT_DATA : RF chip I/Os initialized in bootloader
 * default value : empty
 */
#ifndef RADIO_INIT_DATA
#error("RADIO_INIT_DATA must be defined !\r\n"\
    "Please remove your PRODUCT_TYPE definition and keep only MODULE_REVISION definition.\r\n"\
    "if PRODUCT_TYPE is externally defined, all SYSTEM define must also occurs")
#endif

	GPIO_Port_TypeDef const CONFIG_RADIO_CS_PORT = RADIO_CS_PORT;
	uint8_t const CONFIG_RADIO_CS_BIT = RADIO_CS_BIT;
	GPIO_Port_TypeDef const CONFIG_RADIO_IRQ_PORT = RADIO_IRQ_PORT;
	uint8_t const CONFIG_RADIO_IRQ_BIT = RADIO_IRQ_BIT;
    uint8_t const CONFIG_RADIO_INFO_PIN = RADIO_INFO_PIN;
    GPIO_Port_TypeDef const CONFIG_RADIO_INFO_PORT = (GPIO_Port_TypeDef)
    RADIO_INFO_PORT;\
    uint8_t const CONFIG_RADIO_INFO_BIT = RADIO_INFO_BIT;
	TD_GPIO_Port_TypeDef const CONFIG_POWER_CRYSTAL_PORT = (TD_GPIO_Port_TypeDef)
	POWER_CRYSTAL_PORT;
	uint8_t const CONFIG_POWER_CRYSTAL_BIT = POWER_CRYSTAL_BIT;
    TD_GPIO_Port_TypeDef const CONFIG_SHTD_PORT = (TD_GPIO_Port_TypeDef) SHTD_PORT;
    uint8_t const CONFIG_SHTD_BIT = SHTD_BIT;
    TD_GPIO_Port_TypeDef const CONFIG_RADIO_POWER_PORT = RADIO_POWER_PORT;
    uint8_t const CONFIG_RADIO_POWER_BIT = RADIO_POWER_BIT;
    TIMER_TypeDef * const CONFIG_RADIO_POWER_TIMER = RADIO_POWER_TIMER;
    uint8_t const CONFIG_RADIO_POWER_TIMER_CC = RADIO_POWER_TIMER_CC;
    TD_GPIO_Port_TypeDef const CONFIG_RADIO_INT_PA_TX_PORT = RADIO_INT_PA_TX_PORT;
    uint8_t const CONFIG_RADIO_INT_PA_TX_BIT = RADIO_INT_PA_TX_BIT;
    TD_GPIO_Port_TypeDef const CONFIG_RADIO_PA_TX_PORT = RADIO_PA_TX_PORT;
    uint8_t const CONFIG_RADIO_PA_TX_BIT = RADIO_PA_TX_BIT;
    TD_GPIO_Port_TypeDef const CONFIG_RADIO_PA_RX_LNA_PORT = RADIO_PA_RX_LNA_PORT;
    uint8_t const CONFIG_RADIO_PA_RX_LNA_BIT = RADIO_PA_RX_LNA_BIT;
    TD_GPIO_Port_TypeDef const CONFIG_RADIO_PA_ON_PORT = RADIO_PA_ON_PORT;
    uint8_t const CONFIG_RADIO_PA_ON_BIT = RADIO_PA_ON_BIT;
	uint8_t const CONFIG_RADIO_BOOT_RF_SHUTDOWN = RADIO_BOOT_RF_SHUTDOWN;
	uint8_t const CONFIG_FORCE_RADIO_RESET = FORCE_RADIO_RESET;
    uint8_t const CONFIG_RADIO_USE_TCXO = RADIO_USE_TCXO;
	bool const CONFIG_USE_FREQUENCY_CAL = USE_FREQUENCY_CAL;
	uint8_t const CONFIG_RADIO_PA_POWER = RADIO_PA_POWER;
	uint8_t const CONFIG_RADIO_PA_MAX = RADIO_PA_MAX;
	uint8_t const CONFIG_RADIO_XO_TUNE = RADIO_XO_TUNE;
    uint8_t const CONFIG_TD_RF_SI4461_CLASS_E = TD_RF_SI4461_CLASS_E;
    uint8_t const CONFIG_TD_RF_PA_CUSTOM = TD_RF_PA_CUSTOM;
	uint8_t const CONFIG_ITU_ISM_REGION = ITU_ISM_REGION;
    int8_t const CONFIG_TD_RF_POWER_CALCULATION_COEF[] = TD_RF_POWER_CALCULATION_COEF;
    uint8_t const *CONFIG_RADIO_PATCH_BLOB = RADIO_PATCH_BLOB;
    int const CONFIG_RADIO_PATCH_SIZE = RADIO_PATCH_SIZE;
    uint8_t const CONFIG_RADIO_INIT_DATA[] = {
    	6, 0x13, RADIO_INIT_DATA, 0
    };

#ifdef __cplusplus
}
#endif

#endif // __TD_CONFIG_RADIO_H
/** @endcond */
