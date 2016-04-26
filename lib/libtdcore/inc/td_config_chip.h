/** @cond TD_CONFIG */
/***************************************************************************//**
 * @file
 * @brief Configuration file for the TDxxxx RF standard chips.
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
#ifndef __TD_CONFIG_CHIP_H
#define __TD_CONFIG_CHIP_H

#include <stdint.h>
#include <stdbool.h>

#include <td_config_ext.h>

#include <td_gpio.h>

#ifdef __cplusplus
extern "C" {
#endif
/* Check MANDATORY variables and set others to default values */

/**************************************************************************//**
 * Standard TD1202 chip configuration
 ******************************************************************************/
#ifdef CHIP_TD1202

#define TARGET_COMPILER_IAR							/**< IAR compiler for size optimization */

#define RADIO_USE_TCXO 			1					/**< RF uses a TCXO */

#define POWER_CRYSTAL_PORT      gpioPortF           /**< Power RF Crystal port */
#define POWER_CRYSTAL_BIT       2					/**< Power RF Crystal bit */
#define	SHTD_PORT               gpioPortC           /**< RF chip shutdown port */
#define	SHTD_BIT                13					/**< RF chip shutdown bit */
#define RADIO_INIT_DATA			GPIO_TRISTATE, GPIO_NOP, GPIO_TRISTATE, \
    GPIO_RX_RAW_DATA_OUT, GPIO_NOP					/**< RF chip GIOs */
#define RADIO_INFO_PIN			2					/**< RF chip GPIO2 is Info pin */
#define RADIO_INFO_PORT 		gpioPortB			/**< RF Info port on CPU side */
#define RADIO_INFO_BIT 			14					/**< RF Info bit on CPU side */

#define TD_GEOLOC_LOGGER_REMOVE_CODE

#ifndef	TD_GEOLOC_USE_CODE
#define TD_GEOLOC_USE_CODE		0
#endif

#endif // CHIP_TD1202

/**************************************************************************//**
 * Standard TD1204 module configuration
 ******************************************************************************/
#ifdef CHIP_TD1204

#define TARGET_COMPILER_GCC

#ifndef TD_STACK_PROTECT
#define TD_STACK_PROTECT 		1
#endif

#define RADIO_USE_TCXO 			1

#define POWER_CRYSTAL_PORT      gpioPortF           /**< Power RF Crystal port */
#define POWER_CRYSTAL_BIT       2					/**< Power RF Crystal bit */
#define	SHTD_PORT               gpioPortC           /**< RF chip shutdown port */
#define	SHTD_BIT                13					/**< RF chip shutdown bit */
#define RADIO_INFO_PIN			0
#define RADIO_INFO_PORT 		gpioPortB
#define RADIO_INFO_BIT 			14

#ifndef PRODUCT_INIT_DATA

#define PRODUCT_INIT_DATA\
    {PIP(gpioPortB,13,PI_DISABLED,1),	/* CS pull-up to 1 for GPS and accelerometer */ \
    PIP(gpioPortC,15,PI_OUTPUT,1)}
#define RADIO_INIT_DATA			GPIO_TRISTATE, GPIO_RX_RAW_DATA_OUT, \
    GPIO_TRISTATE, GPIO_DRIVE0, GPIO_NOP

#endif // PRODUCT_INIT_DATA

#define GPS_CS_PORT 			gpioPortB
#define GPS_CS_BIT 				13
#define GPS_IRQ_PORT 			gpioPortC
#define GPS_IRQ_BIT 			1
#define GPS_RESET_PORT 			TD_GPIO_PortRadio
#define GPS_RESET_BIT 			2
#define GPS_VBCKP_PORT 			TD_GPIO_PortRadio 	/**< port 0x10 is Silabs */
#define GPS_VBCKP_BIT 			3
#define GPS_VIO_PORT 			USR2_PORT
#define GPS_VIO_BIT 			USR2_BIT
#define ACCELERO_CS_PORT		USR1_PORT
#define ACCELERO_CS_BIT			USR1_BIT
#define ACCELERO_IRQ_PORT		USR4_PORT
#define ACCELERO_IRQ_BIT		USR4_BIT

#ifndef	TD_GEOLOC_USE_CODE
#define TD_GEOLOC_USE_CODE		1
#endif

#endif // CHIP_TD1204

/**************************************************************************//**
 * Standard TD1205 module configuration
 ******************************************************************************/
#ifdef CHIP_TD1205

#define TARGET_COMPILER_GCC

#ifndef TD_STACK_PROTECT
#define TD_STACK_PROTECT		1
#endif

#define RADIO_USE_TCXO 			1

#define POWER_CRYSTAL_PORT      gpioPortA           /**< Power RF Crystal port */
#define POWER_CRYSTAL_BIT       0					/**< Power RF Crystal bit */
#define	SHTD_PORT               gpioPortC           /**< RF chip shutdown port */
#define	SHTD_BIT                13					/**< RF chip shutdown bit */
#define RADIO_INFO_PIN			1
#define RADIO_INFO_PORT 		gpioPortB
#define RADIO_INFO_BIT 			14

#define RADIO_PA_POWER			20

#define RADIO_INIT_DATA			GPIO_IN_RX_STATE, GPIO_TRISTATE, \
    GPIO_IN_TX_STATE, GPIO_TRISTATE, GPIO_NOP

#ifndef PRODUCT_INIT_DATA
#define PRODUCT_INIT_DATA\
    {PIP(gpioPortB,13,PI_DISABLED, 1),	/* CS pull-up to 1 for GPS and accelerometer */ \
    PIP(gpioPortC,15,PI_DISABLED, 1)}
#endif // PRODUCT_INIT_DATA

#define GPS_CS_PORT 			USR0_PORT
#define GPS_CS_BIT 				USR0_BIT
#define GPS_IRQ_PORT 			USR3_PORT
#define GPS_IRQ_BIT 			USR3_BIT
#define GPS_RESET_PORT 			gpioPortB
#define GPS_RESET_BIT 			11
#define GPS_VBCKP_PORT 			gpioPortF
#define GPS_VBCKP_BIT 			2
#define GPS_VIO_PORT 			USR2_PORT
#define GPS_VIO_BIT 			USR2_BIT

#define ACCELERO_CS_PORT		USR1_PORT
#define ACCELERO_CS_BIT			USR1_BIT
#define ACCELERO_IRQ_PORT		USR4_PORT
#define ACCELERO_IRQ_BIT		USR4_BIT

#ifndef	TD_GEOLOC_USE_CODE
#define TD_GEOLOC_USE_CODE		1
#endif

#endif // CHIP_TD1205

/**************************************************************************//**
 * Standard TD1205P module configuration
 ******************************************************************************/
#ifdef CHIP_TD1205P

#define TARGET_COMPILER_GCC

#define RADIO_USE_TCXO 			1

#define CHIP_EZR_LG				                    /**< Internal EFM32 / Si446x bonding */

#define PRODUCT_LED_PORT		gpioPortD
#define PRODUCT_LED_BIT			5
#define PRODUCT_LED_DRIVE		gpioDriveModeLowest

#define RADIO_USE_TCXO          1

//power crystal is driven by silabs
#define POWER_CRYSTAL_PORT      TD_GPIO_PortF       /**< Power RF Crystal port */
#define POWER_CRYSTAL_BIT       8					/**< Power RF Crystal bit */
#define RADIO_INIT_DATA			GPIO_IN_TX_STATE, GPIO_TRISTATE, GPIO_TRISTATE, GPIO_TRISTATE, GPIO_TRISTATE

#define ACCELERO_POWER_PORT		TD_GPIO_PortNull
#define ACCELERO_POWER_BIT		0
#define ACCELERO_CS_PORT		gpioPortD
#define ACCELERO_CS_BIT			3
#define ACCELERO_IRQ_PORT		gpioPortC
#define ACCELERO_IRQ_BIT		14
#define ACCELERO_SPI_BUS		SPI_BUS(SPI_BUS1,SPI_LOC1)
#define ACCELERO_SPI_MODE		usartClockMode0/*|USART_CTRL_LOOPBK*/
#define ACCELERO_SPI_SPEED		10000000

#define GPS_CS_PORT 			gpioPortE
#define GPS_CS_BIT 				3
#define GPS_IRQ_PORT 			gpioPortF
#define GPS_IRQ_BIT 			12
#define GPS_RESET_PORT 			gpioPortB
#define GPS_RESET_BIT 			11
#define GPS_VBCKP_PORT 			gpioPortF
#define GPS_VBCKP_BIT 			3
#define GPS_VIO_PORT 			gpioPortF
#define GPS_VIO_BIT 			2
#define GPS_SPI_BUS 			SPI_BUS(SPI_BUS1,SPI_LOC1)

#define RADIO_INT_SW_TX_PORT	gpioPortA			/**< Internal RF switch TX port */
#define RADIO_INT_SW_TX_BIT		15					/**< Internal RF PA TX bit */
#define RADIO_INT_SW_TX_PIN		0					/**< Internal RF PA TX RF chip GPIO1 */
#define RADIO_SW_TX_PORT		gpioPortA			/**< RF PA TX port */
#define RADIO_SW_TX_BIT			1					/**< RF PA TX bit */

#ifndef	TD_GEOLOC_USE_CODE
#define TD_GEOLOC_USE_CODE		1
#endif

#ifndef PRODUCT_INIT_DATA
#define PRODUCT_INIT_DATA\
		{PIP(LED_GREEN_PORT,LED_GREEN_BIT,PI_OUTPUT,1),			/* Common led value set to 1 */ \
		PIP(LED_RED_PORT,LED_RED_BIT,PI_OUTPUT,1),			/* Common led value set to 1 */ \
		PIP(ACCELERO_CS_PORT,ACCELERO_CS_BIT,PI_OUTPUT,1),	/* Accelero CS set to 1 */ \
		PIP(RADIO_INT_SW_TX_PORT,RADIO_INT_SW_TX_BIT,PI_INPUT,0),	/* Internal bonding to RF GPIO0 */\
		PIP(RADIO_SW_TX_PORT,RADIO_SW_TX_BIT,PI_OUTPUT,0),		/* External RF GPIO0 via PRS */\
		PIA(1,PRS_CH_CTRL_SOURCESEL_GPIOH,PRS_CH_CTRL_SIGSEL_GPIOPIN15)} /* PRS routing of PA15 to PRS channel 1 output (PA1) */
#endif // PRODUCT_INIT_DATA

#define LED_GREEN_PORT			gpioPortD
#define LED_GREEN_BIT			5

#define LED_RED_PORT			gpioPortD
#define LED_RED_BIT				8

#define TD_UART_COUNT		2
#define TD_STREAM_COUNT		2
#define LEUART_DEVICE		LEUART1
#define LEUART_LOCATION		0

#endif // CHIP_TD1205P

/**************************************************************************//**
 * Standard TD1208 module configuration
 ******************************************************************************/
#ifdef CHIP_TD1208

#define TARGET_COMPILER_GCC

#ifndef TD_STACK_PROTECT
#define TD_STACK_PROTECT		1
#endif

#define RADIO_USE_TCXO 			1

#define POWER_CRYSTAL_PORT      gpioPortF           /**< Power RF Crystal port */
#define POWER_CRYSTAL_BIT       2					/**< Power RF Crystal bit */
#define	SHTD_PORT               gpioPortC           /**< RF chip shutdown port */
#define	SHTD_BIT                13					/**< RF chip shutdown bit */
#define RADIO_INFO_PIN			2
#define RADIO_INFO_PORT 		gpioPortB
#define RADIO_INFO_BIT 			14

#ifndef RADIO_INIT_DATA
#define RADIO_INIT_DATA			GPIO_TRISTATE, GPIO_NOP, GPIO_TX_DATA_CLK_OUT, \
    GPIO_NOP, GPIO_NOP
#endif

#define TD_GEOLOC_LOGGER_REMOVE_CODE

#ifndef	TD_GEOLOC_USE_CODE
#define TD_GEOLOC_USE_CODE		0
#endif

#endif // CHIP_TD1208

/**************************************************************************//**
 * Standard TD1508 module configuration
 ******************************************************************************/
#ifdef CHIP_TD1508

#define TARGET_COMPILER_GCC

#define RADIO_USE_TCXO 			1

#define CHIP_EZR_LG									/**< Internal EFM32 / Si446x bonding */

#define ITU_ISM_REGION			2					/**< ITU ISM region 2 (Americas) */

#define POWER_CRYSTAL_PORT      gpioPortF			/**< Power RF Crystal port */
#define POWER_CRYSTAL_BIT       8					/**< Power RF Crystal bit */
#define RADIO_INT_PA_TX_PORT	gpioPortA			/**< Internal RF PA TX port */
#define RADIO_INT_PA_TX_BIT		15					/**< Internal RF PA TX bit */
#define RADIO_INT_PA_TX_PIN		0					/**< Internal RF PA TX RF chip GPIO */
#define RADIO_PA_TX_PORT		gpioPortA			/**< RF PA TX port */
#define RADIO_PA_TX_BIT			1					/**< RF PA TX bit */
#define RADIO_PA_RX_LNA_PORT	gpioPortB			/**< RF PA RX LNA port */
#define RADIO_PA_RX_LNA_BIT		4					/**< RF PA RX LNA bit */
#define RADIO_PA_ON_PORT		gpioPortB			/**< RF PA ON port */
#define RADIO_PA_ON_BIT			6					/**< RF PA ON bit */
#define RADIO_INIT_DATA		GPIO_DRIVE0, GPIO_TX_DATA_CLK_OUT, GPIO_TRISTATE, GPIO_TRISTATE, GPIO_TRISTATE
#define RADIO_PA_MAX			127

#define PRODUCT_INIT_DATA		{ \
    PIR(PRODUCT_DATA_INIT_ROUTE_DISABLE),								/* Do not power-up BLE at startup */\
    PIP(RADIO_INT_PA_TX_PORT, RADIO_INT_PA_TX_BIT, PI_INPUT, 0),		/* Internal bonding to RF GPIO0 */\
    PIP(RADIO_PA_TX_PORT, RADIO_PA_TX_BIT, PI_OUTPUT, 0),				/* External RF GPIO0 via PRS */\
    PIP(RADIO_PA_ON_PORT, RADIO_PA_ON_BIT, PI_OUTPUT, 0),				/* PA_ON set to 0 */\
    PIP(RADIO_PA_RX_LNA_PORT, RADIO_PA_RX_LNA_BIT, PI_OUTPUT, 0),		/* PA_RX_LNA set to 0 */\
    PIA(1, PRS_CH_CTRL_SOURCESEL_GPIOH, PRS_CH_CTRL_SIGSEL_GPIOPIN15)	/* PRS routing of PA15 to PRS channel 0 output (PA0) */\
}

//#define PRODUCT_LED_PORT		gpioPortD
//#define PRODUCT_LED_BIT			7
//#define PRODUCT_LED_BLINK		1
//#define PRODUCT_LED_DRIVE		gpioDriveModeLowest

#define TD_UART_COUNT			2
#define TD_STREAM_COUNT			2
#define LEUART_DEVICE			LEUART1
#define LEUART_LOCATION			0

#endif // CHIP_TD1508

/**************************************************************************//**
 * Standard REF_DESIGN_FCC module configuration
 ******************************************************************************/
#ifdef CHIP_REF_DESIGN_FCC

#define TARGET_COMPILER_GCC

#define RADIO_USE_TCXO 			1

#define CHIP_EZR_LG									/**< Internal EFM32 / Si446x bonding */

#define ITU_ISM_REGION			2					/**< ITU ISM region 2 (Americas) */

#define POWER_CRYSTAL_PORT      gpioPortF			/**< Power RF Crystal port */
#define POWER_CRYSTAL_BIT       8					/**< Power RF Crystal bit */
#define RADIO_INT_PA_TX_PORT	gpioPortA			/**< Internal RF PA TX port */
#define RADIO_INT_PA_TX_BIT		15					/**< Internal RF PA TX bit */
#define RADIO_INT_PA_TX_PIN		0					/**< Internal RF PA TX RF chip GPIO */
#define RADIO_PA_TX_PORT		gpioPortA			/**< RF PA TX port */
#define RADIO_PA_TX_BIT			1					/**< RF PA TX bit */
#define RADIO_PA_RX_LNA_PORT	gpioPortB			/**< RF PA RX LNA port */
#define RADIO_PA_RX_LNA_BIT		4					/**< RF PA RX LNA bit */
#define RADIO_PA_ON_PORT		gpioPortB			/**< RF PA ON port */
#define RADIO_PA_ON_BIT			6					/**< RF PA ON bit */
#define RADIO_INIT_DATA		GPIO_DRIVE0, GPIO_TX_DATA_CLK_OUT, GPIO_TRISTATE, GPIO_TRISTATE, GPIO_TRISTATE
#define RADIO_PA_MAX			127

#define PRODUCT_INIT_DATA		{ \
    PIR(PRODUCT_DATA_INIT_ROUTE_DISABLE),								/* Do not power-up BLE at startup */\
    PIP(RADIO_INT_PA_TX_PORT, RADIO_INT_PA_TX_BIT, PI_INPUT, 0),		/* Internal bonding to RF GPIO0 */\
    PIP(RADIO_PA_TX_PORT, RADIO_PA_TX_BIT, PI_OUTPUT, 0),				/* External RF GPIO0 via PRS */\
    PIP(RADIO_PA_ON_PORT, RADIO_PA_ON_BIT, PI_OUTPUT, 0),				/* PA_ON set to 0 */\
    PIP(RADIO_PA_RX_LNA_PORT, RADIO_PA_RX_LNA_BIT, PI_OUTPUT, 0),		/* PA_RX_LNA set to 0 */\
    PIA(1, PRS_CH_CTRL_SOURCESEL_GPIOH, PRS_CH_CTRL_SIGSEL_GPIOPIN15)	/* PRS routing of PA15 to PRS channel 0 output (PA0) */\
}

//#define PRODUCT_LED_PORT		gpioPortD
//#define PRODUCT_LED_BIT			7
//#define PRODUCT_LED_BLINK		1
//#define PRODUCT_LED_DRIVE		gpioDriveModeLowest

#define TD_UART_COUNT			2
#define TD_STREAM_COUNT			2
#define LEUART_DEVICE			LEUART1
#define LEUART_LOCATION			0

#endif // CHIP_REF_DESIGN_FCC

/**************************************************************************//**
 * Standard REF_DESIGN_ETSI module configuration
 ******************************************************************************/
#ifdef CHIP_REF_DESIGN_ETSI

#define TARGET_COMPILER_GCC

#define RADIO_USE_TCXO 			1

#define CHIP_EZR_LG									/**< Internal EFM32 / Si446x bonding */

#define ITU_ISM_REGION			1					/**< ITU ISM region 1 (Europe) */

#define POWER_CRYSTAL_PORT      gpioPortF			/**< Power RF Crystal port */
#define POWER_CRYSTAL_BIT       8					/**< Power RF Crystal bit */
//#define RADIO_INT_PA_TX_PORT	gpioPortA			/**< Internal RF PA TX port */
//#define RADIO_INT_PA_TX_BIT		15					/**< Internal RF PA TX bit */
//#define RADIO_INT_PA_TX_PIN		0					/**< Internal RF PA TX RF chip GPIO */
//#define RADIO_PA_TX_PORT		gpioPortA			/**< RF PA TX port */
//#define RADIO_PA_TX_BIT			1					/**< RF PA TX bit */
//#define RADIO_PA_RX_LNA_PORT	gpioPortB			/**< RF PA RX LNA port */
//#define RADIO_PA_RX_LNA_BIT		4					/**< RF PA RX LNA bit */
//#define RADIO_PA_ON_PORT		gpioPortB			/**< RF PA ON port */
//#define RADIO_PA_ON_BIT			6					/**< RF PA ON bit */
#define RADIO_INIT_DATA		GPIO_DRIVE0, GPIO_TX_DATA_CLK_OUT, GPIO_TRISTATE, GPIO_TRISTATE, GPIO_TRISTATE
#define RADIO_PA_MAX			127

#define PRODUCT_INIT_DATA		{ \
    PIR(PRODUCT_DATA_INIT_ROUTE_DISABLE),								/* Do not power-up BLE at startup */\
    /*PIP(RADIO_INT_PA_TX_PORT, RADIO_INT_PA_TX_BIT, PI_INPUT, 0),*/		/* Internal bonding to RF GPIO0 */\
    /*PIP(RADIO_PA_TX_PORT, RADIO_PA_TX_BIT, PI_OUTPUT, 0),		*/		/* External RF GPIO0 via PRS */\
    /*PIP(RADIO_PA_ON_PORT, RADIO_PA_ON_BIT, PI_OUTPUT, 0),		*/		/* PA_ON set to 0 */\
    /*PIP(RADIO_PA_RX_LNA_PORT, RADIO_PA_RX_LNA_BIT, PI_OUTPUT, 0),*/		/* PA_RX_LNA set to 0 */\
    PIA(1, PRS_CH_CTRL_SOURCESEL_GPIOH, PRS_CH_CTRL_SIGSEL_GPIOPIN15)	/* PRS routing of PA15 to PRS channel 0 output (PA0) */\
}

#define PRODUCT_LED_PORT		gpioPortD
#define PRODUCT_LED_BIT			5
#define PRODUCT_LED_DRIVE		gpioDriveModeLowest


#define TD_UART_COUNT			2
#define TD_STREAM_COUNT			2
#define LEUART_DEVICE			LEUART1
#define LEUART_LOCATION			0

#endif // CHIP_REF_DESIGN_ETSI

/**************************************************************************//**
 * Common EZR Internal bonding configuration
 ******************************************************************************/
#ifdef CHIP_EZR_LG

#define	SHTD_PORT               gpioPortE           /**< RF shutdown port (internal to EFM32 no outside pin) */
#define	SHTD_BIT                8					/**< RF shutdown port (internal to EFM32 no outside pin) */
#define RADIO_CS_PORT			gpioPortE           /**< RF chip select port (internal to EFM32 no outside pin) */
#define RADIO_CS_BIT			9					/**< RF chip select port (internal to EFM32 no outside pin) */
#define RADIO_IRQ_PORT			gpioPortE           /**< RF IRQ port (internal to EFM32 no outside pin) */
#define RADIO_IRQ_BIT			13					/**< RF IRQ port (internal to EFM32 no outside pin) */
#define RADIO_INFO_PORT 		gpioPortE           /**< RF shutdown port (internal to EFM32 no outside pin) */
#define RADIO_INFO_BIT 			14					/**< RF shutdown port (internal to EFM32 no outside pin) */
#define RADIO_INFO_PIN			1

extern const uint8_t Si446xPatchCommandsEZR[][8];
#define	RADIO_PATCH_BLOB		(const uint8_t *) &Si446xPatchCommandsEZR	/**< RF patch blob */
#define RADIO_PATCH_SIZE		1312				/**< RF patch size */

#define CHIP_EZR				1

#else // !CHIP_EZR_LG

extern const uint8_t Si446xPatchCommands[][8];
#define	RADIO_PATCH_BLOB		(const uint8_t *) &Si446xPatchCommands		/**< RF patch blob */
#define RADIO_PATCH_SIZE		1960				/**< RF patch size */

#define CHIP_EZR				0

#endif // CHIP_EZR_LG

bool const CONFIG_CHIP_EZR	= CHIP_EZR;

#ifdef __cplusplus
}
#endif

#endif // __TD_CONFIG_CHIP_H
/** @endcond */
