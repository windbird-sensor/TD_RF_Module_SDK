/***************************************************************************//**
 * @file
 * @brief Main program initialization for the TDxxxx RF modules.
 * @author Telecom Design S.A.
 * @version 1.0.0
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2012-2014 Telecom Design S.A., http://www.telecomdesign.fr</b>
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

#include <em_cmu.h>

#include "td_boot.h"
#include "td_sigfox.h"
#include "td_cmu.h"
#include "td_rtc.h"
#include "td_gpio.h"
#include "td_flash.h"
#include "td_uart.h"
#include "td_core.h"
#include "td_printf.h"
#include "td_scheduler.h"
#include "td_config_ext.h"

/***************************************************************************//**
 * @addtogroup MAIN Main
 * @brief Main program for the TDxxxx RF modules
 * @{
 ******************************************************************************/

/*******************************************************************************
 **************************   PUBLIC VARIABLES   *******************************
 ******************************************************************************/

/** @addtogroup MAIN_GLOBAL_VARIABLES Global Variables
 * @{ */

/** Boot handler */
extern TD_BOOT_Handler_t const TD_BOOT_Handler;

/** Stack protect handler */
extern TD_StackProtect_t const TD_StackProtect;

/** @} */

/*******************************************************************************
 *************************   PROTOTYPES   **************************************
 ******************************************************************************/

/** @addtogroup MAIN_PROTOTYPES External Declarations
 * @{ */

/** User setup function called once */
extern void TD_USER_Setup(void);

/** @} */

/*******************************************************************************
 **************************   PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup MAIN_USER_FUNCTIONS User Functions
 * @{ */

/** Macro containing all the common main initialization */
#define MAIN_INIT\
	EARLY_DEBUG_INIT\
	EARLY_DEBUG_PRINTF("Stack Protect\r\n");\
	if (TD_StackProtect) {\
		TD_StackProtect(true);\
	}\
	\
	EARLY_DEBUG_PRINTF("TD_BOOT_Init(0x%08X)\r\n", TD_BOOT_Handler);\
	if (TD_BOOT_Handler) {\
		TD_BOOT_Handler(CONFIG_PRODUCT_LED_POLARITY, CONFIG_PRODUCT_TYPE);\
	}\
	/* Initialize the clock Management Unit*/\
	EARLY_DEBUG_PRINTF("TD_CMU_Init\r\n");\
	TD_CMU_Init(true);\
	\
	/* Initialize the RTC clock*/\
	EARLY_DEBUG_PRINTF("TD_RTC_Init\r\n");\
	TD_RTC_Init(0);\
	\
	/* Initialize GPIOs*/\
	EARLY_DEBUG_PRINTF("TD_GPIO_Init\r\n");\
	TD_GPIO_Init();\
	\
	/* Initialize Flash Variables Management */\
	EARLY_DEBUG_PRINTF("TD_FLASH_SetVariablesVersion\r\n");\
	TD_FLASH_SetVariablesVersion(CONFIG_TD_FLASH_VARIABLES_VERSION);\
	\
	/* Initialize SigFox (can be bypassed by : TD_SIGFOX_REMOVE_CODE) */\
	EARLY_DEBUG_PRINTF("TD_SIGFOX_Init:0x%08X\r\n",TD_SIGFOX_Init);\
	TD_SIGFOX_Init(true);\
	/* Initialize Scheduler */\
	EARLY_DEBUG_PRINTF("TD_SCHEDULER_Init\r\n");\
	if (TD_SCHEDULER_Init) {\
		TD_SCHEDULER_Init();\
	}\
	\
	/* Call user setup function */\
	EARLY_DEBUG_PRINTF("TD_USER_Setup\r\n");\
	TD_USER_Setup();\
	\
	/* Complete SigFox Init by writing to flash if required (proxy got paired)
	 can be nopped by : TD_SIGFOX_PROXY_REMOVE_CODE*/\
	EARLY_DEBUG_PRINTF("TD_SIGFOX_CompleteInit\r\n");\
	TD_SIGFOX_PROXY_CompleteInit();\
	\
	EARLY_DEBUG_PRINTF("Entering Main Loop ...\r\n");


/* EARLY_DEBUG system is able to trace bootup events on serial port
 * It uses 115200,8N1 (to be compatible with serial loader speed)
 *
 * Two functions are defined, one with, one without early debug in order to
 * provide a link-time activation capability.
 *
 * This define allows to use tfp_printf even during the early boot stage
 * UART_LOADER in bootloader is compliant with this flag, as long as 115200
 * baudrate is kept
 * */

/** Null early debug print macro */
#define EARLY_DEBUG_PRINTF(...)

/** Null early debug initialization */
#define EARLY_DEBUG_INIT
/***************************************************************************//**
 * @brief
 *   Main Init (without early_debug) function for the TDxxxx RF modules.
 *
 * @details
 *   This function is part of main()
 ******************************************************************************/
void main_init(void)
{
	MAIN_INIT
}

#undef EARLY_DEBUG_PRINTF
#undef EARLY_DEBUG_INIT

/** Early debug print macro */
#define EARLY_DEBUG_PRINTF(...) tfp_printf(__VA_ARGS__)

/** Early debug initialization */
#define EARLY_DEBUG_INIT\
	BITBAND_Peripheral(&(CMU ->HFPERCLKDIV), \
		(cmuClock_HFPER >> CMU_EN_BIT_POS) & CMU_EN_BIT_MASK, 1);\
	BITBAND_Peripheral(&(CMU ->HFPERCLKEN0), \
		((cmuClock_GPIO) >> CMU_EN_BIT_POS) & CMU_EN_BIT_MASK, 1);\
	CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_CORELEDIV2);\
	CMU_ClockDivSet(cmuClock_LEUART0, cmuClkDiv_4);\
	/* Enable the LEUART0 clock*/\
	CMU_ClockEnable(cmuClock_LEUART0, true);\
	init_printf(TD_UART_Init(115200, true, false),\
		TD_UART_Putc,\
		TD_UART_Start,\
		TD_UART_Stop);\
	tfp_printf("--BOOT %s--\r\n", __TIME__);
/** printf function used for early debug purposes */

/***************************************************************************//**
 * @brief
 *   Main Init (with early_debug) function for the TDxxxx RF modules.
 *
 * @details
 *   This function is part of main()
 ******************************************************************************/
void main_init_debug(void)
{
	MAIN_INIT
}

/** @} */

/** @} (end addtogroup MAIN) */
