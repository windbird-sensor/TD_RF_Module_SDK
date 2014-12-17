/***************************************************************************//**
 * @file
 * @brief UART/RF bootloader API for the TDxxxx RF modules.
 * @author Telecom Design S.A.
 * @version 2.2.0
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

#ifndef __TD_BOOT_H
#define __TD_BOOT_H

#include <stdbool.h>
#include <stdint.h>

#include <td_config_ext.h>

#ifdef __cplusplus
extern "C" {
#endif

	/***********************************************************************//**
	 * @addtogroup BOOT Bootloader
	 * @brief RF bootloader API for the TDxxxx RF modules
	 * @{
	 **************************************************************************/

	/***************************************************************************
	 *************************   DEFINES   *************************************
	 **************************************************************************/

	/** @addtogroup BOOT_DEFINES Defines
	 * @{ */

	/** PLL setting for channel 255 */
#define BOOT_FREQ_CHAN255		BOOT_FREQ(0x40, 934571)					// 855.173185

	/** PLL setting for channel 254 */
#define BOOT_FREQ_CHAN254		BOOT_FREQ(0x40, 1000107)				// 856.798185

	/** PLL setting for channel 25 */
#define BOOT_FREQ_CHAN25		BOOT_FREQ(0x41, 927586 + 8066 * 2)		// 868.4

	/** PLL setting for channel 26 */
#define BOOT_FREQ_CHAN26		BOOT_FREQ(0x41, 927586 + 36297 * 2)		// 869.80

	/** Macro to define RF channel constant */
#define BOOT_FREQ_CHAN(x)		uint32_t const boot_freq_data = BOOT_FREQ_CHAN ## x; \
uint8_t boot_freq = x;

	/** Macro to combine integer and fractional part of PLL setting */
#define BOOT_FREQ(inte, frac)	(((frac) << 8) | (inte))

	/** @} */

	/***************************************************************************
	 ***********************   ENUMERATIONS   **********************************
	 **************************************************************************/

	/** @addtogroup BOOT_ENUMERATIONS Enumerations
	 * @{ */

	/** Bootloader operation modes */
	typedef enum {
		MODE_POWER_ON = 1,												///< Initialize SPI, power on TCXO and RF chip
		MODE_CONFIG = 2,												///< Configure the RF chip
		MODE_GPIO = 4,													///< Configure GPIOs on RF chip
		MODE_BOOT_EXIT = 8,												///< Put the RF chip back to sleep
		MODE_BOOT = MODE_CONFIG | MODE_GPIO,							///< Use the RF chip in bootloader mode
		MODE_FULL_APP = MODE_POWER_ON | MODE_CONFIG | MODE_BOOT_EXIT,	///< Fully reconfigure the RF chip in application mode and put back to sleep
		MODE_FULL_ON = MODE_POWER_ON | MODE_CONFIG 						///< Fully reconfigure the RF chip in application mode and stays on
	} TD_BOOT_radio_mode_t;

	/** @} */

	/***********************************************************************
	 *************************   TYPEDEFS   ****************************************
	 **************************************************************************/

	/** @addtogroup BOOT_USER_TYPEDEFS Typedefs
	 * @{ */

	/** Bootloader handling function pointer */
	typedef void (*TD_BOOT_Handler_t)(bool LedPolarity, uint8_t ProductType);

	/** @} */

	/*******************************************************************************
	 *************************   PROTOTYPES   **************************************
	 ******************************************************************************/

	/** @addtogroup BOOT_USER_FUNCTIONS User Functions
	 * @{ */

	void TD_BOOT_Init(bool LedPolarity, uint8_t ProductType);
	void TD_BOOT_ConfigureRadio(TD_BOOT_radio_mode_t Mode);
	void TD_BOOT_Exec_InitData(unsigned short const *data, unsigned char len);
	DECLARE_DYNAMIC(void, TD_BOOT_UartLoader, bool enable);

	/** @} */

	/** @} (end addtogroup BOOT) */

#ifdef __cplusplus
}
#endif

#endif // __TD_BOOT_H
