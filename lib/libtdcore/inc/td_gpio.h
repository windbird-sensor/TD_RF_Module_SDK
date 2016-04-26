/***************************************************************************//**
 * @file
 * @brief General Purpose IO (GPIO) peripheral API for the TDxxxx RF modules.
 * @author Telecom Design S.A.
 * @version 2.3.0
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2012-2016 Telecom Design S.A., http://www.telecomdesign.fr</b>
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

#ifndef __TD_GPIO_H
#define __TD_GPIO_H

#include <stdbool.h>
#include <em_gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

	/***********************************************************************//**
	 * @addtogroup GPIO
	 * @brief General Purpose IO (GPIO) peripheral API for the TDxxxx RF modules
	 * @{
	 **************************************************************************/

	/***************************************************************************
	 *************************   DEFINES   *************************************
	 **************************************************************************/

	/** @addtogroup GPIO_DEFINES Defines
	 * @{ */

#define TD_GPIO_USER		0								///< User GPIO IRQ hook
#define TD_GPIO_SYSTEM		1								///< System GPIO IRQ hook
#define TD_GPIO_EVEN		0								///< Even GPIO IRQ hook
#define TD_GPIO_ODD			2								///< Odd GPIO IRQ hook
#define TD_GPIO_SYSTEM_ODD	(TD_GPIO_SYSTEM | TD_GPIO_ODD)	///< Odd System GPIO IRQ hook
#define TD_GPIO_SYSTEM_EVEN	(TD_GPIO_SYSTEM | TD_GPIO_EVEN)	///< Even System GPIO IRQ hook
#define TD_GPIO_USER_ODD	(TD_GPIO_USER | TD_GPIO_ODD)	///< Odd User GPIO IRQ hook
#define TD_GPIO_USER_EVEN	(TD_GPIO_USER | TD_GPIO_EVEN)	///< Even User GPIO IRQ hook
#define TD_GPIO_MAX_HOOKS	4								///< Maximum number of GPIO IRQ hooks
#define TD_GPIO_ODD_MASK	0xAAAAAAAA						///< Mask for odd interrupts
#define TD_GPIO_EVEN_MASK	0x55555555						///< Mask for even interrupts

	/** @} */

	/***************************************************************************
	 ***********************   ENUMERATIONS   **********************************
	 **************************************************************************/

	/** @addtogroup GPIO_ENUMERATIONS Enumerations
	 * @{ */

	/** TD GPIO ports identifiers */
	typedef enum {
		TD_GPIO_PortA = 0, 		/**< Port A */
		TD_GPIO_PortB = 1, 		/**< Port B */
		TD_GPIO_PortC = 2, 		/**< Port C */
		TD_GPIO_PortD = 3, 		/**< Port D */
		TD_GPIO_PortE = 4, 		/**< Port E */
		TD_GPIO_PortF = 5,  	/**< Port F */
		TD_GPIO_PortRadio = 16, /**< RF port */
		TD_GPIO_PortNull = 255  /**< Null port */
	} TD_GPIO_Port_TypeDef;

	/** @} */

	/***************************************************************************
	 *************************   TYPEDEFS   ************************************
	 **************************************************************************/

	/** @addtogroup GPIO_TYPEDEFS Typedefs
	 * @{ */

	/** Interrupt process callback function. mask contains bitfield of one or
	 * multiple I/O that triggered IRQ */
	typedef void (*TD_GPIO_callback_t)(uint32_t mask);

	/** GPIO IRQ hook structure */
	typedef struct {
		TD_GPIO_callback_t callback;	///< callback function to handle this GPIO IRQ
		uint32_t mask;					///< mask to apply for matching this GPIO IRQ
	} TD_GPIO_hook_t;

	/** @} */

	/***************************************************************************
	 *************************   PROTOTYPES   **********************************
	 **************************************************************************/

	/** @addtogroup GPIO_USER_FUNCTIONS User Functions
	 * @{ */

	/***********************************************************************//**
	 * @brief
	 *   Default GPIO IRQ processing function.
	 **************************************************************************/
	static __INLINE void TD_GPIO_DefaultHook(void)
	{
	}

	/** @addtogroup GPIO_PROTOTYPES Prototypes
	 * @{ */

	void TD_GPIO_Init(void);
	void TD_GPIO_Dump(void);
	TD_GPIO_callback_t TD_GPIO_SetCallbackExtended(uint8_t bit,
		TD_GPIO_callback_t callback);
	TD_GPIO_callback_t TD_GPIO_GetCallbackExtended(uint8_t bit);
	void TD_GPIO_PinModeSet(TD_GPIO_Port_TypeDef port, unsigned int bit,
		GPIO_Mode_TypeDef mode, unsigned int out);
	void TD_GPIO_DriveModeSet(TD_GPIO_Port_TypeDef port,
		GPIO_DriveMode_TypeDef mode);

	// Depreciated
	void TD_GPIO_SetCallback(int type, TD_GPIO_callback_t callback,
		uint32_t mask);
	void TD_GPIO_GetIntConfig(unsigned int pin, GPIO_Port_TypeDef *port,
		bool *risingEdge, bool *fallingEdge, bool *enable);

	/** @} */
	/** @} */

	/** @} (end addtogroup GPIO) */

#ifdef __cplusplus
}
#endif

#endif // __TD_GPIO_H
