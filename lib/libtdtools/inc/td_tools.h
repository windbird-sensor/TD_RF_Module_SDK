/***************************************************************************//**
 * @file
 * @brief Tools API for the TDxxxx RF modules.
 * @author Telecom Design S.A.
 * @version 1.1.0
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2014-2015 Telecom Design S.A., http://www.telecomdesign.fr</b>
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

#ifndef __TD_TOOLS_H
#define __TD_TOOLS_H

#ifdef __cplusplus
extern "C" {
#endif

	/***********************************************************************//**
	 * @addtogroup TOOLS
	 * @brief Tools API for the TDxxxx RF modules
	 * @{
	 **************************************************************************/

	/***************************************************************************
	 *************************   DEFINES   *************************************
	 **************************************************************************/

	/** @addtogroup TOOLS_DEFINES Defines
	 * @{ */

	/** Flag for direct LED polarity */
#define TD_TOOLS_LED_DIRECT				true

	/** Flag for inverted LED polarity */
#define TD_TOOLS_LED_INVERTED			false

	/** Do power voltage compensation */
#define TD_TOOLS_LED_VCOMPENS			true

	/** No power voltage compensation */
#define TD_TOOLS_LED_NO_VCOMPENS		false

	/** Flag for active high switch polarity */
#define TD_TOOLS_SWITCH_ON_HIGH			false

	/** Flag for active low switch polarity */
#define TD_TOOLS_SWITCH_ON_LOW			true

	/** Flag for on/off switch type */
#define TD_TOOLS_SWITCH_ON_OFF			false

	/** Flag for pushbutton switch type */
#define TD_TOOLS_SWITCH_PUSH_BUTTON		true

	/** Value for unused long switch processing */
#define TD_TOOLS_SWITCH_LONG_NOT_USED	-1

	/** Helper macro for UI table */
#define UI(x)							CONFIG_TD_UI_Id[x]

	/** UI direct LED type */
#define PRODUCT_UI_LED					0

	/** UI inverted LED type */
#define PRODUCT_UI_LED_INV				1

	/** UI switch type */
#define PRODUCT_UI_SWITCH				2

	/** UI direct LED type with PWM control */
#define PRODUCT_UI_LED_PWM				3

	/** UI direct LED type with PWM control and charge pump*/
#define PRODUCT_UI_LED_PWM_PUMP			4

	/** UI push pull type */
#define PRODUCT_UI_PUSH_PULL			5

	/** Macro to pack direct LED flags */
#define PUI_LED(id, port, bit, drive, direct) \
	(id << 3) | PRODUCT_UI_LED, port | (bit << 3) | (direct << 7), drive, port

	/** Macro to pack direct push pull flags */
#define PUI_PP(id, port, bit, drive, direct) \
(id << 3) | PRODUCT_UI_PUSH_PULL, port | (bit << 3) | (direct << 7), drive, port

	/** Macro to pack inverted LED flags */
#define PUI_LED_INV(id1, id2) \
	(id1 << 3) | PRODUCT_UI_LED_INV, id2

	/** Macro to pack direct LED flags */
#define PUI_LED_PWM(id, port, bit, drive, direct, out, period, value, vcomp) \
	(id << 3) | PRODUCT_UI_LED_PWM, port | (bit << 3) | (direct << 7), \
	drive, port, out | (vcomp<<6), period, value

	/** Macro to pack direct LED flags */
#define PUI_LED_PWM_PUMP(id, port_pump, bit_pump, drive_pump, direct, out, \
	period, value, vcomp, port, bit, drive) (id << 3) | \
	PRODUCT_UI_LED_PWM_PUMP, port_pump | (bit_pump << 3) | (direct << 7), \
	drive_pump, port_pump, out | (vcomp<<6), period, value,drive | (bit << 2), \
	port

	/** Macro to pack switch flags */
#define PUI_SWITCH(id, port, bit, drive, direct, push_button) \
	(id << 3) | PRODUCT_UI_SWITCH, port | (bit << 3) | (direct << 7), \
	drive | (push_button << 4)

	/** @} */

	/***************************************************************************
	 *************************   PROTOTYPES   **********************************
	 **************************************************************************/

	/** @addtogroup TOOLS_USER_FUNCTIONS User Functions
	 * @{ */

void TD_TOOLS_Full_Init(const uint8_t *data);

	/** @} */

	/** @} (end addtogroup TOOLS) */

#ifdef __cplusplus
}
#endif

#endif //__TD_TOOLS_H
