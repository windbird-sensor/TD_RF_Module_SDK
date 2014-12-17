/***************************************************************************//**
 * @file
 * @brief LED API for the TDxxxx RF modules.
 * @author Telecom Design S.A.
 * @version 1.0.0
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2014 Telecom Design S.A., http://www.telecomdesign.fr</b>
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

#include <td_module.h>
#include <td_gpio.h>
#include <td_scheduler.h>
#include <td_core.h>
#include <td_printf.h>

#include "td_tools.h"
#include "td_tools_led.h"
#include "td_tools_switch.h"

/***************************************************************************//**
 * @addtogroup TOOLS
 * @brief Tools API for the TDxxxx RF modules
 * @{
 ******************************************************************************/

/*******************************************************************************
 *************************   DEFINES   *****************************************
 ******************************************************************************/

/** @addtogroup TOOLS_DEFINES Defines
 * @{ */

//#define TOOLS_DEBUG

#ifdef TOOLS_DEBUG
/** Turn on trace mode if tfp_printf not commented */
#define DEBUG_PRINTF(...) tfp_printf(__VA_ARGS__)
#else
/** Turn on trace mode if tfp_printf not commented */
#define DEBUG_PRINTF(...)
#endif

/** @} */

/*******************************************************************************
 **************************   PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup TOOLS_USER_FUNCTIONS User Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *  Initialize LED and switches.
 *
 * @param[in] data
 *   Pointer to the buffer containing the initialization data.
 ******************************************************************************/
void TD_TOOLS_Full_Init(const uint8_t *data)
{
	uint8_t id;

#ifdef TOOLS_DEBUG
	const uint8_t *data_mem = data;

	tfp_printf("ToolsInit\r\n");
	while (*data != 0xFF){
		tfp_printf("%d,", *data++);
	}
	tfp_printf("\r\n");
	data = data_mem;
#endif

	while (*data != 0xFF){
		id = *data >> 2;
		DEBUG_PRINTF("TOOLS_Init %d %d\r\n", data, (*data) &0x3);
		switch ((*data) & 0x3){
		case PRODUCT_UI_LED:
			CONFIG_TD_UI_Id[id] = TD_TOOLS_LED_Init(data[1] & 0x7,
				(data[1] >> 3) & 0xF, data[2] & 0xF,(data[1] >> 7));
			data += 3;
			break;
		case PRODUCT_UI_LED_INV:
			TD_TOOLS_LED_InitInverted(CONFIG_TD_UI_Id[id],
				CONFIG_TD_UI_Id[data[1]]);
			data += 2;
			break;
		case PRODUCT_UI_SWITCH:
			CONFIG_TD_UI_Id[id] = TD_TOOLS_SWITCH_Init(data[1] & 0x7,
				(data[1] >> 3) & 0xF, data[2] & 0xF, (data[1] >> 7),
				data[2] >> 4, -1, -1, -1);
			data += 3;
			break;
		}
	}
}

/** @} */

/** @} (end addtogroup TOOLS) */
