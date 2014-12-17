/***************************************************************************//**
 * @file
 * @brief CMU (Clock Management Unit) peripheral API for the TDxxxx RF modules.
 * @author Telecom Design S.A.
 * @version 2.0.3
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

#include <efm32.h>
#include <em_cmu.h>

#include "td_core.h"
#include "td_printf.h"
#include "td_cmu.h"

/***************************************************************************//**
 * @addtogroup CMU
 * @brief Clock management unit (CMU) Peripheral API for the TDxxxx RF modules
 * @{
 ******************************************************************************/

/*******************************************************************************
 *******************************   DEFINES   ***********************************
 ******************************************************************************/

/** @addtogroup CMU_DEFINES Defines
 * @{ */

/** Macro to dump a clock status */
#define DUMP_CLOCK_EN(lst)\
	first = true;\
	for (i = 0; i < sizeof (lst) / sizeof (char*); i++){\
		if (status & (1 << i)){\
			if (!first){\
				tfp_printf(",");\
			}\
			tfp_printf(lst[i]);\
			first = false;\
		}\
	}\
	tfp_printf("\r\n");

/** @} */

/*******************************************************************************
 **************************   PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup CMU_GLOBAL_FUNCTIONS Global Functions
 * @{ */

/***************************************************************************//**
 * @brief Initialize the CMU device.
 *
 * @param[in] external
 *   Use external crystal if true, otherwise use internal RC if false.
 ******************************************************************************/
void TD_CMU_Init(bool external)
{
	if (external) {

		// Start LFXO, and use LFXO for low-energy modules
		CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);
		CMU_OscillatorEnable(cmuOsc_LFXO, true, true);
	} else {
		CMU_OscillatorEnable(cmuOsc_LFRCO, true, true);
	}

	// Enable the clock to the interface of the low energy modules
	CMU_ClockEnable(cmuClock_CORELE, true);
}

/***************************************************************************//**
 * @brief Dump the CMU configuration.
 *
 * @param[in] cmu
 *   Pointer to the CMU configuration.
 ******************************************************************************/
void TD_CMU_Dump(CMU_TypeDef *cmu)
{
	uint32_t status, clk_sel;
	uint8_t i, first;
	static char *lst_hf_core[]={"AES","DMA","LEbus","EBI"};
	static char *lst_hf_per[]={"USART0","USART1","USART2","UART0","TIM0","TIM1",
		"TIM2","ACMP0","ACMP1","PRS","DAC0","GPIO","VCMP","ADC0","I2C0"};
	static char *lst_lfa[]={"RTC","LETIMER","LCD"};
	static char *lst_lfb[]={"LEUART0","LEUART1"};
	static char *clk_lst[]={"Dis/ULFRC","LFRCO","LFXO","HFCOREDIV2"};

	if (!cmu) {
		cmu = CMU;
	}
	tfp_printf("CMU_CTRL:0x%08X\r\n", cmu->CTRL);
	status = cmu->STATUS;
	tfp_printf("        |EN|RDY|HFCLK\r\n");
	tfp_printf("HFRCO   |%c | %c |%c\r\n",
		status&CMU_STATUS_HFRCOENS ? 'X':' ',
		status&CMU_STATUS_HFRCORDY ? 'X':' ',
		status&CMU_STATUS_HFRCOSEL ? 'X':' ');
	tfp_printf("HFXO    |%c | %c |%c\r\n",
		status&CMU_STATUS_HFXOENS ? 'X':' ',
		status&CMU_STATUS_HFXORDY ? 'X':' ',
		status&CMU_STATUS_HFXOSEL ? 'X':' ');
	tfp_printf("AUXHFRCO|%c | %c |%c\r\n",
		status&CMU_STATUS_AUXHFRCOENS ? 'X':' ',
		status&CMU_STATUS_AUXHFRCORDY ? 'X':' ',
		' ');
	tfp_printf("LFRCO   |%c | %c |%c\r\n",
		status&CMU_STATUS_LFRCOENS ? 'X':' ',
		status&CMU_STATUS_LFRCORDY ? 'X':' ',
		status&CMU_STATUS_LFRCOSEL ? 'X':' ');
	tfp_printf("LFXO    |%c | %c |%c\r\n",
		status&CMU_STATUS_LFXOENS ? 'X':' ',
		status&CMU_STATUS_LFXORDY ? 'X':' ',
		status&CMU_STATUS_LFXOSEL ? 'X':' ');
	tfp_printf("CMU_HFCORECLKDIV:%d\r\n", CMU->HFCORECLKDIV);
	tfp_printf("CMU_HFPERCLKDIV:0x%02X\r\n", CMU->HFPERCLKDIV);
	status = cmu->HFCORECLKEN0;
	clk_sel = cmu->LFCLKSEL;
	tfp_printf("HF Core Clk En:");
	DUMP_CLOCK_EN(lst_hf_core)
	status = cmu->HFPERCLKEN0;
	tfp_printf("HF Periph Clk En:");
	DUMP_CLOCK_EN(lst_hf_per)
	status = cmu->LFACLKEN0;
	tfp_printf("LF Periph Clk A %s En:", clk_lst[clk_sel & 3]);
	DUMP_CLOCK_EN(lst_lfa)
	status = cmu->LFBCLKEN0;
	tfp_printf("LF Periph Clk B %s En:", clk_lst[(clk_sel >> 2) & 3]);
	DUMP_CLOCK_EN(lst_lfb)
}

/** @} */

/** @} (end addtogroup CMU) */
