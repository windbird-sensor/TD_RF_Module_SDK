/**************************************************************************//**
 * @file
 * @brief Capacitive sense peripheral API for the TDxxxx RF modules.
 * @author Telecom Design S.A.
 * @version 1.1.0
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
 *****************************************************************************/

#include <em_device.h>
#include <em_emu.h>
#include <em_acmp.h>

#include <td_printf.h>
#include <td_capsense.h>
#include <td_measure.h>
#include <td_utils.h>

#include "td_config_ext.h"

/***************************************************************************//**
 * @addtogroup CAPSENSE
 * @brief Capacitive sense peripheral API for the TDxxxx RF modules
 * @{
 ******************************************************************************/

/*******************************************************************************
 *******************************   DEFINES   ***********************************
 ******************************************************************************/

/** @addtogroup CAPSENSE_DEFINES Defines
 * @{ */

//#define DEBUG_CAPSENSE
#ifdef DEBUG_CAPSENSE
	#define DEBUG_PRINTF(...) tfp_printf(__VA_ARGS__);
#else

/** Macro to print warnings when debugging */
#define DEBUG_PRINTF(...)
#endif



/** @} */

/*******************************************************************************
 *******************************   PRIVATE VARIABLES   *************************
 ******************************************************************************/

/** @addtogroup CAPSENSE_LOCAL_VARIABLES Local Variables
 * @{ */

/** CAPSENSE measured value */
static volatile uint32_t CapSenseValue = 0;

/** CAPSENSE value history array */
extern uint32_t CapsenseHistory[];

/** CAPSENSE value history pointer */
uint32_t * const TD_CAPSENSE_ValueHistory = CapsenseHistory;

/** CAPSENSE value history current index */
static uint32_t CapSenseValueHistoryIndex;

/** CAPSENSE maximum value */
static uint32_t MaxValue = 0xFFFFFFFF;

/** CAPSENSE voltage when maximum value was read */
static int32_t MaxValueVoltage = 0;

/** CAPSENSE value */
uint16_t CapSenseVal = 0;

/** CAPESENSE referecne voltage */
uint16_t CapSenseRef = 0;

/** @} */

/*******************************************************************************
 **************************  PRIVATE FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup CAPSENSE_LOCAL_FUNCTIONS Local Functions
 * @{ */

/**************************************************************************//**
 * @brief Read measured value.
 *
 * @return
 *  Returns true if pressed, false otherwise.
 *****************************************************************************/
static bool CAPSENSE_getPressed(void)
{
	int32_t delta;

	delta = (((int32_t) (CapSenseValue - MaxValue)) * 1000) / (int32_t) MaxValue;
	DEBUG_PRINTF("D%d C%d M%d\r\n", delta, CapSenseValue, MaxValue);
	if (delta <= -CONFIG_CAPSENSE_DELTA_CHANGE_THRESHOLD) {
		CapSenseVal = (uint16_t) CapSenseValue;
		CapSenseRef = (uint16_t) MaxValue;
		if ((MaxValueVoltage -
			TD_MEASURE_VoltageTemperatureExtended(false)) < 200) {
			return true;
		}
	}
	return false;
}

/**************************************************************************//**
 * @brief This function iterates through all the capacitive sensors and
 *     initiates a read. Uses EM1 while waiting for the result from each sensor.
 *****************************************************************************/
static void CAPSENSE_Measure(void)
{
	int i;
	uint32_t mask;
	uint32_t previous_max;
	uint32_t previous_irq;

#if 1
	// Enable TIMER0, TIMER1, CONFIG_ACMP_CAPSENSE and PRS clock
	CMU->HFPERCLKDIV |= CMU_HFPERCLKDIV_HFPERCLKEN;
	CMU->HFPERCLKEN0 |= CMU_HFPERCLKEN0_TIMER0
		| CMU_HFPERCLKEN0_TIMER1
		| CONFIG_ACMP_CAPSENSE_CLKEN
		| CMU_HFPERCLKEN0_PRS;

	// Initialize TIMER0 - Prescaler 2^9, top value 10, interrupt on overflow
	TIMER0->CTRL = TIMER_CTRL_PRESC_DIV512;
	TIMER0->TOP = 20;
	TIMER0->IEN = TIMER_IEN_OF;
	TIMER0->CNT = 0;

	// Initialize TIMER1 - Prescaler 2^10, clock source CC1, top value 0xFFFF
	TIMER1->CTRL = TIMER_CTRL_PRESC_DIV1024 | TIMER_CTRL_CLKSEL_CC1;
	TIMER1->TOP  = 0xFFFF;

	// Set up TIMER1 CC1 to trigger on PRS channel 0
	TIMER1->CC[1].CTRL = TIMER_CC_CTRL_MODE_INPUTCAPTURE /* Input capture */
		| TIMER_CC_CTRL_PRSSEL_PRSCH0   /* PRS channel 0 */
		| TIMER_CC_CTRL_INSEL_PRS       /* PRS input selected */
		| TIMER_CC_CTRL_ICEVCTRL_RISING /* PRS on rising edge */
		| TIMER_CC_CTRL_ICEDGE_BOTH;    /* PRS on rising edge */

	// Set up PRS channel 0 to trigger on ACMP1 output
	PRS->CH[0].CTRL = PRS_CH_CTRL_EDSEL_POSEDGE		/* Posedge triggers action */
		| CONFIG_PRS_CH_CTRL_SOURCESEL_ACMP_CAPSENSE      	/* PRS source */
		| CONFIG_PRS_CH_CTRL_SIGSEL_ACMPOUT_CAMPSENSE;    	/* PRS source */

#endif

	// Use the default STK capacitive sensing setup and enable it
	ACMP_Enable(CONFIG_ACMP_CAPSENSE);
	ACMP_CapsenseChannelSet(CONFIG_ACMP_CAPSENSE,
		(ACMP_Channel_TypeDef) CONFIG_ACMP_CAPSENSE_CHANNEL);

	// Reset timers
	TIMER0->CNT = 0;
	TIMER1->CNT = 0;

	// Mask all interrupts but timer0
	// Prevent all interrupts to avoid interrupt to get disabled between save
	// and stop
	mask = __get_PRIMASK();
	__set_PRIMASK(1);

	// Save all active interrupts
	previous_irq = NVIC->ISER[0];

	// Stop all interrupts but timer0
	NVIC->ICER[0] = ~(1 << ((uint32_t) (TIMER0_IRQn) & 0x1F));

	// Enable interrupts
	__set_PRIMASK(mask);

	// Start timers
	TIMER0->CMD = TIMER_CMD_START;
	TIMER1->CMD = TIMER_CMD_START;

	// When out of EM1, we are done for sure as all other interrupt are masked
	EMU_EnterEM1();

	// Restore interrupts as before
	NVIC->ISER[0] = previous_irq;

	// Disable ACMP while not sensing to reduce power consumption
	ACMP_Disable(CONFIG_ACMP_CAPSENSE);
	TD_CAPSENSE_ValueHistory[CapSenseValueHistoryIndex++] = CapSenseValue;
	CapSenseValueHistoryIndex %= CONFIG_CAPSENSE_HISTORY_DEPTH;
	previous_max = MaxValue;
	MaxValue = 0;
	for (i = 0; i < CONFIG_CAPSENSE_HISTORY_DEPTH; i++) {
		if (TD_CAPSENSE_ValueHistory[i] > MaxValue) {
			MaxValue = TD_CAPSENSE_ValueHistory[i];
		}
	}
	if (MaxValue != previous_max) {
		MaxValueVoltage = TD_MEASURE_VoltageTemperatureExtended(false);
		DEBUG_PRINTF("New max:%d at Voltage:%d\r\n",
			MaxValue,
			MaxValueVoltage);
	}
}

/** @} */

/*******************************************************************************
 **************************   PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup CAPSENSE_USER_FUNCTIONS User Functions
 * @{ */

/**************************************************************************//**
 * @brief TIMER0 interrupt handler.
 *     When TIMER0 expires the number of pulses on TIMER1 is inserted into
 *     channelValues. If this values is bigger than what is recorded in
 *     channelMaxValues, channelMaxValues is updated.
 *     Finally, the next ACMP channel is selected.
 *****************************************************************************/
void TIMER0_IRQHandler(void)
{
	//tfp_printf("*");
  // Stop timers
  TIMER0->CMD = TIMER_CMD_STOP;
  TIMER1->CMD = TIMER_CMD_STOP;

  // Clear interrupt flag
  TIMER0->IFC = TIMER_IFC_OF;

  // Read out value of TIMER1
  CapSenseValue = (TIMER1->CNT);
  //DEBUG_PRINTF("C%d\t", CapSenseValue);
}

/**************************************************************************//**
 * @brief Start a measure and update state.
 *
 * @return
 *   Returns true if button is pushed, false otherwise.
 *****************************************************************************/
bool TD_CAPSENSE_Sense(void)
{
	CAPSENSE_Measure();
	return CAPSENSE_getPressed();
}

/**************************************************************************//**
 * @brief Initializes the capacitive sense system.
 *     Capacitive sensing uses two timers: TIMER0 and TIMER1 as well as ACMP.
 *     ACMP is set up in cap-sense (oscillator mode).
 *     TIMER1 counts the number of pulses generated by CONFIG_ACMP_CAPSENSE.
 *     When TIMER0 expires it generates an interrupt.
 *     The number of pulses counted by TIMER0 is then stored in channelValues.
 *****************************************************************************/
void TD_CAPSENSE_Init(void)
{
	// Use the default STK capacative sensing setup
	ACMP_CapsenseInit_TypeDef capsenseInit = {
		false,              		/* fullBias */                    	\
		false,              		/* halfBias */                    	\
		0x7,                	/* biasProg */                    	\
		acmpWarmTime512,   		/* 512 cycle warmup to be safe */ 	\
		acmpHysteresisLevel5,                                 		\
		acmpResistor3,      	/* Res3 offers better variation*/	\
		false,              	/* low power reference */         	\
		0x3D,               	/* VDD level */                   	\
		true                	/* Enable after init. */          	\
	};

	// Enable TIMER0, TIMER1, CONFIG_ACMP_CAPSENSE and PRS clock
	CMU->HFPERCLKDIV |= CMU_HFPERCLKDIV_HFPERCLKEN;
	CMU->HFPERCLKEN0 |= CMU_HFPERCLKEN0_TIMER0
		| CMU_HFPERCLKEN0_TIMER1
		| CONFIG_ACMP_CAPSENSE_CLKEN
		| CMU_HFPERCLKEN0_PRS;

	// Initialize TIMER0 - Prescaler 2^9, top value 10, interrupt on overflow
	TIMER0->CTRL = TIMER_CTRL_PRESC_DIV512;
	TIMER0->TOP = 20;
	TIMER0->IEN = TIMER_IEN_OF;
	TIMER0->CNT = 0;

#if 0
	// Initialize TIMER1 - Prescaler 2^10, clock source CC1, top value 0xFFFF
	TIMER1->CTRL = TIMER_CTRL_PRESC_DIV1024 | TIMER_CTRL_CLKSEL_CC1;
	TIMER1->TOP  = 0xFFFF;

	// Set up TIMER1 CC1 to trigger on PRS channel 0
	TIMER1->CC[1].CTRL = TIMER_CC_CTRL_MODE_INPUTCAPTURE /* Input capture */
		| TIMER_CC_CTRL_PRSSEL_PRSCH0   /* PRS channel 0 */
		| TIMER_CC_CTRL_INSEL_PRS       /* PRS input selected */
		| TIMER_CC_CTRL_ICEVCTRL_RISING /* PRS on rising edge */
		| TIMER_CC_CTRL_ICEDGE_BOTH;    /* PRS on rising edge */

	// Set up PRS channel 0 to trigger on ACMP1 output
	PRS->CH[0].CTRL = PRS_CH_CTRL_EDSEL_POSEDGE		/* Posedge triggers action */
		| PRS_CH_CTRL_SOURCESEL_ACMP_CAPSENSE      	/* PRS source */
		| PRS_CH_CTRL_SIGSEL_ACMPOUT_CAMPSENSE;    	/* PRS source */
#endif

	// Set up ACMP0 in capsense mode
	ACMP_CapsenseInit(CONFIG_ACMP_CAPSENSE, &capsenseInit);

	// Enable TIMER0 interrupt
	NVIC_EnableIRQ(TIMER0_IRQn);
	CapSenseValueHistoryIndex = 0;
	memset((uint8_t *) TD_CAPSENSE_ValueHistory, 0, CONFIG_CAPSENSE_HISTORY_DEPTH);
	MaxValue = 0xFFFFFFFF;
	MaxValueVoltage = 0;
	CAPSENSE_Measure();
}


/**************************************************************************//**
 * @brief Get pressed-CapSense value
 *****************************************************************************/
uint32_t TD_CAPSENSE_GetValue(void)
{
	return (uint16_t) CapSenseVal << 16 | (uint16_t) CapSenseRef;
}

/** @} */

/** @} (end addtogroup CAPSENSE) */
