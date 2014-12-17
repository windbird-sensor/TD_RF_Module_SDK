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

#include "td_tools_led.h"

//#define LED_DEBUG

#ifdef LED_DEBUG
/** Turn on trace mode if tfp_printf not commented */
#define DEBUG_PRINTF(...) tfp_printf(__VA_ARGS__)
#else
/** Turn on trace mode if tfp_printf not commented */
#define DEBUG_PRINTF(...)
#endif

/***************************************************************************//**
 * @addtogroup LED
 * @brief LED API for the TDxxxx RF modules
 * @{
 ******************************************************************************/

/*******************************************************************************
 *************************   DEFINES   *****************************************
 ******************************************************************************/

/** @addtogroup LED_DEFINES Defines
 * @{ */

/** Maximum number of LEDs */
#define MAX_LED 4

/** @} */

/*******************************************************************************
 *************************   TYPEDEFS   ****************************************
 ******************************************************************************/

/** @addtogroup LED_TYPEDEFS Typedefs
 * @{ */

/** LED structure */
typedef struct {
	TD_GPIO_Port_TypeDef port;
	GPIO_DriveMode_TypeDef drive;
	uint8_t bit;
	uint8_t inverted;
	bool faint;
	uint8_t timer;
	volatile bool state;
	bool polarity;
} Led_t;

/** @} */

/*******************************************************************************
 ************************   PRIVATE VARIABLES   ********************************
 ******************************************************************************/

/** @addtogroup LED_LOCAL_VARIABLES Local Variables
 * @{ */

/** Current number of LEDs */
static uint8_t LedCount = 0;

/** LED table */
static Led_t Leds[MAX_LED];

/** @} */

/*******************************************************************************
 **************************  PRIVATE FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup LED_LOCAL_FUNCTIONS Local Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *  Set a LED state.
 *
 * @param[in] id
 *   The LED id.
 *
 * @param[in] state
 *   The state to set.
 ******************************************************************************/
static void SetState(uint8_t id, bool state)
{
	Led_t *p = &Leds[id];

	DEBUG_PRINTF("SetState: id:%d, state:%d\r\n",id,state);
	if (state) {
		TD_GPIO_PinModeSet(p->port,	p->bit,	gpioModePushPullDrive,
			p->polarity ? 1 : 0);
		if (p->inverted != 0xFF) {
			TD_GPIO_PinModeSet(Leds[p->inverted].port,Leds[p->inverted].bit,
					gpioModePushPullDrive, Leds[p->inverted].polarity ? 0 : 1);
			Leds[p->inverted].state = false;
		}
	} else {
		TD_GPIO_PinModeSet(p->port,p->bit, gpioModeDisabled,
			p->polarity ? 0 : 1);
	}
	p->state = state;
}

/***************************************************************************//**
 * @brief
 *  Toggle timer callback function.
 *
 * @param[in] id
 *   The callback argument containing the LED id.
 *
 * @param[in] repeats
 *   The number of repeats left, unused.
 ******************************************************************************/

static void TimerToggle(uint32_t arg, uint8_t repetition)
{
	int i = 200;

	// For timer without infinite count
	if (repetition == 0) {
		Leds[arg].timer = 0xFF;
	}
	if (!Leds[arg].faint) {
		TD_TOOLS_LED_Toggle(arg);
	} else {

		// Wait in interrupt...
		while (i--) {
			SetState(arg, true);
		}
		SetState(arg,false);
	}
}

/** @} */

/*******************************************************************************
 **************************   PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup LED_USER_FUNCTIONS User Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *  Initialize LED processing on a GPIO.
 *
 * @param[in] port
 *   The GPIO port to use.
 *
 * @param[in] bit
 *   The GPIO bit to use
 *
 * @param[in] drive
 *   The GPIO drive mode to use.
 *
 * @param[in] polarity
 *   Flag set to true if the GPIO is active high, false otherwise.
 *
 * @return
 *   Returns the switch ID if OK, -1 if there is no more switch available.
 ******************************************************************************/
// Polarity, true is ON when pin at VCC
int TD_TOOLS_LED_Init(TD_GPIO_Port_TypeDef port, uint8_t bit,
	GPIO_DriveMode_TypeDef drive, bool polarity)
{
	Led_t *p = &Leds[LedCount];

	DEBUG_PRINTF("LedInit: port:%c bit:%d drive:%d polarity:%d ==> %d\r\n",
		port + 'A', bit, drive, polarity, LedCount);
	if (LedCount >= MAX_LED) {
		DEBUG_PRINTF("Too many led !\r\n");
		return -1;
	}
	p->port = port;
	p->bit = bit;
	p->drive = drive;
	p->polarity = polarity;
	p->inverted = 0xFF;
	if (drive != gpioDriveModeStandard) {
		TD_GPIO_DriveModeSet(port, drive);
	}

	// Initialize timer
	p->timer = 0xFF;

	// Switch off and set state
	SetState(LedCount, false);
	return LedCount++;
}

/***************************************************************************//**
 * @brief
 *  Declare 2 LEDs that must be on the same GPIO as being inverted.
 *
 * @param[in] id1
 *   The first LED id.
 *
 * @param[in] id2
 *   The second LED id.
 *
 * @return
 *   Returns false if an id is invalid, true otherwise.
 ******************************************************************************/
// Declare two leds as inversed parallel
// When one will be switch on the other will be switched off
bool TD_TOOLS_LED_InitInverted(uint8_t id1, uint8_t id2)
{
	if (id1 >= LedCount ||id2 >= LedCount) {
		return false;
	}
	Leds[id1].inverted = id2;
	Leds[id2].inverted = id1;
	return true;
}

/***************************************************************************//**
 * @brief
 *  Set a LED state.
 *
 * @param[in] id
 *   The LED id.
 *
 * @param[in] state
 *   The state to set.
  *
 * @return
 *   Returns false if id is invalid, true otherwise.
 ******************************************************************************/
bool TD_TOOLS_LED_Set(uint8_t id, bool state)
{
	if (id >= LedCount) {
		return false;
	}
	SetState(id, state);
	return true;
}

/***************************************************************************//**
 * @brief
 *  Toggle a LED state.
 *
 * @param[in] id
 *   The LED id.
 *
 * @return
 *   Returns false if id is invalid, true otherwise.
 ******************************************************************************/
bool TD_TOOLS_LED_Toggle(uint8_t id)
{
	if (id >= LedCount) {
		return false;
	}
	SetState(id, !Leds[id].state);
	return true;
}

/***************************************************************************//**
 * @brief
 *  Start blinking a LED.
 *
 * @param[in] id
 *   The LED id.
 *
 * @param[in] seconds
 *   The blink period integer number of seconds.
 *
 * @param[in] ticks
 *   The blink period fractional number of seconds in ticks.
 *
 * @param[in] repeats
 *   The number of blinks to perform, set to TD_SCHEDULER_INFINITE for infinite
 *   blinking.
 *
 * @return
 *   Returns false if id is invalid, true otherwise.
 ******************************************************************************/
bool TD_TOOLS_LED_StartBlink(uint8_t id, uint8_t seconds, uint16_t ticks,
  uint8_t repeats)
{
	Led_t *p = &Leds[id];

	if (id >= LedCount) {
		return false;
	}

	// If timer not already started
	if (p->timer == 0xFF) {
		p->timer = TD_SCHEDULER_AppendIrq(
			seconds,
			ticks,
			0,
			repeats,
			TimerToggle,
			id);
		}
		return true;
	}

/***************************************************************************//**
 * @brief
 *  Start blinking faintly a LED.
 *
 * @param[in] id
 *   The LED id.
 *
 * @param[in] seconds
 *   The blink period integer number of seconds.
 *
 * @return
 *   Returns false if id is invalid, true otherwise.
 ******************************************************************************/
bool TD_TOOLS_LED_StartFaintBlink(uint8_t id, uint8_t seconds)
{
	if (id >= LedCount) {
		return false;
	}

	// If timer not already started
	if (Leds[id].timer == 0xFF) {
		Leds[id].faint = true;
		Leds[id].timer = TD_SCHEDULER_AppendIrq(seconds, 0, 0, 0xFF,
			TimerToggle, id);
	}
	return true;
}

/***************************************************************************//**
 * @brief
 *  Stop blinking a LED.
 *
 * @param[in] id
 *   The LED id.
 *
 * @param[in] nice_stop
 *   If set to true, wait until the LED is off to stop timer, stops immediately
 *   otherwise.
 *
 * @return
 *   Returns false if id is invalid, true otherwise.
 ******************************************************************************/
bool TD_TOOLS_LED_StopBlink(uint8_t id, bool nice_stop)
{
	Led_t *p = &Leds[id];

	if (id >= LedCount) {
		return false;
	}

	if (p->timer != 0xFF) {

		// Wait until led is off for nice stop
		while (nice_stop && p->state) {
			;
		}
		TD_SCHEDULER_Remove(p->timer);
		p->timer = 0xFF;
	}
	if (!nice_stop) {
		SetState(id, false);
	}
	return true;
}

/** @} */

/** @} (end addtogroup LED) */
