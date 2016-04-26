/***************************************************************************//**
 * @file
 * @brief LED API for the TDxxxx RF modules.
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

#include <em_cmu.h>
#include <em_letimer.h>

#include <td_module.h>
#include <td_gpio.h>
#include <td_scheduler.h>
#include <td_core.h>
#include <td_printf.h>
#include <td_rtc.h>
#include <td_measure.h>

#include "td_tools_led.h"

//#define LED_DEBUG
//#define LED_DEBUG_COMP

#ifdef LED_DEBUG
/** Turn on trace mode if tfp_printf not commented */
#define DEBUG_PRINTF(...) tfp_printf(__VA_ARGS__)
#else
/** Turn on trace mode if tfp_printf not commented */
#define DEBUG_PRINTF(...)
#endif

#ifdef LED_DEBUG_COMP
/** Turn on trace mode if tfp_printf not commented */
#define DEBUG_PRINTF_COMP(...) tfp_printf(__VA_ARGS__)
#else
/** Turn on trace mode if tfp_printf not commented */
#define DEBUG_PRINTF_COMP(...)
#endif

/** Tick compensation duration in ms */
#define TICK_COMPENS	TMS(30)

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
	uint16_t pwm_route;
	TD_GPIO_Port_TypeDef port;
	GPIO_DriveMode_TypeDef drive;
	LETIMER_TypeDef *letimer;
	TD_GPIO_Port_TypeDef port_aux;
	GPIO_DriveMode_TypeDef drive_aux;
	uint8_t timer_compens;
	volatile uint8_t timer;
	uint8_t bit;
	uint8_t bit_aux;
	uint8_t inverted;
	uint8_t pwm_value;
	uint8_t pwm_period;
	bool faint;
	volatile bool state;
	bool polarity;
	bool vcomp;
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
 *  Compens timer callback function.
 *
 * @param[in] id
 *   The callback argument containing the LED id.
 *
 * @param[in] repeats
 *   The number of repeats left, unused.
 ******************************************************************************/
static void TimerCompens(uint32_t arg, uint8_t repetition)
{
	int32_t compens;
	Led_t *p = &Leds[arg];
#ifdef LED_DEBUG_COMP
	uint32_t rtc = RTC->CNT;
#endif
	int mv = TD_MEASURE_VoltageRepeat();

	DEBUG_PRINTF_COMP("CLK:%d rtc:%d arg:%d letimer:0x%08X\r\n",
		((CMU->HFPERCLKEN0 >> _CMU_HFPERCLKEN0_ADC0_SHIFT)),
		RTC->CNT - rtc, arg, p->letimer);
	if (mv > 3300) {
		mv = 3300;
	}
	if (p->port_aux != TD_GPIO_PortNull) {
		compens = (((uint32_t) p->pwm_period) * (3300 - mv)) >> 10;
		if (mv < 2600) {
			compens -= ((uint32_t) p->pwm_period * (2600 - mv)) >> 11;
		} else {
			compens -= ((uint32_t) p->pwm_period * (mv - 2600)) >> 11;
		}
	} else {
		compens = (((uint32_t) p->pwm_value) *
			((4 << 10) + (3300 - mv) * 8)) >> 12;
	}
	if (compens > 254) {
		compens = 254;
	}
	if (compens < 1){
		compens = 1;
	}
	DEBUG_PRINTF_COMP("mv:%d csgne:%d, out:%d\r\n",
		mv, p->pwm_value, compens);
	//LETIMER_CompareSet(p->letimer, 1, compens);

	// Size optimization
	p->letimer->COMP1 = compens;
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
 ******************************************************************************/
static void SetState(uint8_t id, bool state)
{
	Led_t *p = &Leds[id];

	DEBUG_PRINTF("SetState: id:%d, state:%d p->letimer:%d p->vcomp:%d\r\n",
		id, state, p->letimer, p->vcomp);
	if (state) {
		TD_GPIO_DriveModeSet(p->port, p->drive);
		TD_GPIO_PinModeSet(p->port,	p->bit,	gpioModePushPullDrive, p->polarity);
		if (p->inverted != 0xFF) {
			TD_GPIO_PinModeSet(Leds[p->inverted].port,Leds[p->inverted].bit,
					gpioModePushPullDrive, Leds[p->inverted].polarity ? 0 : 1);
			Leds[p->inverted].state = false;
		}
		if (p->letimer){
			p->letimer->ROUTE=p->pwm_route;
			TD_GPIO_DriveModeSet(p->port_aux, p->drive_aux);
			TD_GPIO_PinModeSet(p->port_aux,	p->bit_aux,	gpioModePushPullDrive,
				p->polarity ? 1 : 0);
			TD_TOOLS_LED_SetPWM(id,p->pwm_period,p->pwm_value);
			LETIMER_Enable(p->letimer, true);
			if (p->vcomp && p->timer_compens==0xFF) {
				p->timer_compens = TD_SCHEDULER_AppendIrq(
				0, TICK_COMPENS, 0, 0xFF, TimerCompens, id);
			}
		}
	} else {
		if (p->letimer) {
			LETIMER_Enable(p->letimer, false);
			p->letimer->ROUTE = 0;
			if (p->timer_compens != 0xFF) {
				TD_SCHEDULER_Remove(p->timer_compens);
				p->timer_compens = 0xFF;
				CMU_ClockEnable(cmuClock_ADC0, false);
			}
		}

		 // Floating
		 TD_GPIO_PinModeSet(p->port,p->bit, gpioModeDisabled, 0);
		TD_GPIO_PinModeSet(p->port_aux,	p->bit_aux,	gpioModeDisabled, 0);
		if (p->inverted != 0xFF) {
			TD_GPIO_PinModeSet(Leds[p->inverted].port,Leds[p->inverted].bit,
					gpioModeDisabled, Leds[p->inverted].polarity ? 0 : 1);
			Leds[p->inverted].state = false;
		}
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
 *   Flag set to true if the led is on when pin is high, false otherwise.
 *
 * @return
 *   Returns the led ID if OK, -1 if there is no more led available.
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
	p->drive = drive;
	p->bit = bit;
	p->polarity = polarity;
	p->inverted = 0xFF;

	// Initialize timer
	p->timer = 0xFF;

	// PWM
	p->timer_compens = 0xFF;

	// PWM aux
	p->port_aux = TD_GPIO_PortNull;
	// Switch off and set state
	SetState(LedCount, false);
	return LedCount++;
}

/***************************************************************************//**
 * @brief
 *  Initialize LED processing on a GPIO with LETIMER.
 *
 * @param[in] port_pump
 *   The GPIO port to use.
 *
 * @param[in] bit_pump
 *   The GPIO bit to use
 *
 * @param[in] drive_pump
 *   The GPIO drive mode to use.
 *
 * @param[in] polarity
 *   Flag set to true if the led is on when pin is high, false otherwise.
 *
 * @param[in] letimerId
 *   LETIMERx associated with led pin
 *
 * @param[in] period
 *   PWM period
 *
 * @param[in] value
 *   PWM value
 *
 * @param[in] vcomp
 *   Enable power voltage compensation
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
 * @return
 *   Returns the Led ID if OK, -1 if there is no more led available.
 ******************************************************************************/
// Polarity, true is ON when pin at VCC
int TD_TOOLS_LED_InitPWM_Pump(TD_GPIO_Port_TypeDef port_pump, uint8_t bit_pump,
	GPIO_DriveMode_TypeDef drive_pump, bool polarity,uint8_t letimerId,
	uint8_t period, uint8_t value, uint8_t vcomp, TD_GPIO_Port_TypeDef port,
	uint8_t bit, GPIO_DriveMode_TypeDef drive)
{
	static const LETIMER_Init_TypeDef init = {
		false, true, false, false, true, false, true, true,
		letimerUFOAPwm, letimerUFOAPwm, letimerRepeatFree};
	int handle;
	Led_t *p;
	uint32_t loc = 0;
	CMU_Clock_TypeDef clk;

	handle = TD_TOOLS_LED_Init(port_pump, bit_pump, drive_pump, polarity);
	p = &Leds[handle];
	DEBUG_PRINTF("LedInitPWM: port:%c bit:%d drive:%d polarity:%d TIMER:%d out:%d loc:%d ==> %d\r\n",
		port_pump + 'A',
		bit_pump,
		drive_pump,
		polarity,
		letimerId & 0x3,
		((letimerId >> 2) & 0x1),
		letimerId >> 3,
		handle);
	switch (letimerId & 0x3) {
	case 0:
		clk = cmuClock_LETIMER0;
		p->letimer = LETIMER0;
		break;

	default:
		TD_Trap(TRAP_TIMER_CONFIG, letimerId);
		break;
	}
	CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);
	CMU_ClockEnable(clk, true);
	//LETIMER_Reset(p->letimer);
	LETIMER_Init(p->letimer, &init);
	switch ((letimerId >> 3) & 0x7) {
	case LED_LOCATION0:
		loc = LETIMER_ROUTE_LOCATION_LOC0;
		break;

	case LED_LOCATION1:
		loc = LETIMER_ROUTE_LOCATION_LOC1;
		break;

	//case LED_LOCATION2:
		//loc = LETIMER_ROUTE_LOCATION_LOC2;
		//break;

	//case LED_LOCATION3:
		//loc = LETIMER_ROUTE_LOCATION_LOC3;
		//break;

	default:
		TD_Trap(TRAP_TIMER_CONFIG, letimerId);
		break;
	}
	p->pwm_route = (((letimerId >> 2) & 0x1) ?
		LETIMER_ROUTE_OUT1PEN : LETIMER_ROUTE_OUT0PEN) | loc;
	p->vcomp = vcomp;
	p->port_aux = port;
	p->bit_aux = bit;
	p->drive_aux = drive;
	LETIMER_RepeatSet(p->letimer, 0, 0x01);
	LETIMER_RepeatSet(p->letimer, 1, 0x01);
	if (!TD_TOOLS_LED_SetPWM(handle, period, value)) {
		return -1;
	}
	DEBUG_PRINTF("LedInitPWM_pump: port:%c bit:%d drive:%d ==> %d\r\n",
		port_pump + 'A', bit_pump, drive_pump, handle);
	return handle;
}

/***************************************************************************//**
 * @brief
 *  Re-init LETIMER for LED PWM operation
 *
 * @param[in] id
 *   LED id.
 *
 * @param[in] period
 *   PWM period (0 : do not change actual value)
 *
 * @param[in] value
 *   PWM value
 *
 * @return
 *   Returns false if an id is invalid, true otherwise.
 ******************************************************************************/
void TD_TOOLS_LED_ReinitPWM (uint8_t id, uint8_t period, uint8_t value)
{
	Led_t *p;
	p = &Leds[id];
	static const LETIMER_Init_TypeDef init = {
		false,
		true,
		false,
		false,
		true,
		false,
		true,
		true,
		letimerUFOAPwm,
		letimerUFOAPwm,
		letimerRepeatFree
	};

	CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);
	CMU_ClockEnable(cmuClock_LETIMER0, true);
	LETIMER_Init(p->letimer, &init);
	LETIMER_RepeatSet(p->letimer, 0, 0x01);
	LETIMER_RepeatSet(p->letimer, 1, 0x01);
	TD_TOOLS_LED_SetPWM(id, period, value);
}

/***************************************************************************//**
 * @brief
 *  Set PWM period/value for led (do not light on or off a led, just set period)
 *
 * @param[in] id
 *   LED id.
 *
 * @param[in] period
 *   PWM period (0 : do not change actual value)
 *
 * @param[in] value
 *   PWM value
 *
 * @return
 *   Returns false if an id is invalid, true otherwise.
 ******************************************************************************/
int TD_TOOLS_LED_SetPWM(uint8_t id,uint8_t period, uint8_t value)
{
	Led_t *p;

	DEBUG_PRINTF("LedSetPWM: id:%d => %d/%d\r\n", id, value, period);
	p = &Leds[id];
	if (id >= LedCount) {
		return false;
	}
	if (period) {
		p->pwm_period = period;
		LETIMER_CompareSet(p->letimer, 0, period);
	}
	p->pwm_value = value;
	LETIMER_CompareSet(p->letimer, 1, value);
	return true;
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
	if (id1 >= LedCount || id2 >= LedCount) {
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
 *  Set a PUSH PULL state.
 *
 * @param[in] id
 *   The PUSH PULL id.
 *
 * @param[in] state
 *   The state to set.
  *
 * @return
 *   Returns false if id is invalid, true otherwise.
 ******************************************************************************/
bool TD_TOOLS_PUSHPULL_Set(uint8_t id, bool state)
{
	Led_t *p;

	if (id >= LedCount) {
		return false;
	}
	p = &Leds[id];
	if (state) {
		GPIO_PinOutSet((GPIO_Port_TypeDef) p->port, p->bit);
	} else {
		GPIO_PinOutClear((GPIO_Port_TypeDef) p->port, p->bit);
	}
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

	// Up to 127 blinks
	if (repeats < 255 && repeats > 128) {
		return false;
	}

	// If timer not already started
	if (p->timer == 0xFF) {
		SetState(id, true);
		p->timer = TD_SCHEDULER_AppendIrq(
			seconds,
			ticks,
			0,
			repeats == 0xff ? 0xFF : (repeats * 2) - 1,
			TimerToggle,
			id);
	} else {
		return false;
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
