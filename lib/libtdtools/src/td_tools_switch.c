/***************************************************************************//**
 * @file
 * @brief Switch API for the TDxxxx RF modules.
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

#include <stdbool.h>
#include <stdint.h>

#include <efm32.h>
#include <em_gpio.h>

#include <td_gpio.h>
#include <td_rtc.h>
#include <td_scheduler.h>
#include <td_core.h>
#include <td_printf.h>

#include "td_tools_switch.h"

/***************************************************************************//**
 * @addtogroup SWITCH
 * @brief Switch API for the TDxxxx RF modules
 * @{
 ******************************************************************************/

/*******************************************************************************
 *************************   DEFINES   *****************************************
 ******************************************************************************/

/** @addtogroup SWITCH_DEFINES Defines
 * @{ */

//#define SWITCH_DEBUG

#ifdef SWITCH_DEBUG
/** Turn on trace mode if tfp_printf not commented */
#define DEBUG_PRINTF(...) tfp_printf(__VA_ARGS__)
#else
/** Turn on trace mode if tfp_printf not commented */
#define DEBUG_PRINTF(...)
#endif


/** Maximum number of switches */
#define MAX_SWITCH 5

// Debounce definitions

/** Read state period in ms */
#define CHECK_MSEC 5

 /** Stable delay before registering pressed in ms */
#define PRESS_MSEC 40

 /** Stable delay before registering released in ms */
#define RELEASE_MSEC 40

 /** Stable delay before giving up in ms */
#define GIVEUP_MSEC 40

/** How much push-time spaced up and down to trigger multiple push even in ms */
#define MULTIPLE_CLICK_MAX_INTERVAL (350 << 15) / 1000 // 350ms

/** @} */

/*******************************************************************************
 *************************   TYPEDEFS   ****************************************
 ******************************************************************************/

/** @addtogroup SWITCH_TYPEDEFS Typedefs
 * @{ */

/** Switch structure */
typedef struct {
	uint32_t last_push;								///< Tick count of last push
	TD_TOOLS_SWITCH_Events_t switch_event;			///< Current switch event
	TD_TOOLS_SWITCH_Events_t switch_pending_event;	///< Last switch event
	GPIO_Port_TypeDef port;							///< Switch GPIO port
	uint8_t debounce_timer;							///< Debounce timer ID
	uint8_t giveup_count;							///< Count after which we give up
	uint8_t check_count;							///< Number of check to perform
	uint8_t push_count;								///< Number of button presses
	uint8_t bit;									///< Switch GPIO bit
	uint8_t last_push_count;						///< Last number of button presses
	uint8_t long_push_timer;						///< Timer ID for long push detection
	int8_t long_push_time;							///< Duration of long push in ms
	int8_t very_long_push_time;						///< Duration of very long push in ms
	int8_t extra_long_push_time;					///< Duration of extra long push in ms
	bool debounced_switch_state;					///< Switch debounce state
	bool debounced;									///< Debounce flag
	bool enable;									///< Switch enable flag
	bool polarity;									///< Switch on state
	bool pushbutton;								///< Simple pushbutton flag
} Switch_t;

/** @} */

/*******************************************************************************
 ************************   PRIVATE VARIABLES   ********************************
 ******************************************************************************/

/** @addtogroup SWITCH_LOCAL_VARIABLES Local Variables
 * @{ */

/** Current number of switches */
static uint8_t SwitchCount = 0;

/** Switch table */
static Switch_t Switch[MAX_SWITCH];

/** @} */

/*******************************************************************************
 **************************  PRIVATE FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup SWITCH_LOCAL_FUNCTIONS Local Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *  Debounce function called every CHECK_MSEC after interrupt until debounce
 *  is done.
 *
 * @param[in] arg
 *   The callback argument containing the switch id.
 *
 * @param[in] repeats
 *   The number of repeats left, unused.
 *
 * @note
 *   Called from IRQ.
 ******************************************************************************/
static void Debounce(uint32_t arg, uint8_t repeats)
{
	bool read_state;
	Switch_t *p = &Switch[arg];

	read_state = GPIO_PinInGet(p->port, p->bit);
	p->debounced = false;
	if (read_state == p->debounced_switch_state &&
		--(p->giveup_count) != 0) {

		// Set the timer which allows a change from current state.
		if (p->debounced_switch_state != p->polarity) {
			p->check_count = RELEASE_MSEC / CHECK_MSEC;
		} else {
			p->check_count = PRESS_MSEC / CHECK_MSEC;
		}
	} else {

		// Key has changed - wait for new state to become stable.
		if (--(p->check_count) == 0 || p->giveup_count == 0) {

			// Timer expired - accept the change.
			p->debounced_switch_state = read_state;
			p->debounced=true;
			TD_WakeMainLoop();

			// Remove debounce timer
			TD_SCHEDULER_Remove(p->debounce_timer);
			p->debounce_timer = 0xFF;

			// If push was released and long push timer still running stop it
			// right now
			// Cannot afford to wait for switch process to remove it in case of
			// delay
			if (p->debounced_switch_state == p->polarity) {

				// Remove long push timer
				if (p->long_push_timer != 0xFF) {
					TD_SCHEDULER_Remove(p->long_push_timer);
					p->long_push_timer = 0xFF;
				}
			}

			// And reset the timer.
			if (p->debounced_switch_state != p->polarity) {
				p->check_count = RELEASE_MSEC / CHECK_MSEC;
			} else {
				p->check_count = PRESS_MSEC / CHECK_MSEC;
			}
			p->giveup_count = GIVEUP_MSEC / CHECK_MSEC;

			// Fast multiple push
			if (TD_RTC_TimeDiff(p->last_push) <
				MULTIPLE_CLICK_MAX_INTERVAL) {

				// Count
				p->push_count++;
			} else {
				p->push_count = 0;
			}
			p->last_push = RTC->CNT;
		} else {
			p->giveup_count = RELEASE_MSEC / CHECK_MSEC;
		}
	}
}

/***************************************************************************//**
 * @brief
 *  Button IRQ callback function.
 *
 * @param[in] mask
 *   The IRQ mask with bits corresponding to pending IRQs set.
 ******************************************************************************/
static void ButtonIrq(uint32_t mask)
{
	int i;

	// Find right switch in list (could have several)
	for (i = 0; i < SwitchCount; i++) {
		if (((1 << Switch[i].bit) & mask) && Switch[i].enable) {
			if (Switch[i].debounce_timer == 0xFF) {
				Switch[i].debounce_timer = TD_SCHEDULER_AppendIrq(
					0,
					(CHECK_MSEC * 32768) / 1000,
					0,
					TD_SCHEDULER_INFINITE,
					Debounce,
					i);
			}
		}
	}
}

/***************************************************************************//**
 * @brief
 *  Button polling callback function.
 *
 * @param[in] arg
 *   The callback argument containing the switch id.
 *
 * @param[in] repeats
 *   The number of repeats left, unused.
 *
 * @note
 *   Called from IRQ.
 ******************************************************************************/
static void PollingCallback(uint32_t arg, uint8_t repeats)
{
	Switch_t *p = &Switch[arg];

	if (arg >= SwitchCount) {
		return;
	}
	if (p->debounced_switch_state !=
		(bool) GPIO_PinInGet(p->port, p->bit)) {
		p->debounced_switch_state = !p->debounced_switch_state;
		p->debounced = true;
		TD_WakeMainLoop();
	}
	if (p->debounced_switch_state == p->polarity) {

		// Remove long push timer
		if (p->long_push_timer != 0xFF) {
			TD_SCHEDULER_Remove(p->long_push_timer);
			p->long_push_timer = 0xFF;
		}
	}

	// Fast multiple push
	if (TD_RTC_TimeDiff(p->last_push) < MULTIPLE_CLICK_MAX_INTERVAL) {

		// Count
		p->push_count++;
	} else {
		p->push_count = 0;
	}
	p->last_push = RTC->CNT;
}

/***************************************************************************//**
 * @brief
 *  Long push timer callback function.
 *
 * @param[in] arg
 *   The callback argument containing the switch id.
 *
 * @param[in] repeats
 *   The number of repeats left, unused.
 *
 * @note
 *   Called from IRQ.
 ******************************************************************************/
static void LongPushEvent(uint32_t arg, uint8_t repeats)
{
	Switch_t *p = &Switch[arg];

	if (p->switch_pending_event == TD_TOOLS_SWITCH_NO_EVENT) {
		p->switch_pending_event = TD_TOOLS_SWITCH_LONG_PUSHED;
		p->switch_event = TD_TOOLS_SWITCH_LONG_PUSHED;
		if (p->very_long_push_time > p->long_push_time) {
			p->long_push_timer = TD_SCHEDULER_Append(
				p->very_long_push_time - p->long_push_time,
				0,
				0,
				1,
				LongPushEvent,
				arg);
		} else {
			p->long_push_timer = 0xFF;
		}
	} else if(p->switch_pending_event == TD_TOOLS_SWITCH_LONG_PUSHED) {
		p->switch_pending_event = TD_TOOLS_SWITCH_VERY_LONG_PUSHED;
		p->switch_event = TD_TOOLS_SWITCH_VERY_LONG_PUSHED;
		if (p->extra_long_push_time > p->very_long_push_time) {
			p->long_push_timer = TD_SCHEDULER_Append(
				p->extra_long_push_time - p->very_long_push_time,
				0,
				0,
				1,
				LongPushEvent,
				arg);
		} else {
			p->long_push_timer = 0xFF;
		}
	} else if(p->switch_pending_event == TD_TOOLS_SWITCH_VERY_LONG_PUSHED) {
		p->switch_pending_event = TD_TOOLS_SWITCH_EXTRA_LONG_PUSHED;
		p->switch_event = TD_TOOLS_SWITCH_EXTRA_LONG_PUSHED;
		p->long_push_timer = 0xFF;
	}
}

/***************************************************************************//**
 * @brief
 *  Internal processing function.
 *
 * @param[in] id
 *   The switch id.
 ******************************************************************************/
static void Process(uint8_t id)
{
	uint8_t count;
	Switch_t *p = &Switch[id];

	if (p->debounced) {
		p->debounced = false;

		// On push start timer
		if (p->debounced_switch_state != p->polarity) {
			if (p->pushbutton) {
				p->switch_event = TD_TOOLS_SWITCH_SHORT_PUSHED;
				if (p->long_push_time != -1) {
					if (p->long_push_timer == 0xFF) {
						p->long_push_timer = TD_SCHEDULER_Append(
							p->long_push_time,
							0,
							0,
							1,
							LongPushEvent,
							id);
					}
				}
			} else {
				p->switch_event = TD_TOOLS_SWITCH_ON;
			}
		} else {

			// On release
			if (p->pushbutton) {

				// If still no pending event
				if (p->switch_pending_event == TD_TOOLS_SWITCH_NO_EVENT) {
					count = p->push_count;

					// Wait in case another push happens
					// Could be improved by actually computing real time for
					// going to push validated to here
					TD_RTC_Delay(MULTIPLE_CLICK_MAX_INTERVAL);

					// No change ? done
					if (count == p->push_count) {

						// No push count++ ? short
						if (count <= 1) {
							p->switch_event = TD_TOOLS_SWITCH_SHORT_RELEASED;
						} else {

							// Two or more
							if (count == 3) {
								p->switch_event = TD_TOOLS_SWITCH_DOUBLE_CLICK;
							} else if (count == 5) {
								p->switch_event = TD_TOOLS_SWITCH_TRIPLE_CLICK;
							} else {
								p->switch_event =
									TD_TOOLS_SWITCH_MULTIPLE_CLICK_DONE;
							}
							p->last_push_count = (count + 1) / 2;
							p->push_count = 0;
						}
					} else {
						p->last_push_count = (count + 1) / 2;
						p->switch_event = TD_TOOLS_SWITCH_MULTIPLE_CLICK_PENDING;
					}
				} else {
					if (p->switch_pending_event == TD_TOOLS_SWITCH_LONG_PUSHED) {
						p->switch_event = TD_TOOLS_SWITCH_LONG_RELEASED;
					} else if (p->switch_pending_event ==
						TD_TOOLS_SWITCH_VERY_LONG_PUSHED) {
						p->switch_event = TD_TOOLS_SWITCH_VERY_LONG_RELEASED;
					} else if(p->switch_pending_event ==
						TD_TOOLS_SWITCH_EXTRA_LONG_PUSHED) {
						p->switch_event = TD_TOOLS_SWITCH_EXTRA_LONG_RELEASED;
					} else {
						p->switch_event = p->switch_pending_event;
					}
					p->switch_pending_event = TD_TOOLS_SWITCH_NO_EVENT;
				}
			} else {
				p->switch_event = TD_TOOLS_SWITCH_OFF;
			}
		}
	}
}

/***************************************************************************//**
 * @brief
 *  Stop processing for a switch.
 *
 * @param[in] id
 *   The switch id.
 ******************************************************************************/
static void StopSwitch(uint8_t id)
{
	Switch_t *p = &Switch[id];

	// Disable
	p->enable = false;
	if (p->debounce_timer != 0xFF) {
		TD_SCHEDULER_Remove(p->debounce_timer);
		p->debounce_timer = 0xFF;
	}
	if (p->long_push_timer != 0xFF) {
		TD_SCHEDULER_Remove(p->long_push_timer);
		p->long_push_timer = 0xFF;
	}
	p->switch_pending_event = TD_TOOLS_SWITCH_NO_EVENT;
	p->switch_event = TD_TOOLS_SWITCH_NO_EVENT;
	p->debounced = false;
	p->push_count = 0;
	p->last_push = 0;
}

/***************************************************************************//**
 * @brief
 *  Start processing for a switch.
 *
 * @param[in] id
 *   The switch id.
 ******************************************************************************/
static void StartSwitch(uint8_t id)
{
	Switch_t *p = &Switch[id];

	p->enable = true;
	if ((bool) GPIO_PinInGet(p->port, p->bit) != p->polarity) {
		p->switch_pending_event = TD_TOOLS_SWITCH_STARTUP_PUSH;
		p->debounced_switch_state = true;
		p->check_count = RELEASE_MSEC / CHECK_MSEC;
	} else {
		p->debounced_switch_state = false;
		p->check_count = PRESS_MSEC / CHECK_MSEC;
	}
	p->giveup_count = GIVEUP_MSEC / CHECK_MSEC;
}

/** @} */

/*******************************************************************************
 **************************   PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup SWITCH_USER_FUNCTIONS User Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *  Initialize switch processing on a GPIO.
 *
 * @param[in] port
 *   The GPIO port to use.
 *
 * @param[in] bit
 *   The GPIO bit to use
 *
 * @param[in] mode
 *   The GPIO mode to use.
 *
 * @param[in] polarity
 *   Flag set to true if the GPIO is active high, false otherwise.
 *
 * @param[in] pushbutton
 *   Flag set to true if the GPIO is a push button, false for a switch.
 *
 * @param[in] long_time
 *   Delay for a long press in ms.
 *
 * @param[in] very_long_time
 *   Delay for a very long press in ms.
 *
 * @param[in] extra_long_time
 *   Delay for an extra long press in ms.
 *
 * @return
 *   Returns the switch ID if OK, -1 if there is no more switch available.
 ******************************************************************************/
int TD_TOOLS_SWITCH_Init(GPIO_Port_TypeDef port, uint8_t bit,
	GPIO_Mode_TypeDef mode, bool polarity, bool pushbutton, int8_t long_time,
	int8_t very_long_time, int8_t extra_long_time)
{
	Switch_t *p = &Switch[SwitchCount];

	DEBUG_PRINTF("Switch init Port:%c%d mode:%d polarity:%d bp:%d times:%d,%d,%d\r\n",
		port + 'A', bit,mode, polarity, pushbutton, long_time, very_long_time,
		extra_long_time);
	if (SwitchCount >= MAX_SWITCH) {
		DEBUG_PRINTF("Switch Failed\r\n");
		return -1;
	}

	// Set GPIO state
	GPIO_PinModeSet(port, bit, mode, polarity);

	// Save pull and mask
	p->polarity = polarity;
	p->port = port;
	p->bit = bit;

	// True for push button, false for simple on/off switch
	p->pushbutton = pushbutton;
	p->long_push_time = long_time;
	p->very_long_push_time = very_long_time;
	p->extra_long_push_time = extra_long_time;
	p->giveup_count = GIVEUP_MSEC / CHECK_MSEC;

	// Timer
	p->debounce_timer = 0xFF;
	p->long_push_timer = 0xFF;

	// Debounce
	p->debounced = false;

	// State machine
	p->switch_event = TD_TOOLS_SWITCH_NO_EVENT;
	p->switch_pending_event = TD_TOOLS_SWITCH_NO_EVENT;

	// Counter
	p->push_count = 0;
	p->last_push = 0;
	p->last_push_count = 0;

	// Enable
	p->enable = true;
	return SwitchCount++;
}

/***************************************************************************//**
 * @brief
 *  Switch process function called from main user loop.
 *
 * @param[in] id
 *   The switch id.
 *
 * @return
 *   Returns the last event for the given switch.
 ******************************************************************************/
// Called by user_loop, return current event and clear it
TD_TOOLS_SWITCH_Events_t TD_TOOLS_SWITCH_Process(uint8_t id)
{
	TD_TOOLS_SWITCH_Events_t last_event;

	if (id >= SwitchCount) {
		return TD_TOOLS_SWITCH_NO_EVENT;
	}
	Process(id);
	last_event = Switch[id].switch_event;
	Switch[id].switch_event = TD_TOOLS_SWITCH_NO_EVENT;
	return last_event;
}

/***************************************************************************//**
 * @brief
 *  Start processing for a switch handled with IRQs.
 *
 * @param[in] id
 *   The switch id.
 *
 * @return
 *   Returns false if id is invalid, true otherwise.
 ******************************************************************************/
bool TD_TOOLS_SWITCH_Start(uint8_t id)
{
	Switch_t *p = &Switch[id];

	if (id >= SwitchCount) {
		return false;
	}
	TD_GPIO_SetCallbackExtended(p->bit, ButtonIrq);
	StartSwitch(id);

	// Clear flag
	GPIO_IntClear(1 << p->bit);

	// Enable interrupt
	GPIO_IntConfig(p->port, p->bit, true, true, true);
	return true;
}

/***************************************************************************//**
 * @brief
 *  Start processing for a switch handled with polling.
 *
 * @param[in] id
 *   The switch id.
 *
 * @param[in] interval
 *   Polling interval in ticks.
 *
 * @return
 *   Returns false if id is invalid, true otherwise.
 *
 * @note: don't expect efficient multiple click handling with a high interval.
 ******************************************************************************/
bool TD_TOOLS_SWITCH_StartPolling(uint8_t id, uint32_t interval)
{
	Switch_t *p = &Switch[id];

	if (id >= SwitchCount) {
		return false;
	}
	StartSwitch(id);
	p->giveup_count = GIVEUP_MSEC / CHECK_MSEC;
	p->debounce_timer = TD_SCHEDULER_AppendIrq(
		0,
		interval,
		0,
		TD_SCHEDULER_INFINITE,
		PollingCallback,
		id);
	return true;
}

/***************************************************************************//**
 * @brief
 *  Stop processing for a switch handled with IRQ.
 *
 * @param[in] id
 *   The switch id.
 *
 * @return
 *   Returns false if id is invalid, true otherwise.
 ******************************************************************************/
bool TD_TOOLS_SWITCH_Stop(uint8_t id)
{
	Switch_t *p = &Switch[id];

	if (id >= SwitchCount) {
		return false;
	}
	if (p->enable) {

		// Stop interrupt
		GPIO_IntConfig(p->port, p->bit, true, true, false);

		// Clear flag
		GPIO_IntClear(1 << p->bit);
		StopSwitch(id);
	}
	return true;
}

/***************************************************************************//**
 * @brief
 *  Configure long, very long and extra long push durations.
 *
 * @param[in] id
 *   The switch id.
 *
 * @param[in] long_time
 *   The long push duration in ticks.
 *
 * @param[in] very_long_time
 *   The very long push duration in ticks.
 *
 * @param[in] extra_long_time
 *   The extra long push duration in ticks.
 *
 * @return
 *   Returns false if id is invalid, true otherwise.
 ******************************************************************************/
bool TD_TOOLS_SWITCH_SetPushTime(uint8_t id, int8_t long_time,
	int8_t very_long_time, int8_t extra_long_time)
{
	Switch_t *p = &Switch[id];

	if (id >= SwitchCount) {
		return false;
	}
	p->long_push_time = long_time;
	p->very_long_push_time = very_long_time;
	p->extra_long_push_time = extra_long_time;

	return true;
}

/***************************************************************************//**
 * @brief
 *  Stop processing for a switch handled with polling.
 *
 * @param[in] id
 *   The switch id.
 *
 * @return
 *   Returns false if id is invalid, true otherwise.
 ******************************************************************************/
bool TD_TOOLS_SWITCH_StopPolling(uint8_t id)
{
	if (id >= SwitchCount) {
		return false;
	}
	if (Switch[id].enable) {
		StopSwitch(id);
	}
	return true;
}

/***************************************************************************//**
 * @brief
 *  Get the number of presses for a given switch.
 *
 * @param[in] id
 *   The switch id.
 *
 * @return
 *   Returns the number presses.
 ******************************************************************************/
uint8_t TD_TOOLS_SWITCH_GetMultipleClickCount(uint8_t id)
{
	return Switch[id].last_push_count;
}

/***************************************************************************//**
 * @brief
 *  Simulate a switch event for debug purposes.
 *
 * @param[in] id
 *   The switch id.
 *
 * @param[in] event
 *   The event to simulate.
 *
 * @return
 *   Returns false if id is invalid, true otherwise.
 ******************************************************************************/
bool TD_TOOLS_SWITCH_Simulate(uint8_t id, TD_TOOLS_SWITCH_Events_t event)
{
	if (id >= SwitchCount) {
		return false;
	}
	Switch[id].switch_event = event;
	return true;
}

/** @} */

/** @} (end addtogroup SWITCH) */
