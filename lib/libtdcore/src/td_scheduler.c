/***************************************************************************//**
 * @file
 * @brief Scheduler API for the TDxxxx RF modules.
 * @author Telecom Design S.A.
 * @version 2.0.0
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2013 Telecom Design S.A., http://www.telecom-design.com</b>
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

#include <time.h>
#include <stdint.h>
#include <stdbool.h>
#include <td_rtc.h>
#include "td_core.h"
#include "td_printf.h"
#include "td_scheduler.h"

/***************************************************************************//**
 * @addtogroup SCHEDULER
 * @brief Scheduler for the TD1202 module
 * @{
 ******************************************************************************/

/*******************************************************************************
 *************************   DEFINES   *****************************************
 ******************************************************************************/

/** @addtogroup SCHEDULER_DEFINES Defines
 * @{ */

/** Maximum allowed timers */
#ifdef EFM32TG210F32
#define TD_SCHEDULER_MAX_TIMER 20
#elif EFM32G210F128
#define TD_SCHEDULER_MAX_TIMER 100
#endif

/** Maximum allowed callback in queue */
#define TD_SCHEDULER_MAX_IRQ_QUEUE 5

/** @} */

/*******************************************************************************
 *************************   TYPEDEFS   ****************************************
 ******************************************************************************/

/** @addtogroup SCHEDULER_TYPEDEFS Typedefs
 * @{ */

/** Scheduler timer structure */
typedef struct {
	uint64_t interval;						///< Interval at which the callback is called
	uint64_t last_time;						///< Last time at which the callback was called
	void (*callback)(uint32_t, uint8_t);	///< Function to be called
	uint32_t arg;							///< Argument to be passed to the callback when being called
	uint8_t repetition;						///< Timer repetition, 0xFF for infinite
} TD_SCHEDULER_timer_t;

/** Callback structure */
typedef struct {
	uint8_t index;
	void (*callback)(uint32_t, uint8_t);	///< Function to be called
	uint32_t arg;							///< Argument to be passed to the callback when being called
	uint8_t repetition;						///< Timer repetition, 0xFF for infinite
} TD_SCHEDULER_callback_t;

/** @} */

/*******************************************************************************
 *************************   PRIVATE VARIABLES   *******************************
 ******************************************************************************/

/** @addtogroup SCHEDULER_PRIVATE_VARIABLES Private Variables
 * @{ */

/** Array of timers */
static TD_SCHEDULER_timer_t TD_SCHEDULER_Timer[TD_SCHEDULER_MAX_TIMER];

/** Count of active timers */
static uint8_t TD_SCHEDULER_TimerCount = 0;

/** Active timer index */
static uint8_t TD_SCHEDULER_ActiveTimer = 0xFF;

/** Flag to reject undesired irq as comp1 always have a value */
static bool TD_SCHEDULER_WaitIrq = false;

/** Queue for timer irqs */
static TD_SCHEDULER_callback_t CallbackQueue[TD_SCHEDULER_MAX_IRQ_QUEUE];

/** Callback count in queue */
static uint8_t CallbackQueueCount = 0;

/** Current callback index in queue */
static uint8_t CallbackQueueIndex = 0;

/** @} */

/*******************************************************************************
 *************************   PRIVATE FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup SCHEDULER_PRIVATE_FUNCTIONS Private Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Return current time in 1/32768 ticks
 ******************************************************************************/
static uint64_t TD_SCHEDULER_GetTime(void) {
	uint64_t time_now;
	uint32_t time_tick, overflow_counter;

	overflow_counter = TD_RTC_GetOverflowCounter();
	time_tick = RTC_CounterGet();

	time_now = overflow_counter;
	time_now = time_now << 24; //<<15 & <<9
	time_now |= time_tick; //+ tick

	return time_now;
}

/***************************************************************************//**
 * @brief
 *   Arm the next timer IRQ according to all required timer.
 ******************************************************************************/
static void TD_SCHEDULER_Manager(void) {
	int i, j;
	uint64_t delta = 0xFFFFFFFFFFFFFFFF, temp_delta = 0, diff, time_now;
	bool set_alarm = false;
	bool bias;

	time_now = TD_SCHEDULER_GetTime();
	j = 0;
	for (i = 0; i < TD_SCHEDULER_MAX_TIMER; i++) {

		// Stop when all active timer have been processed
		if (j >= TD_SCHEDULER_TimerCount) {
			break;
		}
		if (TD_SCHEDULER_Timer[i].repetition == 0) {
			continue;
		}

		//Count valid timer
		j++;
		bias = false;

		// In case of a bias
		if (TD_SCHEDULER_Timer[i].last_time > time_now) {
			diff = TD_SCHEDULER_Timer[i].last_time - time_now
					+ TD_SCHEDULER_Timer[i].interval;
			bias = true;
		} else {
			diff = time_now - TD_SCHEDULER_Timer[i].last_time;
		}

		// If we are late, go for it right now
		if (diff >= TD_SCHEDULER_Timer[i].interval && !bias) {
			delta = 0;
			TD_SCHEDULER_ActiveTimer = i;
			set_alarm = true;
			break;
		} else {

			// Otherwise compute delta
			if (!bias) {
				temp_delta = TD_SCHEDULER_Timer[i].interval - diff;
			} else {
				temp_delta = diff;
			}

			// Wait until next overflow
			if (temp_delta >= 0x1000000) {
				continue;
			}
		}

		// Get smallest delta
		if (temp_delta < delta) {
			delta = temp_delta;
			TD_SCHEDULER_ActiveTimer = i;
			set_alarm = true;
		}
	}

	//Set alarm
	if (set_alarm) {
		TD_SCHEDULER_WaitIrq = true;
		TD_RTC_UserAlarmAfter(delta);
	}
}

/***************************************************************************//**
 * @brief
 *   Timer IRQ handler. Add the corresponding callback and argument in the queue so that it
 *   can be safely processed.
 ******************************************************************************/
static void TD_SCHEDULER_TimerIRQHandler(void) {
	uint8_t index;

	if (TD_SCHEDULER_WaitIrq) {
		if (TD_SCHEDULER_ActiveTimer != 0xFF) {
			TD_SCHEDULER_WaitIrq = false;
			TD_SCHEDULER_Timer[TD_SCHEDULER_ActiveTimer].last_time =
					TD_SCHEDULER_GetTime();
			index = CallbackQueueIndex + CallbackQueueCount;
			if (index >= TD_SCHEDULER_MAX_IRQ_QUEUE) {
				index -= TD_SCHEDULER_MAX_IRQ_QUEUE;
			}
			CallbackQueue[index].arg =
					TD_SCHEDULER_Timer[TD_SCHEDULER_ActiveTimer].arg;
			CallbackQueue[index].callback =
					TD_SCHEDULER_Timer[TD_SCHEDULER_ActiveTimer].callback;
			CallbackQueue[index].repetition =
					TD_SCHEDULER_Timer[TD_SCHEDULER_ActiveTimer].repetition;
			CallbackQueue[index].index = TD_SCHEDULER_ActiveTimer;
			CallbackQueueCount++;
		}
	}
	TD_RTC_ClearUserInterrupts();
}

/** @} */

/*******************************************************************************
 *************************   PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup SCHEDULER_PUBLIC_FUNCTIONS Public Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Safely process all callback in queue.
 ******************************************************************************/
void TD_SCHEDULER_Process(void) {
	void (*callback)(uint32_t, uint8_t);
	uint32_t arg;
	uint8_t repetition, index;

	// While in case a timer irq happens in between
	while (CallbackQueueCount > 0) {
		callback = CallbackQueue[CallbackQueueIndex].callback;
		arg = CallbackQueue[CallbackQueueIndex].arg;
		repetition = CallbackQueue[CallbackQueueIndex].repetition;
		index = CallbackQueue[CallbackQueueIndex].index;
		CallbackQueueCount--;
		CallbackQueueIndex++;
		if (CallbackQueueIndex == TD_SCHEDULER_MAX_IRQ_QUEUE) {
			CallbackQueueIndex = 0;
		}
		if (TD_SCHEDULER_Timer[index].repetition != 0xFF) {
			TD_SCHEDULER_Timer[index].repetition--;
			if (TD_SCHEDULER_Timer[index].repetition == 0) {
				TD_SCHEDULER_Remove(index);
			}
		}
		if (callback != 0) {
			(*callback)(arg, repetition - 1);
		}
		if (TD_SCHEDULER_TimerCount > 0) {
			TD_SCHEDULER_Manager();
		}
	}
	if (TD_RTC_IsOverflowed() == true && TD_SCHEDULER_TimerCount > 0) {
		TD_SCHEDULER_Manager();
	}
}

/***************************************************************************//**
 * @brief
 *   Initialize the scheduler.
 ******************************************************************************/
void TD_SCHEDULER_Init(void) {
	int i;

	for (i = 0; i < TD_SCHEDULER_MAX_TIMER; i++) {
		TD_SCHEDULER_Timer[i].repetition = 0;
	}
}

/***************************************************************************//**
 * @brief
 *   Append a timer on schedule.
 *
 * @param[in] interval
 *  Interval integer part in seconds at which the callback is called.
 *
 * @param[in] tick
 * Interval fractional part in ticks (1/32768) at which the callback is called
 *
 * @param[in] delay
 *  Time to wait before actually scheduling the timer.
 *
 * @param[in] repetition
 * Timer repetition, 0xFF for infinite repetitions.
 *
 * @param[in] callback
 *  Function to be called.
 *
 * @param[in] arg
 *  Argument to be passed to the callback when being called.
 *
 * @return
 * 	Returns a timer id if successful or 0xFF if no more timer can be added in
 * 	the list.
 ******************************************************************************/
uint8_t TD_SCHEDULER_Append(uint32_t interval, uint16_t tick, uint32_t delay,
		uint8_t repetition, void (*callback)(uint32_t, uint8_t), uint32_t arg) {
	uint8_t index;

	// Look for an available slot
	if (repetition > 0 && TD_SCHEDULER_TimerCount < TD_SCHEDULER_MAX_TIMER
			&& (interval != 0 || tick != 0)) {
		for (index = 0; index < TD_SCHEDULER_MAX_TIMER; index++) {
			if (TD_SCHEDULER_Timer[index].repetition != 0) {
				continue;
			} else {
				if (TD_SCHEDULER_TimerCount == 0) {
					TD_RTC_SetUserHandler(&TD_SCHEDULER_TimerIRQHandler);
					TD_RTC_EnableUserInterrupts(true);
				}
				TD_SCHEDULER_Timer[index].interval = interval;
				TD_SCHEDULER_Timer[index].interval =
						TD_SCHEDULER_Timer[index].interval << 15;
				TD_SCHEDULER_Timer[index].interval |= tick;
				TD_SCHEDULER_Timer[index].callback = callback;
				TD_SCHEDULER_Timer[index].repetition = repetition;
				TD_SCHEDULER_Timer[index].last_time = TD_SCHEDULER_GetTime()
						+ (delay << 15);
				TD_SCHEDULER_Timer[index].arg = arg;
				TD_SCHEDULER_TimerCount++;
				TD_SCHEDULER_Manager();
				return index;
			}
		}
	}
	return 0xFF;
}

/***************************************************************************//**
 * @brief
 *   Change a timer interval on the fly.
 *
 * @param[in] id
 *  Timer id.
 *
 * @param[in] interval
 *  Interval integer part in seconds at which the callback should now be called.
 *
 * @param[in] tick
 *  Interval fractional part in ticks at which the callback should now be called.
 *
 * @param[in] delay
 *  Time to wait in seconds before actually scheduling the timer.
 ******************************************************************************/
void TD_SCHEDULER_SetInterval(uint8_t id, uint32_t interval, uint16_t tick, uint32_t delay) {
	TD_SCHEDULER_Timer[id].interval = interval;
	TD_SCHEDULER_Timer[id].interval = TD_SCHEDULER_Timer[id].interval << 15;
	TD_SCHEDULER_Timer[id].interval |= tick;
	TD_SCHEDULER_Timer[id].last_time = TD_SCHEDULER_GetTime() + (delay << 15);
	TD_SCHEDULER_Manager();
}

/***************************************************************************//**
 * @brief
 *   Change a timer argument on the fly.
 *
 * @param[in] id
 *  Timer id.
 *
 * @param[in] arg
 *  New timer argument to be passed to the callback.
 ******************************************************************************/
void TD_SCHEDULER_SetArg(uint8_t id, uint32_t arg) {
	TD_SCHEDULER_Timer[id].arg = arg;
}

/***************************************************************************//**
 * @brief
 *   Change a timer argument on the fly.
 *
 * @param[in] id
 *  Timer id.
 *
 * @return
 * 	Remaining repetitions for the given Timer id.
 ******************************************************************************/
uint8_t TD_SCHEDULER_GetRepetition(uint8_t id) {
	return TD_SCHEDULER_Timer[id].repetition;
}

/***************************************************************************//**
 * @brief
 *   Remove a timer from the scheduler.
 *
 * @param[in] id
 *  Timer id.
 ******************************************************************************/
void TD_SCHEDULER_Remove(uint8_t id) {
	if (id == TD_SCHEDULER_ActiveTimer) {
		TD_SCHEDULER_ActiveTimer = 0xFF;
	}
	if (TD_SCHEDULER_TimerCount > 0) {
		TD_SCHEDULER_Timer[id].repetition = 0;
		TD_SCHEDULER_TimerCount--;
		if (TD_SCHEDULER_TimerCount == 0) {
			TD_RTC_EnableUserInterrupts(false);
			TD_RTC_SetUserHandler(0);
			TD_RTC_UserAlarmAfter(0);
		} else {
			TD_SCHEDULER_Manager();
		}
	}
}

/** @} */

/** @} (end addtogroup SCHEDULER) */
