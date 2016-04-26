/** @cond TD_CONFIG */
/***************************************************************************//**
 * @file
 * @brief Configuration file for the TDxxxx RF modules timer scheduler.
 * @author Telecom Design S.A.
 * @version 1.0.0
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2013-2015 Telecom Design S.A., http://www.telecomdesign.fr</b>
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
#ifndef __TD_CONFIG_SCHEDULER_H
#define __TD_CONFIG_SCHEDULER_H

#include <stdint.h>
#include <stdbool.h>

#include <td_config_ext.h>

#include <td_scheduler.h>

#ifdef __cplusplus
extern "C" {
#endif
/* TD_SCHEDULER_MAX_TIMER : total number of Scheduler Timer. Cost gain :
 * ram usage, system performance.
 * Be careful, standard libraries use some timers ...
 * default value : TD1202 = 10 others = 100 */
#ifndef TD_SCHEDULER_MAX_TIMER

#ifdef EFM32TG210F32
#define TD_SCHEDULER_MAX_TIMER	10
#else
#define TD_SCHEDULER_MAX_TIMER	100
#endif

#endif // TD_SCHEDULER_MAX_TIMER

/* TD_SCHEDULER_MAX_QUEUE : size of timer execution queue. Cost gain : ram
 * usage.
 * Be careful, if timer queue is too short, some timer events will get lost!
 * default value : TD1202 = 10 others = 100 */
#ifndef TD_SCHEDULER_MAX_QUEUE

#ifdef EFM32TG210F32
#define TD_SCHEDULER_MAX_QUEUE	10
#else
#define TD_SCHEDULER_MAX_QUEUE	100
#endif

#endif // TD_SCHEDULER_MAX_QUEUE

#ifndef TD_SCHEDULER_REMOVE_CODE

    /** Timer list */
    uint8_t const CONFIG_TD_SCHEDULER_MAX_TIMER = TD_SCHEDULER_MAX_TIMER;
    static TD_SCHEDULER_timer_t SchedulerTimer[TD_SCHEDULER_MAX_TIMER];
    TD_SCHEDULER_timer_t * TD_SCHEDULER_Timer = SchedulerTimer;

    /** Scheduler callback queue */
    uint8_t const CONFIG_TD_SCHEDULER_MAX_QUEUE = TD_SCHEDULER_MAX_QUEUE;
    static TD_SCHEDULER_callback_t SchedulerCallbackQueue[TD_SCHEDULER_MAX_QUEUE];
    TD_SCHEDULER_callback_t * TD_SCHEDULER_CallbackQueue  = SchedulerCallbackQueue;

/** Scheduler don't trap on queue overflow flag */
#ifdef TD_SCHEDULER_DONT_OVF_QUEUE
    bool const CONFIG_TD_SCHEDULER_DONT_OVF_QUEUE = true;
#else
    bool const CONFIG_TD_SCHEDULER_DONT_OVF_QUEUE = false;
#endif

#endif // TD_SCHEDULER_REMOVE_CODE

#ifdef TD_SCHEDULER_REMOVE_CODE
NULL_DYNAMIC(TD_SCHEDULER_Init);
NULL_DYNAMIC(TD_SCHEDULER_Process);
#else
INIT_DYNAMIC(TD_SCHEDULER_Init);
INIT_DYNAMIC(TD_SCHEDULER_Process);
#endif

#ifdef __cplusplus
}
#endif

#endif // __TD_CONFIG_SCHEDULER_H
/** @endcond */
