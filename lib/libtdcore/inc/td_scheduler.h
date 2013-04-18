/***************************************************************************//**
 * @file
 * @brief Scheduler API for the TDxxxx RF modules.
 * @author Telecom Design S.A.
 * @version 2.0.1
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

#ifndef __TD_SCHEDULER_H_
#define __TD_SCHEDULER_H_

#include <stdbool.h>
#include <stdint.h>

/***************************************************************************//**
 * @addtogroup SCHEDULER
 * @brief Scheduler for the TD1202 module
 * @{
 ******************************************************************************/

/*******************************************************************************
 *************************   PROTOTYPES   **************************************
 ******************************************************************************/

/** @addtogroup SCHEDULER_USER_FUNCTIONS User Functions
 * @{ */

/** @addtogroup SCHEDULER_PROTOTYPES Prototypes
 * @{ */

void TD_SCHEDULER_Init(void);
uint8_t TD_SCHEDULER_Append(uint32_t interval, uint16_t tick, uint32_t delay, uint8_t repetition, void  (*callback)(uint32_t, uint8_t),  uint32_t arg);
void TD_SCHEDULER_Remove(uint8_t id);
void TD_SCHEDULER_SetInterval(uint8_t id, uint32_t interval, uint16_t tick, uint32_t delay);
void TD_SCHEDULER_SetArg(uint8_t id, uint32_t arg);
uint8_t TD_SCHEDULER_GetRepetition(uint8_t id);
void TD_SCHEDULER_Process(void);

/** @} */
/** @} */

/** @} (end addtogroup SCHEDULER) */

#endif // __TD_SCHEDULER_H_
