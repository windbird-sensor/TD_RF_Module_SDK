/***************************************************************************//**
 * @file
 * @brief Configuration file for the TDxxxx RF modules SIGFOX stack.
 * @author Telecom Design S.A.
 * @version 1.1.0
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
#ifndef __TD_CONFIG_SIGFOX_H
#define __TD_CONFIG_SIGFOX_H

#include <stdint.h>
#include <stdbool.h>

#include <td_config_ext.h>

#include <td_sigfox.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Sigfox handling system
 *
 *   +=============+               +=======+
 *   |local module |   \/\/\/\---> |booster|
 *   +=============+               +=======+
 *
 *
 *                     | Use local emit | Use forward   |   Use proxy    |   Define                         |   Remarks
 *   :-----------------|:--------------:|:-------------:|:--------------:|:--------------------------------:|:----------------
 *                     | ID module      | ID booster    | ID module      |                                  |
 *   :-----------------|:--------------:|:-------------:|:--------------:|:--------------------------------:|:----------------
 *   default           |     USE        |      -        |    USE         | TD_SIGFOX_TRANSMIT_DEFAULT       | no define
 *   forward only      |      -         |     USE       |     -          | TD_SIGFOX_TRANSMIT_FORWARD_ONLY  | SIGFOX_REMOVE_CODE and SIGFOX_PROXY_USE_FORWARD
 *   local only        |     USE        |      -        |     -          | TD_SIGFOX_TRANSMIT_LOCAL_ONLY    | SIGFOX_PROXY_REMOVE_CODE
 *   none              |      -         |      -        |     -          | TD_SIGFOX_TRANSMIT_NONE          | SIGFOX_REMOVE_CODE and SIGFOX_PROXY_REMOVE_CODE
 *   :-----------------|:--------------:|:-------------:|:--------------:|:--------------------------------:|:----------------
 *   SIGFOX_REMOVE_CODE|    remove      |      -        |   remove       |                                  |
 *   PROXY_REMOVE_CODE |      -         |      -        |   remove       |                                  |
 *   PROXY_USE_FORWARD |      -         |     USE       |                |                                  |
 *
 */
/** @cond TD_CONFIG */

#include <td_sigfox.h>

/* Proxy class */
#ifndef TD_SIGFOX_PROXY_CLASS
#define TD_SIGFOX_PROXY_CLASS	9
#endif

/* RF Proxy class config */
uint8_t const CONFIG_TD_SIGFOX_PROXY_CLASS = TD_SIGFOX_PROXY_CLASS;

#ifdef TD_SIGFOX_TRANSMIT_DEFAULT
#define SIGFOX_TRANSMIT_DEFINED
#endif

#ifdef TD_SIGFOX_TRANSMIT_FORWARD_ONLY

#ifdef SIGFOX_TRANSMIT_DEFINED
#error("Only one SIGFOX_TRANSMIT_xxxx can be used at once !")
#endif

#define SIGFOX_TRANSMIT_DEFINED
#define TD_SIGFOX_REMOVE_CODE
#define TD_SIGFOX_PROXY_USE_FORWARD

#endif // TD_SIGFOX_TRANSMIT_FORWARD_ONLY

#ifdef TD_SIGFOX_TRANSMIT_LOCAL_ONLY

#ifdef SIGFOX_TRANSMIT_DEFINED
#error("Only one SIGFOX_TRANSMIT_xxxx can be used at once !")
#endif

#define SIGFOX_TRANSMIT_DEFINED
#define TD_SIGFOX_PROXY_REMOVE_CODE

#endif // TD_SIGFOX_TRANSMIT_LOCAL_ONLY

#ifdef TD_SIGFOX_TRANSMIT_NONE

#ifdef SIGFOX_TRANSMIT_DEFINED
#error("Only one SIGFOX_TRANSMIT_xxxx can be used at once !")
#endif

#define SIGFOX_TRANSMIT_DEFINED
#define TD_SIGFOX_REMOVE_CODE
#define TD_SIGFOX_PROXY_REMOVE_CODE

/* If we cannot transmit, we cannot receive */
#define TD_SIGFOX_DOWNLINK_REMOVE_CODE

#else // !TD_SIGFOX_TRANSMIT_NONE

#ifdef TD_SIGFOX_RECEIVE_NONE
#define TD_SIGFOX_DOWNLINK_REMOVE_CODE
#endif

#endif // TD_SIGFOX_TRANSMIT_NONE

#if defined(TD_SIGFOX_PROXY_REMOVE_CODE) && defined(TD_SIGFOX_PROXY_USE_FORWARD)
#error ("TD_SIGFOX_PROXY_REMOVE_CODE and TD_SIGFOX_PROXY_USE_FORWARD can't be used together. TD_SIGFOX_PROXY_USE_FORWARD Requires Proxy")
#endif

#if defined(TD_SIGFOX_REMOVE_CODE) &&!defined(TD_SIGFOX_PROXY_REMOVE_CODE) && !defined(TD_SIGFOX_PROXY_USE_FORWARD)
#error ("If you don't want to use proxy please use TD_SIGFOX_PROXY_REMOVE_CODE. Otherwise don't use TD_SIGFOX_REMOVE_CODE")
#endif

#if (defined(TD_SIGFOX_REMOVE_CODE) && defined(TD_SIGFOX_PROXY_REMOVE_CODE))

/* We want to remove all Sigfox possibilities */
NOP_DYNAMIC(TD_SIGFOX_Init);
NULL_DYNAMIC(TD_SIGFOX_SendRaw);
NULL_DYNAMIC(TD_SIGFOX_DirectTransmission);
NOP_DYNAMIC(TD_SIGFOX_ComputeFrameByMode);
#else

#if defined(TD_SIGFOX_PROXY_USE_FORWARD) && !defined(TD_SIGFOX_PROXY_REMOVE_CODE)

/* We are in forward mode only -> no sigfox capabilities */
INIT_DYNAMIC_ALT(TD_SIGFOX_Init, TD_SIGFOX_PROXY_InitForward);
NULL_DYNAMIC(TD_SIGFOX_SendRaw);
NULL_DYNAMIC(TD_SIGFOX_DirectTransmission);
NOP_DYNAMIC(TD_SIGFOX_ComputeFrameByMode);
#else
/* We are in full mode */
INIT_DYNAMIC(TD_SIGFOX_Init);
INIT_DYNAMIC(TD_SIGFOX_SendRaw);
INIT_DYNAMIC(TD_SIGFOX_DirectTransmission);
INIT_DYNAMIC(TD_SIGFOX_ComputeFrameByMode);
#endif

#endif // (defined(TD_SIGFOX_REMOVE_CODE) && defined(TD_SIGFOX_PROXY_REMOVE_CODE))

#if (defined(TD_SIGFOX_REMOVE_CODE) && !defined(TD_SIGFOX_PROXY_USE_FORWARD)) || \
    defined(TD_SIGFOX_PROXY_REMOVE_CODE)

/* We remove all code and don't forward, or we want only local transmit */
NOP_DYNAMIC(TD_SIGFOX_PROXY_Discover);
NOP_DYNAMIC(TD_SIGFOX_PROXY_Init);
NOP_DYNAMIC(TD_SIGFOX_PROXY_CompleteInit);
#else

/* We would use proxy or forward */
INIT_DYNAMIC(TD_SIGFOX_PROXY_Discover);
INIT_DYNAMIC(TD_SIGFOX_PROXY_Init);
INIT_DYNAMIC(TD_SIGFOX_PROXY_CompleteInit);
#endif

#if defined(TD_SIGFOX_REMOVE_CODE) && defined(TD_SIGFOX_PROXY_REMOVE_CODE)
NOP_DYNAMIC(TD_SIGFOX_Send);
NOP_DYNAMIC(TD_SIGFOX_SendV1);
NOP_DYNAMIC(TD_SIGFOX_symbol_IRQ);
#else
INIT_DYNAMIC(TD_SIGFOX_Send);
INIT_DYNAMIC(TD_SIGFOX_symbol_IRQ);

#ifdef TD_SIGFOX_SEND_V1_OVERLOAD
const TD_SIGFOX_SendV1_t TD_SIGFOX_SendV1 = TD_SIGFOX_SEND_V1_OVERLOAD;
#else
INIT_DYNAMIC(TD_SIGFOX_SendV1);
#endif

#endif // defined(TD_SIGFOX_REMOVE_CODE) && defined(TD_SIGFOX_PROXY_REMOVE_CODE)

#ifdef TD_SIGFOX_PROXY_REMOVE_CODE

/* We don't use proxy at all */
NULL_DYNAMIC(TD_SIGFOX_PROXY_Transmission);
NOP_DYNAMIC(TD_SIGFOX_PROXY_Send);
NULL_DYNAMIC(TD_SIGFOX_PROXY_ForwardTransmission);
NULL_DYNAMIC(TD_SIGFOX_PROXY_Process);
#else


#ifdef TD_SIGFOX_PROXY_USE_FORWARD
/* We use proxy in forward mode */
NULL_DYNAMIC(TD_SIGFOX_PROXY_Transmission);
INIT_DYNAMIC(TD_SIGFOX_PROXY_Send);
INIT_DYNAMIC(TD_SIGFOX_PROXY_ForwardTransmission);
#else

/* We use proxy in transparent mode */
INIT_DYNAMIC(TD_SIGFOX_PROXY_Transmission);
INIT_DYNAMIC(TD_SIGFOX_PROXY_Send);
NULL_DYNAMIC(TD_SIGFOX_PROXY_ForwardTransmission);
#endif

#ifdef TD_SIGFOX_DOWNLINK_REMOVE_CODE

/* We use only proxy for transmit not receive */
NULL_DYNAMIC(TD_SIGFOX_PROXY_Process);
#else

/* We use proxy for transmit and receive */
INIT_DYNAMIC(TD_SIGFOX_PROXY_Process);
#endif

#endif // TD_SIGFOX_PROXY_REMOVE_CODE

#if defined(TD_SIGFOX_REMOVE_CODE) || defined(TD_SIGFOX_DOWNLINK_REMOVE_CODE)

/* We don't use Sigfox receive */
NULL_DYNAMIC(TD_SIGFOX_DOWNLINK_DirectProcess);
#else

/* We use Sigfox receive */
INIT_DYNAMIC(TD_SIGFOX_DOWNLINK_DirectProcess);
#endif

#ifdef TD_SIGFOX_USE_TEST_CARRIER_CODE
INIT_DYNAMIC(TD_SIGFOX_TestCarrier);
#else
NULL_DYNAMIC(TD_SIGFOX_TestCarrier);
#endif

#ifdef TD_SIGFOX_USE_DOWNLINK_TEST_CODE
INIT_DYNAMIC(TD_SIGFOX_DOWNLINK_ReceiveTest);
#else
NULL_DYNAMIC(TD_SIGFOX_DOWNLINK_ReceiveTest);
#endif

/** FCC macro channel default configuration */
#ifndef SIGFOX_FCC_MACRO_CHANNEL_BITMASK_MSB
#define SIGFOX_FCC_MACRO_CHANNEL_BITMASK_MSB (0x0)
#define SIGFOX_FCC_MACRO_CHANNEL_BITMASK_MID (0x0)
#define SIGFOX_FCC_MACRO_CHANNEL_BITMASK_LSB (0x1FF)
#define SIGFOX_FCC_DEFAULT_MACRO_CHANNEL (1)
#endif

uint32_t const CONFIG_SIGFOX_FCC_MACRO_CHANNEL_BITMASK_MSB = SIGFOX_FCC_MACRO_CHANNEL_BITMASK_MSB;
uint32_t const CONFIG_SIGFOX_FCC_MACRO_CHANNEL_BITMASK_MID = SIGFOX_FCC_MACRO_CHANNEL_BITMASK_MID;
uint32_t const CONFIG_SIGFOX_FCC_MACRO_CHANNEL_BITMASK_LSB = SIGFOX_FCC_MACRO_CHANNEL_BITMASK_LSB;
uint16_t const CONFIG_SIGFOX_FCC_DEFAULT_MACRO_CHANNEL = SIGFOX_FCC_DEFAULT_MACRO_CHANNEL;

#ifdef __cplusplus
}
#endif

#endif // __TD_CONFIG_SIGFOX_H
/** @endcond */
