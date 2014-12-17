/***************************************************************************//**
 * @file
 * @brief SIGFOX down-link API for the TDxxxx RF modules.
 * @author Telecom Design S.A.
 * @version 1.0.0
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2013-2014 Telecom Design S.A., http://www.telecomdesign.fr</b>
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
#ifndef __TD_SIGFOX_DOWNLINK_H
#define __TD_SIGFOX_DOWNLINK_H

#include <stdint.h>
#include <stdbool.h>

#include <td_core.h>
#include <td_rtc.h>

#ifdef __cplusplus
extern "C" {
#endif

	/***************************************************************************//**
	 * @addtogroup SIGFOX SIGFOX
	 * @brief SIGFOX API for the TDxxxx RF modules
	 * @{
	 ******************************************************************************/

	/*******************************************************************************
	 *************************   DEFINES   *****************************************
	 ******************************************************************************/

	/** @addtogroup SIGFOX_DEFINES Defines
	 * @{ */

	/** SIGFOX down-link priority */
#define TD_SIGFOX_DOWNLINK_PRIORITY			15

/** SIGFOX down-link center frequency */
#define TD_SIGFOX_DOWNLINK_FREQUENCY		869525000

	/** SIGFOX down-link startup delay */
#define TD_SIGFOX_DOWNLINK_DELAY			20

	/** SIGFOX down-link duration */
#define TD_SIGFOX_DOWNLINK_DURATION			25

	/** Received down-link payload size */
#define TD_SIGFOX_DOWNLINK_PAYLOAD_SIZE		8

	/** @} */

	/***************************************************************************
	 ***********************   ENUMERATIONS   **********************************
	 **************************************************************************/

	/** @addtogroup SIGFOX_ENUMERATIONS Enumerations
	 * @{ */

	/** SIGFOX down-link states */
	typedef enum {
		TD_SIGFOX_DOWNLINK_STATE_IDLE,  // Not active, default state
		TD_SIGFOX_DOWNLINK_STATE_DELAY,
		TD_SIGFOX_DOWNLINK_STATE_DOZE,
		TD_SIGFOX_DOWNLINK_STATE_START, // Begin reception
		TD_SIGFOX_DOWNLINK_STATE_RECEIVE,
		TD_SIGFOX_DOWNLINK_STATE_DECODE,
		TD_SIGFOX_DOWNLINK_STATE_SWITCH,
		TD_SIGFOX_DOWNLINK_STATE_SEND, // Send ack-ack
		TD_SIGFOX_DOWNLINK_STATE_STOP // Completed
	} TD_SIGFOX_downlink_state_t;

	/** SIGFOX down-link events */
	typedef enum {
		TD_SIGFOX_DOWNLINK_EVENT_NONE,
		TD_SIGFOX_DOWNLINK_EVENT_DELAY,
		TD_SIGFOX_DOWNLINK_EVENT_DELAY_TIMEOUT,
		TD_SIGFOX_DOWNLINK_EVENT_DOZE_TIMEOUT,
		TD_SIGFOX_DOWNLINK_EVENT_RECEIVE_TIMEOUT,
		TD_SIGFOX_DOWNLINK_EVENT_FRAME_RECEIVED,
		TD_SIGFOX_DOWNLINK_EVENT_ABORT,
		TD_SIGFOX_DOWNLINK_EVENT_SWITCH_TIMEOUT,
		TD_SIGFOX_DOWNLINK_EVENT_ERROR,
		TD_SIGFOX_DOWNLINK_EVENT_DONE
	} TD_SIGFOX_downlink_event_t;

	/** @} */

	/***************************************************************************
	 *************************   TYPEDEFS   ************************************
	 **************************************************************************/

	/** @addtogroup SIGFOX_TYPEDEFS Typedefs
	 * @{ */

	/** SIGFOX down-link callback */
	typedef int (*TD_SIGFOX_DOWNLINK_callback_t)(uint8_t *rx_frame,
		uint8_t length);

	/** SIGFOX down-link configuration structure */
	typedef struct {
		uint32_t frequency;		///< Down-link frequency in Hz
		uint16_t delay;			///< Delay before receiving in seconds
		uint16_t duration;		///< Down-link receive duration in seconds
	} TD_SIGFOX_DOWNLINK_config_t;

	#pragma pack(1)
	/** SIGFOX down-link OOB acknowledge frame */
	typedef struct {
		uint8_t type;			///< OOB type, fixed to 9 for SIGFOX down-link
								// acknowledgment frames
		uint16_t idle_voltage;	///< Idle voltage in mV
		uint16_t tx_voltage;	///< Voltage during RF TX in mV
		uint16_t temperature;	///< Temperature in 1/10 °C
		uint8_t rssi;			///< RSSI - 100 measured during down-link receive
	} TD_SIGFOX_downlink_acknowedge_t;
	#pragma pack()

	/** @} */

	/***************************************************************************
	 *************************   PROTOTYPES   **********************************
	 **************************************************************************/

	/** @addtogroup SIGFOX_USER_FUNCTIONS User Functions
	 * @{ */

	void TD_SIGFOX_DOWNLINK_Abort(void);
	bool TD_SIGFOX_DOWNLINK_IsRxPending(void);
	TD_SIGFOX_DOWNLINK_callback_t TD_SIGFOX_DOWNLINK_SetUserCallback(
		TD_SIGFOX_DOWNLINK_callback_t callback);
	TD_SIGFOX_DOWNLINK_callback_t TD_SIGFOX_DOWNLINK_GetUserCallback(void);
	bool TD_SIGFOX_DOWNLINK_Process(void);
	int TD_SIGFOX_DOWNLINK_GetRSSI(void);

	/** @} */

	/** @addtogroup SIGFOX_GLOBAL_FUNCTIONS Global Functions
	 * @{ */

	DECLARE_DYNAMIC(bool,TD_SIGFOX_DOWNLINK_DirectProcess, void);
	DECLARE_DYNAMIC(bool, TD_SIGFOX_DOWNLINK_ReceiveTest, uint32_t sequence,
		uint16_t frequency, uint16_t duration);
	void TD_SIGFOX_DOWNLINK_Configure(TD_SIGFOX_DOWNLINK_config_t *config);
	bool TD_SIGFOX_DOWNLINK_Receive(uint32_t sequence, uint16_t frequency,
		uint32_t central_frequency, uint32_t elapsed, uint32_t id);
	TD_SIGFOX_DOWNLINK_config_t *TD_SIGFOX_DOWNLINK_GetConfig(void);
	bool TD_SIGFOX_DOWNLINK_CheckSignature(uint8_t *buffer, uint16_t signature,
		uint32_t sequence);
	void TD_SIGFOX_DOWNLINK_SetRSSI(int rssi);
	void TD_SIGFOX_DOWNLINK_SetBuffer(uint8_t *buffer, uint16_t length,
		uint8_t *count);
	bool TD_SIGFOX_DOWNLINK_SendAck(void);

	/** @} */

	/** @} (end addtogroup SIGFOX) */

#ifdef __cplusplus
}
#endif

#endif // __TD_SIGFOX_DOWNLINK_H
