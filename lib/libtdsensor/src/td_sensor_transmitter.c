/***************************************************************************//**
 * @file td_sensor_transmitter.c
 * @brief Sensor Transmitter
 * @author Telecom Design S.A.
 * @version 1.1.0
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

#include <stdbool.h>
#include <stdint.h>

#include <td_scheduler.h>
#include <td_sigfox.h>

#include "sensor_private.h"
#include "sensor_config.h"
#include "sensor_send.h"
#include "sensor_send_private.h"
#include "td_sensor.h"
#include "td_sensor_lan.h"

#include "td_sensor_transmitter.h"

/***************************************************************************//**
 * @addtogroup TD_SENSOR_TRANSMITTER Sensor Transmitter
 * @brief
 *  Interface the Sigfox API by handling Sensor headers and retransmissions
 * @{
 ******************************************************************************/

/*******************************************************************************
 **************************  DEFINES   ****************************************
 ******************************************************************************/

/** @addtogroup TD_SENSOR_TRANSMITTER_DEFINES Defines
 * @{ */

/** Minimum interval in seconds between two Sigfox Transmissions. Should only apply to Gateways */
#define TRANSMISSION_INTERVAL 0

/** @} */

/*******************************************************************************
 **************************  TYPEDEFS   **************************************
 ******************************************************************************/

/** @addtogroup TD_SENSOR_TRANSMITTER_TYPEDEFS Typedefs
 * @{ */

/***************************************************************************//**
 * @brief
 *   Holds retransmission parameters, these parameters are saved by the Scheduler
 ******************************************************************************/
typedef struct {
	uint8_t index :8; 	///<index in the retransmission list
	uint8_t entry_id :4;		///<device entry_id
	SensorFrameType frame_type :4;		///<frame type
	uint8_t payload_count :4;		///<payload count
	uint8_t stamp :4;		///<stamp
}__PACKED RetransmissionParam;

/***************************************************************************//**
 * @brief
 *   Holds retransmission data and id of timer in charge of the retransmission
 ******************************************************************************/
typedef struct {
	uint8_t payload[10];
	uint8_t timer; //keep the timer to be able to stop it when receiving an ACK

} Retransmission;

/***************************************************************************//**
 * @brief
 *   Holds a Sigfox frame.
 ******************************************************************************/
typedef struct {
	uint8_t data[12];
	uint8_t count;

} Transmission;

/** @} */

/*******************************************************************************
 **************************  PRIVATE VARIABLES   *******************************
 ******************************************************************************/

/** @addtogroup TD_SENSOR_TRANSMITTER_PRIVATE_VARIABLES Private Variables
 * @{ */

/** List of future retransmissions */
static Retransmission RetransmissionList[MAX_RETRANSMISSION];

/** Sigfox frames queue */
static Transmission TransmissionQueue[MAX_TRANSMISSION_QUEUE];

/** Count of future retransmission */
static uint8_t QueueCount = 0;

/**Index of future retransmission*/
static uint8_t QueueCurrentIndex = 0;

/*Gateway frame Counter*/
static uint8_t GatewayCpt = 0;

/** Transmission regulation to allow LAN */
static bool CanEmit = true;

/** Transmission regulation timer */
static uint8_t QueueTimer = 0xFF;


/** @} */

/*******************************************************************************
 **************************  PRIVATE FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup TD_SENSOR_TRANSMITTER_PRIVATE_FUNCTIONS Private Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Emit first sigfox frame in the queue.
 * @return
 *   True if Sigfox transmission went fine. False otherwise.
 ******************************************************************************/

static bool TD_SENSOR_TRANSMITTER_EmitFirstInQueue()
{
	if (TD_SIGFOX_Send(TransmissionQueue[QueueCurrentIndex].data, TransmissionQueue[QueueCurrentIndex].count, 1)) {
		QueueCurrentIndex++;
		if (QueueCurrentIndex == MAX_TRANSMISSION_QUEUE) {
			QueueCurrentIndex = 0;
		}
		QueueCount--;
		return true;
	}
	return false;
}

/***************************************************************************//**
 * @brief
 *   Append a sigfox framet to the queue. If the queue is full send first in queue
 *   and then append frame. Must be called out of irq context.
 *
 * @param[in] data
 *	Data to be sent.
 *
 * @param[in] count
 *	Data count.
 ******************************************************************************/
static void TD_SENSOR_TRANSMITTER_AppendInQueue(uint8_t * data, uint8_t count)
{
	int i;
	int index = 0;

	//if circular buffer is full, send anyway
	if (QueueCount >= MAX_TRANSMISSION_QUEUE) {
		//can't afford to lose any frame
		TD_SENSOR_TRANSMITTER_EmitFirstInQueue();
	}

	//append to circular buffer
	index = QueueCurrentIndex + QueueCount;
	if (index >= MAX_TRANSMISSION_QUEUE) {
		index -= MAX_TRANSMISSION_QUEUE;
	}

	for (i = 0; i < count; i++) {
		TransmissionQueue[index].data[i] = data[i];
	}
	TransmissionQueue[index].count = count;
	QueueCount++;
}

/***************************************************************************//**
 * @brief
 *   Allows to emit again in Sigfox. Called by timer.
 *
 * @param[in] arg
 *	Timer parameter. Not used.
 *
 * @param[in] repetitions
 *	Timer parameter. Not used.
 ******************************************************************************/
static void TD_SENSOR_TRANSMITTER_QueueManagerCallback(uint32_t arg, uint8_t repetitions)
{
	CanEmit = true;
}

/***************************************************************************//**
 * @brief
 *  Sigfox Queue Manager. Regulate Sigfox transmissions to minimize LAN collisions.
 *
 ******************************************************************************/
static void TD_SENSOR_TRANSMITTER_QueueManager()
{
	//if we can emit and emit list is not empty
	if (QueueCount > 0 && CanEmit) {
		CanEmit = false;
		TD_SENSOR_TRANSMITTER_EmitFirstInQueue();

		//add timer to set CanEmit
		QueueTimer = TD_SCHEDULER_Append(TRANSMISSION_INTERVAL, 0, 0, 1, TD_SENSOR_TRANSMITTER_QueueManagerCallback, 0);

		//if timer count overflow then don't care about waiting
		if (QueueTimer == 0xFF) {
			CanEmit = true;
		}
	}

}

/***************************************************************************//**
 * @brief
 *   Send a Sigfox - Sensor Encoded - Frame.
 *
 * @param[in] frame
 *	SensorFrame
 *
 * @param[in] count
 *	SensorFrame payload count
 *
 * @param[in] retry
 *	Set to true if the frame is a retransmission.
 *
 * @return
 * 	Always true
 *
 ******************************************************************************/
static bool TD_SENSOR_TRANSMITTER_SendSigfoxPrivate(SensorFrame * frame, uint8_t count, bool retry)
{
	frame->header.retry = retry;
	frame->header.cpt = GatewayCpt;
	GatewayCpt = (GatewayCpt + 1) & 0xF;

	TD_SENSOR_TRANSMITTER_AppendInQueue((uint8_t *) frame, count);
	TD_SENSOR_TRANSMITTER_QueueManager();

	return true;
}

/***************************************************************************//**
 * @brief
 *   Retransmission handler being called by timer. Save arguments information
 *   for out-of-context retransmission.
 *
 * @param[in] arg
 *	Timer argument, containing RetransmissionParam
 *
 * @param[in] repetitions
 *	Timer repetitions left.
 *
 ******************************************************************************/
static void TD_SENSOR_TRANSMITTER_RetransmissionHandler(uint32_t arg, uint8_t repetitions)
{
	int i;
	SensorFrame frame;
	RetransmissionParam param;

	//compiler refuses cast
	param.index = (arg >> 20) & 0xFF;
	param.frame_type = (SensorFrameType) ((arg >> 16) & 0xF);
	param.entry_id = (arg >> 12) & 0xF;
	param.stamp = (arg >> 8) & 0x7;
	param.payload_count = (arg >> 4) & 0xF;

	for (i = 0; i < param.payload_count; i++) {
		frame.payload[i] = RetransmissionList[param.index].payload[i];
	}

	frame.header.stamp = param.stamp;
	frame.header.type = param.frame_type;
	frame.header.entry_id = param.entry_id;

	TD_SENSOR_TRANSMITTER_SendSigfoxPrivate(&frame, param.payload_count, 1);

	if (repetitions == 0) {
		RetransmissionList[param.index].timer = 0xFF;
	}
}

/***************************************************************************//**
 * @brief
 *   Add a Retransmission in the List
 *
 * @param[in] frame
 *	SensorFrame
 *
 * @param[in] count
 *	SensorFrame payload count
 *
 * @param[in] interval
 *	Interval in seconds at which the frame should be retransmitted.
 *
 * @param[in] repetitions
 *	How many times the frame should be retransmitted.
 *
 * @return
 * 	True is the retransmission was added to the list, false if RetransmissionList is full.
 *
 ******************************************************************************/
static bool TD_SENSOR_TRANSMITTER_AddRetransmission(SensorFrame * frame, uint8_t count, uint32_t interval, uint8_t repetitions)
{
	int i, index;
	RetransmissionParam param;
	uint32_t uint_param = 0;

	//look for first available slot
	for (index = 0; index < MAX_RETRANSMISSION; index++) {
		if (RetransmissionList[index].timer != 0xFF) {
			continue;
		} else {
			param.frame_type = frame->header.type;
			param.entry_id = frame->header.entry_id;
			param.stamp = frame->header.stamp;
			param.payload_count = count;
			param.index = index;

			//copy payload
			for (i = 0; i < count; i++) {
				RetransmissionList[index].payload[i] = frame->payload[i];
			}

			//encode by hand as compiler refuses cast
			uint_param = ((param.index & 0xFF) << 20) | ((param.frame_type & 0xF) << 16) | ((param.entry_id & 0xF) << 12) | ((param.stamp & 0x7) << 8)
					| ((param.payload_count & 0xF) << 4);

			//setup timer on interval with given repetitions
			RetransmissionList[index].timer = TD_SCHEDULER_Append(interval, 0, 0, repetitions, TD_SENSOR_TRANSMITTER_RetransmissionHandler, uint_param);

			return true;
		}
	}
	return false;
}

/** @} */

/*******************************************************************************
 **************************  PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup TD_SENSOR_TRANSMITTER_PUBLIC_FUNCTIONS Public Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Send a Sensor Frame to Sigfox and save retransmission profile.
 *
 * @param[in] frame
 *	SensorFrame to send.
 *
 * @param[in] count
 *	SensorFrame payload count
 *
 * @param[in] entry_id
 *	Entry Id of the module sending the frame.
 *
 * @param[in] profile
 *	Retransmission profile
 *
 * @return
 * 	Always true
 *
 ******************************************************************************/
bool TD_SENSOR_TRANSMITTER_SendSigfox(SensorFrame * frame, uint8_t count, uint8_t entry_id, TransmitProfile * profile)
{
	frame->header.entry_id = entry_id;

	TD_SENSOR_TRANSMITTER_AddRetransmission(frame, count + sizeof(SensorFrameHeader), profile->interval, profile->repetition);
	TD_SENSOR_TRANSMITTER_SendSigfoxPrivate(frame, count + sizeof(SensorFrameHeader), 0);

	return true;
}

/***************************************************************************//**
 * @brief
 *   Process the sigfox frames queue.
 *
 ******************************************************************************/
void TD_SENSOR_TRANSMITTER_Process()
{
	TD_SENSOR_TRANSMITTER_QueueManager();
}

/***************************************************************************//**
 * @brief
 *   Sensor Transmitter init.
 *
 ******************************************************************************/

void TD_SENSOR_TRANSMITTER_Init()
{
	int i;

	//set all retransmissions timer to unused
	for (i = 0; i < MAX_RETRANSMISSION; i++) {
		RetransmissionList[i].timer = 0xFF;
	}

	//reset all values
	GatewayCpt = 0;
	QueueCount = 0;
	QueueCurrentIndex = 0;
	QueueTimer = 0xFF;
	CanEmit = true;
}

/** @} */

/** @} */
