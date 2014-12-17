/***************************************************************************//**
 * @file
 * @brief Sensor LAN
 * @author Telecom Design S.A.
 * @version 1.2.0
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

#ifndef __TD_SENSOR_LAN_H
#define __TD_SENSOR_LAN_H

#include <stdbool.h>
#include <stdint.h>

#include <td_lan.h>
#include <td_utils.h>
#include <td_config_ext.h>

#include "sensor_send.h"

#ifdef __cplusplus
extern "C" {
#endif

	/***********************************************************************//**
	 * @addtogroup TD_SENSOR_LAN Sensor LAN
	 *
	 * @{
	 **************************************************************************/

	/***************************************************************************
	 **************************  DEFINES   ************************************
	 **************************************************************************/

	/** @addtogroup TD_SENSOR_LAN_DEFINES Defines
	 * @{ */

#define BROADCAST_ADDRESS							0x000000
#define BROADCAST_MASK								0xFFFFFF
#define NETWORK_MASK								0xFFFF00

#define TD_SENSOR_LAN_PAYLOAD_SIZE (TD_LAN_PAYLOAD_SIZE - 1)

#define LOCALSENSORFRAME_COUNT_WIDTH				4				/**< Count flag field width */
#define LOCALSENSORFRAME_COUNT_SHIFT				0				/**< Count flag field shift */

#define LOCALSENSORFRAME_TYPE_WIDTH					4				/**< Type flag field width */
#define LOCALSENSORFRAME_TYPE_SHIFT					4				/**< Type flag field shift */

#define LOCALFORWARDFRAME_REPETITION_WIDTH			4				/**< Repetition flag field width */
#define LOCALFORWARDFRAME_REPETITION_SHIFT			0				/**< Repetition flag field shift */

#define LOCALFORWARDFRAME_INTERVAL_WIDTH			28				/**< Interval flag field width */
#define LOCALFORWARDFRAME_INTERVAL_SHIFT			4				/**< Interval flag field shift */

/** Get the Sensor LAN count field */
#define GET_LOCALSENSORFRAME_COUNT(b)				GET_BITFIELD(b, LOCALSENSORFRAME_COUNT)

/** Set the Sensor LAN count field */
#define SET_LOCALSENSORFRAME_COUNT(b, v)			SET_BITFIELD(b, LOCALSENSORFRAME_COUNT, v)

/** Get the Sensor LAN type field */
#define GET_LOCALSENSORFRAME_TYPE(b)				GET_BITFIELD(b, LOCALSENSORFRAME_TYPE)

/** Set the Sensor LAN type field */
#define SET_LOCALSENSORFRAME_TYPE(b, v)				SET_BITFIELD(b, LOCALSENSORFRAME_TYPE, v)

/** Get the Sensor LAN repetition field */
#define GET_LOCALFORWARDFRAME_REPETITION(b)			GET_BITFIELD(b, LOCALFORWARDFRAME_REPETITION)

/** Set the Sensor LAN repetition field */
#define SET_LOCALFORWARDFRAME_REPETITION(b, v)		SET_BITFIELD(b, LOCALFORWARDFRAME_REPETITION, v)

/** Get the Sensor LAN interval field */
#define GET_LOCALFORWARDFRAME_INTERVAL(b)			GET_BITFIELD(b, LOCALFORWARDFRAME_INTERVAL)

/** Set the Sensor LAN interval field */
#define SET_LOCALFORWARDFRAME_INTERVAL(b, v)		SET_BITFIELD(b, LOCALFORWARDFRAME_INTERVAL, v)

	/** @} */

	/***************************************************************************
	 ***********************   ENUMERATIONS   **********************************
	 **************************************************************************/

	/** @addtogroup TD_SENSOR_LAN_ENUMERATIONS Enumerations
	 * @{ */

	/** Sensor LAN frames types */
	typedef enum {
		LOCAL_REGISTER = 1,
		LOCAL_FORWARD = 2,
		LOCAL_KEEPALIVE = 3,
		LOCAL_DATA = 4,
		LOCAL_FASTLAN_START = 5,
		LOCAL_FASTLAN_DATA = 6,
		LOCAL_RAW_FORWARD = 7,
		LOCAL_DATA_DOWNLINK = 8
	} TD_SENSOR_LAN_FrameType_t;

	/** Sensor LAN FastLAN types */
	typedef enum {
		FASTLAN_RELAY = 1
	} TD_SENSOR_FastLANType_t;

	/** Sensor LAN result types */
	typedef enum {
		NOT_ACKED,
		ACK_OK,
		ACK_ERROR,
		ACK_REGISTRATION_DISABLED,
		ACK_ALREADY_REGISTERED,
		NOT_SENT,
		GATEWAY_BUSY,
		TRANSMITTER_BUSY,
		TRANSMITTER_SENT,
		SENSOR_LAN_QUEUED,
		SENSOR_LAN_FRAME_ERROR,
		LAN_ERROR,
		LAN_CHANNEL_BUSY,
		ACK_OK_PARTIAL, //stream frame, got an ack at some point
		BROADCAST_OK,
	} TD_SENSOR_LAN_AckCode_t;

	/** @} */

	/***************************************************************************
	 **************************  TYPEDEFS   ************************************
	 **************************************************************************/

	/** @addtogroup TD_SENSOR_LAN_TYPEDEFS Typedefs
	 * @{ */

	/** LAN address structure */
	typedef struct {
		uint32_t address;
		uint32_t mask;
	} TD_SENSOR_LAN_Address_t;

	/** LAN pairing structure */
	typedef struct {
		TD_SENSOR_LAN_Address_t lanAddress;
		uint8_t pairingType;
	} TD_SENSOR_LAN_PairingParams_t;

	/** LAN frame structure
	 * The LAN frame is made of:
	 *
	 *   | header  | data     |
	 *   | :-----: | :------: |
	 *   | 1 byte  | 17 bytes |
	 *
	 * The LAN frame header is made of:
	 *
	 *   | frame type | count  |
	 *   | :--------: | -----: |
	 *   | 4 bits     | 4 bits |
	 *
	 *   The LAN frame type is a TD_SENSOR_LAN_FrameType_t value
	 *   The LAN count contains the actual useful data size as the data field is
	 *   fixed to TD_SENSOR_LAN_PAYLOAD_SIZE byte long
	 * */
	typedef struct {
		uint8_t header;
		uint8_t data[TD_SENSOR_LAN_PAYLOAD_SIZE];
	} TD_SENSOR_LAN_Frame_t;

	/** Register frame structure
	 *  The LAN REGISTER frame is made of:
	 *
	 *   | SIGFOX ID | Device Class |
	 *   | :-------: | :----------: |
	 *   | 4 bytes   | 2 bytes      |
	 *
	 */
	typedef struct {
		uint32_t SigfoxID;
		uint16_t device_class;
	} TD_SENSOR_LAN_RegisterFrame_t;

	/** Forward frame structure
	 * The LAN FORWARD frame is made of:
	 *
	 *   | Profile | SIGFOX Data |
	 *   | :-----: | :---------: |
	 *   | 4 bytes | 12 bytes    |
	 *
	 * The sigfox field contains the SIGFOX data payload to forward.
	 *
	 * The profile field is made of:
	 *
	 *   | Repetition | Interval |
	 *   | :--------: | :------: |
	 *   | 4 bits     | 28 bits  |
	 *
	 * */
	typedef struct {
		uint32_t profile;	/** Transmit Profile */
		uint8_t sigfox[12]; /** Sigfox data */
	} TD_SENSOR_LAN_ForwardFrame_t;

	/** Keepalive frame structure*/
	typedef struct {
		uint32_t interval;
		int8_t level_low;
		int8_t level_ok;
		bool keepalive;
		bool rssi;
		uint8_t voltage_level;
		uint8_t temp_level;
		bool battery;
	} TD_SENSOR_LAN_KeepAliveFrame_t;

	/** @} */

	/***************************************************************************
	 *************************   PROTOTYPES   **********************************
	 **************************************************************************/

	/** @addtogroup TD_SENSOR_LAN_GLOBAL_FUNCTIONS Global Functions
	 * @{ */

	DECLARE_DYNAMIC(bool, TD_SENSOR_LAN_Init, bool gateway,
		uint32_t lan_frequency, int16_t lan_power_level);
	void TD_SENSOR_LAN_DeclareFlashVariable(void);
	TD_SENSOR_LAN_AckCode_t TD_SENSOR_LAN_SendFrame(TD_SENSOR_LAN_Frame_t *frame,
		uint8_t *data_rx);
	TD_SENSOR_LAN_AckCode_t TD_SENSOR_LAN_SendFrameTo(uint32_t address,
		TD_SENSOR_LAN_FrameType_t type, uint8_t *payload, uint8_t count,
		uint8_t *data_rx);
	uint16_t TD_SENSOR_LAN_ComputeAddressTo16bits(uint32_t address);
	uint8_t TD_SENSOR_LAN_ComputeAddressTo8bits(uint32_t address);
	bool TD_SENSOR_LAN_setLanAddress(uint32_t adress, uint32_t mask);
	void TD_SENSOR_LAN_SetFrameRetry(uint8_t retry);
	uint8_t TD_SENSOR_LAN_GetFrameRetry(void);
	void TD_SENSOR_LAN_Stop(void);
	void TD_SENSOR_LAN_Restart(void);
	void TD_SENSOR_LAN_RestartTest(void);
	void TD_SENSOR_LAN_SetTxPeriod(uint32_t tx_period);

	/** @} */

	/** @addtogroup TD_SENSOR_LAN_USER_FUNCTIONS User Functions
	 * @{ */

	const TD_SENSOR_LAN_Address_t *TD_SENSOR_LAN_GetAddress(void);

	/** @} */

	/** @} (end addtogroup TD_SENSOR_LAN) */

#ifdef __cplusplus
}
#endif

#endif // __TD_SENSOR_LAN_H
