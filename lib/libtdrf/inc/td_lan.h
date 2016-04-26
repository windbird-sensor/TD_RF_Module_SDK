/***************************************************************************//**
 * @file
 * @brief Local RF for the TDxxxx RF modules.
 * @author Telecom Design S.A.
 * @version 3.2.0
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2012-2016 Telecom Design S.A., http://www.telecomdesign.fr</b>
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

#ifndef __TD_LAN_H
#define __TD_LAN_H

#include <stdint.h>
#include <stdbool.h>

#include <td_utils.h>

#ifdef __cplusplus
extern "C" {
#endif

	/***********************************************************************//**
	 * @addtogroup TD_LAN TD LAN
	 * @brief Local RF for the TDxxxx RF modules
	 * @{
	 **************************************************************************/

	/***************************************************************************
	 *************************   DEFINES   *************************************
	 **************************************************************************/

	/** @addtogroup LAN_DEFINES Defines
	 * @{ */

#define ACK_WIDTH				1									/**< Ack flag field width */
#define ACK_SHIFT				31									/**< Ack flag field shift */
#define FRAME_NUMBER_WIDTH		(ACK_SHIFT - CONFIG_LAN_ADDRESS_SIZE)	/**< Frame number field width */
#define FRAME_NUMBER_SHIFT		CONFIG_LAN_ADDRESS_SIZE				/**< Frame number field shift */
#define ADDRESS_WIDTH			CONFIG_LAN_ADDRESS_SIZE				/**< Address field width */
#define ADDRESS_SHIFT			0									/**< Address field shift */

	/** Macro to compute bit mask given its width */
#define MASK(w)					((1 << w) - 1)

#define GET_ACK(b)				GET_BITFIELD(b, ACK)				/**< Get the ack flag value */
#define SET_ACK(b, v)			SET_BITFIELD(b, ACK, v)				/**< Set the ack flag value */
#define GET_FRAME_NUMBER(b)		GET_BITFIELD(b, FRAME_NUMBER)		/**< Get the frame number value */
#define SET_FRAME_NUMBER(b, v)	SET_BITFIELD(b, FRAME_NUMBER, v)	/**< Set the frame number value */
#define GET_ADDRESS(b)			GET_BITFIELD(b, ADDRESS)			/**< Get the address value */
#define SET_ADDRESS(b, v)		SET_BITFIELD(b, ADDRESS, v)			/**< Set the address value */

	/** TD LAN preamble + synchro size */
#define TD_LAN_PRE_SIZE			9

	/** TD LAN payload size */
#define TD_LAN_PAYLOAD_SIZE		17

	/** TD LAN frame size */
#define TD_LAN_FRAME_SIZE		(TD_LAN_PAYLOAD_SIZE + sizeof (uint32_t))

	/** TD FastLAN payload size */
#define TD_FASTLAN_PAYLOAD_SIZE		(TD_LAN_PAYLOAD_SIZE - 1)

	/** Negative acknowledgment */
#define TD_LAN_NACK				0U

	/** Positive acknowledgment */
#define TD_LAN_ACK				1U

	/** Do LBT send, send default transmit count frame, go to receive mode
	 * (LBT = Listen Before Talk) */
#define TD_LAN_CNT_TRANSCEIVER	-1

	/** Do LBT send, send default transmit count frame */
#define TD_LAN_CNT_TRANSMITTER	0xFF

	/** Send one frame, go to receive mode */
#define TD_LAN_CNT_TRANSCEIVER_FAST	0xFE

	/** Do LBT ACK, send one frame */
#define TD_LAN_CNT_ACK			1

/** Infinite TD LAN receive mode */
#define TD_LAN_INFINITE		0xFFFFFFFF

/** Autoamtic TD LAN receive mode */
#define TD_LAN_AUTOMATIC		0

/** Set ACK mode*/
#define TD_LAN_ACK_SET			1

/** Reset ACK mode */
#define TD_LAN_ACK_RESET		2

/** Automatic ACK mode */
#define TD_LAN_ACK_AUTO			0

	/** @} */

	/***************************************************************************
	 ***********************   ENUMERATIONS   **********************************
	 **************************************************************************/

	/** @addtogroup LAN_ENUMERATIONS Enumerations
	 * @{ */

	/** LAN RF transaction modes */
	typedef enum {
		MODE_SLEEP,					/**< Idle mode */
		MODE_RECEIVER,				/**< Periodic windowed receive mode */
		MODE_TRANSCEIVER,			/**< LBT, Transmit mode, then switch to immediate receive mode */
		MODE_ACK,					/**< Immediate receive mode (no Windowing) */
		MODE_TRANSMITTER,			/**< Transmit mode */
		MODE_NOT_USED,				/**< Reserved mode */
		MODE_TRANSCEIVER_FAST,		/**< Fast Transceive mode (no LBT) */
		MODE_TRANSMITTER_CUSTOM,	/**< Custom Transmit mode (user callback write into fifo)*/
		MODE_RECEIVER_CUSTOM,		/**< Custom Receive mode (user callback write from fifo and stop rx)*/
		MODE_TRANSMITTER_FAST		/**< Fast Transmit mode (no LBT) */
	} TD_LAN_mode_t;

	/** LAN result states */
	typedef enum {
		RESULT_PENDING,				/**< Pending radio transaction */
		RESULT_COMPLETE,			/**< Radio transaction completed */
		RESULT_ABORT				/**< No or aborted radio transaction */
	} TD_LAN_result_t;

	/** LAN error code */
	typedef enum {
		ERROR_NONE,					/**< No error */
		ERROR_CANT_SWAP_RADIO,		/**< After a radio priority change, can't restore LAN radio state */
		ERROR_USER_ABORT,			/**< User have aborted radio transmission */
		ERROR_LAN_BUSY,				/**< LAN is busy (RSSI detected over timeout period) */
		ERROR_RECEIVE_TIMEOUT,		/**< Receive has timeout (no message available) */
		ERROR_TRANSMIT,				/**< Transmit error (no IRQ received) */
		ERROR_SYNC_TIMEOUT,			/**< User timeout during synchronous operation */
		ERROR_CANT_START_RECEIVE,	/**< A start receive failed. Radio chip probably messed up */
		ERROR_LAN_BUSY_ACK			/**< An ack reception failed and RSSI was high at start-up and end */
	} TD_LAN_error_t;

	/** @} */

	/***************************************************************************
	 *************************   TYPEDEFS   ************************************
	 **************************************************************************/

	/** @addtogroup LAN_TYPEDEFS Typedefs
	 * @{ */

	/** LAN RF frame structure
	 *
	 * @details
	 *   The TD_LAN frame consists in a a header and a user payload field:
	 *
	 *   | Header  | Payload  |
	 *   | :-----: | :------: |
	 *   | 4 bytes | 17 bytes |
	 *
	 *   The header field is further made up of several fields:
	 *
	 *   | Addr Low | Addr Med | Addr High | Ack   | Frame Counter |
	 *   | :------: | :------: | :-------: | :---: | :-----------: |
	 *   | 8 bits   | 8 bits   | 8 bits    | 1 bit | 7 bits        |
	 *
	 *   The 24-bit address field contains the sender source address for a
	 *   message or the destination address for an answer.
	 *
	 *   The Ack flag is set by the receiver to acknowledge a received message.
	 *
	 *   The frame counter is a monotonic upwards counter that is decremented
	 *   for each retransmission of a message frame during a single retry. This
	 *   counter is used by network devices to compute the delay until the end
	 *   of the current message.
	 *
	 *   Please note that the actual partition between the addres and frame
	 *   counter fields is adjustable using the LAN_ADDRESS_SIZE configuration
	 *   parameter in order to accommodate for longer transmission periods.
	 */

	typedef struct {
		uint32_t    header;					///< Header
		uint8_t     payload[TD_LAN_PAYLOAD_SIZE];	///< User payload
	} TD_LAN_frame_t;

	/** User LAN callback function that will be called when an ack frame is
	 * received with pointers to both the sent and received frames. */
	typedef int (*TD_LAN_callback_t)(TD_LAN_frame_t *tx_frame,
		TD_LAN_frame_t *rx_frame);

	/** LAN callback structure */
	typedef struct {
		TD_LAN_callback_t FrameReceived;	///< User callback, on frame received
		void (*Interrupt)(int);				///< User callback on chip interrupt
		void (*Start)(void);				///< User callback on mode start
		void (*LanWaitAckTimeout)(volatile TD_LAN_result_t *LanResult);	///< User callback, called when timeout is reached when waiting ack frame
	} TD_LAN_UserCallbacks_t;

	/** LAN configuration structure */
	typedef struct {
		uint32_t RxDelay; 					///< Delay before receive
		uint32_t RxPeriod; 					///< RF receive window period
		TD_LAN_UserCallbacks_t Callbacks;	///< User callbacks
		uint32_t Address; 					///< Local RF Address
		uint32_t Mask; 						///< Local RF mask
		uint32_t Match;						///< Local RF match
		uint32_t Frequency; 				///< Currently used frequency
		uint8_t *RXFrame;					///< Pointer to RX frame
		uint8_t *TXFrame;					///< Pointer to TX frame
		const uint8_t *RFConfig;			///< Pointer to RF Config
		int16_t PowerLevel; 				///< Currently used power level
		uint16_t FrameSize;					///< Frame size
		uint32_t Baudrate;					///< Baudrate
		bool LanActive; 					///< LAN Status flag
		bool LanSessionOpen;				///< LAN Session opened
		bool AbortRTCDelayOnRx;				///< Abort RTC Delay if packet received
	} TD_LAN_Configuration_t;

	/** LAN bidirectional exchange structure */
	typedef struct{
		TD_LAN_mode_t Mode;					///< Transfer mode type (LAN_Task state machine parameter)
		bool Synchronous;					///< If true, activate synchronous mode (wait TimeLimit max)
		uint32_t RxPeriod;					///< Time between reception attempt. If > 0 we are in window mode
		TD_LAN_frame_t *TxFrame;			///< Pointer to frame to transmit
		TD_LAN_frame_t *RxFrame;			///< Pointer to frame to receive
		bool Retry;							///< This transmission is a retry. Wait random time before beginning
		uint16_t TxCount;					///< Force specific Frame Count to transmit (0 : automatic - based on RxPeriod)
		uint32_t TimeLimit;					///< In synchronous mode, global timeout for reception (can be set to TD_LAN_INFINITE or TD_LAN_AUTOMATIC(default) = based on RxPeriod)
		uint8_t AckMode;					///< TD_LAN_ACK_SET (force ack mode), TD_LAN_ACK_RESET (reset ack mode), TD_LAN_ACK_AUTO (legacy mode, based on frame count)
		uint8_t MaxTxCount;					///< 0 : automatic, 0xFF : set MaxTxCount to 0, other : force MaxTxCount to value.
		uint32_t MaxTxListen;				///< 0 : automatic, other : force MaxTxListen.
		uint32_t DelayRx;					///< 0 : automatic, other : force delay before RX in transceiver mode.
		bool AbortRTCDelayOnRx;				///< 0 : standard, 1 : force a RTC delay abort on packet RX.
	} TD_LAN_ExchangeParam_t;

	/** @} */

	/***************************************************************************
	 *************************   PROTOTYPES   **********************************
	 **************************************************************************/

	/** @addtogroup LAN_USER_FUNCTIONS User Functions
	 * @{ */
	void TD_LAN_FastAck(TD_LAN_mode_t mode);
	bool TD_LAN_Exchange(TD_LAN_ExchangeParam_t *param);
	char *TD_LAN_ErrorToStr(TD_LAN_error_t error);
	bool TD_LAN_Init(bool init, uint32_t address, uint32_t mask);
	bool TD_LAN_InitExt(bool init, uint32_t address, uint32_t mask, uint32_t match);
	bool TD_LAN_Release(void);
	bool TD_LAN_Process(void);
	void TD_LAN_Abort(void);
	bool TD_LAN_SetFrequencyLevel(uint32_t frequency, int16_t level);
	bool TD_LAN_SetPeriod(uint32_t period);
	uint32_t TD_LAN_GetFrequency(void);
	int TD_LAN_ReadAFCFrequencyOffset(void);
	int16_t TD_LAN_GetPowerLevel(void);
	bool TD_LAN_isActive(void);
	uint32_t TD_LAN_PowerVoltageExtended(void);
	void TD_LAN_SetRFConfig(const uint8_t * config, uint32_t baudrate);
	void TD_LAN_SaveCurrentConfiguration(TD_LAN_Configuration_t *config);
	void TD_LAN_ApplyConfiguration(TD_LAN_Configuration_t *config);
	void TD_LAN_SetUserCallback(TD_LAN_callback_t callback);
	void TD_LAN_SetUserCallbackExt(TD_LAN_UserCallbacks_t *callbacks);
	TD_LAN_callback_t TD_LAN_GetUserCallback(void);
	uint32_t TD_LAN_ComputeAddress(uint32_t id);
	bool TD_LAN_SetAddress(uint32_t address, uint32_t mask);
	bool TD_LAN_SetAddressMatch(uint32_t address, uint32_t mask,
		uint32_t match);
	bool TD_LAN_SendReceive(int count, uint8_t retries,
		TD_LAN_frame_t *tx_frame, TD_LAN_frame_t *rx_frame);
	bool TD_LAN_SendFrame(int32_t txcount, TD_LAN_frame_t *tx_frame,
		TD_LAN_frame_t *rx_frame);
	bool TD_LAN_ReceiveFrame(uint32_t rx_period, uint32_t limit,
		TD_LAN_frame_t *rx_frame);
	bool TD_LAN_ReceiveFrameSync(TD_LAN_frame_t *rx_frame);
	bool TD_LAN_ReceiveFrameContinuousSync(TD_LAN_frame_t *rx_frame);
	int TD_LAN_ReadLatchedRSSI(void);
	TD_LAN_error_t TD_LAN_LastError(void);
	void TD_LAN_GetMinMaxPower(int16_t *min_power, int16_t *max_power);

	/** @} */

	/** @addtogroup LAN_GLOBAL_FUNCTIONS Global Functions
	 * @{ */

	bool TD_LAN_Start(TD_LAN_mode_t transfer_mode, uint32_t count,
		TD_LAN_frame_t *tx_frame, TD_LAN_frame_t *rx_frame);
	bool TD_LAN_Stop(TD_LAN_result_t result);
	void TD_LAN_Restart(void);
	void TD_LAN_Task(void);
	void TD_LAN_SetMaxWaitAck(int8_t waitack);
	bool TD_LAN_SendFrameCustom(uint32_t limit);
	bool TD_LAN_ReceiveFrameCustom(uint32_t rx_period, uint32_t limit);
	bool TD_LAN_SendFrameAsync(TD_LAN_frame_t *tx_frame, TD_LAN_frame_t *rx_frame);
	uint32_t TD_LAN_GetCallBackStartTime(void);
	void TD_LAN_Block(bool block);
	const uint8_t *TD_LAN_GetConfig9600(void);
	void TD_LAN_SetThreshold(int value);
	/** @} */

	/** @} (end addtogroup TD_LAN) */

#ifdef __cplusplus
}
#endif

#endif // __TD_LAN_H
