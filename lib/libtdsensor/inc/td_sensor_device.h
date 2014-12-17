/***************************************************************************//**
 * @file
 * @brief Sensor LAN Device
 * @author Telecom Design S.A.
 * @version 1.3.0
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

#ifndef __TD_SENSOR_DEVICE_H
#define __TD_SENSOR_DEVICE_H

#include <td_config_ext.h>

#include "td_sensor_gateway.h"
#include "td_sensor_lan.h"
#include "sensor_send.h"

#ifdef __cplusplus
extern "C" {
#endif

	/***********************************************************************//**
 * @addtogroup TD_SENSOR_DEVICE Sensor LAN Device
 * @brief
	 *	Devices LAN functions to Register, Forward Sensor Frames, send custom
	 *	Data and send Keep-alive to the gateway.
 *
 * @{
	 **************************************************************************/

	/***************************************************************************
	 **************************  DEFINES   *************************************
	 **************************************************************************/

/** @addtogroup TD_SENSOR_DEVICE_DEFINES Defines
 * @{ */

/** For compatibility */
#define TD_SENSOR_DEVICE_ForwardAsynch(p, c, r, i) \
		TD_SENSOR_DEVICE_Forward(p, c, r, i)

/** @} */


	/***************************************************************************
	****************************   TYPEDEFS   **********************************
	***************************************************************************/

/** @addtogroup TD_SENSOR_DEVICE_TYPEDEFS Typedefs
 * @{ */

/** Device queue message entry */
typedef struct {
	TD_SENSOR_LAN_Frame_t frame;///< Data frame to transmit
	bool tx_on_lan_fail;		///< Transfer over SIGFOX upon LAN failure flag
	uint8_t *data_rx;			///< Received data frame
} TD_SENSOR_DEVICE_DeviceQueueFrame_t;

/** @} */

	/***************************************************************************
	 *************************   PROTOTYPES   **********************************
	 **************************************************************************/

/** @addtogroup TD_SENSOR_DEVICE_GLOBAL_FUNCTIONS Global Functions
 * @{ */

DECLARE_DYNAMIC(int, TD_SENSOR_DEVICE_FrameReceived, TD_LAN_frame_t *tx_frame,
	TD_LAN_frame_t *rx_frame);
DECLARE_DYNAMIC(TD_SENSOR_LAN_AckCode_t, TD_SENSOR_DEVICE_Forward,
	uint8_t *payload, uint8_t count, uint8_t repetition, uint32_t interval);
DECLARE_DYNAMIC(void, TD_SENSOR_DEVICE_Process, void);
TD_SENSOR_LAN_AckCode_t TD_SENSOR_DEVICE_KeepAlive(bool keepalive,
	uint32_t interval, bool rssi, int8_t level_low, int8_t level_ok);

/** @} */

/** @addtogroup TD_SENSOR_DEVICE_USER_FUNCTIONS User Functions
 * @{ */

TD_SENSOR_LAN_AckCode_t TD_SENSOR_DEVICE_Data(uint8_t *data, uint8_t count,
	uint8_t *data_rx);
TD_SENSOR_LAN_AckCode_t TD_SENSOR_DEVICE_Register(void);
bool TD_SENSOR_DEVICE_isRegistered(void);
void TD_SENSOR_DEVICE_Reset(void);
void TD_SENSOR_DEVICE_SetDataCallback(int8_t (*user_data_callback)(
		bool broadcast, uint32_t address, uint8_t *data, uint8_t len,
		uint8_t *reply));
void TD_SENSOR_DEVICE_SetKeepAliveCallback(int8_t (*user_keepalive_callback)(
		bool broadcast, uint32_t address, uint32_t interval, int8_t rssi,
		uint8_t *data, uint8_t len, uint8_t *reply));
void TD_SENSOR_DEVICE_StartReception(void);
void TD_SENSOR_DEVICE_StartBroadcastReception(uint32_t mask);
void TD_SENSOR_DEVICE_StartNetworkReception(uint32_t mask);
bool TD_SENSOR_DEVICE_StartSynchReception(void);
void TD_SENSOR_DEVICE_StopReception(void);
bool TD_SENSOR_DEVICE_isReceptionEnabled(void);
bool TD_SENSOR_DEVICE_isBroadcastReceptionEnabled(void);
bool TD_SENSOR_DEVICE_isNetworkReceptionEnabled(void);
void TD_SENSOR_DEVICE_SetTxOnLanFail(bool enable);
bool TD_SENSOR_DEVICE_GetTxOnLanFail();
void TD_SENSOR_DEVICE_SetTxSkipLan(bool enable);
bool TD_SENSOR_DEVICE_GetTxSkipLan(void);
uint32_t TD_SENSOR_DEVICE_GetAddress(void);
TD_SENSOR_GATEWAY_Registration_t TD_SENSOR_DEVICE_GetRegistrationType(void);
void TD_SENSOR_DEVICE_SetUserCallback(void (*user_callback)(void));
void TD_SENSOR_DEVICE_SetTxRxPeriod(uint32_t tx_period, uint32_t rx_period);
bool TD_SENSOR_DEVICE_SetAddress(uint32_t address, uint32_t mask);
uint32_t TD_SENSOR_DEVICE_GetBroadcastMask(void);
void TD_SENSOR_DEVICE_SetAsynchronousForward(bool asynch);
bool TD_SENSOR_DEVICE_IsAsynchronousForward(void);

/** @} */

/** @} (end addtogroup TD_SENSOR_DEVICE) */

#ifdef __cplusplus
}
#endif

#endif // __TD_SENSOR_DEVICE_H
