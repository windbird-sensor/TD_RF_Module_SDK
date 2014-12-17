/***************************************************************************//**
 * @file
 * @brief SIGFOX V1 API for the TDxxxx RF modules.
 * @author Telecom Design S.A.
 * @version 4.0.0
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2012-2014 Telecom Design S.A., http://www.telecomdesign.fr</b>
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
#ifndef __TD_SIGFOX_H
#define __TD_SIGFOX_H

#include <stdint.h>
#include <stdbool.h>
#include <td_config_ext.h>

#include "td_sigfox_downlink.h"
#include "td_sigfox_proxy.h"

#ifdef __cplusplus
extern "C" {
#endif

	/***********************************************************************//**
	 * @addtogroup SIGFOX SIGFOX
	 * @brief SIGFOX API for the TDxxxx RF modules
	 * @{
	 **************************************************************************/

	/***************************************************************************
	 ***********************   TYPEDEFS   **************************************
	 **************************************************************************/

	/** @addtogroup SIGFOX_TYPEDEFS Typedefs
	 * @{ */

	/** SIGFOX transmit modes type */
	typedef enum {
		MODE_BIT = 0,				///< Single bit mode
		MODE_FRAME,					///< Byte frame mode
		MODE_OOB,					///< Out Of Band mode
		MODE_OOB_ACK				///< Out Of Band Acknowledge mode
	} TD_SIGFOX_mode_t;

	/** SIGFOX RF configuration */
	typedef struct {
		uint32_t central_frequency;	///< Central frequency
		uint16_t max_channel;		///< Maximum allowed channel number
		uint16_t channel_width;		///< Channel width in Hz
		uint8_t	power_level;		///< Power level in dBm
	} __PACKED TD_SIGFOX_RF_config_t;

	/** @} */

	/***************************************************************************
	 *************************   PROTOTYPES   **********************************
	 **************************************************************************/

	/** @addtogroup SIGFOX_USER_FUNCTIONS User Functions
	 * @{ */

	// Now declared in "td_sigfox_proxy.h"
	//DECLARE_DYNAMIC(bool,TD_SIGFOX_Init,bool init);
	DECLARE_DYNAMIC(bool, TD_SIGFOX_Send, uint8_t *message, uint8_t size,
		uint8_t retry);
	bool TD_SIGFOX_SendV1(uint8_t mode, bool value, uint8_t *message,
		uint8_t size, uint8_t retry, bool ack, bool reserved);
	bool TD_SIGFOX_SendTest(uint16_t count, uint8_t time, uint16_t slot,
		uint8_t retry);
	bool TD_SIGFOX_RfPower(uint8_t power);
	uint8_t TD_SIGFOX_PowerVoltage(void);
	uint32_t TD_SIGFOX_PowerVoltageExtended(void);
	void TD_SIGFOX_KeepAlive(void);
	char const *TD_SIGFOX_VersionLib(void);
	uint32_t TD_SIGFOX_GetId(void);

	/** @} */

	/** @addtogroup SIGFOX_GLOBAL_FUNCTIONS Global Functions
	 * @{ */

	DECLARE_DYNAMIC(uint8_t, TD_SIGFOX_ComputeFrameByMode, uint8_t mode,
		bool value, uint8_t *message, uint8_t size, bool ack, bool reserved);
	DECLARE_DYNAMIC(bool, TD_SIGFOX_DirectTransmission,
		uint8_t global_data_length, uint8_t size, uint8_t retry, bool ack);
	DECLARE_DYNAMIC(bool, TD_SIGFOX_SendRaw, uint8_t *buffer, uint16_t length,
		TD_SIGFOX_RF_config_t *config, bool *ack, uint32_t id);
	bool TD_SIGFOX_SendOOB(uint8_t *message, uint8_t size, uint8_t retry);
	bool TD_SIGFOX_FrequencyConf(uint32_t frequency, uint16_t allowed,
		uint16_t forbidden, uint16_t hop);
	uint16_t TD_SIGFOX_FrequencyHopping(uint16_t Slot, bool Apply,
		bool downlink);
	bool TD_SIGFOX_PinConf(uint8_t config);
	bool TD_SIGFOX_SetKeepAliveRetries(uint8_t count);
	void TD_SIGFOX_SetPowerVoltageExtended(uint32_t voltage);
	void TD_SIGFOX_SetReady(void);
	void TD_SIGFOX_Unlock(void);
	void TD_SIGFOX_GetRFConfig(TD_SIGFOX_RF_config_t *config);
	void TD_SIGFOX_SetRFConfig(TD_SIGFOX_RF_config_t *config);
	uint8_t *TD_SIGFOX_GetRawFrame(void);
	uint16_t TD_SIGFOX_GetChannel(void);
	void TD_SIGFOX_SetChannel(uint16_t channel);
	void TD_SIGFOX_ApplyChannel(uint16_t channel, uint32_t base_frequency);
	void TD_SIGFOX_ComputeRawFrame(uint8_t tx_cpt, uint8_t size,
		uint8_t global_data_length);
	DECLARE_DYNAMIC(void, TD_SIGFOX_TestCarrier, uint32_t level,
		uint32_t frequency, bool high_performance);
	bool TD_SIGFOX_SendTestPA(uint16_t slot, uint8_t mode, uint32_t count,
		uint32_t freq);
	void TD_SIGFOX_AllowExtendedUse(uint32_t key);

	/** @} */

	/** @} (end addtogroup SIGFOX) */

#ifdef __cplusplus
}
#endif

#endif // __TD_SIGFOX_H
