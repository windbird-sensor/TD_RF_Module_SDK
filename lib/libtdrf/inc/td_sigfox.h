/***************************************************************************//**
 * @file
 * @brief SIGFOX V1 API for the TDxxxx RF modules.
 * @author Telecom Design S.A.
 * @version 5.2.0
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

	/** SIGFOX ITU ISM SRD region */
	typedef enum {
		REGION_UNKNOWN = 0,
		REGION_ETSI = 1,
		REGION_FCC = 2,
		REGION_ARIB = 3
	} TD_SIGFOX_region_t;

	/** SIGFOX transmit modes type */
	typedef enum {
		MODE_BIT = 0,					///< Single bit mode
		MODE_FRAME,						///< Byte frame mode
		MODE_OOB,						///< Out Of Band mode
		MODE_OOB_ACK					///< Out Of Band Acknowledge mode
	} TD_SIGFOX_mode_t;

	/** SIGFOX Modulation type */
	typedef enum {
		MOD_BPSK100 = 0,				///< BPSK 100 bps
		MOD_BPSK600						///< BPSK 600 bps
	} TD_SIGFOX_modulation_t;

	/** SIGFOX Frequency plan index */
	typedef enum {
		PLAN_868MHZ = 0,				///< 868 MHz
		PLAN_902MHZ,					///< 902 MHz
		PLAN_915MHZ						///< 915 MHz
	} TD_SIGFOX_plan_index_t;

	/** SIGFOX RF configuration */
	typedef struct {
		uint32_t central_frequency;		///< Central frequency
		uint16_t max_channel;			///< Maximum allowed channel number
		uint16_t channel_width;			///< Channel width in Hz
		uint8_t power_level;			///< Power level in dBm
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

	// TD_SIGFOX_SendV1_t already defined in td_config_ext.h so use "ALT" config
	// to just setp fonction
	DECLARE_DYNAMIC_ALT(bool, TD_SIGFOX_SendV1, TD_SIGFOX_SendV1, uint8_t mode,
		bool value, uint8_t *message, uint8_t size, uint8_t retry, bool ack,
		bool reserved);
	bool TD_SIGFOX_SendTest(uint16_t count, uint8_t time, uint16_t channel,
		uint8_t retry);
	bool TD_SIGFOX_RfPower(uint8_t power);
	uint8_t TD_SIGFOX_PowerVoltage(void);
	uint32_t TD_SIGFOX_PowerVoltageExtended(void);
	void TD_SIGFOX_KeepAlive(void);
	char const *TD_SIGFOX_VersionLib(void);
	uint32_t TD_SIGFOX_GetId(void);
	bool TD_SIGFOX_SetRegion(TD_SIGFOX_region_t region);
	TD_SIGFOX_region_t TD_SIGFOX_GetRegion(void);
	bool TD_SIGFOX_SetMacroChannel(int16_t macro_channel);
	void TD_SIGFOX_SetUplinkFrequency(uint32_t frequency);
	void TD_SIGFOX_SetDownlinkFrequency(uint32_t frequency);
	uint32_t TD_SIGFOX_GetUplinkFrequency(void);
	uint32_t TD_SIGFOX_GetDownlinkFrequency(void);
	int16_t TD_SIGFOX_GetMacroChannel(void);
	int8_t TD_SIGFOX_GetValidPower(int8_t *tab_valid_power, uint8_t max_len,
		uint8_t *return_type);
	int8_t TD_SIGFOX_GetDefaultPower(TD_SIGFOX_region_t region);
	bool TD_SIGFOX_CertifTest(uint16_t mode, uint16_t param1, uint16_t param2);
	bool TD_SIGFOX_SetPublicKey(bool public_id);
	bool TD_SIGFOX_GetPublicKey(void);

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
	int16_t TD_SIGFOX_ComputeChannel(int16_t channel, bool downlink,
		uint8_t repeat);
	bool TD_SIGFOX_PinConf(uint8_t config);
	bool TD_SIGFOX_SetKeepAliveRetries(uint8_t count);
	void TD_SIGFOX_SetPowerVoltageExtended(uint32_t voltage);
	bool TD_SIGFOX_SetTestModulation(uint8_t modulation);
	void TD_SIGFOX_SetReady(void);
	void TD_SIGFOX_Unlock(void);
	void TD_SIGFOX_GetRFConfig(TD_SIGFOX_RF_config_t *config);
	void TD_SIGFOX_SetRFConfig(TD_SIGFOX_RF_config_t *config);
	uint8_t *TD_SIGFOX_GetRawFrame(void);
	int16_t TD_SIGFOX_GetChannel(void);
	void TD_SIGFOX_SetChannel(int16_t channel);
	void TD_SIGFOX_ApplyChannel(uint16_t channel, bool downlink,
		bool modulation);
	void TD_SIGFOX_ApplyChannelExt(uint16_t channel, bool downlink,
			bool modulation, int32_t offset);
	void TD_SIGFOX_ComputeRawFrame(uint8_t tx_cpt, uint8_t size,
		uint8_t global_data_length);
	DECLARE_DYNAMIC(void, TD_SIGFOX_TestCarrier, uint32_t level,
		uint32_t frequency, bool high_performance);
	bool TD_SIGFOX_SendTestPA(uint16_t channel, uint8_t mode, uint32_t count,
		uint32_t freq);
	bool TD_SIGFOX_SendTestPA_carac(uint8_t max_amp, uint8_t max_level,
		uint32_t freq, uint32_t time);
	void TD_SIGFOX_AllowExtendedUse(uint32_t key);
	bool TD_SIGFOX_GetReady(void);
	DECLARE_DYNAMIC(void, TD_SIGFOX_InitPA, void);
	DECLARE_DYNAMIC_ALT(void, TD_SIGFOX_InitPA, TD_SIGFOX_InitWithoutPA, void);
	DECLARE_DYNAMIC_ALT(void, TD_SIGFOX_InitPA, TD_SIGFOX_InitWithoutPA_EZR, void);
	DECLARE_DYNAMIC(void, TD_SIGFOX_symbol_IRQ, uint16_t intFlags);
	bool TD_SIGFOX_SetIDKey(uint32_t id, uint8_t *key);
	bool TD_SIGFOX_SetMacroChannelBitmask(uint32_t *bitmask);
	bool TD_SIGFOX_SetDefaultMacroChannel(uint16_t ChannelDefault);
	bool TD_SIGFOX_ResetSequenceNumber(uint32_t key);

	/** @} */

	/** @} (end addtogroup SIGFOX) */

#ifdef __cplusplus
}
#endif

#endif // __TD_SIGFOX_H
