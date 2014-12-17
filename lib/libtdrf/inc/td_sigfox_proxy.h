/***************************************************************************//**
 * @file
 * @brief SIGFOX proxy API for the TDxxxx RF module.
 * @author Telecom Design S.A.
 * @version 1.0.0
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2014 Telecom Design S.A., http://www.telecomdesign.fr</b>
 ******************************************************************************
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

#ifndef __TD_SIGFOX_PROXY_H
#define __TD_SIGFOX_PROXY_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

	/***********************************************************************//**
	 * @addtogroup SIGFOX SIGFOX
	 * @brief SIGFOX API for the TDxxxx RF modules
	 * @{
	 **************************************************************************/

	/***************************************************************************
	 ***********************   ENUMERATIONS   **********************************
	 **************************************************************************/

	/** @addtogroup SIGFOX_ENUMERATIONS Enumerations
	 * @{ */

	/** SIGFOX transmit transport type */
	typedef enum {
		TD_SIGFOX_AUTO = 0,				///< Auto handling of proxy */
		TD_SIGFOX_USE_DIRECT,			///< Force direct transmission */
		TD_SIGFOX_USE_PROXY,			///< Force proxy transmission */
	} TD_SIGFOX_Transport_t;

	/** @} */

	/***************************************************************************
	 *************************   TYPEDEFS   ************************************
	 **************************************************************************/

	/** @addtogroup SIGFOX_TYPEDEFS Typedefs
	 * @{ */

	/** SIGFOX proxy config - not to be saved in flash */
	typedef struct {
		uint32_t retry_interval;		///< Interval to wait (ticks) between each retry_count
		uint32_t frame_low;				///< Frame to send for battery low event
		uint16_t class;					///< Proxy class = CONFIG_TD_SIGFOX_PROXY_CLASS
		uint8_t retry_count;			///< Number of time to retry to reach a nearby proxy
		uint8_t give_up;				///< Proxy give up count. 0 never give up.
		uint8_t failed_count;			///< Successive lan failed counter.
		bool monitor_battery;			///< Will send frame on battery state transition
		TD_SIGFOX_Transport_t transport;///< Current sigfox transport used.
	} TD_SIGFOX_ProxyConfig_t;

	/** SIGFOX proxy status - to be saved in flash */
	typedef struct {
		uint32_t address;				///< Proxy Address
		uint32_t id;					///< Proxy Unique ID
		uint32_t frequency;				///< RW window period
		uint32_t period;				///< RW window period in ticks
		uint16_t soft_version;			///< Proxy soft version
		uint8_t udm_release;			///< Proxy udm release
		uint8_t udm_subrelease;			///< Proxy udm sub release
		uint8_t temp;					///< Encoded local temperature
		uint8_t voltage;				///< Encoded local voltage
		bool battery;					///< True is ok
		bool paired;					///< True if paired to a proxy
		bool available; 				///< True if last proxy transmission succeeded
	} TD_SIGFOX_ProxyStatus_t;

	/** SIGFOX proxy status */
	typedef struct {
		TD_SIGFOX_ProxyConfig_t config;	///< SIGFOX proxy configuration
		TD_SIGFOX_ProxyStatus_t status; ///< SIGFOX proxy status
	} TD_SIGFOX_Proxy_t;

	/** @} */

	/***************************************************************************
	 *************************   PROTOTYPES   **********************************
	 **************************************************************************/

	/** @addtogroup SIGFOX_USER_FUNCTIONS User Functions
	 * @{ */

	void TD_SIGFOX_PROXY_SetTransportLayer(TD_SIGFOX_Transport_t transport);
	TD_SIGFOX_Transport_t TD_SIGFOX_PROXY_GetTransportLayer(void);
	bool TD_SIGFOX_PROXY_IsRxPending(void);
	const TD_SIGFOX_Proxy_t *TD_SIGFOX_PROXY_Get(void);
	void TD_SIGFOX_PROXY_SetBatteryMonitoring(bool enable, uint32_t frame_low);
	void TD_SIGFOX_PROXY_SetRetryInterval(uint8_t retries, uint32_t interval);
	void TD_SIGFOX_PROXY_ClearPairing(void);
	void TD_SIGFOX_PROXY_SetGiveUp(uint8_t give_up);
	bool TD_SIGFOX_PROXY_IsPaired(void);
	void TD_SIGFOX_PROXY_DefaultConfig(void);
	DECLARE_DYNAMIC(bool, TD_SIGFOX_PROXY_Send, uint8_t *data, uint8_t length,
		uint8_t *reply);
	void TD_SIGFOX_PROXY_Dump(void);

	/** @} */

	/** @addtogroup SIGFOX_GLOBAL_FUNCTIONS Global Functions
	 * @{ */
	DECLARE_DYNAMIC(bool,TD_SIGFOX_Init,bool init);
	DECLARE_DYNAMIC_ALT(bool, TD_SIGFOX_Init, TD_SIGFOX_PROXY_InitForward, bool init);
	DECLARE_DYNAMIC(void, TD_SIGFOX_PROXY_Init);
	DECLARE_DYNAMIC(void,TD_SIGFOX_PROXY_CompleteInit);
	DECLARE_DYNAMIC(bool,TD_SIGFOX_PROXY_Discover);
	DECLARE_DYNAMIC(bool, TD_SIGFOX_PROXY_Transmission, uint8_t size,
		uint8_t global_data_length, uint8_t retry, bool ack,
		uint32_t mem_sequence);
	DECLARE_DYNAMIC(bool, TD_SIGFOX_PROXY_ForwardTransmission,
		uint8_t global_data_length, uint8_t mode,  bool value, uint8_t *message,
		uint8_t size, uint8_t retry, bool ack, bool reserved);
	void TD_SIGFOX_PROXY_DownlinkAbort(void);
	DECLARE_DYNAMIC(bool, TD_SIGFOX_PROXY_Process);

	/** @} */

	/** @} (end addtogroup SIGFOX) */

#ifdef __cplusplus
}
#endif

#endif // __TD_SIGFOX_PROXY_H
