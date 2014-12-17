/***************************************************************************//**
 * @file
 * @brief Service Send
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

#ifndef __SENSOR_SEND_H
#define __SENSOR_SEND_H

#include <stdbool.h>
#include <stdint.h>

#include <efm32.h>
#include <em_assert.h>

#ifdef __cplusplus
extern "C" {
#endif

	/***********************************************************************//**
	 * @addtogroup SENSOR_SEND Sensor Send
	 * @{
	 **************************************************************************/

	/***************************************************************************
	 ************************   DEFINES   **************************************
	 **************************************************************************/

	/** @addtogroup SENSOR_REGISTER_DEFINES Defines
	 * @{ */

	/** Sensor header size in bytes */
#define TD_SENSOR_HEADER_SIZE		2

	/** Sensor payload size in bytes */
#define TD_SENSOR_PAYLOAD_SIZE		10

	/** @} */

	/***************************************************************************
	 ***********************   ENUMERATIONS   **********************************
	 **************************************************************************/

	/** @addtogroup SENSOR_SEND_ENUMERATIONS Enumerations
	 * @{ */

	/** Sensor Frame Type */
	typedef enum {
		SRV_FRM_EVENT = 0,
		SRV_FRM_DATA = 1,
		SRV_FRM_REGISTER = 2,
		SRV_FRM_KEEPALIVE = 3,
		SRV_FRM_RAW = 4,
		SRV_FRM_SERVICE = 5,
		SRV_FRM_GEOLOC = 6,
		SRV_FRM_PING = 7,
		SRV_FRM_MAX
	} TD_SENSOR_FrameType_t;

	/** @} */

	/***************************************************************************
	 **********************************  TYPEDEFS   ****************************
	 **************************************************************************/

	/** @addtogroup SENSOR_SEND_TYPEDEFS Typedefs
	 * @{ */

#pragma pack(1)
	/** Transmission Profile */
	typedef struct {
		uint8_t repetition : 4;					///< Number of repetitions
		uint32_t interval : 28;					///< Interval between repetitions in seconds
	} TD_SENSOR_TransmitProfile_t;
#pragma pack()

	/** Transmission configuration */
	typedef struct {
		TD_SENSOR_TransmitProfile_t profile;	///< Transmission profile
		uint8_t stamp;							///< Per Sensor type frame counter
	} TD_SENSOR_TransmitConfig_t;

	/** Sensor frame */
	typedef struct {
		uint8_t header[TD_SENSOR_HEADER_SIZE];
		uint8_t payload[TD_SENSOR_PAYLOAD_SIZE];
	} TD_SENSOR_Frame_t;

	/** @} */

	/***************************************************************************
	 **************************   PUBLIC VARIABLES   ***************************
	 **************************************************************************/

	/** @addtogroup SENSOR_SEND_GLOBAL_VARIABLES Global Variables
	 * @{ */
	/** @addtogroup SENSOR_SEND_EXTERN External Declarations
	 * @{ */

	/** Array of Sensor transmission profile indexed by frame tpye */
	extern TD_SENSOR_TransmitConfig_t TD_SENSOR_TransmitConfig[SRV_FRM_MAX];

	/** @} */
	/** @} */

	/***************************************************************************
	 *************************   PROTOTYPES   **********************************
	 **************************************************************************/

	/** @addtogroup SENSOR_SEND_GLOBAL_FUNCTIONS Global Functions
	 * @{ */

	bool TD_SENSOR_SendUDM(uint8_t id, TD_SENSOR_FrameType_t frame_type,
		TD_SENSOR_Frame_t *frame, uint8_t length);
	bool TD_SENSOR_Send(TD_SENSOR_TransmitProfile_t *profile,
		TD_SENSOR_FrameType_t frame_type, uint8_t stamp, uint8_t *payload,
		uint8_t length);
	void TD_SENSOR_SEND_SetUserCallback(bool (*user_callback)(uint8_t *payload,
		uint8_t count, uint8_t repetition, uint32_t interval));

	/***********************************************************************//**
	 * @brief
	 *   Set retransmission profile for each frame type.
	 *
	 * @param[in] type
	 * 	The Sensor frame type.
	 *
	 * @param[in] repetition
	 * 	Number of times the frame should be retransmitted.
	 *
	 * @param[in] interval
	 * 	Interval in seconds at which the retransmission should occur.Maximum
	 * 	600 s.
	 **************************************************************************/
	//TODO: Breaks API!!!
	static void __INLINE TD_SENSOR_SetTransmissionProfile(
		TD_SENSOR_FrameType_t type,
		uint8_t repetition, uint32_t interval)
	{
		TD_SENSOR_TransmitConfig[type].profile.repetition = repetition;
		TD_SENSOR_TransmitConfig[type].profile.interval = interval;
	}

	/** @} */

	/** @} (end addtogroup SENSOR_SEND) */

#ifdef __cplusplus
}
#endif

#endif // __SENSOR_SEND_H
