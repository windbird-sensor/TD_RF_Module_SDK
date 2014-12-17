/***************************************************************************//**
 * @file
 * @brief GPS management
 * @author Telecom Design S.A.
 * @version 1.1.0
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
#ifndef TD_GEOLOC_H_
#define TD_GEOLOC_H_

#include <stdbool.h>
#include <stdint.h>

#include <td_config_ext.h>

#include <ubx7.h>

#ifdef __cplusplus
extern "C" {
#endif

	/***********************************************************************//**
	 * @addtogroup TD_GEOLOC Geolocalization
	 * @{
	 **************************************************************************/

	/***************************************************************************
 ***********************   DEFINES   *******************************************
 ******************************************************************************/

/** @addtogroup TD_GEOLOC_DEFINES Defines
 * @{ */

/** Maximum number of satellites */
#define TD_GEOLOC_MAX_SATELLITES 16

/** @} */

/*******************************************************************************
	 ***********************   ENUMERATIONS   **********************************
	 **************************************************************************/

	/** @addtogroup TD_GEOLOC_ENUMERATIONS Enumerations
	 * @{ */

	/** GPS Power mode types */
	typedef enum {
		TD_GEOLOC_OFF = 0,
		TD_GEOLOC_HW_BCKP = 1,
		TD_GEOLOC_SOFT_BCKP = 2 ,
		TD_GEOLOC_NAVIGATION = 3,
		TD_GEOLOC_POWER_SAVE_MODE = 4,
		TD_GEOLOC_NAVIGATION_COLD_START = 5,	// Force cold start
		TD_GEOLOC_NAVIGATION_WARM_START = 6,	// Force warm start
		TD_GEOLOC_NAVIGATION_HOT_START = 7,		// Force hot start
		TD_GEOLOC_NAVIGATION_IDLE = 8			// Stop GNSS receive
	} TD_GEOLOC_PowerMode_t;

	/** GPS fix structure types */
	typedef enum {
		TD_GEOLOC_NO_FIX = 0,
		TD_GEOLOC_TIME_FIX = 1,
		TD_GEOLOC_DATE_FIX = 2,
		TD_GEOLOC_2D_FIX = 3,
		TD_GEOLOC_3D_FIX = 4
	} TD_GEOLOC_FixType_t;

	/** GPS fix special timeout values */
	typedef enum {
		TD_GEOLOC_INFINITE = 0xFFFF
	} TD_GEOLOC_FixTimeout_t;

	/** @} */

	/***************************************************************************
	 **************************  TYPEDEFS   ************************************
	 **************************************************************************/

	/** @addtogroup TD_GEOLOC_TYPEDEFS Typedefs
	 * @{ */

	/** Quality structure */
	typedef struct {
		uint16_t hdop;					///< Horizontal Degree Of Precision, 0 .. 20 * 100
		uint16_t hAcc;					///< Horizontal estimated Accuracy in meters
		uint8_t sat;					///< Number of satellites used for fix, maximum 20
	} TD_GEOLOC_Quality_t;


	/** Satellites structure */
	typedef struct {
		TD_UBX7_NAV_SVInfo_t sv_info; 	///< Sat info
		uint8_t usabled;				///< Computed value of usable satellites (>30)
		uint8_t used_autonomous;		///< Satellites used in solution based on autonomous
	} TD_GEOLOC_Satellites_t;

	/** Position structure, data stored as degrees, decimals minutes */
	typedef struct {
		int32_t latitude;				///< Latitude in 1/100000 degrees, South if < 0, North otherwise
		int32_t longitude;				///< Longitude in 1/100000 degrees, West if < 0, East otherwise
		int16_t altitude;				///< Altitude in meters above sea level
	} TD_GEOLOC_Position_t;

	/** Speed structure */
	typedef struct {
		uint16_t speed_knot;			///< Speed in knots
		uint16_t speed_kmh;				///< Speed in km/h
		int16_t true_track;				///< True track (unused)
		int16_t magnetic_track;			///< Magnetic track (unused)
	} TD_GEOLOC_Speed_t;

	/** Date & Time structure */
	typedef struct {
		uint16_t year;					///< Year UTC
		uint8_t seconds;				///< Seconds 0..59
		uint8_t minutes;				///< Minutes 0..59
		uint8_t hours;					///< Hours 0..23
		uint8_t day;					///< Days 1.31
		uint8_t month;					///< Month 1..12
	} TD_GEOLOC_DateTime_t;

	/** Hardware Status structure */
	typedef struct {
		bool rtc_calibrated;			///< RTC calibrated flag
	} TD_GEOLOC_HardwareStatus_t;

	/** Autonomous Status structure */
	typedef struct {
		uint32_t sats;					///< Bitfield indicating which sats have auto data av.
		bool idle;						///< Assist Now Autonomous idle flag
		bool enabled;					///< Assist Now Autonomous enabled flag
	} TD_GEOLOC_Autonomous_t;

	/** Full structure containing all updated informations */
	typedef struct {
		TD_GEOLOC_DateTime_t datetime;		///< Date & time information
		TD_GEOLOC_Position_t position;		///< Position information
		TD_GEOLOC_Speed_t speed;			///< Speed information
		TD_GEOLOC_Quality_t quality;		///< Quality information
		TD_GEOLOC_FixType_t type;			///< Type of fix information available
		TD_GEOLOC_Satellites_t sats;		///< Satellites in view detailed and computed information
		TD_GEOLOC_HardwareStatus_t hard;	///< Hardware status
		TD_GEOLOC_Autonomous_t autonomous;	///< Autonomous
		uint32_t duration;					///< Fix duration in seconds
	} TD_GEOLOC_Fix_t;

	/** Geoloc flash configuration */
	typedef struct {
		bool logger;					///< Logger status to save in flash
	} TD_GEOLOC_Config_t;

	/** Geoloc information saved by logger */
	typedef struct {
		TD_GEOLOC_Position_t position;	///< Position information
		TD_GEOLOC_DateTime_t datetime;	///< Date & time information
	} TD_GEOLOC_LogValue_t;

	/** Geoloc ephemeris entries */
	typedef struct {
		TD_UBX7_AID_Eph_t *ephemeris;	///< Pointer towards ephemeris array
		uint8_t count;					///< Valid ephemeris count
	} TD_GEOLOC_AidingEphemeris_t;

	/** @} */

	/***************************************************************************
	 *************************   PROTOTYPES   **********************************
	 **************************************************************************/

	/** @addtogroup TD_GEOLOC_GLOBAL_FUNCTIONS Global Functions
	 * @{ */

	void TD_GEOLOC_Dump(void);

	/** @} */

	/** @addtogroup TD_GEOLOC_USER_FUNCTIONS User Functions
	 * @{ */

	void TD_GEOLOC_Init(void);
	bool TD_GEOLOC_Process(void);
	void TD_GEOLOC_TryToFix(TD_GEOLOC_PowerMode_t mode, uint16_t timeout,
		void (*callback)(TD_GEOLOC_Fix_t *fix, bool timeout));
	void TD_GEOLOC_StopFix(TD_GEOLOC_PowerMode_t end_mode);
	void TD_GEOLOC_PrintfFix(TD_GEOLOC_Fix_t *fix);
	void TD_GEOLOC_FixInit(TD_GEOLOC_Fix_t *fix);
	DECLARE_DYNAMIC(void, TD_GEOLOC_Log, TD_GEOLOC_Fix_t *fix);
	void TD_GEOLOC_SetLogger(bool enable);
	void TD_GEOLOC_ResetLogger(void);
	bool TD_GEOLOC_ReadLog(TD_GEOLOC_LogValue_t *log);
	void TD_GEOLOC_PrintfFixLog(TD_GEOLOC_LogValue_t *log);
	uint32_t TD_GEOLOC_PowerVoltageExtended(void);
	void TD_GEOLOC_Dump(void);
	void TD_GEOLOC_AddSecondsToDateTime(TD_GEOLOC_DateTime_t * time,
		int32_t seconds);
	uint8_t TD_GEOLOC_PollEphemeris(TD_UBX7_NAV_SVInfo_t *sv_info,
		TD_UBX7_AID_Eph_t * ephemeris, uint8_t max);
	void TD_GEOLOC_SendEphemeris(TD_UBX7_AID_Eph_t * ephemeris, uint8_t count);
	bool TD_GEOLOC_EnableAutonomous(bool enable, uint8_t min_sv,
		uint8_t min_cno, bool force_3d, uint16_t max_error);
	uint8_t TD_GEOLOC_PollAutonomous(uint32_t sats,
		TD_UBX7_AID_Aop_t *autonomous, uint8_t max);
	void TD_GEOLOC_SendAutonomous(TD_UBX7_AID_Aop_t * autonomous,
		uint8_t count);
	void TD_GEOLOC_SendHui(uint8_t * data, uint32_t tow, uint16_t wno);
	/** @} */

/** @} */

	/** @} (end addtogroup TD_GEOLOC) */

#ifdef __cplusplus
}
#endif

#endif // TD_GEOLOC_H_
