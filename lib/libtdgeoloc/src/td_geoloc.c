/***************************************************************************//**
 * @file
 * @brief GPS management for TD12xx RF modules.
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

#include <stdbool.h>
#include <stdint.h>

#include <em_gpio.h>

#include <td_core.h>
#include <td_gpio.h>
#include <td_rtc.h>
#include <td_flash.h>
#include <td_trap.h>
#include <td_module.h>
#include <td_scheduler.h>
#include <td_spi.h>
#include <td_printf.h>
#include <td_measure.h>
#include <td_utils.h>
#include <td_config_ext.h>
#include <td_uart.h>

#include "sensor_data.h"
#include "sensor_data_geoloc.h"
#include "nmea_parser.h"
#include "ubx_parser.h"
#include "ubx7.h"
#include "td_geoloc.h"

/***************************************************************************//**
 * @addtogroup TD_GEOLOC Geolocalization
 * @brief API for managing the on-board GPS chip
 * @{
 ******************************************************************************/

/*******************************************************************************
 ************************   DEFINES   ********************************
 ******************************************************************************/

/** @addtogroup TD_GEOLOC_DEFINES Defines
 * @{ */

//#define GEOLOC_DEBUG
//#define GEOLOC_DEBUG_TIME

/** Turn on trace mode if tfp_printf not commented */
#ifdef GEOLOC_DEBUG
#define DEBUG_PRINTF(...)		tfp_printf(__VA_ARGS__)
#else
#define DEBUG_PRINTF(...)
#endif

/** Amount of data being read at once on SPI Bus */
#define GEOLOC_BUFFER_DATA		64

/** Time in seconds on which the battery level is tested when GPS ON */
#define VOLTAGE_CHECK_INTERVAL	60

/** To enable nmea decoding */
#define DECODE_NMEA				0

/** @} */

/*******************************************************************************
 **************************  TYPEDEFS   ****************************************
 ******************************************************************************/

/** @addtogroup TD_GEOLOC_TYPEDEFS Typedefs
 * @{ */

/** Geoloc information saved by logger */
typedef struct {
	TD_UBX7_AidIni_t aid_ini;
	TD_UBX7_AID_Eph_t *ephemeris;
	uint32_t poll_time;
} TD_GEOLOC_Aiding_t;

typedef struct {
	uint16_t max_error;
	bool enable;
	uint8_t min_sv;
	uint8_t min_cno;
	bool force_3d;
	bool configured;
} TD_GEOLOC_AutonomousConfig_t;

/** @} */

/*******************************************************************************
 ************************   PRIVATE VARIABLES   ********************************
 ******************************************************************************/

/** @addtogroup TD_GEOLOC_LOCAL_VARIABLES Local Variables
 * @{ */

/** Contains Current position */
static TD_GEOLOC_Fix_t CurrentFix;

 /** Current GPS mode */
static TD_GEOLOC_PowerMode_t CurrentMode = TD_GEOLOC_OFF;

/** Timer ID for fix timeout */
static uint8_t FixTimer = 0xFF;

/** Configuration which must be saved in flash */
static TD_GEOLOC_Config_t Geoloc;

/** Logger flag for complete read */
static bool LogResetRead = false;

/** Current Fix start time */
static uint32_t StartFixTime = 0;

/** Last Voltage check time */
static uint32_t LastVoltageCheckTime = 0;

/** User Callback */
static void (*FixCallback)(TD_GEOLOC_Fix_t *fix, bool timeout) = 0;

/** Flag for fix timeout */
static volatile bool FixTimeout = false;
static volatile bool FirstFixTimeout = false;

/** Voltage level while using GPS */
static uint32_t PowerVoltageExtended = 0;

/** Aiding information including ephemeris and time */
static TD_GEOLOC_Aiding_t Aiding;

static TD_GEOLOC_AutonomousConfig_t AutonomousConfig;

static uint8_t UpdatedFields = 0;

/* Data to be filled by parser */
#if DECODE_NMEA
static TD_NMEA_GPGGA_t Gpgga;
static TD_NMEA_GPRMC_t Gprmc;
#endif

static TD_UBX7_MON_Hw_t MON_Hw;

static TD_UBX7_NAV_AopStatus_t NAV_AopStatus;

static TD_UBX7_NAV_Pvt_t NAV_Pvt;
/** @} */

/*******************************************************************************
 ************************   EXTERN  ********************************************
 ******************************************************************************/

/** @addtogroup TD_GEOLOC_EXTERN Extern
 * @{ */

/** End of Code in Flash memory from linker script */
extern const char __cs3_regions_end;

/** End of Flash memory from linker script */
extern const char __cs3_region_end_rom;

/** @} */

/*******************************************************************************
 **************************  PRIVATE FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup TD_GEOLOC_LOCAL_FUNCTIONS Local Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Read all structures updated by parser and fill the TD_GEOLOC_fix with it.
 *****************************************************************************/
static bool FixUpdate(void)
{
	unsigned int i;
	int16_t lat_deg, long_deg;

#if DECODE_NMEA

	// Copy GPPGA information into fix
	if (TD_NMEA_PARSER_IsMessageUpdated(TD_NMEA_GPGGA)) {
		CurrentFix.position.latitude = Gpgga.latitude;
		CurrentFix.position.longitude = Gpgga.longitude;
		CurrentFix.position.altitude = Gpgga.altitude;
		CurrentFix.quality.sat = Gpgga.sat;
		CurrentFix.quality.hdop = Gpgga.hdop;
		CurrentFix.datetime.hours = Gpgga.hour;
		CurrentFix.datetime.minutes = Gpgga.minute;
		CurrentFix.datetime.seconds = Gpgga.second;
	}

	// Copy GPRMC information into fix
	if (TD_NMEA_PARSER_IsMessageUpdated(TD_NMEA_GPRMC)) {
		CurrentFix.position.latitude = Gprmc.latitude;
		CurrentFix.position.longitude = Gprmc.longitude;
		CurrentFix.datetime.hours = Gprmc.hour;
		CurrentFix.datetime.minutes = Gprmc.minute;
		CurrentFix.datetime.seconds = Gprmc.second;
		CurrentFix.datetime.year = Gprmc.year;
		CurrentFix.datetime.month = Gprmc.month;
		CurrentFix.datetime.day = Gprmc.day;
		CurrentFix.speed.speed_knot = Gprmc.speed;

		// Compute speed in kmh
		if (Gprmc.speed != 0xFFFF) {

			// 1 knot = 1.852 km/h, but we pre-multiply by 2 to get
			// 1 more bit of accuracy for rounding purposes
			CurrentFix.speed.speed_kmh = ((Gprmc.speed << 1) * 1852) / 1000;

			// Rounding to nearest integer
			if (CurrentFix.speed.speed_kmh & 1) {
				CurrentFix.speed.speed_kmh++;
			}
			CurrentFix.speed.speed_kmh >>= 1;
		} else {
			CurrentFix.speed.speed_kmh = 0xFFFF;
		}
	}
#endif

	if (TD_UBX_PARSER_IsMessageUpdated(TD_UBX7_MON, TD_UBX7_MON_HW)) {
		UpdatedFields |= 1 << 0;
		CurrentFix.hard.rtc_calibrated = MON_Hw.flags&1;
	}
	if (TD_UBX_PARSER_IsMessageUpdated(TD_UBX7_NAV, TD_UBX7_NAV_SVINFO)) {
		UpdatedFields |= 1 << 1;
		CurrentFix.sats.used_autonomous = 0;
		CurrentFix.sats.usabled = 0;
		CurrentFix.quality.sat = 0;
		for (i = 0; i < CurrentFix.sats.sv_info.channel_count; i++) {

			// If used
			if (CurrentFix.sats.sv_info.channels[i].flags & 1) {
				CurrentFix.quality.sat++;
				//tfp_printf("Flag %d %04x\r\n",i,CurrentFix.sats.sv_info.channels[i].flags);
				if (CurrentFix.sats.sv_info.channels[i].flags & 0x40) {
					CurrentFix.sats.used_autonomous++;
				}
			}
			if(CurrentFix.sats.sv_info.channels[i].level > 30) {
				CurrentFix.sats.usabled++;
			}
		}
	}
	if (TD_UBX_PARSER_IsMessageUpdated(TD_UBX7_NAV, TD_UBX7_NAV_AOPSTATUS)) {
		UpdatedFields |= 1 << 2;
		CurrentFix.autonomous.idle = NAV_AopStatus.status ? false : true;
		CurrentFix.autonomous.enabled = NAV_AopStatus.aopCfg & 1 ? true : false;
		CurrentFix.autonomous.sats = NAV_AopStatus.availGPS;
	}
	if (TD_UBX_PARSER_IsMessageUpdated(TD_UBX7_NAV, TD_UBX7_NAV_PVT)) {
		UpdatedFields |= 1 << 3;

		// Valid time
		if (NAV_Pvt.valid & 2) {
			CurrentFix.datetime.hours = NAV_Pvt.hour;
			CurrentFix.datetime.minutes = NAV_Pvt.min;
			CurrentFix.datetime.seconds = NAV_Pvt.sec;
		} else {
			CurrentFix.datetime.hours = 0xFF;
			CurrentFix.datetime.minutes = 0xFF;
			CurrentFix.datetime.seconds = 0xFF;
		}

		// Valid data
		if (NAV_Pvt.valid & 1) {
			CurrentFix.datetime.year = NAV_Pvt.year;
			CurrentFix.datetime.month = NAV_Pvt.month;
			CurrentFix.datetime.day = NAV_Pvt.day;
		} else {
			CurrentFix.datetime.year = 0xFF;
			CurrentFix.datetime.month = 0xFF;
			CurrentFix.datetime.day = 0xFF;
		}

		// Valid flag
		if (NAV_Pvt.flags & 1) {
			CurrentFix.position.latitude = NAV_Pvt.lat;
			CurrentFix.position.longitude = NAV_Pvt.lon;

			// Convert to degree, minute decimal for historical reasons...

			// Get degrees
			lat_deg = NAV_Pvt.lat / 10000000;
			long_deg = NAV_Pvt.lon / 10000000;

			// Convert to decimal degrees
			CurrentFix.position.latitude = lat_deg * 10000000 +
				(((NAV_Pvt.lat - (lat_deg * 10000000)) * 60) / 100);
			CurrentFix.position.longitude = long_deg * 10000000 +
				(((NAV_Pvt.lon - (long_deg * 10000000)) * 60) / 100);
			CurrentFix.position.altitude = NAV_Pvt.height/1000;
			CurrentFix.quality.hAcc = NAV_Pvt.hAcc/1000;
			if (NAV_Pvt.gSpeed >= 0) {
				CurrentFix.speed.speed_kmh =  (((uint32_t)NAV_Pvt.gSpeed)* 3600)
					/ 1000000;
			} else {
				CurrentFix.speed.speed_kmh = (((uint32_t)(-NAV_Pvt.gSpeed)) *
					3600) / 1000000;
			}
			CurrentFix.speed.speed_knot = (CurrentFix.speed.speed_kmh * 1000) /
				1852;
			CurrentFix.quality.hdop = NAV_Pvt.pDop;
			CurrentFix.quality.sat = NAV_Pvt.numSv;
		} else {
			CurrentFix.position.altitude = 0x7FFF;
			CurrentFix.position.longitude = 0x7FFFFFFF;
			CurrentFix.position.latitude = 0x7FFFFFFF;
			CurrentFix.quality.hAcc = 0;
			CurrentFix.speed.speed_kmh = 0xFFFF;
			CurrentFix.quality.hdop = 0xFFFF;
			CurrentFix.quality.sat = 0;
		}
	}

	// Only call user callback when each fields have been updated once
	// Avoid shift in information such as sv used/sv autonomous
	if (UpdatedFields == 0x0F) {
		UpdatedFields = 0;
		return true;
	}
	return false;
}

/***************************************************************************//**
 * @brief
 *   Find current fix type according to latitude/longitude/sat values.
 *
 * @param [in] fix
 *   Structure containing fix values/
 *****************************************************************************/
static void SetFixType(TD_GEOLOC_Fix_t *fix)
{
	if (fix->position.latitude != 0x7FFFFFFF ||
		fix->position.longitude != 0x7FFFFFFF) {
		if (fix->quality.sat == 3) {
			fix->type = TD_GEOLOC_2D_FIX;
		} else if (fix->quality.sat >= 4) {
			fix->type = TD_GEOLOC_3D_FIX;
		} else {
			fix->type = TD_GEOLOC_NO_FIX;
		}
	} else if (fix->datetime.year != 0xFF &&
		fix->datetime.month != 0xFF &&
		fix->datetime.day != 0xFF) {
		fix->type = TD_GEOLOC_DATE_FIX;
	} else if (fix->datetime.hours != 0xFF &&
		fix->datetime.minutes != 0xFF &&
		fix->datetime.seconds != 0xFF) {
		fix->type = TD_GEOLOC_TIME_FIX;
	} else {
		fix->type = TD_GEOLOC_NO_FIX;
	}
}

/***************************************************************************//**
 * @brief
 *   Change GPS mode if different from current mode.
 *
 * @param[in] mode
 *   New GPS mode.
 *****************************************************************************/
static void SetPowerMode(TD_GEOLOC_PowerMode_t mode)
{
#ifdef GEOLOC_DEBUG_TIME
	uint32_t cnt = RTC->CNT;
	tfp_printf("SetPowerMode:%d\r\n", mode);
#endif

	if (mode != CurrentMode) {
		switch (mode) {
		case TD_GEOLOC_OFF:
			TD_UBX7_PowerOff();
			break;

		case TD_GEOLOC_HW_BCKP:
			TD_UBX7_HardwareBackup();
			break;

		case TD_GEOLOC_SOFT_BCKP:
			//TODO:
			//saveAiding
			//saveephemeris
			break;

		case TD_GEOLOC_POWER_SAVE_MODE:
		case TD_GEOLOC_NAVIGATION:
		case TD_GEOLOC_NAVIGATION_COLD_START:
		case TD_GEOLOC_NAVIGATION_WARM_START:
		case TD_GEOLOC_NAVIGATION_HOT_START:
		case TD_GEOLOC_NAVIGATION_IDLE:
			if (!TD_UBX7_PowerUp(mode == TD_GEOLOC_POWER_SAVE_MODE, false)) {
				TD_Trap(TRAP_GPS_HARD_ERR, 0);
			}
			if (mode == TD_GEOLOC_NAVIGATION_COLD_START) {
				TD_UBX7_ConfigureStartMode(TD_UBX7_COLD);
				mode = TD_GEOLOC_NAVIGATION;
			} else if (mode == TD_GEOLOC_NAVIGATION_WARM_START) {
				TD_UBX7_ConfigureStartMode(TD_UBX7_WARM);
				mode = TD_GEOLOC_NAVIGATION;
			} else if (mode == TD_GEOLOC_NAVIGATION_HOT_START) {
				TD_UBX7_ConfigureStartMode(TD_UBX7_HOT);
				mode = TD_GEOLOC_NAVIGATION;
			} else if (mode == TD_GEOLOC_NAVIGATION_IDLE) {
				TD_UBX7_StartGPS(false);
				mode = TD_GEOLOC_NAVIGATION;
			} else if (CurrentMode == TD_GEOLOC_SOFT_BCKP) {

				//TODO:
				//send aiding
				//send ephemeris

			}
			break;
		}
		CurrentMode = mode;
	}

#ifdef GEOLOC_DEBUG_TIME
	if (((RTC->CNT - cnt) & 0xFFFFFF) > 10) {
		tfp_printf("Set power mode%d:%d\r\n", mode, (RTC->CNT - cnt) & 0xFFFFFF);
	}
#endif
}

/***************************************************************************//**
 * @brief
 *  Printf fix information as GPGGA
 *
 * @param[in] fix
 *  Pointer to fix structure to be printed
 ******************************************************************************/
static void PrintfGPGGA(TD_GEOLOC_Fix_t *fix)
{
	char position[100];
	int checksum = 0;
	int32_t temp;
	long calc;
	int i;
	char ctemp[4] = ",N,";
	ctemp[3] = 0;

	if (fix->datetime.hours == 0xFF ||
		fix->datetime.minutes == 0xFF ||
		fix->datetime.seconds == 0xFF) {
		// tfp_sprintf(position,"$GPGGA,,");
		position[7] = ',';
		position[7] = 0;
	} else {
		tfp_sprintf(position, "$GPGGA,%02d%02d%02d.00,",
					fix->datetime.hours,
					fix->datetime.minutes,
					fix->datetime.seconds);
	}
	for (i = 1; position[i] != 0; i++) {
		checksum ^= position[i];
	}
	tfp_printf(position);
	temp = fix->position.latitude;
	if (temp < 0) {
		ctemp[1] = 'S';
		temp = -temp;
	}
	// tfp_printf("a%d\r\n",temp);
	if (temp != 0x7FFFFFFF) {
		calc = temp / 100000;
		tfp_sprintf(position, "%04d.%03d", calc, temp - (calc * 100000));
		for (i = 0; position[i] != 0; i++) {
			checksum ^= position[i];
		}
		tfp_printf(position);
	}
	tfp_sprintf(position, ctemp);
	for (i = 0; position[i] != 0; i++) {
		checksum ^= position[i];
	}
	tfp_printf(position);
	temp = fix->position.longitude;
	ctemp[1] = 'E';
	if (temp < 0) {
		ctemp[1] = 'W';
		temp = -temp;
	}
	// tfp_printf("a%d\r\n",temp);
	if (temp != 0x7FFFFFFF) {
		calc = temp / 100000;
		tfp_sprintf(position, "%05d.%03d", calc, temp - (calc * 100000));
		for (i = 0; position[i] != 0; i++) {
			checksum ^= position[i];
		}
		tfp_printf(position);
	}
	tfp_sprintf(position, ctemp);
	for (i = 0; position[i] != 0; i++) {
		checksum ^= position[i];
	}
	tfp_printf(position);
	if (fix->position.altitude != 0x7FFF) {
		tfp_sprintf(position,
					"1,%02d,%d,%d,M,0,M,,",
					fix->quality.sat,
					fix->quality.hdop / 100,
					fix->position.altitude);
	} else {
		tfp_sprintf(position,
					"1,%02d,%d,,M,0,M,,",
					fix->quality.sat,
					fix->quality.hdop / 100);
	}
	for (i = 0; position[i] != 0; i++) {
		checksum ^= position[i];
	}
	tfp_printf(position);
	tfp_sprintf(position, "*%02x\r\n", checksum);
	tfp_printf(position);
	tfp_printf("\r\n");
}

/***************************************************************************//**
 * @brief
 *  Convert a date to day index.
 *
 * @param[in] year
 *  Number of year.
 *
 * @param[in] month
 *  Number of month.
 *
 * @param[in] day
 *  Number of days.
 *
 * @return
 *   Returns the day index.
 *
 * @see http://alcor.concordia.ca/~gpkatch/gdate-algorithm.html
 ******************************************************************************/
// Return a day index from a date
// http://alcor.concordia.ca/~gpkatch/gdate-algorithm.html
static int DateToDay(int year, int month, int day)
{
	month = (month + 9) % 12;
	year = year - month / 10;
	return 365 * year + year / 4 - year / 100 + year / 400 + (month * 306 + 5) /
		10 + (day - 1);
}

/***************************************************************************//**
 * @brief
 *  Convert a day index to a date.
 *
 * @param[in] index
 *  Day index to convert.
 *
 * @param[out] year
 *  Pointer to a buffer that will receive the converted year.
 *
 * @param[out] month
 *  Pointer to a buffer that will receive the converted month.
 *
 * @param[out] day
 *  Pointer to a buffer that will receive the converted day.
 *
 * @see http://alcor.concordia.ca/~gpkatch/gdate-algorithm.html
 ******************************************************************************/
static void DayToDate(int index, uint16_t *year, uint8_t *month, uint8_t *day)
{
	int y, m, d;
	int mi, dd;

	y = (10000 * index + 14780) / 3652425;
	dd = index - (365 * y + y / 4 - y / 100 + y / 400);
	if (dd < 0) {
		y = y - 1;
		dd = index - (365 * y + y / 4 - y / 100 + y / 400);
	}
	mi = (100 * dd + 52) / 3060;
	m = (mi + 2) % 12 + 1;
	y = y + (mi + 2) / 12;
	d = dd - (mi * 306 + 5) / 10 + 1;
	*year = y;
	*month = m;
	*day = d;
}

/***************************************************************************//**
 * @brief
 *  Convert time variables to number of seconds.
 *
 * @param[in] h
 *  Number of hours.
 *
 * @param[in] m
 *  Number of minutes.
 *
 * @param[in] s
 *  Number of seconds.
 *
 * @result
 *  Retuns the total number of seconds.
 ******************************************************************************/
static int TimeToSec(int h, int m, int s)
{
	return h * 3600 + m * 60 + s;
}

/***************************************************************************//**
 * @brief
 *  Convert time in seconds to time variables.
 *
 * @param[in] time
 *  Time in seconds.
 *
 * @param[out] hours
 *  Pointer that will receive the number of hours.
 *
 * @param[out] minutes
 *  Pointer that will receive the number of minutes.
 *
 * @param[out] seconds
 *  Pointer that will receive the number of remaining seconds.
 ******************************************************************************/

static void SecToTime(int time, uint8_t *hours, uint8_t *minutes,
	uint8_t *seconds)
{
	*hours = time / 3600;
	time -= (*hours) * 3600;
	*minutes = time / 60;
	time -= (*minutes) * 60;
	*seconds = time;
}

/***************************************************************************//**
 * @brief
 *   Process NMEA updates by updating fix type and duration and push
 *   it to the user callback.
 ******************************************************************************/
static void OnUpdate(void)
{
	uint32_t now = (TD_SCHEDULER_GetTime() >> 15);

	// Set fix duration and type
	CurrentFix.duration = (now - StartFixTime);

	// Measure battery level every minute
	if (LastVoltageCheckTime - now > VOLTAGE_CHECK_INTERVAL) {
		PowerVoltageExtended = TD_MEASURE_VoltageExtended();
		LastVoltageCheckTime = now;
	}
	SetFixType(&CurrentFix);
	if (FixCallback != 0) {
		FixCallback(&CurrentFix, FixTimeout);
	}
}

/***************************************************************************//**
 * @brief
 *  Fix timeout handler. Set the Fix timeout flag to true.
 *
 * @param[in] arg
 *  Unused.
 *
 * @param[in] repeat_count
 *  Unused.
 *****************************************************************************/
static void OnFixTimeout(uint32_t arg, uint8_t repeat_count)
{
	FixTimer = 0xFF;
	if (FixCallback != 0) {
		FixTimeout = true;
		FirstFixTimeout = true;
	}
}

/** @} */

/*******************************************************************************
 **************************  PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup TD_GEOLOC_GLOBAL_FUNCTIONS Global Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *  Dump GPS state.
 ******************************************************************************/
void TD_GEOLOC_Dump(void)
{
	TD_UBX7_Dump();
}

/** @} */

/** @addtogroup TD_GEOLOC_USER_FUNCTIONS User Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Start the GPS and try to get a fix. If the GPS is already running then no
 *   effect.
 *
 * @param[in] mode
 *   Power mode
 *
 * @param[in] timeout
 *  timeout in seconds. On timeout expiration the timeout parameter of the
 *  callback will be set to true. No further action is being performed and
 *  shutting down the GPS or not is left to the user application and should be
 *  done using TD_GEOLOC_StopFix.
 *
 * @param[in] callback
 *   Pointer to a function that will be called each time a parameter
 *   is being updated (time, position, speed, etc..).
 *   Fix is a pointer to a structure containing fix information.
 *   Timeout will be set to true when the specified timeout expired
 *
 *
 ******************************************************************************/
void TD_GEOLOC_TryToFix(TD_GEOLOC_PowerMode_t mode, uint16_t timeout,
	void (*callback)(TD_GEOLOC_Fix_t *fix, bool timeout))
{
	if (callback != 0) {
		FixCallback = callback;
		StartFixTime = TD_SCHEDULER_GetTime() >> 15;
		TD_GEOLOC_FixInit(&CurrentFix);
		SetPowerMode(mode);

		// Enable NAV sol message
		if (TD_UBX7_ConfigureMessageRate(TD_UBX7_NAV, TD_UBX7_NAV_PVT, 1) !=
			TD_UBX7_CONFIG_ACKED) {
			//TODO: trap
		}

		// Enable SVINFO message
		if (TD_UBX7_ConfigureMessageRate(TD_UBX7_NAV, TD_UBX7_NAV_SVINFO, 1) !=
			TD_UBX7_CONFIG_ACKED) {
			//TODO: trap
		}

		// Enable RTC calibrated message
		if (TD_UBX7_ConfigureMessageRate(TD_UBX7_MON, TD_UBX7_MON_HW, 1) !=
			TD_UBX7_CONFIG_ACKED) {
			//TODO: trap
		}

		// Enable autonomous status message
		if (TD_UBX7_ConfigureMessageRate(TD_UBX7_NAV, TD_UBX7_NAV_AOPSTATUS, 1) !=
			TD_UBX7_CONFIG_ACKED) {
			//TODO: trap
		}

		// Apply autonomous config
		if (AutonomousConfig.configured) {
			TD_GEOLOC_EnableAutonomous(AutonomousConfig.enable,
				AutonomousConfig.min_sv,
				AutonomousConfig.min_cno,
				AutonomousConfig.force_3d,
				AutonomousConfig.max_error);
		}

		FixTimeout = false;
		FirstFixTimeout = false;
		if (timeout != TD_GEOLOC_INFINITE && FixTimer == 0xFF) {
			FixTimer = TD_SCHEDULER_AppendIrq(timeout,
				0,
				0,
				TD_SCHEDULER_ONE_SHOT,
				OnFixTimeout,
				0);
		}
	}
}

/***************************************************************************//**
 * @brief
 *   Stop the current fix timer if activated and switch to specified power mode.
 *
 * @param[in] end_mode
 *   GPS mode to switch to.
 *****************************************************************************/
void TD_GEOLOC_StopFix(TD_GEOLOC_PowerMode_t end_mode)
{
	if (FixTimer != 0xFF) {
		TD_SCHEDULER_Remove(FixTimer);
		FixTimer = 0xFF;
	} else {

		// May happen if fix timeout but user choose to keep it running
		// or if no timeout was set
	}
	if (end_mode == TD_GEOLOC_SOFT_BCKP) {
		//TD_GEOLOC_SaveAiding();
		end_mode = TD_GEOLOC_OFF;
	}

	// If full stop gps and autonomous disabled, don't configure it on next
	// startup
	if (end_mode == TD_GEOLOC_OFF && !AutonomousConfig.enable) {
		AutonomousConfig.configured = false;
	}
	SetPowerMode(end_mode);
}

/***************************************************************************//**
 * @brief
 *   Parse the GPS for incoming NMEA data if available and call user callback on
 *   updates. Must be called in User_Loop for proper GPS operation.
 *****************************************************************************/
bool TD_GEOLOC_Process(void)
{
	bool process, ret;

	ret = false;
	// Poll NMEA from GPS chip if on
	if (CurrentMode == TD_GEOLOC_NAVIGATION ||
		CurrentMode == TD_GEOLOC_POWER_SAVE_MODE) {

		// Read data from ublox
		ret = TD_UBX7_Process();

		// Execute callback if fix update
		process = TD_UBX_PARSER_Process();
		process |= TD_NMEA_PARSER_Process();
		if (FirstFixTimeout) {
			FirstFixTimeout = false;
			OnUpdate();
		}
		if (process) {
			if (FixUpdate() ) {
				OnUpdate();
			}
		}
	}
	return ret;
}

/***************************************************************************//**
 * @brief
 *   Enables or disables GPS logger.
 *
 * @param[in] enable
 *   True to enable, false to disable.
 *****************************************************************************/
void TD_GEOLOC_SetLogger(bool enable)
{
	if (Geoloc.logger != true && enable) {
		TD_FLASH_InitLogger(true,
			1,
			sizeof (TD_GEOLOC_LogValue_t),
			(uint32_t) &__cs3_regions_end,
			(uint32_t) &__cs3_region_end_rom);
	}
	Geoloc.logger = enable;
}

/***************************************************************************//**
 * @brief
 *   Reset GPS logger.
 *****************************************************************************/
void TD_GEOLOC_ResetLogger(void)
{
	TD_FLASH_InitLogger(true,
		1,
		sizeof (TD_GEOLOC_LogValue_t),
		(uint32_t) &__cs3_regions_end,
		(uint32_t) &__cs3_region_end_rom);
}

/***************************************************************************//**
 * @brief
 *  Read each log value one by one.
 *
 * @param log
 *  Pointer to log structure (position and date) to read from flash
 *
 *  @return
 *   return true if all log has been read
 *
 *****************************************************************************/
bool TD_GEOLOC_ReadLog(TD_GEOLOC_LogValue_t *log)
{
	bool result = false;

	if (!LogResetRead) {
		TD_FLASH_LoggerResetRead(1);
		LogResetRead = true;
	}
	result = TD_FLASH_LoggerReadNext(1, (uint32_t *) log);

	// Everything has been read
	if (!result) {
		LogResetRead = false;
	}
	return result;
}

/***************************************************************************//**
 * @brief
 *  Log a fix position and time into flash.
 *
 * @param fix
 *  Pointer to fix to save.
 ******************************************************************************/
void DYNAMIC(TD_GEOLOC_Log)(TD_GEOLOC_Fix_t *fix)
{
	TD_GEOLOC_LogValue_t log;

	// Copy interesting values
	memcpy(&log.position, &fix->position, sizeof (TD_GEOLOC_Position_t));
	memcpy(&log.datetime, &fix->datetime, sizeof (TD_GEOLOC_DateTime_t));

	// Write in flash
	TD_FLASH_LoggerWrite(1, (uint32_t *) &log);
}

/***************************************************************************//**
 * @brief
 *  Print a fix log as GPGGA.
 *
 * @param log
 *  Pointer to log value to print.
 ******************************************************************************/
void TD_GEOLOC_PrintfFixLog(TD_GEOLOC_LogValue_t *log)
{
	TD_GEOLOC_Fix_t fix;

	memcpy(&fix.position, &log->position, sizeof (TD_GEOLOC_Position_t));
	memcpy(&fix.datetime,&log->datetime, sizeof (TD_GEOLOC_DateTime_t));
	PrintfGPGGA(&fix);
}

/***************************************************************************//**
 * @brief
 *  Print a fix as GPGGA.
 *
 * @param fix
 *  Pointer to fix value to print.
 ******************************************************************************/
void TD_GEOLOC_PrintfFix(TD_GEOLOC_Fix_t *fix)
{
	PrintfGPGGA(fix);
}

/***************************************************************************//**
 * @brief
 *   Retrieve the power supply voltage measured during current GPS session if
 *   GPS on or last GPS sesion if off. When ON this value is outdated of
 *   maximum VOLTAGE_CHECK_INTERVAL in seconds.
 *
 * @return
 *   Returns the last measured power supply voltage value in mV.
 ******************************************************************************/
uint32_t TD_GEOLOC_PowerVoltageExtended(void)
{
	return PowerVoltageExtended;
}

/***************************************************************************//**
 * @brief
 *   Enable or disable autonomous calculation. If enabled one must wait until
 *   hard-> autonomous_idle is true before stopping gps in order to properly
 *   use autonomous capabilities.
 *
 *   If gps is on will send command right away, otherwise command will be send
 *   on next start-up
 *
 * @param enable
 *  true to enable autonomous, false to disable.
 *
 * @param min_sv
 *  Minimum number of sat to allow using autonomous
 *
 * @param min_cno
 *  Minimum cno in dbHz for a sat to allow using autonomous. Default to 6.
 *
 * @param force_3d
 *  Set to true to only allow 3D autonomous fix
 *
 * @param max_error
 *  Max acceptable error due to autonomous calculation in meter. Max is 300
 ******************************************************************************/
bool TD_GEOLOC_EnableAutonomous(bool enable, uint8_t min_sv, uint8_t min_cno,
	bool force_3d, uint16_t max_error)
{
	uint8_t AidHui[72];

	if (CurrentMode > TD_GEOLOC_SOFT_BCKP) {

		// Enable/disable autonomous
		if (TD_UBX7_ConfigureAutonomous(enable, min_sv, min_cno, force_3d,
			max_error) != TD_UBX7_CONFIG_ACKED) {
			return false;
		}
		if (enable) {

			// Fake leap seconds by saying utc valid to enable autonomous
			// TODO: real leap second should be sent
			memset(AidHui, 0, 72);
			TD_GEOLOC_SendHui(AidHui, 0, 0);
		}
	}
	AutonomousConfig.configured = true;
	AutonomousConfig.enable = enable;
	AutonomousConfig.min_sv = min_sv;
	AutonomousConfig.min_cno = min_cno;
	AutonomousConfig.force_3d = force_3d;
	AutonomousConfig.max_error = max_error;
	return true;
}

/***************************************************************************//**
 * @brief
 *  Appends seconds to a datetime and compute the updated time.
 *
 * @param[in] time
 *  Pointer to datetime structure to update.
 *
 * @param[in] seconds
 *  Seconds to append.
 *
 * @note Seconds can be negative (remove) or positive (append)
 ******************************************************************************/
void TD_GEOLOC_AddSecondsToDateTime(TD_GEOLOC_DateTime_t *time, int32_t seconds)
{
	int days;
	int day_index;
	bool add = true;
	int time_index;

	// Add or remove?
	if (seconds < 0) {
		add = false;
		seconds = -seconds;
	}

	// Convert seconds to seconds / minutes / hours / days;
	days = seconds / 86400;
	seconds -= days * 86400;

	// Seconds now only contains less that one day
	// Get time as day index and second index
	day_index = DateToDay(time->year - 2000, time->month, time->day);
	time_index = TimeToSec(time->hours, time->minutes, time->seconds);

	// Compute time index
	if (add) {
		time_index += seconds;

		// If one more day
		if (time_index >= 86400) {
			time_index -= 86400;
			days++;
		}
		day_index += days;
	} else {
		time_index -= seconds;

		// If one more day
		if (time_index < 0) {
			time_index += 86400;
			days++;
		}
		if (day_index > days) {
			day_index -= days;
		} else {
			day_index = 0;
		}
	}

	// Update time with computed values
	SecToTime(time_index, &time->hours, &time->minutes, &time->seconds);
	DayToDate(day_index, &time->year, &time->month, &time->day);
	time->year += 2000;
}

/***************************************************************************//**
 * @brief
 *  Initialize a fix structure with unknown position values.
 *
 * @param[in] fix
 *  Pointer to fix structure to initialize.
 ******************************************************************************/
void TD_GEOLOC_FixInit(TD_GEOLOC_Fix_t *fix)
{
	if (fix != 0) {
		fix->duration = 0;
		fix->position.altitude = 0x7FFF;
		fix->position.longitude = 0x7FFFFFFF;
		fix->position.latitude = 0x7FFFFFFF;
		fix->quality.hdop = 9999;
		fix->quality.sat = 0;
		memset(&fix->speed, 0xFF, sizeof (TD_GEOLOC_Speed_t));
		memset(&fix->datetime, 0xFF, sizeof (TD_GEOLOC_DateTime_t));
		fix->type = TD_GEOLOC_NO_FIX;
		fix->autonomous.idle = true;
		fix->autonomous.enabled = false;
		fix->autonomous.sats = 0;
		fix->hard.rtc_calibrated = false;
	}
}

/***************************************************************************//**
 * @brief
 *   Poll all ephemeris data
 *
 * @param[in] sv_info
 *   SV_info value from current fix in order to poll av. sat only. If null will
 *   poll all ephemeris.
 *
 * @param[in] ephemeris
 *   Ephemeris array to be filled
 *
 * @param[in] max
 *   Max ephemeris to poll.
 *
 * @return
 *  Polled ephemeris count
 ******************************************************************************/
uint8_t TD_GEOLOC_PollEphemeris(TD_UBX7_NAV_SVInfo_t *sv_info,
	TD_UBX7_AID_Eph_t *ephemeris, uint8_t max)
{
	int eph_count = 0;
	int sv_count = 0;
	TD_UBX7_AID_Eph_t *p;
	int i;
	uint8_t msg[1];

	p = ephemeris;
	for (i = 1; i <= 32; i++) {
		//show_uart_stats();
		if (sv_info != NULL) {
			if (sv_count >= sv_info->channel_count) {
				return eph_count;
			} else {
				i = sv_info->channels[sv_count].svid;
				sv_count++;
			}
		}
		msg[0] = i;
		if (TD_UBX7_Poll(TD_UBX7_AID, TD_UBX7_AID_EPH, msg, 1, p, NULL, 0)) {
			if (p->svid != 0 && p->svid < 100 && p->how != 0) {
				eph_count++;

				// If max reached, stop
				if (eph_count >= max) {
					return eph_count;
				}
				if (ephemeris != NULL) {
					p = &ephemeris[eph_count];
				}
			}
		}
	}
	return eph_count;
}

/***************************************************************************//**
 * @brief
 *   Poll autonomous data for all satellites
 *
 * @param[in] sats
 *   Binary mapping of satellites to poll
 *
 * @param[in] autonomous
 *   Pointer to array to be filled
 *
 * @param[in] max
 *   Max autonomous data to poll
 *
 * @return
 *  Valid polled autonomous count
 ******************************************************************************/
uint8_t TD_GEOLOC_PollAutonomous(uint32_t sats, TD_UBX7_AID_Aop_t *autonomous,
	uint8_t max)
{
	int aop_count = 0;
	int i;
	uint8_t msg[1];
	TD_UBX7_AID_Aop_t *p;

	p = autonomous;
	if (sats == 0) {
		return 0;
	}
	for (i = 0; i < 32; i++) {
		if (!((1 << i) & sats)) {
			continue;
		}

		// Satellite IDs go from 1 to 32
		msg[0] = i + 1;
		if (TD_UBX7_Poll(TD_UBX7_AID, TD_UBX7_AID_AOP, msg, 1, p, NULL, 0)) {
			if (p->svid != 0) {
				aop_count++;

				// If max reached, stop
				if (aop_count >= max) {
					return aop_count;
				}
				if (autonomous != NULL) {
					p = &autonomous[aop_count];
				}
			}
		}
	}
	return aop_count;
}

/***************************************************************************//**
 * @brief
 *   Send autonomous data.
 *
 * @param[in] autonomous
 *   Pointer to array containing autonomous data
 *
 * @param[in] count
 *   Autonomous data array count
 ******************************************************************************/
void TD_GEOLOC_SendAutonomous(TD_UBX7_AID_Aop_t *autonomous, uint8_t count)
{
	int i;

	for (i = 0; i < count; i++) {
		TD_UBX7_SendCommand(TD_UBX7_AID, TD_UBX7_AID_AOP,
			(uint8_t *) &autonomous[i], 204);
	}
}

/***************************************************************************//**
 * @brief
 *   Send Health/Utc/Ionospheric parameters.
 *
 * @param[in] data
 *   Buffer used to send data. Must be 72 bytes.
 *
 * @param[in] tow
 *   Time of week
 *
 * @param[in] wno
 *   Week number
 ******************************************************************************/
void TD_GEOLOC_SendHui(uint8_t *data, uint32_t tow, uint16_t wno)
{
	memset(data, 0xFF, 4);
	data[20] = tow & 0xFF;
	data[21] = (tow >> 8) & 0xFF;
	data[22] = (tow >> 16) & 0xFF;
	data[23] = tow >> 24;
	data[24] = wno & 0xFF;
	data[25] = (wno >> 8) & 0xFF;
	data[32] = 16;
	data[26] = 0;
	data[68] = 3;
	TD_UBX7_SendCommand(TD_UBX7_AID, TD_UBX7_AID_HUI, data, 72);
}

/***************************************************************************//**
 * @brief
 *   Send Aiding Initialization data. Required to activate autonomous.
 *
 * @param[in] tow
 *   Time of week
 *
 * @param[in] wno
 *   Week number
 ******************************************************************************/
void TD_GEOLOC_SendIni(uint32_t tow, uint16_t wno)
{
	uint8_t msg[48];

	memset(msg,0,48);
	msg[20] = tow & 0xFF;
	msg[21] = (tow >> 8) & 0xFF;
	msg[22] = (tow >> 16) & 0xFF;
	msg[23] = tow >> 24;
	msg[18] = wno & 0xFF;
	msg[19] = (wno >> 8) & 0xFF;
	msg[44] = 2;
	TD_UBX7_SendCommand(TD_UBX7_AID, TD_UBX7_AID_INI, msg, 48);
}

/***************************************************************************//**
 * @brief
 *   Send the ephemeris.
 *
 * @param[in] ephemeris
 *   Pointer to the ephemeris array to send.
 *
 * @param[in] count
 *   Number of ephemeris to send.
 ******************************************************************************/
void TD_GEOLOC_SendEphemeris(TD_UBX7_AID_Eph_t *ephemeris, uint8_t count)
{
	int i;

	for (i = 0; i < count; i++) {
		TD_UBX7_SendEphemeris(&ephemeris[i]);
	}
}

/***************************************************************************//**
 * @brief
 *  Initialize Geolocation. Must be called in User Setup for
 *  proper GPS operation.
 ******************************************************************************/
void TD_GEOLOC_Init(void)
{
	TD_NMEA_Init();
	TD_UBX_PARSER_Init();

#if DECODE_NMEA
	TD_NMEA_PARSER_SetDataPointer(TD_NMEA_GPGGA, &Gpgga);
	TD_NMEA_PARSER_SetDataPointer(TD_NMEA_GPRMC, &Gprmc);
#endif

	TD_UBX_PARSER_SetDataPointer(TD_UBX7_NAV, TD_UBX7_NAV_SVINFO,
		&CurrentFix.sats.sv_info,0);
	TD_UBX_PARSER_SetDataPointer(TD_UBX7_MON, TD_UBX7_MON_HW, &MON_Hw, 0);
	TD_UBX_PARSER_SetDataPointer(TD_UBX7_NAV, TD_UBX7_NAV_AOPSTATUS,
		&NAV_AopStatus, 0);
	TD_UBX_PARSER_SetDataPointer(TD_UBX7_NAV, TD_UBX7_NAV_PVT, &NAV_Pvt, 0);
	TD_UBX7_Init();
	if (!TD_FLASH_DeclareVariable((uint8_t *) &Geoloc,
		sizeof (TD_GEOLOC_Config_t), 0)) {
		Geoloc.logger = false;
	}
	if (Geoloc.logger) {
		if ((void *) TD_FLASH_InitLogger != (void *) TD_TrapHere) {
			TD_FLASH_InitLogger(false,
				1,
				sizeof (TD_GEOLOC_LogValue_t),
				(uint32_t) &__cs3_regions_end,
				(uint32_t) &__cs3_region_end_rom);
			}
	}
	TD_GEOLOC_FixInit(&CurrentFix);
	memset(&Aiding, 0, sizeof (TD_GEOLOC_Aiding_t));
}

/** @} */

/** @} (end addtogroup TD_GEOLOC) */
