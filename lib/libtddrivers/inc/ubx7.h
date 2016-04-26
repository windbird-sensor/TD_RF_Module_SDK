/***************************************************************************//**
 * @file
 * @brief Driver definition for the UBlox 7 GPS used in TD12xx RF modules.
 * @author Telecom Design S.A.
 * @version 1.0.1
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2013-2015 Telecom Design S.A., http://www.telecomdesign.fr</b>
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
#ifndef __UBX7_H
#define __UBX7_H

#include <stdint.h>
#include <stdbool.h>

#include <td_module.h>

#ifdef __cplusplus
extern "C" {
#endif

	/***********************************************************************//**
	 * @addtogroup TD_UBX7 Ublox7
	 * @brief UBlox 7 GPS driver.
	 * @{
	 **************************************************************************/

	/***************************************************************************
	 **************************  DEFINES   *************************************
	 **************************************************************************/

	/** @addtogroup TD_UBX7_DEFINES Defines
	 * @{ */

	/** Maximum number of UBlox 7 SV info channels */
#define TD_UBX7_NAV_SVINFO_MAX_CHANNEL 16

	/** @} */

	/***************************************************************************
	 **************************  TYPEDEFS   ************************************
	 **************************************************************************/

	/** @addtogroup TD_UBX7_TYPEDEFS Typedefs
	 * @{ */

	/** UBlox 7 GPS start modes */
	typedef enum {
		TD_UBX7_HOT = 0,
		TD_UBX7_WARM,
		TD_UBX7_COLD
	} TD_UBX7_StartMode_t;

	/** UBlox 7 GPS configuration result values */
	typedef enum {
		TD_UBX7_CONFIG_TIMEOUT,
		TD_UBX7_CONFIG_ACKED,
		TD_UBX7_CONFIG_NACKED,
		TD_UBX7_CONFIG_SPI_FAILED
	} TD_UBX7_ConfigResult_t;

	/** RXM_RTC5 structure*/
	typedef struct {
		uint32_t freq;				///< Frequency*
		uint32_t tow;				///< Time of week
		uint16_t wno;				///< Week number
		bool valid_tow;				///< Is tow valid
	} TD_UBX7_RxmRTC5_t;

	/** Aid INI structure */
	typedef struct {
		int32_t ecefXOrLat;			///< WGS84 ECEF X coordinate or latitude, depending on flags below
		int32_t ecefYOrLon;			///< WGS84 ECEF Y coordinate or longitude, depending on flags below
		int32_t ecefZOrAlt;			///< WGS84 ECEF Z coordinate or altitude, depending on flags below
		uint32_t posAcc;			///< Position accuracy (stddev)
		uint32_t towOrTime;			///< Actual time of week or DayOfMonth/Hour/Minute/Second (DDHHMMSS), depending on flags below
		int32_t towNs;				///< Fractional part of time of week
		uint32_t tAccMs;			///< Milliseconds part of time accuracy
		uint32_t tAccNs;			///< Nanoseconds part of time accuracy
		int32_t clkDOrFreq;			///< Clock drift or frequency, depending on flags below
		uint32_t clkDAccOrFreqAcc;	///< Accuracy of clock drift or frequency, depending on flags below
		uint32_t flags;				///< Bitmask
		uint16_t tmCfg; 			///< Time mark configuration
		uint16_t wnoOrDate;			///< Actual week number or yearSince2000/Month (YYMM), depending on flags below
	}  TD_UBX7_AidIni_t;

	/** NAV-SVINFO channel structure */
	typedef struct {
		uint8_t svid;				///< Sat in view ID
		uint8_t level;				///< Satellites level
		uint8_t flags; 				///< Type or orbit used for navigation
		uint8_t quality; 			///< Fix quality info (7 = ephemeris)
		bool used;					///< Satellite is used for navigation
	}  TD_UBX7_NAV_SVInfo_Channel_t;

	/** NAV-SVINFO structure */
	typedef struct {
		TD_UBX7_NAV_SVInfo_Channel_t channels[TD_UBX7_NAV_SVINFO_MAX_CHANNEL];	///< Channels
		uint8_t channel_count;		///< Channel number
	}  TD_UBX7_NAV_SVInfo_t;

	/** NAV-Sol structure */
	typedef struct {
		uint32_t itow; 				///< GPS time of week of the navigation epoch.
		int32_t lon;				///< Longitude dg
		int32_t lat;				///< Latitude dg
		int32_t height;				///< Height above Ellipsoid mm
		int32_t hAcc;				///< Horizontal accuracy estimate mm
		int32_t gSpeed;				///< Ground speed mm/s
		int32_t heading;			///< Heading of motion 2-D
		uint16_t pDop;				///< Position DOP
		uint16_t year;				///< UTC year
		uint8_t month;				///< UTC month (1-12)
		uint8_t day;				///< UTC day (1-31)
		uint8_t hour;				///< UTC hour (0-23)
		uint8_t min;				///< UTC day (0-59)
		uint8_t sec;				///< UTC day (0-60)
		uint8_t valid;				///< Validity flag
		uint8_t fixType;			///< GNSSfix Type, 0-> no fix, 2 ->2D; 3->3D; 5-> time
		uint8_t flags;				///< Fix status flag
		uint8_t numSv;				///< Number of sat used in solution
	}  TD_UBX7_NAV_Pvt_t;

	/** NAV-CLOCK structure */
	typedef struct {
		uint32_t tow; 				///< Non reliable gps time
		int32_t clkB; 				///< Clock bias ns
		int32_t clkD; 				///< Clock drift ns/s
		uint32_t tAcc; 				///< Time accuracy estimate ns
		uint32_t fAcc; 				///< Frequency accuracy estimate ps/s
	}  TD_UBX7_NAV_Clock_t;

	/** NAV_AOPSTATUS structure */
	typedef struct {
		uint32_t availGPS; 			///< data availability mask for GPS SVs	(bits 0-31 correspond to GPS PRN 1-32)
		uint8_t aopCfg;				///< Enabled or not
		uint8_t status; 			///< Status (running or not)
	}  TD_UBX7_NAV_AopStatus_t;

	/** RXM-RTC5 structure */
	typedef struct {
		//uint32_t rTagHW; 			///< RTAG high word
		//uint32_t rTagLW; 			///< RTAG low word
		uint32_t freq; 				///< Clock frequency hz
		uint32_t freqFrac; 			///< Clock frequency fractional part hz
		uint32_t towFrac; 			///< Time of week fractional part s
		uint32_t tow; 				///< Time of week s
		uint16_t wno; 				///< GPS week number
		uint8_t towValid;			///< TOW is valid flag
		uint8_t freqValid;			///< Frequency is valid flag
	}  TD_UBX7_RXM_Rtc5_t;

	/** MON-HW structure */
	typedef struct {
		uint8_t flags;				///< Bit 0 is rtc calibration
		uint8_t jamInd; 			///< CW Jamming indicator, scaled (0 = no CW jamming, 255 = strong CW jamming)
	}  TD_UBX7_MON_Hw_t;

	/** Ublox 7 Config Port message structure */
	typedef struct {
		uint8_t port_id			: 8;	///< Port Identifier Number
		uint8_t rsvd0			: 8;	///< Reserved
		bool tx_ready_en		: 1;	///< TX ready enable
		bool tx_ready_pol		: 1;	///< TX ready polarisation (0 high active, 1 low active)
		uint8_t tx_ready_pin	: 5;	///< TX ready IO to use
		uint16_t tx_ready_thres	: 9;	///< TX ready threshold (*8 bytes),0 no threshold, 1 =  8bytes
		uint32_t mode			: 32;	///< A bit mask describing the UART mode
		uint32_t baudrate		: 32;	///< Reserved
		uint16_t in_proto_mask	: 16;	///< Active input protocols
		uint16_t out_proto_mask	: 16;	///< Active output protocols
		uint16_t flags			: 16;	///< Flag bit mask
		uint16_t rsvd5			: 16;	///< Reserved
	} __PACKED TD_UBX7_CFG_PRT_Msg_t;

	/** AID-EPH structure
	 *
	 * NB: sfxd contains subframe data. Ublox is delivering 8 words but all have bits 24..31 that shall be ignored.
	 * As this is a significant data gain (24 bytes per ephemeris) they have been removed here. For the data
	 * to be sent back they should be set again thus requiring a more complex algorithm.
	 *
	 * siv as also been reduced from 32 bits to 8 bits
	 *
	 * */
	typedef struct {
		uint32_t how;				///< Hand-Over Word of first Subframe. This is required if data is sent to the receiver.
		uint8_t sf1d[24];			///< Subframe 1 Words 3..10
		uint8_t sf2d[24];			///< Subframe 2 Words 3..10
		uint8_t sf3d[24];			///< Subframe 3 Words 3..10
		uint8_t svid;				///< SV ID for which this ephemeris data is (Valid Range: 1 .. 32).
	}  TD_UBX7_AID_Eph_t;

	/** Aid structure */
	typedef struct {
		uint8_t data[72];			///< Data
	} TD_UBX7_AID_Hui_t;

	/** Aid aop per satellite structure*/
	typedef struct {
		uint8_t svid;				///< Satellite in view ID
		uint8_t data[59];			///< Data
		uint8_t opt1[48];			///< Option 1
		uint8_t opt2[48];			///< Option 2
		uint8_t opt3[48];			///< Option 3
	} TD_UBX7_AID_Aop_t;

	/** Ublox 7 message class */
	typedef enum {
		TD_UBX7_NAV = 0x01,			///< Navigation Results
		TD_UBX7_RXM = 0x02,			///< Receiver Manager Messages
		TD_UBX7_INF = 0x04,			///< Information Messages
		TD_UBX7_ACK = 0x05,			///< Ack/Nack Messages
		TD_UBX7_CFG = 0x06,			///< Configuration Input Messages
		TD_UBX7_UPD = 0x09,			///< Firmware Update Messages
		TD_UBX7_MON = 0x0A,			///< Monitoring Messages
		TD_UBX7_AID = 0x0B,			///< AssistNow Aiding Messages
		TD_UBX7_TIM = 0x0D,			///< Timing Messages
		TD_UBX7_LOG = 0x21			///< Logging Messages
	} TD_UBX7_Class_t;

	/** Ublox 7 Ack/Nack message id */
	typedef enum {
		TD_UBX7_ACK_NACK = 0x00,
		TD_UBX7_ACK_ACK = 0x01
	} TD_UBX7_ACK_ID_t;

	/** Ublox 7 Config message id */
	typedef enum {
		TD_UBX7_CFG_ANT = 0x13,		///< Antenna control settings
		TD_UBX7_CFG_GNSS = 0x3E,	///< GNSS configuration
		TD_UBX7_CFG_INF = 0x02,		///< Information configuration
		TD_UBX7_CFG_ITFM = 0x39,	///< Interference/jamming configuration
		TD_UBX7_CFG_MSG = 0x01,		///< Message rate configuration
		TD_UBX7_CFG_NAV5 = 0x24,	///< Nav engine settings
		TD_UBX7_CFG_NAVX5 = 0x23,	///< Nav engine expert settings
		TD_UBX7_CFG_NMEA = 0x17,	///< NMEA configuration
		TD_UBX7_CFG_PIO = 0x2C,		///< Config pio
		TD_UBX7_CFG_PRT = 0x00,		///< Port configuration
		TD_UBX7_CFG_PT = 0x45,		///< Production test configuration
		TD_UBX7_CFG_RATE = 0x08,	///< Navigation rate configuration
		TD_UBX7_CFG_TP5 = 0x31 ,	///< Timepulse parameters
		TD_UBX7_CFG_OTP = 0x41,		///< eFuse
		TD_UBX7_CFG_RST = 0x04,		///< Start
		TD_UBX7_CFG_PM2 = 0x3B,		///< Power save mode
		TD_UBX7_CFG_RXM = 0x11,		///< Power mode
		TD_UBX7_CFG_CFG = 0x09		///< Power mode
	} TD_UBX7_CFG_ID_t;

	/** Ublox 7 Monitor message id */
	typedef enum {
		TD_UBX7_MON_HW = 0x09,		///< RTC and pin status
		TD_UBX7_MON_SIG = 0x3E,		///< GNSS configuration
		TD_UBX7_MON_EXCEPT = 0x05,	///< GNSS configuration
		TD_UBX7_MON_VER = 0x04,		///< Version
		TD_UBX7_MON_TX_BUF = 0x08,	///< TX buffer stat
	} TD_UBX7_MON_ID_t;

	/** Ublox 7 Navigation message id */
	typedef enum {
		TD_UBX7_NAV_CLK = 0x22,		///< Clock Solution
		TD_UBX7_NAV_SVINFO = 0x30,	///< Space Vehicle Information
		TD_UBX7_NAV_AOPSTATUS = 0x60,	///< Autonomous status
		TD_UBX7_NAV_PVT = 0x07,		///< Navigation solution information
		TD_UBX7_NAV_TIMEGPS = 0x20	///< Time solution information
	} TD_UBX7_NAV_ID_t;

	/** Ublox 7 Receiver Manager message id */
	typedef enum {
		TD_UBX7_RXM_RTC5 = 0x23,	///< Real Time Clock Status
		TD_UBX7_RXM_EPH = 0x31,		///< Poll GPS Constellation Ephemeris Data
		TD_UBX7_RXM_PMREQ = 0x41,	///< Requests a Power Management task
	} TD_UBX7_RXM_ID_t;

	/** Ublox 7 Receiver Manager message id */
	typedef enum {
		TD_UBX7_AID_EPH = 0x31,		///< Ephemeris aiding
		TD_UBX7_AID_HUI = 0x02,
		TD_UBX7_AID_INI = 0x01,
		TD_UBX7_AID_AOP = 0x33,
	} TD_UBX7_AID_ID_t;

	/** @} */

	/***************************************************************************
	 *************************   PROTOTYPES   **********************************
	 **************************************************************************/

	/** @addtogroup TD_UBX7_GLOBAL_FUNCTIONS Global Functions
	 * @{ */

	bool TD_UBX7_Process(void);
	void TD_UBX7_Init(void);
	void TD_UBX7_Reset(void);
	bool TD_UBX7_PowerUp(bool power_save, bool timepulse);
	void TD_UBX7_PowerOff(void);
	void TD_UBX7_PowerOn(void);
	void TD_UBX7_RegisterOnOffCallback(void (*callback)(bool on));

	// Configuration functions
	TD_UBX7_ConfigResult_t TD_UBX7_SendConfig(TD_UBX7_CFG_ID_t id,
		uint8_t *data, uint16_t len);
	TD_UBX7_ConfigResult_t TD_UBX7_ConfigureNavigationRate(uint16_t mes_rate,
		uint16_t timeref);
	TD_UBX7_ConfigResult_t TD_UBX7_ConfigureStartMode(TD_UBX7_StartMode_t type);
	TD_UBX7_ConfigResult_t TD_UBX7_ConfigureMessageRate(uint8_t class,
		uint8_t id, uint8_t rate);
	TD_UBX7_ConfigResult_t TD_UBX7_ConfigureeFuse(void);
	void TD_UBX7_StartGPS(bool start);
	TD_UBX7_ConfigResult_t TD_UBX7_ConfigureAutonomous(bool enable,
		uint8_t min_sv, uint8_t min_cno, bool force_3d, uint16_t max_error);
	TD_UBX7_ConfigResult_t TD_UBX7_SaveConfig(void);
	void TD_UBX7_ForceNMEA(void);

	// Command functions
	bool TD_UBX7_SendCommand(TD_UBX7_Class_t class, uint8_t id, uint8_t *data,
		uint16_t len);
	bool TD_UBX7_SendEphemeris(TD_UBX7_AID_Eph_t *ephemeris);
	bool TD_UBX7_Sleep(void);
	void TD_UBX7_SoftBackup(void);
	void TD_UBX7_HardwareBackup(void);

	// Poll functions
	bool TD_UBX7_Poll(TD_UBX7_Class_t class, uint8_t id, uint8_t *send,
		uint16_t send_len, void *ptr, uint8_t *data, uint16_t *data_len);
	bool TD_UBX7_isFused(void);

	// Misc
	void TD_UBX7_TestConfiguration(void);
	void TD_UBX7_Dump(void);

	/** @} */

	/** @} (end addtogroup TD_UBX7) */

#ifdef __cplusplus
}
#endif

#endif // __UBX7_H
