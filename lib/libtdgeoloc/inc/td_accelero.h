/***************************************************************************//**
 * @file
 * @brief
 * @author Telecom Design S.A.
 * @version 1.2.0
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2013-2016 Telecom Design S.A., http://www.telecomdesign.fr</b>
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

#ifndef TD_ACCELERO_H_
#define TD_ACCELERO_H_

#ifdef __cplusplus
extern "C" {
#endif

//#define TRACE_RTC_SAMPLE
	/***********************************************************************//**
	 * @addtogroup TD_ACCELERO Accelerometer
	 * @brief Accelerometer API for the TDxxxx RF modules.
	 * @{
	 **************************************************************************/

	/***************************************************************************
	 **************************  DEFINES   *************************************
	 **************************************************************************/

	/** @addtogroup TD_ACCELERO_DEFINES Defines
	 * @{ */

	/** Accelerometer axis masks */
#define TD_ACCELERO_AXIS_X		(1 << 0)	///< X Axis
#define TD_ACCELERO_AXIS_Y		(1 << 1)	///< Y Axis
#define TD_ACCELERO_AXIS_Z		(1 << 2)	///< Z Axis
#define TD_ACCELERO_ALL_AXIS	(TD_ACCELERO_AXIS_X | TD_ACCELERO_AXIS_Y | \
	TD_ACCELERO_AXIS_Z)	///< All axis

	/** Accelerometer IRQ source */
#define TD_ACCELERO_IRQ_XL		(1 << 0)	///< Enable interrupt generation on X low event or on Direction recognition
#define TD_ACCELERO_IRQ_XH		(1 << 1)	///< Enable interrupt generation on X high event or on Direction recognition
#define TD_ACCELERO_IRQ_YL		(1 << 2)	///< Enable interrupt generation on Y low event or on Direction recognition
#define TD_ACCELERO_IRQ_YH		(1 << 3)	///< Enable interrupt generation on Y high event or on Direction recognition
#define TD_ACCELERO_IRQ_ZL		(1 << 4)	///< Enable interrupt generation on Z low event or on Direction recognition
#define TD_ACCELERO_IRQ_ZH		(1 << 5)	///< Enable interrupt generation on Z high event or on Direction recognition
#define LIS3DH_INT1_SRC_IA		(1 << 6)	///< Enable interrupt generation on 6 direction detection
#define TD_ACCELERO_ALL_HIGH_IRQ	(TD_ACCELERO_IRQ_XH | TD_ACCELERO_IRQ_YH | \
	TD_ACCELERO_IRQ_ZH) ///< Enable interrupt generation on all axis high events or on Direction recognition
#define TD_ACCELERO_ALL_LOW_IRQ	(TD_ACCELERO_IRQ_XL | TD_ACCELERO_IRQ_YL | \
	TD_ACCELERO_IRQ_ZL) ///< Enable interrupt generation on all axis low events or on Direction recognition
#define TD_ACCELERO_ALL_IRQ	(TD_ACCELERO_ALL_HIGH_IRQ | TD_ACCELERO_ALL_LOW_IRQ) ///< Enable interrupt generation on all axis events or on Direction recognition

	/** Accelerometer Click IRQ source */
#define TD_ACCELERO_CLICK_IRQ_XS	(1 << 0)	///< Enable interrupt single CLICK-CLICK on X axis.
#define TD_ACCELERO_CLICK_IRQ_XD	(1 << 1)	///< Enable interrupt double CLICK-CLICK on X axis.
#define TD_ACCELERO_CLICK_IRQ_YS	(1 << 2)	///< Enable interrupt single CLICK-CLICK on Y axis.
#define TD_ACCELERO_CLICK_IRQ_YD	(1 << 3)	///< Enable interrupt double CLICK-CLICK on Y axis.
#define TD_ACCELERO_CLICK_IRQ_ZS	(1 << 4)	///< Enable interrupt single CLICK-CLICK on Z axis.
#define TD_ACCELERO_CLICK_IRQ_ZD	(1 << 5)	///< Enable interrupt double CLICK-CLICK on Z axis.

/** Enable interrupt single CLICK-CLICK on all axis */
#define TD_ACCELERO_SCLICK_ALL_IRQ TD_ACCELERO_CLICK_IRQ_XS | \
	TD_ACCELERO_CLICK_IRQ_YS | TD_ACCELERO_CLICK_IRQ_ZS

/** Enable interrupt double CLICK-CLICK on all axis */
#define TD_ACCELERO_DCLICK_ALL_IRQ TD_ACCELERO_CLICK_IRQ_XD | \
	TD_ACCELERO_CLICK_IRQ_YD | TD_ACCELERO_CLICK_IRQ_ZD

/** Root cause for Accelero errors in Polling mode */
#define DRIFT_POLL_ERR				0x01	/**< Drift between samples error */
#define DRIFT_CB_ERR				0x02	/**< Drift at data push error */
#define OVERRUN_ERR					0x04	/**< Overrun error */

/** @} */

	/***************************************************************************
	 ***********************   ENUMERATIONS   **********************************
	 **************************************************************************/

	/** @addtogroup TD_ACCELERO_ENUMERATIONS Enumerations
	 * @{ */

	/** Accelerometer power modes */
	typedef enum {
		TD_ACCELERO_POWER_DOWN = 0,
		TD_ACCELERO_LOW_POWER = 1,
		TD_ACCELERO_NORMAL_POWER = 3,
		TD_ACCELERO_8_BITS = 1,
		TD_ACCELERO_10_BITS = 2,
		TD_ACCELERO_12_BITS = 3
	} TD_ACCELERO_PowerModes_t;

	/** Accelerometer valid sampling rates */
	typedef enum {
		TD_ACCELERO_1HZ = 1,
		TD_ACCELERO_10HZ = 2,
		TD_ACCELERO_25HZ = 3,
		TD_ACCELERO_50HZ = 4,
		TD_ACCELERO_100HZ = 5,
		TD_ACCELERO_200HZ = 6,
		TD_ACCELERO_400HZ = 7,
		TD_ACCELERO_1_25KHZ = 9,				// Normal power only
		TD_ACCELERO_1_6KHZ = 8,					// Low power only
		TD_ACCELERO_5KHZ = 9 					// Low power only
	} TD_ACCELERO_Rates_t;

	/** Accelerometer valid measurement scales */
	typedef enum {
		TD_ACCELERO_2G = 0,
		TD_ACCELERO_4G = 1,
		TD_ACCELERO_8G = 2,
		TD_ACCELERO_16G = 3
	} TD_ACCELERO_Scales_t;

	/** Accelerometer valid FIFO mode */
	typedef enum {
		TD_ACCELERO_BYPASS = 0,
		TD_ACCELERO_FIFO = 1,
		TD_ACCELERO_STREAM = 2,
		TD_ACCELERO_STREAMFIFO = 3
	} TD_ACCELERO_FifoModes_t;

	/** Accelerometer monitoring types */
	typedef enum {
		TD_ACCELERO_NO_MONITORING,
		TD_ACCELERO_MONITOR_DATA,
		TD_ACCELERO_MONITOR_EVENT,
		TD_ACCELERO_MONITOR
	} TD_ACCELERO_Monitoring_t;

	/** @} */

	/***************************************************************************
	 *************************   TYPEDEFS   ************************************
	 **************************************************************************/

	/** @addtogroup TD_ACCELERO_TYPEDEFS Typedefs
	 * @{ */

	/** Accelerometer data structure */
	typedef struct {
		int16_t x;								///< Acceleration along X axis
		int16_t y;								///< Acceleration along Y axis
		int16_t z;								///< Acceleration along Z axis

		//!!WARNING: adding anything here will break MonitorData

#ifdef TRACE_RTC_SAMPLE
		uint8_t delay;							///< Sampling delay for each sample
		uint32_t rtc_before;
		uint32_t rtc_after;
#endif
	} TD_ACCELERO_Data_t;

	/** Accelerometer chip configuration */
	typedef struct {
		bool low_power;							///< Low power flag
		TD_ACCELERO_Rates_t rate;				///< Sampling rate
		TD_ACCELERO_Scales_t scale;				///< Measurement scale
		bool filter;							///< High-pass filter flag
	} TD_ACCELERO_ChipConfig_t;

	/** Accelerometer event monitoring configuration */
	typedef struct {
		bool click;								///< Click monitoring
		uint8_t axis;							///< Axis to monitor
		uint8_t threshold;						///< Monitoring threshold
		uint8_t duration;						///< Event duration
		uint8_t event;							///< Events to monitor
		void (*user_callback)(uint8_t source);	///< Event monitoring callback function
	} TD_ACCELERO_EventMonitoringConfig_t;

	/** Accelerometer data monitoring configuration */
	typedef struct {
		uint8_t axis; ///< Axis to monitor
		void (*user_callback)(TD_ACCELERO_Data_t data[32], uint8_t count, bool overrun); ///< Data monitoring callback function
	} TD_ACCELERO_DataMonitoringConfig_t;

	/** Accelerometer data monitoring configuration */
	typedef struct {
		void (*user_callback)(uint8_t source);	///< Click monitoring callback function
	} TD_ACCELERO_ClickMonitoringConfig_t;

	/** Accelerometer configuration */
	typedef struct {
		TD_ACCELERO_Monitoring_t monitoring;        ///< Monitoring type
		TD_ACCELERO_Monitoring_t monitoring_event;	///< Poll Monitoring type for Event
		TD_ACCELERO_Monitoring_t monitoring_data;	///< Poll Monitoring type for Data
		TD_ACCELERO_ChipConfig_t config;            ///< Accelerometer chip configuration
		TD_ACCELERO_EventMonitoringConfig_t event_config;   ///< Event monitoring configuration
		TD_ACCELERO_DataMonitoringConfig_t data_config;     ///< Data monitoring configuration
		uint8_t irq_source;                         ///< Global interrupt source
		bool allow_spi;			                    ///< SPI locking status
		uint8_t allow_spi_tag;		                ///< SPI locking status
		uint32_t allow_spi_rtc;		                ///< SPI locking status
	} TD_ACCELERO_Config_t;

	/** Accelerometer data callback function */
	typedef void (*TD_ACCELERO_CB)(TD_ACCELERO_Data_t data[32], uint8_t count,
		bool overrun);

	/** Accelerometer IRQ callback function */
	typedef void (*TD_ACCELERO_IRQ_CB)(uint8_t source);

	/** @} */

	/***************************************************************************
	 **************************  PUBLIC FUNCTIONS   ****************************
	 **************************************************************************/

	/** @addtogroup TD_ACCELERO_USER_FUNCTIONS User Functions
	 * @{ */

	bool TD_ACCELERO_Init(void);
	void TD_ACCELERO_Process(void);
	DECLARE_DYNAMIC (void, TD_ACCELERO_MonitorEvent,
			bool enable, TD_ACCELERO_Rates_t rate,
			uint8_t axis, TD_ACCELERO_Scales_t scale, uint8_t event,
			uint8_t threshold, uint8_t duration, int8_t filter,
			TD_ACCELERO_IRQ_CB);
	DECLARE_DYNAMIC_ALT (void, TD_ACCELERO_MonitorEvent,
			TD_ACCELERO_MonitorEventPolling, bool enable, TD_ACCELERO_Rates_t rate,
			uint8_t axis, TD_ACCELERO_Scales_t scale, uint8_t event,
			uint8_t threshold, uint8_t duration, int8_t filter,
			TD_ACCELERO_IRQ_CB callback);
	DECLARE_DYNAMIC(void,TD_ACCELERO_MonitorData,bool enable, bool low_power,
			TD_ACCELERO_Rates_t rate, uint8_t axis, TD_ACCELERO_Scales_t scale,
			int8_t filter, TD_ACCELERO_FifoModes_t fifo, uint8_t watermark,
			TD_ACCELERO_CB);
	DECLARE_DYNAMIC_ALT(void,TD_ACCELERO_MonitorData,
			TD_ACCELERO_MonitorDataPolling,bool enable, bool low_power,
			TD_ACCELERO_Rates_t rate, uint8_t axis, TD_ACCELERO_Scales_t scale,
			int8_t filter, TD_ACCELERO_FifoModes_t fifo, uint8_t watermark,
			TD_ACCELERO_CB callback);
	void TD_ACCELERO_MonitorClickEvent(bool enable,  uint8_t event,
		uint8_t threshold, uint8_t duration, void (*callback)(uint8_t source));
	uint8_t TD_ACCELERO_GetDrift(void);
	TD_ACCELERO_Config_t *TD_ACCELERO_GetConfig(void);
	void TD_ACCELERO_SetConfig(TD_ACCELERO_Config_t *config);
	uint8_t TD_ACCELERO_ReadRegister(uint8_t address);
	void TD_ACCELERO_WriteRegister(uint8_t address, uint8_t value);
	uint8_t TD_ACCELERO_Who(void);
	void TD_ACCELERO_ClearFilter(void);
	void TD_ACCELERO_ProcessMonitorData(TD_ACCELERO_Data_t *data, uint8_t sz);
	void TD_ACCELERO_SetWatermark(uint8_t watermark);
	void TD_ACCELERO_PowerDown(void);
	void TD_ACCELERO_NormalPower(TD_ACCELERO_Rates_t rate, uint8_t axis,
		TD_ACCELERO_Scales_t scale);
	void TD_ACCELERO_LowPower(TD_ACCELERO_Rates_t rate, uint8_t axis,
		TD_ACCELERO_Scales_t scale);
	void TD_ACCELERO_GetXYZ(TD_ACCELERO_Data_t *data);
	void TD_ACCELERO_SampleGetXYZ(TD_ACCELERO_Data_t *data);
	void TD_ACCELERO_RawGetXYZ(TD_ACCELERO_Data_t *data);
	void TD_ACCELERO_RawSampleXYZ(void);
	void TD_ACCELERO_SetBatch(uint8_t data);
	void TD_ACCELERO_PauseMonitor(void);
	void TD_ACCELERO_PauseScheduler(void);
	void TD_ACCELERO_ResumeMonitor(void);
	void TD_ACCELERO_ResumeScheduler(void);
	void TD_ACCELERO_Power(TD_ACCELERO_Rates_t rate, uint8_t axis,
		TD_ACCELERO_Scales_t scale,TD_ACCELERO_PowerModes_t mode);
	TD_ACCELERO_Monitoring_t TD_ACCELERO_MonitorStatus(void);
	TD_ACCELERO_Monitoring_t TD_ACCELERO_MonitorDataStatus(void);
	TD_ACCELERO_Monitoring_t TD_ACCELERO_MonitorEventStatus(void);
	uint8_t TD_ACCELERO_DebugCheck(void);
	uint32_t TD_ACCELERO_ProcessTime(void);
	uint32_t TD_ACCELERO_IrqTime(void);
	uint32_t TD_ACCELERO_CallbackTime(void);
	uint8_t TD_ACCELERO_StatusReg(void);
	uint8_t TD_ACCELERO_GetCause(void);
	void TD_ACCELERO_Polling(uint32_t arg, uint8_t repetition);
	void TD_ACCELERO_PollingGet(void);
	void TD_ACCELERO_PollingSample(void);
	void TD_ACCELERO_InitLETIMER(void);
	void TD_ACCELERO_StopLETIMER(void);
	void  TD_ACCELERO_SetLockingStatus(bool state, uint8_t tag);

	/** @} */

	/** @addtogroup TD_ACCELERO_GLOBAL_FUNCTIONS Global Functions
	 * @{ */

	void TD_ACCELERO_Dump(void);

	/** @} */

	/** @} (end addtogroup TD_ACCELERO) */

#ifdef __cplusplus
}
#endif

#endif // __TD_ACCELERO_H
