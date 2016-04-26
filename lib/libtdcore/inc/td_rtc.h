/***************************************************************************//**
 * @file
 * @brief Real-Time Clock (RTC) peripheral API for the TDxxxx RF modules.
 * @author Telecom Design S.A.
 * @version 2.3.0
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2012-2015 Telecom Design S.A., http://www.telecomdesign.fr</b>
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

#ifndef __TD_RTC_H
#define __TD_RTC_H

#include <stdint.h>
#include <stdbool.h>
#include <time.h>

#include <efm32.h>
#include <em_rtc.h>
#include <em_assert.h>

#ifdef __cplusplus
extern "C" {
#endif

	/***********************************************************************//**
	 * @addtogroup RTC
	 * @brief Real-Time Clock (RTC) peripheral API for the TDxxxx RF modules
	 * @{
	 **************************************************************************/

	/***************************************************************************
	 *************************   DEFINES   *************************************
	 **************************************************************************/

	/** @addtogroup RTC_DEFINES Defines
	 * @{ */

	/***********************************************************************//**
	 * @addtogroup Timing Durations
	 * @brief Duration values in ticks based on 32768 Hz clock
	 * @{
	 **************************************************************************/

	/** Duration values in ticks based on 32768 Hz clock */
#define  T0S               0							/**< 0 s */
#define  TMICRO(x)         ((32768 * (x)) / 1000000)	/**< x �s */
#define  T20MICROS         1							/**< 20 �s */
#define  T50MICROS         2							/**< 50 �s */
#define  T100MICROS        2							/**< 100 �s */
#define  T125MICROS        3							/**< 125 �s */
#define  T150MICROS        4							/**< 150 �s */
#define  T200MICROS        ((32768 * 2 ) / 10000)		/**< 200 �s */
#define  T300MICROS        ((32768 * 3 ) / 10000)		/**< 300 �s */
#define  T340MICROS        ((32768 * 34 ) / 100000)		/**< 340 �s */
#define  T400MICROS        ((32768 * 4 ) / 10000)		/**< 400 �s */
#define  T480MICROS        ((32768 * 48 )/ 100000)		/**< 480 �s */
#define  T500MICROS        ((32768 * 5 ) / 10000)		/**< 500 �s */
#define  T600MICROS        ((32768 * 6 ) / 10000)		/**< 600 �s */
#define  T700MICROS        ((32768 * 7 ) / 10000)		/**< 700 �s */
#define  T800MICROS        ((32768 * 8 ) / 10000)		/**< 800 �s */
#define  T900MICROS        ((32768 * 9 ) / 10000)		/**< 900 �s */
#define  TMS(x)            ((32768 * (x)) / 1000)		/**< x ms */
#define  T1MS              ((32768 * 10) / 10000)		/**< 1 ms */
#define  T1_25MS           ((32768 * 125) / 100000)		/**< 1.25 ms */
#define  T1_6MS            ((32768 * 160) / 100000)		/**< 1.6 ms */
#define  T2MS              ((32768 * 20) / 10000)		/**< 2 ms */
#define  T2_5MS            ((32768 * 25) / 10000)		/**< 2.5 ms */
#define  T3_32MS           ((32768 * 33) / 10000)		/**< 3.32 ms */
#define  T3_8MS            ((32768 * 38) / 10000)		/**< 3.8 ms */
#define  T5MS              ((32768 * 50) / 10000)		/**< 5 ms */
#define  T5_5MS            ((32768 * 55) / 10000)		/**< 5.5 ms */
#define  T7MS              ((32768 * 70) / 10000)		/**< 6 ms */
#define  T7_5MS            ((32768 * 75) / 10000)		/**< 7.5 ms */
#define  T8_5MS            ((32768 * 85) / 10000)		/**< 8.5 ms */
#define  T10MS             ((32768 * 100) / 10000)		/**< 10 ms */
#define  T11MS             ((32768 * 110) / 10000)		/**< 11 ms */
#define  T12MS             ((32768 * 120) / 10000)		/**< 12 ms */
#define  T15MS             ((32768 * 150) / 10000)		/**< 15 ms */
#define  T16MS             ((32768 * 160) / 10000)		/**< 16 ms */
#define  T17MS             ((32768 * 171) / 10000)		/**< 17 ms */
#define  T19_2MS           ((32768 * 192) / 10000)		/**< 19.2 ms */
#define  T20MS             ((32768 * 200) / 10000)		/**< 20 ms */
#define  T20_2MS           ((32768 * 202) / 10000)		/**< 20 ms */
#define  T24MS             ((32768 * 240) / 10000)		/**< 24 ms */
#define  T24_5MS           ((32768 * 245) / 10000)		/**< 24 ms */
#define  T26MS			   ((32768 * 260) / 10000)		/**< 26 ms */
#define  T26_6MS           ((32768 * 266) / 10000)		/**< 26.6 ms */
#define  T26_7MS           ((32768 * 267) / 10000)		/**< 26.7 ms */
#define  T27_6MS           ((32768 * 276) / 10000)		/**< 27.6 ms */
#define  T38MS             ((32768 * 380) / 10000)		/**< 38 ms */
#define  T40MS             ((32768 * 400) / 10000)		/**< 40 ms */
#define  T44MS             ((32768 * 440) / 10000)		/**< 44 ms */
#define  T50MS             ((32768 * 500) / 10000)		/**< 50 ms */
#define  T60MS             ((32768 * 600) / 10000)		/**< 60 ms */
#define  T70MS             ((32768 * 700) / 10000)		/**< 70 ms */
#define  T100MS            ((32768 * 1000) / 10000)		/**< 100 ms */
#define  T160MS            ((32768 * 1600) / 10000)		/**< 160 ms */
#define  T200MS            ((32768 * 2000) / 10000)		/**< 200 ms */
#define  T250MS            ((32768 * 2500) / 10000)		/**< 250 ms */
#define  T300MS            ((32768 * 3000) / 10000)		/**< 300 ms */
#define  T350MS            ((32768 * 3500) / 10000)		/**< 350 ms */
#define  T400MS            ((32768 * 4000) / 10000)		/**< 400 ms */
#define  T500MS            ((32768 * 5000) / 10000)		/**< 500 ms */
#define  T600MS            ((32768 * 6000) / 10000)		/**< 600 ms */
#define  T700MS            ((32768 * 7000) / 10000)		/**< 700 ms */
#define  T800MS            ((32768 * 8000) / 10000)		/**< 800 ms */
#define  T900MS            ((32768 * 9000) / 10000)		/**< 900 ms */
#define  T950MS            ((32768 * 9500) / 10000)		/**< 950 ms */
#define  T1S               32768						/**< 1 s */
#define  T1060MS           ((32768 * 10600) / 10000)	/**< 1.06 s */
#define  T1100MS           ((32768 * 11000) / 10000)	/**< 1.1 s */
#define  T1200MS           ((32768 * 12000) / 10000)	/**< 1.2 s */
#define  T1400MS           ((32768 * 14000) / 10000)	/**< 1.4 s */
#define  T2S               (T1S * 2)					/**< 2 s */
#define  T3S               (T1S * 3)					/**< 3 s */
#define  T4S               (T1S * 4)					/**< 4 s */
#define  T5S               (T1S * 5)					/**< 5 s */
#define  T8S               (T1S * 8)					/**< 8 s */
#define  T10S              (T1S * 10)					/**< 10 s */
#define  T15S              (T1S * 15)					/**< 15 s */
#define  T17S              (T1S * 17)					/**< 17 s */
#define  T20S              (T1S * 20)					/**< 20 s */
#define  T30S              (T1S * 30)					/**< 30 s */
#define  T1MN              (T1S * 60)					/**< 1 min */
#define  T2MN              (T1S * 60 * 2)				/**< 2 min */
#define  T8MN              (T1S * 60 * 8)				/**< 8 min */
#define  T8H               (T1S * 3600 * 8)				/**< 8 h */
#define  T24H              0xA8C00000					/**< 24 h */
#define  TICK8MMBYDAY      (60 * 3)						/**< Number of 8 min ticks /day */
#define  TICK32768_BYDAY   (T24H)						/**< Number of 32768 hz ticks /day */
#define  TICK1MMBYDAY      (24 * 60)					/**< Number of 1 min ticks /day */

	/** @} (end addtogroup Timing Durations) */

	/** The RTC comparator number used by system */
#define TD_RTC_SYSTEM		0

	/** The RTC comparator number used by user */
#define TD_RTC_USER			1

	/** Macro to get the current RTC timestamp */
#define TD_RTC_Now()        RTC_CounterGet()

	/* Use this to trace wakeup cause*/
//#define WAKE_MAIN_LOOP_DEBUG

#ifdef WAKE_MAIN_LOOP_DEBUG
	/** Macro to force a main loop iteration */
#define TD_WakeMainLoop()		{ \
	tfp_printf("W%s:%d\r\n", __FILE__, __LINE__);\
	BackgroundRoundWanted = true; \
}
#else
	/** Macro to force a main loop iteration */
#define TD_WakeMainLoop()		{ \
	BackgroundRoundWanted = true; \
}
#endif

	/** Overloaded version of TD_RTC_SetPowerMode() which adds mode and line number */
#define TD_RTC_SetPowerMode(mode) TD_RTC_SetPowerModeInternal(mode,__LINE__)

	/** @} */

	/***************************************************************************
	 ***********************   ENUMERATIONS   **********************************
	 **************************************************************************/

	/** @addtogroup RTC_ENUMERATIONS Enumerations
	 * @{ */

	/** RTC Low power mode */
	typedef enum {
		TD_RTC_EM0,
		TD_RTC_EM1,
		TD_RTC_EM2,
		TD_RTC_EM3,
		TD_RTC_EM4
	} TD_RTC_PowerMode_t;

	/** @} */

	/***************************************************************************
	 *************************   TYPEDEFS   ************************************
	 **************************************************************************/

	/** @addtogroup RTC_TYPEDEFS Typedefs
	 * @{ */

	/** RTC Interrupt handler function type */
	typedef void (*TD_RTC_handler_t)(void);

	/** @} */

	/***************************************************************************
	 **************************   PUBLIC VARIABLES   ***************************
	 **************************************************************************/

	/** @addtogroup RTC_GLOBAL_VARIABLES Global Variables
	 * @{ */

	/** Flag to ask for an additional background loop round upon IRQ wake-up */
	extern volatile bool BackgroundRoundWanted;

	/** @} */

	/***************************************************************************
	 *************************   PROTOTYPES   **********************************
	 **************************************************************************/

	/** @addtogroup RTC_USER_FUNCTIONS User Functions
	 * @{ */

	/***********************************************************************//**
	 * @brief
	 *   Program an absolute alarm.
	 *
	 * @param[in] count
	 *   Absolute count in 1/32768 s ticks at which the alarm will occur.
	 *   If count is less than 2 unit more than actual count, timer will wait a
	 *   full loop (0xFFFFFF)
	 *   If count is between 2 and 4 unit more than actual count, behavior is
	 *   not defined (full loop or immediate)
	 *   Alarm will only trigger handler set TD_RTC_SetSystemHandler.
	 *************************************************************************/
	static __INLINE void TD_RTC_AlarmAt(uint32_t count)
	{
		RTC_CompareSet(TD_RTC_SYSTEM, count & 0xFFFFFF);
	}

	/***********************************************************************//**
	 * @brief
	 *   Abort an RTC delay.
	 **************************************************************************/
	static __INLINE void TD_RTC_Abort(void)
	{
		extern volatile bool TD_RTC_AbortDelay;
		TD_RTC_AbortDelay = true;
	}

	/***********************************************************************//**
	 * @brief
	 *   Acknowledge a keep-alive interrupt.
	 **************************************************************************/
	static __INLINE void TD_RTC_AckKeepAlive(void)
	{
		extern volatile bool TD_RTC_KeepAlive;
		TD_RTC_KeepAlive = false;
	}

	/***********************************************************************//**
	 * @brief
	 *   Enable/disable the system RTC IRQ.
	 *
	 * @param[in] enable
	 *   The enable state.
	 **************************************************************************/
	static __INLINE void TD_RTC_EnableSystemInterrupts(bool enable)
	{
		if (enable) {
			RTC_IntEnable(RTC_IF_COMP0);
		} else {
			RTC_IntDisable(RTC_IF_COMP0);
		}
	}

	/***********************************************************************//**
	 * @brief
	 *   Enable/disable the user RTC IRQ.
	 *
	 * @param[in] enable
	 *   The enable state.
	 **************************************************************************/
	static __INLINE void TD_RTC_EnableUserInterrupts(bool enable)
	{
		if (enable) {
			RTC_IntEnable(RTC_IF_COMP1);
		} else {
			RTC_IntDisable(RTC_IF_COMP1);
		}
	}

	/***********************************************************************//**
	 * @brief
	 *   Clear the system RTC IRQ.
	 **************************************************************************/
	static __INLINE void TD_RTC_ClearSystemInterrupts(void)
	{
		RTC_IntClear(RTC_IFC_COMP0);
	}

	/***********************************************************************//**
	 * @brief
	 *   Clear the user RTC IRQ.
	 **************************************************************************/
	static __INLINE void TD_RTC_ClearUserInterrupts(void)
	{
		RTC_IntClear(RTC_IFC_COMP1);
	}

	void TD_RTC_Init(TD_RTC_handler_t function);
	void TD_RTC_Process(void);
	bool TD_RTC_Delay(uint32_t duration);
	void TD_RTC_Sleep(void);
	TD_RTC_handler_t TD_RTC_SetSystemHandler(TD_RTC_handler_t function);
	TD_RTC_handler_t TD_RTC_SetUserHandler(TD_RTC_handler_t function);
	void TD_RTC_SetKeepAliveHandler(TD_RTC_handler_t function, uint32_t period);
	void TD_RTC_CalibratedDelay(uint32_t udelay);
	uint32_t TD_RTC_TimeDiff(uint32_t reference);
	int32_t TD_RTC_SignedTimeDiff(uint32_t reference);
	uint32_t TD_RTC_GetOverflowCounter(void);
	bool TD_RTC_IsOverflowed(void);
	void TD_RTC_ClearOverflow(void);
	void TD_RTC_SetOffsetTime(int delta);
	TD_RTC_PowerMode_t TD_RTC_GetPowerMode(void);
	void TD_RTC_AlarmAfter(int32_t delay);
	void TD_RTC_UserAlarmAfter(int32_t delay);
	int32_t TD_RTC_SignedTimeDiff(uint32_t reference);
	time_t __time32(time_t *timer);
	void TD_RTC_SetPowerModeInternal(TD_RTC_PowerMode_t mode, uint32_t line);
	void TD_RTC_EnterPowerMode(void);
	void TD_RTC_DelayAbort(void);

	/** @} */

	/** @} (end addtogroup RTC) */

#ifdef __cplusplus
}
#endif

#endif // __TD_RTC_H
