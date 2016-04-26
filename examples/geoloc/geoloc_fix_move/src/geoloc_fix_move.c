/******************************************************************************
 * @file
 * @brief Simple GPS fix upon movement detection application for the TDxxxx RF modules.
 * @author Telecom Design S.A.
 * @version 1.2.0
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2014-2016 Telecom Design S.A., http://www.telecomdesign.fr</b>
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

#include "config.h"

#include <stdint.h>
#include <stdbool.h>

#include <efm32.h>

#include <td_core.h>
#include <td_rtc.h>
#include <td_uart.h>
#include <td_printf.h>
#include <td_stream.h>
#include <td_flash.h>
#include <td_watchdog.h>

#include <td_sensor.h>
#include <sensor_data_geoloc.h>

#include <td_accelero.h>
#include <td_geoloc.h>

#include <td_config.h>

/*******************************************************************************
 ******************************  DEFINES  ************************************
 ******************************************************************************/

/** Flash variable version ID */
#define VARIABLES_VERSION 0

/** Stop trying to fix after timeout (seconds) */
#define FIX_TIMEOUT 120

/** Movement ignore period after a first movement is detected */
#define MOVE_IGNORE_TIMEOUT 0

/** Acceptable minimum horizontal accuracy, 800 to be very accurate */
#define FIX_HDOP 800

/** Boot monitoring, 1 to enable */
#define BOOT_MONITORING 1

/** Keepalive monitoring interval in hours, 0 to disable */
#define KEEPALIVE_INTERVAL 24

/*******************************************************************************
 ******************************  VARIABLES  ************************************
 ******************************************************************************/

/** Flag to enable GPS fixes */
static bool FixingEnabled = true;

/*******************************************************************************
 ******************************  GLOBAL FUNCTIONS  ****************************
 ******************************************************************************/

/***************************************************************************//**
 * @brief
 *  GPS fix callback
 *
 * @param[in] fix
 *   The GPS fix data structure.
 *
 * @param[in] timeout
 *   Flag that indicates whether a timeout occurred if set to true.
 ******************************************************************************/
static void GPSFix(TD_GEOLOC_Fix_t *fix, bool timeout)
{
	if ((fix->type >= TD_GEOLOC_2D_FIX && fix->hard.rtc_calibrated) || timeout) {
		tfp_printf("Stop fixing\r\n");

		// Stop GPS in hardware backup mode
		TD_GEOLOC_StopFix(TD_GEOLOC_HW_BCKP);

		// Send position
		TD_SENSOR_SendDataPosition(GPS_DATA_XYZ_SV_HDOP, fix, 0, 0);
		if (MOVE_IGNORE_TIMEOUT == 0) {

			// No ignore period, restart fixing right away
			tfp_printf("Fixing on move enabled\r\n");
			FixingEnabled = true;
		}
	}
}

/***************************************************************************//**
 * @brief
 *  Restart fixing on movement detection.
 *
 * @param[in] arg
 *  Generic argument set by the user that is passed along to the callback
 *  function.
 *
 * @param[in] repeat_count
 *  Updated repeat count, decremented at each timer trigger, unless it is an
 *  infinite timer.
 ******************************************************************************/
static void RestartMoveDetection(uint32_t arg, uint8_t repeat_count)
{
	tfp_printf("Fixing on move enabled\r\n");
	FixingEnabled = true;
}

/***************************************************************************//**
 * @brief
 *  Called on move detection
 *
 * @param[in] source
 *   The event source mask that triggered the event.
 ******************************************************************************/
void MoveDetected(uint8_t source)
{
	tfp_printf("Movement detected\r\n");
	if (FixingEnabled) {

		// Disable restarting the GPS until a position is found
		FixingEnabled = false;
		tfp_printf("Start GPS\r\n");
		TD_GEOLOC_TryToFix(TD_GEOLOC_NAVIGATION, FIX_TIMEOUT, GPSFix);
		if (MOVE_IGNORE_TIMEOUT > 0 ) {

			// Launch a one-shot timer to trigger after the given timeout
			TD_SCHEDULER_Append(MOVE_IGNORE_TIMEOUT,
				0,
				0,
				TD_SCHEDULER_ONE_SHOT,
				RestartMoveDetection,
				0);
		}
	}
}

/***************************************************************************//**
 * @brief
 *  User Setup function.
 ******************************************************************************/
void TD_USER_Setup(void)
{
	TD_UART_Options_t options = {LEUART_DEVICE, LEUART_LOCATION, 9600, 8, 'N',
		1, false};

	// Open an I/O stream using LEUART0
	TD_UART_Open(&options, TD_STREAM_RDWR);

	// Set flash variables version
	TD_FLASH_SetVariablesVersion(VARIABLES_VERSION);

	// Use a 64 s automatic watchdog
	TD_WATCHDOG_Init(64);
	TD_WATCHDOG_Enable(true, true);

	// Sensor as transmitter
	TD_SENSOR_Init(SENSOR_TRANSMITTER, 0, 0);

	// Geoloc and accelerometer initialization
	TD_GEOLOC_Init();
	TD_ACCELERO_Init();

#if BOOT_MONITORING

	// Will only send a boot monitor frame on NEXT reboot
	TD_SENSOR_MonitorBoot(true, 0);
#endif

#if KEEPALIVE_INTERVAL > 0

	// Send a keep-alive frame immediately, then at given interval
	TD_SENSOR_MonitorKeepAlive(true, KEEPALIVE_INTERVAL);
#endif

	// Monitor accelerometer events (movements)
	TD_ACCELERO_MonitorEvent(true,	// Monitoring enabled
		TD_ACCELERO_10HZ, 			// Sampling rate 10 Hz
		TD_ACCELERO_ALL_AXIS,		// Axis mask: all axis
		TD_ACCELERO_2G,				// Scale 2 g
		TD_ACCELERO_ALL_HIGH_IRQ,	// Only monitor high IRQs, as low IRQs are always set in
									// accelerometer registers
		10,							// Threshold in mg = 10 * 2 g / 127 = +- 160 mg (with scale 2 g)
		6,							// Duration in ms = 6 * (1 / 10 Hz) = 600 ms (with rate 10 Hz)
		1,							// High-pass filter enabled
		MoveDetected);
}

/***************************************************************************//**
 * @brief
 *   User loop function.
 ******************************************************************************/
void TD_USER_Loop(void)
{

	// Process local RF events
	TD_LAN_Process();

	// Process Sensor events
	TD_SENSOR_Process();

	// Process Geoloc events
	TD_GEOLOC_Process();

	// Process Accelerometer events
	TD_ACCELERO_Process();
}
