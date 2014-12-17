/***************************************************************************//**
 * @file
 * @brief Accelerometer API for the TDxxxx RF modules.
 * @author Telecom Design S.A.
 * @version 1.0.2
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

#include <td_spi.h>
#include <td_core.h>
#include <td_module.h>
#include <td_trap.h>
#include <td_rtc.h>
#include <td_spi.h>
#include <td_flash.h>
#include <td_printf.h>
#include <td_sensor.h>

#include "lis3dh.h"
#include "td_accelero.h"

/***************************************************************************//**
 * @addtogroup TD_ACCELERO Accelerometer
 * @brief Accelerometer API for the TDxxxx RF modules.
 * @{
 ******************************************************************************/

/*******************************************************************************
 *************************   DEFINES   *******************************
 ******************************************************************************/

/** @addtogroup TD_ACCELERO_DEFINES
 * @{ */

//#define DEBUG_STD

/** Turn on trace mode if tfp_printf not commented */
#ifdef DEBUG_STD
#define DEBUG_PRINTF(...) tfp_printf(__VA_ARGS__)
#else
#define DEBUG_PRINTF(...)
#endif

/** Max number of samples that can be read at once. 32 is FIFO size */
#define ACCELERO_BUF_SIZE	32

/** Soft high pass filter with Fc=1/(2*Pi*(ALPHA/10)) */
#define HIGH_PASS_ALPHA 1

/** @} */

/*******************************************************************************
 *************************   PRIVATE VARIABLES   *******************************
 ******************************************************************************/

/** @addtogroup TD_ACCELERO_LOCAL_VARIABLES Local Variables
 * @{ */

/** Accelerometer data callback function pointer */
static void (*DataCallback)(TD_ACCELERO_Data_t data[32], uint8_t count,
	bool overrun);

/** Accelerometer event callback function pointer */
static void (*EventCallback)(uint8_t source);

/** Accelerometer event callback function pointer */
static void (*ClickEventCallback)(uint8_t source);

/** Current Accelerometer scale */
//static TD_ACCELERO_Scales_t CurrentScale = TD_ACCELERO_2G;

/** Current High-pass filter enable flag */
static bool Filter = false;

/** Accelerometer configuration for the TDxxxx RF module */
static  TD_ACCELERO_Config_t TD_Accelero;

/** Configuration flag */
static bool ConfigInit = false;

/** Current Interrupt flag */
static volatile bool AcceleroIrq = false;

/** @} */

/*******************************************************************************
 **************************  PRIVATE FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup TD_ACCELERO_FUNCTIONS Private Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Accelerometer event IRQ handler. Called by Sensor SwitchIrq Monitoring.
 *
 * @param[in] mask
 *    IRQ mask, unused
 ******************************************************************************/
static void AccelerometerIRQ(uint32_t mask)
{
	DEBUG_PRINTF("I\r\n");
	AcceleroIrq = true;
}

/***************************************************************************//**
 * @brief
 *   Soft High pass as accelerometer is bugged on Z.
 *
 * @param data
 *  Pointer to accelerometry data array to filter
 *
 * @param count
 *  Array data count.
 ******************************************************************************/
static void HighPassFilter(TD_ACCELERO_Data_t *data, uint8_t count)
{
	int i;
	static int16_t xmem = 0, ymem = 0, zmem = 0, xf = 0, yf = 0, zf = 0;

	for (i = 0; i < count; i++) {
		xf = (HIGH_PASS_ALPHA * (xf + data[i].x - xmem)) / 10;
		xmem = data[i].x;
		yf = (HIGH_PASS_ALPHA * (yf + data[i].y - ymem)) / 10;
		ymem = data[i].y;
		zf = (HIGH_PASS_ALPHA * (zf + data[i].z - zmem)) / 10;
		zmem = data[i].z;
		data[i].x = xf;
		data[i].y = yf;
		data[i].z = zf;
	}
}

/***************************************************************************//**
 * @brief
 *   Accelero process. Subset of operation used in MonitorData mode.
 ******************************************************************************/
static void ProcessMonitorData(TD_ACCELERO_Data_t *data, uint8_t sz)
{
	int count = 0;
	bool overrun = false;
	uint8_t st, i;

	if (DataCallback != 0) {

		// Empty FIFO by reading all data
		if (!TD_SPI_Lock(ACCELERO_SPI_ID, NULL)) {
			TD_Trap(TRAP_SPI_NOT_AVAILABLE, __LINE__);
			return;
		}

		// Read current status
		st = TD_LIS3DH_GetFIFOStatus();

		// Has an overrun occured ?
		overrun = st & 0x40;

		// How much should we read from FIFO ?
		count = st & 0x1F;
		if (count > sz) {

			// If we can't get all data, remember to get in another time
			AcceleroIrq = true;
			TD_WakeMainLoop();

			// And clamp ...
			count = sz;
		}
		for (i = 0; i < count; i++) {
			TD_LIS3DH_GetXYZ((uint16_t*)&data[i].x,
				(uint16_t*)&data[i].y,(uint16_t*)&data[i].z);
		}

		// There are still samples but buffer overflows: need an other turn
		if (GPIO_PinInGet(CONFIG_ACCELERO_IRQ_PORT, CONFIG_ACCELERO_IRQ_BIT) &&
			count != 32) {
			DEBUG_PRINTF("IT not clear CNT:%d st:0x%02X!!\r\n", count, st);
		}
		if (!count){
			TD_SPI_UnLock(ACCELERO_SPI_ID);
			return;
		}

		// If filter is activated
		if (Filter) {
			//TD_LIS3DH_SetFilterRef();
			HighPassFilter(data, count);
		}
		TD_SPI_UnLock(ACCELERO_SPI_ID);

		// Push all data to callback
		(*DataCallback)(data, count, overrun);
	}
}

/** @} */

/*******************************************************************************
 **************************  PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup TD_ACCELERO_USER_FUNCTIONS User Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Power down the accelerometer.
 ******************************************************************************/
void TD_ACCELERO_PowerDown(void)
{
	if (TD_SPI_Lock(ACCELERO_SPI_ID, NULL)) {
		TD_LIS3DH_Configure(TD_ACCELERO_POWER_DOWN, 0, 0, 0);
		TD_SPI_UnLock(ACCELERO_SPI_ID);
	} else {
		TD_Trap(TRAP_SPI_NOT_AVAILABLE, __LINE__);
	}
}

/***************************************************************************//**
 * @brief
 *   Set the accelerometer in normal power mode.
 *
 * @param[in] rate
 *    The accelerometer sampling rate taken from the available TD_ACCELERO_Rates.
 *
 * @param[in] axis
 *    The accelerometer axis mask.
 *
 * @param[in] scale
 *   The accelerometer measurement scale taken from the available
 *   TD_ACCELERO_Scales.
 ******************************************************************************/
void TD_ACCELERO_NormalPower(TD_ACCELERO_Rates_t rate, uint8_t axis,
	TD_ACCELERO_Scales_t scale)
{
	if (TD_SPI_Lock(ACCELERO_SPI_ID, NULL)) {
		TD_LIS3DH_Configure(TD_ACCELERO_NORMAL_POWER, rate, axis, scale);
		TD_SPI_UnLock(ACCELERO_SPI_ID);
	} else {
		TD_Trap(TRAP_SPI_NOT_AVAILABLE, __LINE__);
	}
}

/***************************************************************************//**
 * @brief
 *   Set the accelerometer in low power mode.
 *
 * @param[in] rate
 *    The accelerometer sampling rate taken from the available TD_ACCELERO_Rates.
 *
 * @param[in] axis
 *    The accelerometer axis mask.
 *
 * @param[in] scale
 *   The accelerometer measurement scale taken from the available
 *   TD_ACCELERO_Scales.
 ******************************************************************************/
void TD_ACCELERO_LowPower(TD_ACCELERO_Rates_t rate, uint8_t axis,
	TD_ACCELERO_Scales_t scale)
{
	if (TD_SPI_Lock(ACCELERO_SPI_ID, NULL)) {
		TD_LIS3DH_Configure(TD_ACCELERO_LOW_POWER, rate, axis, scale);
		TD_SPI_UnLock(ACCELERO_SPI_ID);
	} else {
		TD_Trap(TRAP_SPI_NOT_AVAILABLE, __LINE__);
	}
}

/** @} */

/** @addtogroup TD_ACCELERO_GLOBAL_FUNCTIONS Global Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Dump all accelerometer registers for debug purposes.
 ******************************************************************************/
void TD_ACCELERO_Dump(void)
{
	uint8_t source;
	int i = 0;

	for (i = 0x07; i <= 0x3D; i++) {
		if (TD_SPI_Lock(ACCELERO_SPI_ID, NULL)) {
			source = TD_SPI_FullReadRegister(ACCELERO_SPI_ID,i);
			TD_SPI_UnLock(ACCELERO_SPI_ID);
			tfp_printf("Reg 0x%02x 0x%02x\r\n", i, source);
		} else {
			TD_Trap(TRAP_SPI_NOT_AVAILABLE, __LINE__);
		}
	}
	if (TD_SPI_Lock(ACCELERO_SPI_ID, NULL)) {
		TD_LIS3DH_Dump();
		TD_SPI_UnLock(ACCELERO_SPI_ID);
	}
}

/** @} */

/** @addtogroup TD_ACCELERO_USER_FUNCTIONS User Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Initialize the accelerometer. Must be called in User_Setup for proper
 *   Accelerometer operation. Will startup monitoring if previously applied
 *   using SetConfig.
 *
 * @return
 *   Returns true if the accelerometer initialized successfully, false otherwise.
 ******************************************************************************/
bool TD_ACCELERO_Init(void)
{
	bool ret = false;

	// Register the accelerometer on the SPI bus
	TD_SPI_Register(ACCELERO_SPI_ID, 0xFF, CONFIG_ACCELERO_SPI_BUS, 10000000,
		(USART_ClockMode_TypeDef) CONFIG_ACCELERO_SPI_MODE);
	TD_SPI_RegisterCS(ACCELERO_SPI_ID, CONFIG_ACCELERO_CS_PORT,
		CONFIG_ACCELERO_CS_BIT);

	// Lock SPI bus
	if (TD_SPI_Lock(ACCELERO_SPI_ID, NULL)) {

		// Driver initialization and register read
		if (TD_LIS3DH_Init()) {
			ret = true;
		}
		if (ret) {

			// If config was loaded using setconfig, use it, otherwise
			// initialize
			if (ConfigInit) {
				switch (TD_Accelero.monitoring) {
				case TD_ACCELERO_MONITOR_DATA:
					TD_ACCELERO_MonitorEvent(false,
						(TD_ACCELERO_Rates_t) 0,
						0,
						(TD_ACCELERO_Scales_t) 0,
						0,
						0,
						0,
						0,
						0);
					TD_ACCELERO_MonitorData(true,
						TD_Accelero.config.low_power,
						TD_Accelero.config.rate,
						TD_Accelero.data_config.axis,
						TD_Accelero.config.scale,
						TD_Accelero.config.filter,
						(TD_ACCELERO_FifoModes_t) 2,
						0,
						TD_Accelero.data_config.user_callback);
					TD_LIS3DH_ClearIRQ();
					break;

				case TD_ACCELERO_MONITOR_EVENT:
					TD_ACCELERO_MonitorData(false, false,
						(TD_ACCELERO_Rates_t) 0,
						0,
						(TD_ACCELERO_Scales_t) 0,
						0,
						(TD_ACCELERO_FifoModes_t) 0,
						0,
						0);
					TD_ACCELERO_MonitorEvent(true,
						TD_Accelero.config.rate,
						TD_Accelero.event_config.axis,
						TD_Accelero.config.scale,
						TD_Accelero.event_config.event,
						TD_Accelero.event_config.threshold,
						TD_Accelero.event_config.duration,
						TD_Accelero.config.filter,
						TD_Accelero.event_config.user_callback);
					TD_LIS3DH_ClearIRQ();
					break;

				default:
					TD_ACCELERO_PowerDown();
					break;
				}
			} else {
				TD_ACCELERO_PowerDown();
				TD_Accelero.monitoring = TD_ACCELERO_NO_MONITORING;
				TD_Accelero.event_config.click = false;
				TD_Accelero.irq_source = 0;
			}
		}
		TD_SPI_UnLock(ACCELERO_SPI_ID);
	} else {
		TD_Trap(TRAP_SPI_NOT_AVAILABLE, __LINE__);
	}
	return ret;
}

/***************************************************************************//**
 * @brief
 *   Return accelerometer configuration
 *
 * @return cutoff
 *    Return pointer to accelerometer configuration structure
 ******************************************************************************/
TD_ACCELERO_Config_t *TD_ACCELERO_GetConfig(void)
{
	return &TD_Accelero;
}

/***************************************************************************//**
 * @brief
 *   Set accelerometer configuration but does NOT apply it
 *
 * @param[in] config
 *    Pointer to the accelerometer configuration structure
 *    containing new configuration
 ******************************************************************************/
void TD_ACCELERO_SetConfig(TD_ACCELERO_Config_t *config)
{
	memcpy(&TD_Accelero, config, sizeof (TD_ACCELERO_Config_t));
	ConfigInit = true;
}

/***************************************************************************//**
 * @brief
 *   Allow reading one register of the accelerometer.
 *
 * @param[in] address
 *  Register address to read the value from.
 *
 * @return[in] value
 *  Returns the read value.
 ******************************************************************************/
uint8_t TD_ACCELERO_ReadRegister(uint8_t address)
{
	uint8_t value = 0;

	if (TD_SPI_Lock(ACCELERO_SPI_ID, NULL)) {
		value = TD_SPI_FullReadRegister(ACCELERO_SPI_ID, address);
		TD_SPI_UnLock(ACCELERO_SPI_ID);
	} else {
		TD_Trap(TRAP_SPI_NOT_AVAILABLE, __LINE__);
	}
	return value;
}

/***************************************************************************//**
 * @brief
 *   Allow writing to the accelerometer register
 *
 * @param[in] address
 *  Register address to write to.
 *
 * @param[in] value
 *  Value to write.
 ******************************************************************************/
void TD_ACCELERO_WriteRegister(uint8_t address, uint8_t value)
{
	if (TD_SPI_Lock(ACCELERO_SPI_ID, NULL)) {
		TD_SPI_FullWriteRegister(ACCELERO_SPI_ID, address, value);
		TD_SPI_UnLock(ACCELERO_SPI_ID);
	} else {
		TD_Trap(TRAP_SPI_NOT_AVAILABLE, __LINE__);
	}
}

/***************************************************************************//**
 * @brief
 *   Clear accelerometer filter.
 ******************************************************************************/
void TD_ACCELERO_ClearFilter(void)
{
	if (TD_SPI_Lock(ACCELERO_SPI_ID, NULL)) {
		TD_LIS3DH_SetFilterRef();
		TD_SPI_UnLock(ACCELERO_SPI_ID);
	}
}

/***************************************************************************//**
 * @brief
 *   Change watermark information.
 *
 * @param[in] watermark
 *   Watermark level.
 ******************************************************************************/
void TD_ACCELERO_SetWatermark(uint8_t watermark)
{
	if (TD_SPI_Lock(ACCELERO_SPI_ID, NULL)) {
		TD_LIS3DH_SetFIFOMode(TD_ACCELERO_STREAM, watermark);

		// To ensure correct processing
		AcceleroIrq = true;
		TD_SPI_UnLock(ACCELERO_SPI_ID);
	} else {
		TD_Trap(TRAP_SPI_NOT_AVAILABLE, __LINE__);
	}
}

/***************************************************************************//**
 * @brief
 *   Setup accelerometer click event monitoring. Provide additional high
 *   value event. Event monitoring must be enabled. Double click not handled.
 *
 * @param[in] enable
 *    Enable accelerometer event monitoring if true, disable if false.
 *
 * @param[in] event
 *    The click event mask to monitor. Available events are
 *    LIS3DH_INT1_CFG_XS
 *    LIS3DH_INT1_CFG_YS
 *    LIS3DH_INT1_CFG_ZS
 *
 * @param[in] threshold
 *    The threshold value to trigger the click.
 *
 * @param[in] duration
 *    The minimum duration to trigger the click.
 *
 * @param[in] callback
 *   Pointer to the function to call back whenever an event occurs. This
 *   function will receive the event source mask as its argument.
 ******************************************************************************/
void TD_ACCELERO_MonitorClickEvent(bool enable, uint8_t event,
	uint8_t threshold, uint8_t duration, void (*callback)(uint8_t source))
{
	TD_Accelero.event_config.click = enable;
	if (TD_SPI_Lock(ACCELERO_SPI_ID, NULL)) {
		if (enable)	{
			TD_LIS3DH_ConfigureClickEvent(event, threshold, duration);
			if(callback != NULL) {
				if(TD_Accelero.irq_source == 0) {
					LIS3DH_EnableInterrupt(AccelerometerIRQ);
				}
				TD_Accelero.irq_source |= 1<<7;
			} else {
				TD_Accelero.irq_source &= 0x7F;
				if(TD_Accelero.irq_source == 0) {
					LIS3DH_DisableInterrupt();
				}
			}
			TD_LIS3DH_ConfigureInterrupt(TD_Accelero.irq_source, true);

			// Clear previous interrupt
			TD_LIS3DH_GetClickEventSource();
			ClickEventCallback = callback;
		} else {

			// Remove interrupt
			TD_Accelero.irq_source &= 0x7F;
			if(TD_Accelero.irq_source == 0) {
				LIS3DH_DisableInterrupt();
			}
			TD_LIS3DH_ConfigureInterrupt(TD_Accelero.irq_source, true);

			// Remove callback and event
			ClickEventCallback = NULL;
			TD_LIS3DH_ConfigureClickEvent(0,0,0);
			TD_LIS3DH_GetClickEventSource();
		}
		TD_SPI_UnLock(ACCELERO_SPI_ID);
	} else {
		TD_Trap(TRAP_SPI_NOT_AVAILABLE, __LINE__);
	}
}

/***************************************************************************//**
 * @brief
 *   Setup accelerometer event monitoring.
 *
 * @param[in] enable
 *    Enable accelerometer event monitoring if true, disable if false.
 *
 * @param[in] rate
 *    The accelerometer sampling rate taken from the available TD_ACCELERO_Rates.
 *
 * @param[in] axis
 *    The accelerometer axis mask.
 *
 * @param[in] scale
 *   The accelerometer measurement scale taken from the available
 *   TD_ACCELERO_Scales.
 *
 * @param[in] event
 *    The event mask to monitor.
 *
 * @param[in] threshold
 *    The threshold value to trigger an event.
 *
 * @param[in] duration
 *    The minimum duration to trigger an event.
 *
 * @param[in] filter
 *   Accelerometer high-pass filter enable flag: enabled if true, disabled if
 *   false.
 *
 * @param[in] callback
 *   Pointer to the function to call back whenever an event occurs. This
 *   function will receive the event source mask as its argument.
 ******************************************************************************/
void TD_ACCELERO_MonitorEvent(bool enable, TD_ACCELERO_Rates_t rate,
	uint8_t axis, TD_ACCELERO_Scales_t scale, uint8_t event, uint8_t threshold,
	uint8_t duration, int8_t filter, void (*callback)(uint8_t source))
{
	if (TD_SPI_Lock(ACCELERO_SPI_ID, NULL)) {
		if (enable)	{
			TD_Accelero.monitoring = TD_ACCELERO_MONITOR_EVENT;
			TD_Accelero.config.low_power = true;
			TD_Accelero.config.rate = rate;
			TD_Accelero.config.scale = scale;
			TD_Accelero.config.filter = filter;
			TD_Accelero.event_config.axis = axis;
			TD_Accelero.event_config.threshold = threshold;
			TD_Accelero.event_config.duration = duration;
			TD_Accelero.event_config.event = event;
			TD_Accelero.event_config.user_callback = callback;
			EventCallback = callback;
			Filter = (filter > 0 ? true : false);

			// Change power
			TD_ACCELERO_LowPower(rate, axis, scale);
			TD_LIS3DH_ConfigureEvent(event, threshold, duration);
			if (callback != NULL) {
				if (TD_Accelero.irq_source == 0) {
					LIS3DH_EnableInterrupt(AccelerometerIRQ);
				}
				TD_Accelero.irq_source |= 1 << 6;
			} else {
				TD_Accelero.irq_source &= 0xBF;
				if(TD_Accelero.irq_source == 0) {
					LIS3DH_DisableInterrupt();
				}
			}
			TD_LIS3DH_ConfigureInterrupt(TD_Accelero.irq_source, true);
			TD_LIS3DH_ClearIRQ();
			TD_LIS3DH_ConfigHighPass(Filter, filter);
		} else {
			TD_Accelero.irq_source &= 0xBF;
			if(TD_Accelero.irq_source == 0) {
				LIS3DH_DisableInterrupt();
			}
			TD_LIS3DH_ConfigureInterrupt(0, true);
			TD_LIS3DH_ConfigureEvent(0, 0, 0);
			TD_LIS3DH_ClearIRQ();
			TD_Accelero.monitoring = TD_ACCELERO_NO_MONITORING;
		}
		TD_SPI_UnLock(ACCELERO_SPI_ID);
	} else {
		TD_Trap(TRAP_SPI_NOT_AVAILABLE, __LINE__);
	}
}

/***************************************************************************//**
 * @brief
 *   Setup accelerometer data monitoring. This function will give you access to
 *   raw xyz data from the accelerometer.
 *
 * @param[in] enable
 *    Enable accelerometer data monitoring if true, disable if false.
 *
 * @param[in] low_power
 *     Enable low-power mode if true, disable if false.
 *
 * @param[in] rate
 *    The accelerometer sampling rate taken from the available TD_ACCELERO_Rates.
 *
 * @param[in] axis
 *    The accelerometer axis mask.
 *
 * @param[in] scale
 *   The accelerometer measurement scale taken from the available
 *   TD_ACCELERO_Scales.
 *
 * @param[in] filter
 *   Accelerometer high-pass filter enable flag: enabled if true, disabled if
 *   false.
 *
 * @param[in] fifo
 *   The FIFO mode to use for the data.
 *
 * @param[in] watermark
 *   The number of bytes to wait before triggering an IRQ.
 *
 * @param[in] callback
 *   Pointer to the function to call back whenever new data arrives. This
 *   function will receive a xyz array with up to 32 accelerometer values as
 *   well as data count and an overrun flag as arguments.
 ******************************************************************************/
void TD_ACCELERO_MonitorData(bool enable, bool low_power,
	TD_ACCELERO_Rates_t rate, uint8_t axis, TD_ACCELERO_Scales_t scale,
	int8_t filter, TD_ACCELERO_FifoModes_t fifo, uint8_t watermark,
	void (*callback)(TD_ACCELERO_Data_t data[32], uint8_t count, bool overrun))
{
	if (TD_SPI_Lock(ACCELERO_SPI_ID, NULL)) {
		if (callback != 0 && enable) {
			TD_Accelero.monitoring = TD_ACCELERO_MONITOR_DATA;
			TD_Accelero.config.low_power = low_power;
			TD_Accelero.config.rate = rate;
			TD_Accelero.config.scale = scale;
			TD_Accelero.config.filter = filter;
			TD_Accelero.data_config.axis = axis;
			TD_Accelero.data_config.user_callback = callback;
			DataCallback = callback;
			//TD_LIS3DH_ClearIRQ();
			LIS3DH_EnableInterrupt(AccelerometerIRQ);
			if (fifo == TD_ACCELERO_FIFO) {
				TD_LIS3DH_ConfigureInterrupt(LIS3DH_IRQ_DRDY1, true);
			}

			// Use stream mode and set IRQ as soon as a sample is available
			TD_LIS3DH_SetFIFOMode(fifo, watermark);
			Filter = (filter > 0 ? true : false);
			DEBUG_PRINTF("MonitorData: low_power:%d rate:%d axis:%d scale:%d\r\n",
				low_power, rate, axis, scale);
			DEBUG_PRINTF("Port:%d Bit:%d\r\n", CONFIG_ACCELERO_IRQ_PORT,
				CONFIG_ACCELERO_IRQ_BIT);
			if (low_power) {
				TD_ACCELERO_LowPower(rate, axis, scale);
			} else {
				TD_ACCELERO_NormalPower(rate, axis, scale);
			}
		} else if (!enable) {
			LIS3DH_DisableInterrupt();
			TD_ACCELERO_PowerDown();
			TD_LIS3DH_ClearIRQ();
			TD_Accelero.monitoring = TD_ACCELERO_NO_MONITORING;
		}
		TD_SPI_UnLock(ACCELERO_SPI_ID);
	} else {
		TD_Trap(TRAP_SPI_NOT_AVAILABLE, __LINE__);
	}
}

/***************************************************************************//**
 * @brief
 *   Accelero process. Subset of operation used in MonitorData mode.
 ******************************************************************************/
void TD_ACCELERO_ProcessMonitorData(TD_ACCELERO_Data_t *data,uint8_t sz)
{
	if (AcceleroIrq) {
		AcceleroIrq = false;
		if (TD_Accelero.monitoring == TD_ACCELERO_MONITOR_DATA) {
			ProcessMonitorData(data, sz);
		}
	}
}

/***************************************************************************//**
 * @brief
 *   Accelero process. Will read data from accelerometer if interrupt flag
 *   is set for current monitoring. Will call user callback. Must be called
 *   in User_Loop for proper accelerometer operation.
 ******************************************************************************/
void TD_ACCELERO_Process(void)
{
	TD_ACCELERO_Data_t data[ACCELERO_BUF_SIZE];
	uint8_t source = 0;
	uint8_t click_source = 0;

	if (AcceleroIrq) {
		AcceleroIrq = false;
		if (TD_Accelero.monitoring == TD_ACCELERO_MONITOR_EVENT) {
			if (EventCallback != 0) {

				// If there is no callback IRQ handling is made somewhere else
				// so don't kill IRQ source unless we have a callback
				if (TD_SPI_Lock(ACCELERO_SPI_ID, NULL)) {

					// Clear event flag on accelerometer and allow IRQ renewal
					source = TD_LIS3DH_GetEventSource();
					TD_SPI_UnLock(ACCELERO_SPI_ID);
				} else {
					TD_Trap(TRAP_SPI_NOT_AVAILABLE, __LINE__);
				}
				if (source & 64) {
					(*EventCallback)(source & TD_Accelero.event_config.event);
				}
			}
			if (ClickEventCallback != 0) {
				if (TD_SPI_Lock(ACCELERO_SPI_ID, NULL)) {
					click_source = TD_LIS3DH_GetClickEventSource();
					TD_SPI_UnLock(ACCELERO_SPI_ID);
				} else {
					TD_Trap(TRAP_SPI_NOT_AVAILABLE, __LINE__);
				}
				if (click_source & 64) {
					ClickEventCallback(click_source);
				}
			}
			if (Filter) {
				if (TD_SPI_Lock(ACCELERO_SPI_ID, NULL)) {
					TD_LIS3DH_SetFilterRef();
					TD_SPI_UnLock(ACCELERO_SPI_ID);
				} else {
					TD_Trap(TRAP_SPI_NOT_AVAILABLE, __LINE__);
				}
			}
		} else if (TD_Accelero.monitoring == TD_ACCELERO_MONITOR_DATA) {
			ProcessMonitorData(data, ACCELERO_BUF_SIZE);
		}
	}
}

/** @} */

/** @} (end addtogroup TD_ACCELERO) */
