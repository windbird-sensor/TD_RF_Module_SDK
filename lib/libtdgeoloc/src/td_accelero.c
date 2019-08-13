/***************************************************************************//**
 * @file
 * @brief Accelerometer API for the TDxxxx RF modules.
 * @author Telecom Design S.A.
 * @version 1.2.1
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2013-2019 Telecom Design S.A., http://www.telecomdesign.fr</b>
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
#include <em_cmu.h>
#include <em_letimer.h>

#include <td_spi.h>
#include <td_core.h>
#include <td_module.h>
#include <td_trap.h>
#include <td_rtc.h>
#include <td_spi.h>
#include <td_flash.h>
#include <td_printf.h>
#include <td_sensor.h>
#include <td_scheduler.h>

#include "lis3dh.h"
#include "td_accelero.h"


/***************************************************************************//**
 * @addtogroup TD_ACCELERO Accelerometer
 * @brief Accelerometer API for the TDxxxx RF modules.
 * @{
 ******************************************************************************/

/*******************************************************************************
 *************************   DEFINES   *****************************************
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

/** Minimum acceleration period in ticks */
#define ACC_PERIOD_MIN		1245	// 38ms

/** Maximum acceleration period in ticks */
#define ACC_PERIOD_MAX		1375	// 42ms

/** Macro to set accelerometer chip select low */
#define CSACC_LOW 	  	{GPIO->P[gpioPortD].DOUTCLR = 1 << 3;}

/** Macro to set accelerometer chip select high */
#define CSACC_HIGH 	  	{GPIO->P[gpioPortD].DOUTSET = 1 << 3;}

/* LIS3DH accelerometer registers */
#define LIS3DH_READ_MODE			0x80			/**< LIS3DH Read Mode mask */
#define LIS3DH_INC_ADD_MODE			0x40			/**< LIS3DH increment address mode */
#define LIS3DH_INT_COUNTER			0x0E			/**< LIS3DH IRQ counter register */
#define LIS3DH_CTRL_REG1			0x20			/**< LIS3DH Control Register 1 */
#define LIS3DH_CTRL_REG1_5000HZ		(0x9 << 4)		/**< LIS3DH 5000 Hz sampling rate mask */
#define LIS3DH_CTRL_REG1_1600HZ		(0x8 << 4)		/**< LIS3DH 2600 Hz sampling rate mask */
#define LIS3DH_CTRL_REG1_400HZ		(0x7 << 4)		/**< LIS3DH 400 Hz sampling rate mask */
#define LIS3DH_CTRL_REG1_200HZ		(0x6 << 4)		/**< LIS3DH 200 Hz sampling rate mask */
#define LIS3DH_CTRL_REG1_100HZ		(0x5 << 4)		/**< LIS3DH 100 Hz sampling rate mask */
#define LIS3DH_CTRL_REG1_50HZ		(0x4 << 4)		/**< LIS3DH 50 Hz sampling rate mask */
#define LIS3DH_CTRL_REG1_25HZ		(0x3 << 4)		/**< LIS3DH 25 Hz sampling rate mask */
#define LIS3DH_CTRL_REG1_10HZ		(0x2 << 4)		/**< LIS3DH 10 Hz sampling rate mask */
#define LIS3DH_CTRL_REG1_1HZ		(0x1 << 4)		/**< LIS3DH 1 Hz sampling rate mask */
#define LIS3DH_CTRL_REG1_OFF		0				/**< LIS3DH sampling off */
#define LIS3DH_CTRL_REG1_ENX		0x1				/**< LIS3DH Enable X axis mask */
#define LIS3DH_CTRL_REG1_ENY		0x2				/**< LIS3DH Enable Y axis mask */
#define LIS3DH_CTRL_REG1_ENZ		0x4				/**< LIS3DH Enable Z axis mask */
#define LIS3DH_CTRL_REG1_LOW_POWER	0x8				/**< LIS3DH Enable low-power mask */
#define LIS3DH_CTRL_REG2			0x21			/**< LIS3DH Control Register 2 */
#define LIS3DH_CTRL_REG3			0x22			/**< LIS3DH Control Register 3 */
#define LIS3DH_CTRL_REG4			0x23			/**< LIS3DH Control Register 4 */
#define LIS3DH_HIGH_RES 			0x08			/**< LIS3DH Enable high-resolution mask */
#define LIS3DH_CTRL_REG5			0x24			/**< LIS3DH Control Register 5 */
#define LIS3DH_REF					0x26			/**< LIS3DH reference register */
#define LIS3DH_OUT_X_L				0x28			/**< LIS3DH X-axis acceleration data register */
#define LIS3DH_INT1CFG				0x30			/**< LIS3DH IRQ1 configuration register */
#define LIS3DH_INT1CSRC				0x31			/**< LIS3DH IRQ1 source register */
#define LIS3DH_INT_THRESOLD			0x32			/**< LIS3DH IRQ thresold register */
#define LIS3DH_INT_DURATION			0x33			/**< LIS3DH IRQ duration register */

/** Accelerometer sampling frequency */
#define ACCELERO_FREQ 				LIS3DH_CTRL_REG1_25HZ

/** Acceleromter low-pass filter delay macro*/
#define DELAY_MS_LP(x)			{\
		RTC->IFC = 2;\
		RTC->COMP0 = (RTC->CNT + (uint16_t) (x * 33)) & 0xFFFFFF; \
		while (!(RTC->IF & 2)) {\
			;\
		}\
}

/** Accelerometer lock context for polling */
#define TD_SPI_LOCK_CONTEXT_ACCELERO_POLLING		254

/** Clear dummy value in UART FIFO */
#define DUMMY_VAL	USART_CMD_CLEARRX | USART_CMD_CLEARTX

//#define SAFE_MODE
//#define LEGACY_MODE
//#define NO_STOP_MODE

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

/** Current Interrupt flags */
static volatile bool AcceleroIrq = false;
static volatile bool AcceleroIrqData = false;
static volatile bool CallbackOnIRQ = false;

/** Current Interrupt stamps */
static volatile uint32_t AcceleroStampProc = 0;
static volatile uint32_t AcceleroStampIrq = 0;
static volatile uint32_t AcceleroStampCB = 0;

/** Accelero data pointer */
static TD_ACCELERO_Data_t DataPoll[8];

/** Current Polling pointer */
static volatile uint8_t CursPoll = 0;

static volatile uint8_t SampleBatch = 8;

/** Current Polling pointer */
static uint8_t PollingID = 0xFF;

/** Ready frame indicator */
static uint8_t ready = 0;

/** Current Drift flag */
static volatile uint8_t AcceleroDrift = 0;

static volatile uint8_t AcceleroDriftMem = 0;

/** RTC */
static uint32_t rtc = 0, rtc_mem = 0;

static uint32_t	sampleVar = 1;

/** @} */

/*******************************************************************************
 **************************  PRIVATE FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup TD_ACCELERO_FUNCTIONS Private Functions
 * @{ */

//#ifdef ACCELERO_POLLING
/***************************************************************************//**
 * @brief
 *  Accelerometer polling callback function.
 ******************************************************************************/
static void AcceleroPollingIRQ(void)
{
	TD_ACCELERO_Polling(0, 0);
}

//#else
//#endif

/***************************************************************************//**
 * @brief
 *   Accelerometer event IRQ handler. Called by Sensor SwitchIrq Monitoring.
 *
 * @param[in] mask
 *    IRQ mask, unused
 ******************************************************************************/
static void AccelerometerIRQ(uint32_t mask)
{
	AcceleroStampProc = RTC->CNT;
	if (CallbackOnIRQ) {
		(*EventCallback)(0);
	} else {
		DEBUG_PRINTF("I\r\n");
		AcceleroIrq = true;
		TD_WakeMainLoop();
	}
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
//#ifndef ACCELERO_POLLING
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
//#endif

/***************************************************************************//**
 * @brief
 *   Accelero process. Subset of operation used in MonitorData mode.
 *
 * @param[in] data
 *   Pointer to the accelerometer data buffer to process.
 *
 * @param[in] sz
 *   Size in bytes of the accelerometer data buffer to process.
 ******************************************************************************/
//#ifdef ACCELERO_POLLING
//#else
static void ProcessMonitorData(TD_ACCELERO_Data_t *data, uint8_t sz)
{
	int count = 0;
	bool overrun = false;
	/** Accelero data pointer */
	TD_ACCELERO_Data_t Backup[8];
	uint8_t st;

	if (DataCallback != 0) {
		if (CONFIG_ACCELERO_POLLING_MODE){
			// tfp_printf("%d*X=%6d|Y=%6d|Z=%6d|S=%d|PROCESS=1\r\n",
			// sampleVar, DataPoll[0].x, DataPoll[0].y, DataPoll[0].z, CursPoll);
			memcpy(Backup, DataPoll, sizeof (TD_ACCELERO_Data_t) * 8);
			if (CursPoll != 0) {
				TD_Trap(TRAP_ACCELERO_MODE, 7);
			}
			AcceleroStampProc = (RTC->CNT - AcceleroStampProc) & 0xFFFFFF;
			AcceleroStampCB = (RTC->CNT - AcceleroStampCB) & 0xFFFFFF;
			if (AcceleroStampCB > T1MS) {
				AcceleroDrift |= DRIFT_CB_ERR;
			}
			overrun = (AcceleroDrift != 0) ? true : false;
			AcceleroDriftMem = AcceleroDrift;

			// Reset Error cause
			AcceleroDrift = 0;

			// Push all data to callback
			(*DataCallback)(Backup, SampleBatch, overrun);
		} else {

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
			TD_LIS3DH_GetXYZExt((uint16_t*)&data[0].x,count);

			// There are still samples but buffer overflows: need an other turn
			if (GPIO_PinInGet(CONFIG_ACCELERO_IRQ_PORT, CONFIG_ACCELERO_IRQ_BIT) &&
					!AcceleroIrq) {
				DEBUG_PRINTF("IT not clear CNT:%d st:0x%02X!!\r\n", count, st);
				TD_Trap(TRAP_ACCELERO_MODE, __LINE__);
				AcceleroIrq = true;
				TD_WakeMainLoop();
			}
			if (!count) {
				TD_SPI_UnLock(ACCELERO_SPI_ID);
				return;
			}

			// If filter is activated
			if (Filter) {
				//TD_LIS3DH_SetFilterRef();
				HighPassFilter(data, count);
			}
			TD_SPI_UnLock(ACCELERO_SPI_ID);

			AcceleroStampProc = (RTC->CNT - AcceleroStampProc)&0xFFFFFF;
			// Push all data to callback
			(*DataCallback)(data, count, overrun);
		}
	}
}
//#endif

/** @} */

/*******************************************************************************
 **************************  PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

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

	memset (&TD_Accelero, 0, sizeof(TD_ACCELERO_Config_t));
	TD_Accelero.allow_spi = true;
	// Register the accelerometer on the SPI bus
	TD_SPI_Register(ACCELERO_SPI_ID, 0xFF, CONFIG_ACCELERO_SPI_BUS,
			CONFIG_ACCELERO_SPI_SPEED,
			(USART_ClockMode_TypeDef) CONFIG_ACCELERO_SPI_MODE);

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
				TD_Accelero.monitoring_event = TD_ACCELERO_NO_MONITORING;
				TD_Accelero.monitoring_data = TD_ACCELERO_NO_MONITORING;
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
 *   Accelero process. Will read data from accelerometer if interrupt flag
 *   is set for current monitoring. Will call user callback. Must be called
 *   in User_Loop for proper accelerometer operation.
 ******************************************************************************/
void TD_ACCELERO_Process(void)
{
	TD_ACCELERO_Data_t data[ACCELERO_BUF_SIZE];

	uint8_t source = 0;
	uint8_t click_source = 0;

	if (CONFIG_ACCELERO_POLLING_MODE) {
		if (AcceleroIrq) {
			AcceleroIrq = false;
			if (TD_Accelero.monitoring_event == TD_ACCELERO_MONITOR) {
				if (EventCallback != 0) {
					(*EventCallback)(source & TD_Accelero.event_config.event);
				}
			}
		}
		if (AcceleroIrqData) {
			AcceleroIrqData = false;
			if (TD_Accelero.monitoring_data == TD_ACCELERO_MONITOR) {
				ProcessMonitorData(data, ACCELERO_BUF_SIZE);
			}
		}
	} else {
		if (AcceleroIrq || GPIO_PinInGet(CONFIG_ACCELERO_IRQ_PORT, CONFIG_ACCELERO_IRQ_BIT)) {
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
 *    The event mask to monitor. a TD_ACCELERO_IRQ_XL,XH,YL,YH,ZL,ZH combination
 *    or TD_ACCELERO_ALL_HIGH_IRQ or TD_ACCELERO_ALL_LOW_IRQ or
 *    TD_ACCELERO_ALL_IRQ
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
void DYNAMIC (TD_ACCELERO_MonitorEvent) (bool enable, TD_ACCELERO_Rates_t rate,
		uint8_t axis, TD_ACCELERO_Scales_t scale, uint8_t event, uint8_t threshold,
		uint8_t duration, int8_t filter, void (*callback)(uint8_t source))
		{
	DEBUG_PRINTF("MonitorEvent %d\r\n", enable);
	if (TD_SPI_Lock(ACCELERO_SPI_ID, NULL)) {
		if (enable)	{
			if (TD_Accelero.monitoring != TD_ACCELERO_NO_MONITORING) {
				TD_Trap(TRAP_ACCELERO_MODE, 1);
			}
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

			// Configure event for irq
			TD_LIS3DH_ConfigureEvent(event, threshold, duration);

			// Configure internal LIS3DH interrupt pipeline : disable all
			// sources
			TD_LIS3DH_ConfigureInterrupt(0, true);

			// Configure high pass filter
			TD_LIS3DH_ConfigHighPass(Filter, filter);

			// LIS3DH configured : Now clear IRQ on LIS3DH side
			TD_LIS3DH_ClearIRQ();

			// Can now configure IRQ on EFM32 side
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

			// Configure internal LIS3DH interrupt pipeline, enable wanted
			// sources
			TD_LIS3DH_ConfigureInterrupt(TD_Accelero.irq_source, true);
		} else {
			if (TD_Accelero.monitoring != TD_ACCELERO_MONITOR_EVENT){
				TD_Trap(TRAP_ACCELERO_MODE,5);
			}
			TD_Accelero.irq_source &= 0xBF;
			if(TD_Accelero.irq_source == 0) {
				LIS3DH_DisableInterrupt();
			}
			TD_LIS3DH_ConfigureInterrupt(0, true);
			TD_LIS3DH_ConfigureEvent(0, 0, 0);
			TD_LIS3DH_ClearIRQ();
			TD_LIS3DH_ConfigHighPass(false, 0);
			TD_Accelero.monitoring = TD_ACCELERO_NO_MONITORING;
			AcceleroIrq = false;
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
 *    The event mask to monitor. a TD_ACCELERO_IRQ_XL,XH,YL,YH,ZL,ZH combination
 *    or TD_ACCELERO_ALL_HIGH_IRQ or TD_ACCELERO_ALL_LOW_IRQ or TD_ACCELERO_ALL_IRQ
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
void DYNAMIC (TD_ACCELERO_MonitorEventPolling) (bool enable,
		TD_ACCELERO_Rates_t rate, uint8_t axis, TD_ACCELERO_Scales_t scale,
		uint8_t event, uint8_t threshold, uint8_t duration, int8_t filter,
		void (*callback)(uint8_t source))
		{
	bool enable_filter = filter > 0 ? true : false;

	DEBUG_PRINTF("MonitorEvent %d\r\n", enable);
	if (enable)	{
		// SPI Is locked during whole Event monitoring (internal lock handled by
		// spi_allow)
		if (TD_SPI_LockContext(ACCELERO_SPI_ID,
				NULL, TD_SPI_LOCK_CONTEXT_ACCELERO_POLLING)) {
			if (TD_Accelero.monitoring_event != TD_ACCELERO_NO_MONITORING) {
				TD_SPI_UnLockContext(ACCELERO_SPI_ID,
					TD_SPI_LOCK_CONTEXT_ACCELERO_POLLING);
				return;
			}
			TD_Accelero.monitoring_event = TD_ACCELERO_MONITOR;
			TD_Accelero.config.low_power = true;
			TD_Accelero.config.rate = rate;
			TD_Accelero.config.scale = scale;
			TD_Accelero.event_config.axis = axis;
			TD_Accelero.event_config.user_callback = callback;

			EventCallback = callback;
			CallbackOnIRQ = true;
			Filter = enable_filter;

			// Low Power
			TD_SPI_FullWriteRegister(ACCELERO_SPI_ID, LIS3DH_CTRL_REG1, 0x3F);

			// Configure event for irq
			if ((TD_Accelero.event_config.threshold!=threshold) ||
					(TD_Accelero.event_config.duration != duration) ||
					(TD_Accelero.event_config.event != event)){
				TD_Accelero.event_config.threshold = threshold;
				TD_Accelero.event_config.duration = duration;
				TD_Accelero.event_config.event = event;
				TD_LIS3DH_ConfigureEvent(event, threshold, duration);
			}

			// Configure internal LIS3DH interrupt pipeline : disable all sources
			TD_LIS3DH_ConfigureInterrupt(0, true);
			if (TD_Accelero.config.filter != enable_filter){
				TD_Accelero.config.filter = enable_filter;

				// Configure high pass filter
				TD_LIS3DH_ConfigHighPass(Filter, enable_filter);
			}
			if (TD_Accelero.config.filter){
				// To have consitant filter detection (DC compomant removal)
				// and no false interrupt trigger operation, we must :
				// -activate sampling (done at top)
				// -read REFERENCE_CAPTURE to set filter ref (probably just a
				//  bit that would tell to do a ref capture on next sampling)
				// -clear IRQ. Why here ? Why it don't work when cleared after?
				//  IRQ state is not enabled immediately, but at each sample ?
				TD_LIS3DH_SetFilterRef();
				TD_LIS3DH_ClearIRQ();
				// -wait for sample to occur. On a LIS3DH it seems it take
				//  about 1.5ms to be done
				TD_RTC_Delay(T2MS);
			}
			// LIS3DH configured : Now clear IRQ on LIS3DH side (in case of no
			// filtering. In case of filtering, have been done above but would
			// it really work in case of no filtering ?)
			TD_LIS3DH_ClearIRQ();

			// Can now configure IRQ on EFM32 side
			if (callback != NULL) {
				if (TD_Accelero.irq_source == 0) {
					LIS3DH_EnableInterrupt(AccelerometerIRQ);
					DEBUG_PRINTF("EnableIRQ %08X\r\n", AccelerometerIRQ);
				}
				TD_Accelero.irq_source |= 1 << 6;
			} else {
				TD_Accelero.irq_source &= 0xBF;
				if(TD_Accelero.irq_source == 0) {
					LIS3DH_DisableInterrupt();
				}
			}

			// Configure internal LIS3DH interrupt pipeline, enable wanted
			// sources
			TD_LIS3DH_ConfigureInterrupt(TD_Accelero.irq_source, true);
		} else {
			TD_Trap(TRAP_SPI_NOT_AVAILABLE, __LINE__);
		}
	} else {
		if (TD_Accelero.monitoring_event != TD_ACCELERO_MONITOR){
			TD_Trap(TRAP_ACCELERO_MODE,5);
		}
		TD_Accelero.irq_source &= 0xBF;
		if(TD_Accelero.irq_source == 0) {
			LIS3DH_DisableInterrupt();
		}
		TD_LIS3DH_ConfigureInterrupt(0, true);

		// Do not clear event (just IRQ) - mandatory watch
		//TD_LIS3DH_ConfigureEvent(0, 0, 0);
		TD_LIS3DH_ClearIRQ();
		// Clear filter only if set
		// Do not clear filter for interrupt
		//if (TD_Accelero.config.filter) {
		//	TD_LIS3DH_ConfigHighPass(false, 0);
		//}
		TD_Accelero.monitoring_event = TD_ACCELERO_NO_MONITORING;
		AcceleroIrq = false;
		CallbackOnIRQ = false;
		TD_SPI_UnLockContext(ACCELERO_SPI_ID, TD_SPI_LOCK_CONTEXT_ACCELERO_POLLING);
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
//#ifndef ACCELERO_POLLING
void DYNAMIC(TD_ACCELERO_MonitorData) (bool enable, bool low_power,
		TD_ACCELERO_Rates_t rate, uint8_t axis, TD_ACCELERO_Scales_t scale,
		int8_t filter, TD_ACCELERO_FifoModes_t fifo, uint8_t watermark,
		TD_ACCELERO_CB callback)
		{
	DEBUG_PRINTF("MonitorData %d\r\n", enable);
	if (TD_SPI_Lock(ACCELERO_SPI_ID, NULL)) {
		if (callback != 0 && enable) {
			if (TD_Accelero.monitoring != TD_ACCELERO_NO_MONITORING) {
				TD_Trap(TRAP_ACCELERO_MODE,2);
			}
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

			Filter = (filter > 0 ? true : false);
			DEBUG_PRINTF("MonitorData: low_power:%d rate:%d axis:%d scale:%d\r\n",
					low_power, rate, axis, scale);
			DEBUG_PRINTF("Port:%d Bit:%d\r\n", CONFIG_ACCELERO_IRQ_PORT,
					CONFIG_ACCELERO_IRQ_BIT);
			TD_ACCELERO_Power(rate, axis, scale,
					low_power ? TD_ACCELERO_LOW_POWER : TD_ACCELERO_12_BITS);

			// Use stream mode and set IRQ as soon as a sample is available
			// Setup FIFO after correct mode setting
			TD_LIS3DH_SetFIFOMode(fifo, watermark);
		} else if (!enable) {
			if (TD_Accelero.monitoring != TD_ACCELERO_MONITOR_DATA) {
				TD_Trap(TRAP_ACCELERO_MODE,6);
			}
			LIS3DH_DisableInterrupt();
			TD_ACCELERO_PowerDown();
			TD_LIS3DH_SetFIFOMode(TD_ACCELERO_BYPASS, 0);
			TD_LIS3DH_ClearIRQ();
			TD_Accelero.monitoring = TD_ACCELERO_NO_MONITORING;
			AcceleroIrq = false;
		}
		TD_SPI_UnLock(ACCELERO_SPI_ID);
	} else {
		TD_Trap(TRAP_SPI_NOT_AVAILABLE, __LINE__);
	}
}

/***************************************************************************//**
 * @brief
 *   Setup accelerometer data monitoring. This function replace the original one
 *   with 40ms RAW access to accelero and callbacks every 8 samples.
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
//#else
void DYNAMIC(TD_ACCELERO_MonitorDataPolling)(bool enable, bool low_power,
		TD_ACCELERO_Rates_t rate, uint8_t axis, TD_ACCELERO_Scales_t scale,
		int8_t filter, TD_ACCELERO_FifoModes_t fifo, uint8_t watermark,
		TD_ACCELERO_CB callback)
		{
	DEBUG_PRINTF("MonitorData %d\r\n", enable);
	if (callback != 0 && enable) {
		if (TD_Accelero.monitoring_data != TD_ACCELERO_NO_MONITORING) {
			return;
		}
		TD_Accelero.monitoring_data = TD_ACCELERO_MONITOR;
		TD_Accelero.config.low_power = low_power;
		TD_Accelero.config.rate = rate;
		TD_Accelero.config.scale = scale;
		TD_Accelero.config.filter = filter;
		TD_Accelero.data_config.axis = axis;
		TD_Accelero.data_config.user_callback = callback;
		DataCallback = callback;
		TD_ACCELERO_InitLETIMER ();
		if (TD_SPI_LockContext(ACCELERO_SPI_ID, NULL,
				TD_SPI_LOCK_CONTEXT_ACCELERO_POLLING)) {
			if (TD_SPI_FullReadRegister(ACCELERO_SPI_ID,
					LIS3DH_CTRL_REG4) != 0x88) {

				// Full resolution
				TD_SPI_FullWriteRegister(ACCELERO_SPI_ID, LIS3DH_CTRL_REG4, 0x88);
			}
		} else {
			TD_Trap(TRAP_SPI_NOT_AVAILABLE, __LINE__);
		}
	} else if (!enable) {
		if (TD_Accelero.monitoring_data != TD_ACCELERO_MONITOR) {
			TD_Trap(TRAP_ACCELERO_MODE, 6);
		}
		TD_ACCELERO_StopLETIMER ();
		TD_Accelero.monitoring_data = TD_ACCELERO_NO_MONITORING;
		CursPoll = 0;
		AcceleroIrqData = false;
		AcceleroDrift = false;
		TD_SPI_UnLockContext(ACCELERO_SPI_ID, TD_SPI_LOCK_CONTEXT_ACCELERO_POLLING);
	}
}
//#endif

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
 *   Return Drift information
 *
 * @return drift
 *   Has drift occured
 ******************************************************************************/
uint8_t TD_ACCELERO_GetDrift(void)
{
	return AcceleroDrift;
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
 *   Return Accelero ID
 ******************************************************************************/
uint8_t TD_ACCELERO_Who(void)
{
	uint8_t value = 0;

	if (TD_SPI_Lock(ACCELERO_SPI_ID, NULL)) {
		value = TD_LIS3DH_Who();
		TD_SPI_UnLock(ACCELERO_SPI_ID);
	} else {
		TD_Trap(TRAP_SPI_NOT_AVAILABLE, __LINE__);
	}
	return value;
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
 *   Accelero process. Subset of operation used in MonitorData mode.
 *   Do not use with TD_ACCELERO_Process().
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
 *    The accelerometer axis mask. (bit0:X, bit1:Y, bit2:Z)
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

/***************************************************************************//**
 * @brief
 *   Get acceleration values.
 *
 * @param[in] data
 *  Pointer to acceleration values to fill.
 ******************************************************************************/
void TD_ACCELERO_GetXYZ(TD_ACCELERO_Data_t * data)
{
	if (TD_SPI_Lock(ACCELERO_SPI_ID, NULL)) {
		TD_LIS3DH_GetXYZ((uint16_t *) &data->x, (uint16_t *) &data->y,
				(uint16_t *) &data->z);
		TD_SPI_UnLock(ACCELERO_SPI_ID);
	} else {
		TD_Trap(TRAP_SPI_NOT_AVAILABLE, __LINE__);
	}
}

/***************************************************************************//**
 * @brief
 *   Get acceleration sample values.
 *
 * @param[in] data
 *  Pointer to the buffer that will receive the acceleration values.
 ******************************************************************************/
void TD_ACCELERO_SampleGetXYZ(TD_ACCELERO_Data_t *data)
{
	CSACC_LOW;
	USART1->CMD = USART_CMD_CLEARRX | USART_CMD_CLEARTX;
	USART1->TXDOUBLE = LIS3DH_CTRL_REG1 + ((ACCELERO_FREQ | LIS3DH_CTRL_REG1_ENX |
			LIS3DH_CTRL_REG1_ENY | LIS3DH_CTRL_REG1_ENZ) << 8);
	while (!(USART1->STATUS & USART_STATUS_TXC)) {
		;
	}
	CSACC_HIGH;
#ifdef SAFE_MODE
	TD_RTC_CalibratedDelay(3000);
#else
	TD_RTC_Delay(TMS(3));
#endif

#ifndef NO_STOP_MODE
	CSACC_LOW;
	USART1->CMD = USART_CMD_CLEARRX | USART_CMD_CLEARTX;
	USART1->TXDOUBLE = LIS3DH_CTRL_REG1 + ((LIS3DH_CTRL_REG1_OFF |
			LIS3DH_CTRL_REG1_ENX | LIS3DH_CTRL_REG1_ENY | LIS3DH_CTRL_REG1_ENZ) << 8);
	while (!(USART1->STATUS & USART_STATUS_TXC)) {
		;
	}
	CSACC_HIGH;
#endif

#ifdef SAFE_MODE
	TD_RTC_CalibratedDelay(200);
#endif

	// Get samples
	/* Enable chip select */
	CSACC_LOW;
	USART1->CMD = USART_CMD_CLEARRX | USART_CMD_CLEARTX;
	USART1->TXDATA = LIS3DH_OUT_X_L | LIS3DH_READ_MODE | LIS3DH_INC_ADD_MODE;
#ifndef SAFE_MODE
	USART1->TXDOUBLE = 0;
#else
	TD_RTC_CalibratedDelay(200);
	USART1->TXDATA = 0;
	TD_RTC_CalibratedDelay(200);
	USART1->TXDATA = 0;
	TD_RTC_CalibratedDelay(200);
#endif
	while (!(USART1->STATUS & USART_STATUS_TXC)) ;
	data->x = USART1->RXDATA;
	data->x = USART1->RXDOUBLE;

#ifndef SAFE_MODE
	USART1->TXDOUBLE = 0;
#else
	USART1->TXDATA = 0;
	TD_RTC_CalibratedDelay(200);
	USART1->TXDATA = 0;
	TD_RTC_CalibratedDelay(200);
#endif

	while (!(USART1->STATUS & USART_STATUS_TXC)) {
		;
	}
	data->y = USART1->RXDOUBLE;
#ifndef SAFE_MODE
	USART1->TXDOUBLE = 0;
#else
	TD_RTC_CalibratedDelay(200);
	USART1->TXDATA = 0;
	TD_RTC_CalibratedDelay(200);
	USART1->TXDATA = 0;
	TD_RTC_CalibratedDelay(200);
#endif

	while (!(USART1->STATUS & USART_STATUS_TXC)) {
		;
	}
	data->z = USART1->RXDOUBLE;
	CSACC_HIGH;
}

/***************************************************************************//**
 * @brief
 *   Get raw acceleration values
 *
 * @param[in] data
 *  Pointer to the buffer that will receive the raw acceleration values.
 ******************************************************************************/
void TD_ACCELERO_RawGetXYZ(TD_ACCELERO_Data_t *data)
{
#ifndef NO_STOP_MODE
	CSACC_LOW;
	USART1->CMD = USART_CMD_CLEARRX | USART_CMD_CLEARTX;
	USART1->TXDOUBLE = LIS3DH_CTRL_REG1 + ((LIS3DH_CTRL_REG1_OFF |
			LIS3DH_CTRL_REG1_ENX | LIS3DH_CTRL_REG1_ENY | LIS3DH_CTRL_REG1_ENZ) << 8);
	while (!(USART1->STATUS & USART_STATUS_TXC)) {
		;
	}
	CSACC_HIGH;
#endif

#ifdef SAFE_MODE
	TD_RTC_CalibratedDelay(200);
#endif

	// Get samples
	/* Enable chip select */
	CSACC_LOW;
	USART1->CMD = USART_CMD_CLEARRX | USART_CMD_CLEARTX;
	USART1->TXDATA = LIS3DH_OUT_X_L | LIS3DH_READ_MODE | LIS3DH_INC_ADD_MODE;
#ifndef SAFE_MODE
	USART1->TXDOUBLE = 0;
#else
	TD_RTC_CalibratedDelay(200);
	USART1->TXDATA = 0;
	TD_RTC_CalibratedDelay(200);
	USART1->TXDATA = 0;
	TD_RTC_CalibratedDelay(200);
#endif
	while (!(USART1->STATUS & USART_STATUS_TXC)) {
		;
	}
	data->x = USART1->RXDATA;
	data->x = USART1->RXDOUBLE;

#ifndef SAFE_MODE
	USART1->TXDOUBLE = 0;
#else
	USART1->TXDATA = 0;
	TD_RTC_CalibratedDelay(200);
	USART1->TXDATA = 0;
	TD_RTC_CalibratedDelay(200);
#endif

	while (!(USART1->STATUS & USART_STATUS_TXC)) {
		;
	}
	data->y = USART1->RXDOUBLE;
#ifndef SAFE_MODE
	USART1->TXDOUBLE = 0;
#else
	TD_RTC_CalibratedDelay(200);
	USART1->TXDATA = 0;
	TD_RTC_CalibratedDelay(200);
	USART1->TXDATA = 0;
	TD_RTC_CalibratedDelay(200);
#endif

	while (!(USART1->STATUS & USART_STATUS_TXC)) {
		;
	}
	data->z = USART1->RXDOUBLE;
	CSACC_HIGH;
}

/***************************************************************************//**
 * @brief
 *   Sample raw accelerometer values.
 ******************************************************************************/
void TD_ACCELERO_RawSampleXYZ(void)
{
#define DUMMY_VAL	USART_CMD_CLEARRX | USART_CMD_CLEARTX
	CSACC_LOW;
	USART1->CMD = USART_CMD_CLEARRX | USART_CMD_CLEARTX;
	USART1->TXDOUBLE = LIS3DH_CTRL_REG1 + ((ACCELERO_FREQ | LIS3DH_CTRL_REG1_ENX |
			LIS3DH_CTRL_REG1_ENY | LIS3DH_CTRL_REG1_ENZ) << 8);
	while (!(USART1->STATUS & USART_STATUS_TXC)) {
		;
	}
	CSACC_HIGH;
}

/***************************************************************************//**
 * @brief
 *   Set accelerometer sample batch for CB
 *
 * @param[in] data
 ******************************************************************************/
void TD_ACCELERO_SetBatch(uint8_t data)
{
	AcceleroIrqData = false;
	SampleBatch = data;
}


/***************************************************************************//**
 * @brief
 *   Set monitoring status to NONE
 *
 ******************************************************************************/
void TD_ACCELERO_PauseMonitor(void)
{
	TD_Accelero.monitoring_data = TD_ACCELERO_NO_MONITORING;
}


/***************************************************************************//**
 * @brief
 *   Set monitoring status to NONE
 *
 ******************************************************************************/
void TD_ACCELERO_PauseScheduler(void)
{
	if (PollingID != 0xFF) {
		TD_SCHEDULER_Remove (PollingID);
		PollingID = 0xFF;
	}
}

/***************************************************************************//**
 * @brief
 *   Set monitoring status to Monitor Data
 *
 ******************************************************************************/
void TD_ACCELERO_ResumeMonitor(void)
{
	TD_Accelero.monitoring_data = TD_ACCELERO_MONITOR;
}

/***************************************************************************//**
 * @brief
 *   Set monitoring status to Monitor Data
 *
 ******************************************************************************/
void TD_ACCELERO_ResumeScheduler(void)
{
	if (PollingID == 0xFF){
		PollingID = TD_SCHEDULER_AppendIrq (0, T40MS, 0, 0xFF, TD_ACCELERO_Polling, 0);
	}
}

/***************************************************************************//**
 * @brief
 *   Set the accelerometer in specific power mode.
 *
 * @param[in] rate
 *    The accelerometer sampling rate taken from the available TD_ACCELERO_Rates.
 *
 * @param[in] axis
 *    The accelerometer axis mask. (bit0:X, bit1:Y, bit2:Z)
 *
 * @param[in] scale
 *   The accelerometer measurement scale taken from the available
 *   TD_ACCELERO_Scales.
 *
 * @param[in] mode
 *   The accelerometer power mode taken from the available
 *   TD_ACCELERO_PowerModes_t.
 ******************************************************************************/
void TD_ACCELERO_Power(TD_ACCELERO_Rates_t rate, uint8_t axis,
		TD_ACCELERO_Scales_t scale,TD_ACCELERO_PowerModes_t mode)
{
	if (TD_SPI_Lock(ACCELERO_SPI_ID, NULL)) {
		TD_LIS3DH_Configure(mode, rate, axis, scale);
		TD_SPI_UnLock(ACCELERO_SPI_ID);
	} else {
		TD_Trap(TRAP_SPI_NOT_AVAILABLE, __LINE__);
	}
}

/***************************************************************************//**
 * @brief
 *   Return current accelero monitoring
 *
 * @return
 *   Returns a pointer to the current accelerometer monitoring structure.
 ******************************************************************************/
TD_ACCELERO_Monitoring_t TD_ACCELERO_MonitorStatus(void)
{
	return TD_Accelero.monitoring;
}


/***************************************************************************//**
 * @brief
 *   Return current accelero monitoring
 ******************************************************************************/
TD_ACCELERO_Monitoring_t TD_ACCELERO_MonitorDataStatus(void)
{
	return TD_Accelero.monitoring_data;
}


/***************************************************************************//**
 * @brief
 *   Return current accelero monitoring
 ******************************************************************************/
TD_ACCELERO_Monitoring_t TD_ACCELERO_MonitorEventStatus(void)
{
	return TD_Accelero.monitoring_event;
}

/***************************************************************************//**
 * @brief
 *   Check if accelerometer IRQ is enabled. Debug purpose only!
 *
 * @return
 *   Returns a debug code.
 ******************************************************************************/
uint8_t TD_ACCELERO_DebugCheck(void)
{
	uint32_t mode;

	if (TD_GPIO_GetCallbackExtended(CONFIG_ACCELERO_IRQ_BIT) !=
			AccelerometerIRQ) {
		return 1;
	}
	mode = (CONFIG_ACCELERO_IRQ_BIT < 8) ?
			GPIO->P[CONFIG_ACCELERO_IRQ_PORT].MODEL :
			GPIO->P[CONFIG_ACCELERO_IRQ_PORT].MODEH;
	if ((mode>>((CONFIG_ACCELERO_IRQ_BIT&0x7)<<2))!=gpioModeInput) {
		return 2;
	}
	mode = (CONFIG_ACCELERO_IRQ_BIT < 8) ? GPIO->EXTIPSELL : GPIO->EXTIPSELH;
	if ((mode >> ((CONFIG_ACCELERO_IRQ_BIT & 0x7) << 2)) !=
			CONFIG_ACCELERO_IRQ_PORT) {
		return 3;
	}
	if (!((GPIO->EXTIRISE>>CONFIG_ACCELERO_IRQ_BIT) & 0x1)) {
		return 4;
	}
	if (((GPIO->EXTIFALL>>CONFIG_ACCELERO_IRQ_BIT) & 0x1)) {
		return 4;
	}
	if (!((GPIO->IEN>>CONFIG_ACCELERO_IRQ_BIT) & 0x1)) {
		return 5;
	}
	return 0;
}

/***************************************************************************//**
 * @brief
 *   Return accelerometer delay time between IRQ and process.
 *
 * @return
 *   Returns accelerometer delay time between IRQ and process.
 ******************************************************************************/
uint32_t TD_ACCELERO_ProcessTime(void)
{
	return AcceleroStampProc;
}

/***************************************************************************//**
 * @brief
 *   Return accelerometer IRQ timestamp.
 *
 * @return
 *   Returns accelerometer IRQ timestamp.
 ******************************************************************************/
uint32_t TD_ACCELERO_IrqTime(void)
{
	return AcceleroStampIrq;
}

/***************************************************************************//**
 * @brief
 *   Return accelerometer callback timestamp.
 *
 * @return
 *   Returns accelerometer callback timestamp.
 ******************************************************************************/
uint32_t TD_ACCELERO_CallbackTime(void)
{
	return AcceleroStampCB;
}

/***************************************************************************//**
 * @brief
 *  Read ready count on a 8 read-poll.
 ******************************************************************************/
uint8_t TD_ACCELERO_StatusReg(void)
{
	uint8_t ret;

	if (CONFIG_ACCELERO_POLLING_MODE) {
		ret = ready;
		ready = 0;
	} else{
		ret = 0;
	}
	return ret;
}

/***************************************************************************//**
 * @brief
 *   Return accelerometer drift.
 *
 * @return
 *   Returns accelerometer drift.
 ******************************************************************************/
uint8_t TD_ACCELERO_GetCause(void)
{
	return AcceleroDriftMem;
}

/***************************************************************************//**
 * @brief
 *   Accelerometer callback function in Polling mode.
 *
 ******************************************************************************/
void TD_ACCELERO_Polling(uint32_t arg, uint8_t repetition)
{
	if (TD_SPI_Lock(ACCELERO_SPI_ID, AcceleroPollingIRQ)) {
		rtc_mem = (RTC->CNT - rtc) & 0xFFFFFF;
		rtc = RTC->CNT;
		if ((rtc_mem < ACC_PERIOD_MIN || rtc_mem > ACC_PERIOD_MAX) &&
				rtc_mem != RTC->CNT) {
			//tfp_printf("drift\r\n");
			AcceleroDrift |= DRIFT_POLL_ERR;
			AcceleroStampIrq = rtc_mem;
		}
		if (AcceleroIrqData) {
			AcceleroDrift |= OVERRUN_ERR;
		}
		TD_GPIO_PinModeSet((TD_GPIO_Port_TypeDef) gpioPortD, 1, gpioModeInputPull, 0);
		TD_GPIO_PinModeSet((TD_GPIO_Port_TypeDef) gpioPortD, 3, gpioModePushPull, 1);

#ifdef TRACE_RTC_SAMPLE
		DataPoll[CursPoll].rtc_before = RTC->CNT;
#endif
		TD_ACCELERO_SampleGetXYZ(&DataPoll[CursPoll]);
#ifdef TRACE_RTC_SAMPLE
		DataPoll[CursPoll].rtc_after = RTC->CNT;
#endif

		//tfp_printf("%d*X=%d|Y=%d|Z=%d\r\n", sampleVar, DataPoll[CursPoll].x,
		// DataPoll[CursPoll].y, DataPoll[CursPoll].z, CursPoll);
		//tfp_printf("%d*S=%d|IRQ=1\r\n", sampleVar, CursPoll);
		sampleVar++;
		TD_SPI_UnLock(ACCELERO_SPI_ID);
		if (CursPoll >= (SampleBatch - 1)) {

			// Push all data to callback
			AcceleroIrqData = true;
			AcceleroStampCB = RTC->CNT;
			CursPoll = 0;
			TD_WakeMainLoop();
		} else {

			// Next sample
			CursPoll++;
		}
	}
}

/***************************************************************************//**
 * @brief
 *   Accelerometer callback function in Polling mode.
 ******************************************************************************/
void TD_ACCELERO_PollingGet(void)
{
	uint8_t batch;

	if (TD_Accelero.allow_spi) {
		TD_ACCELERO_RawGetXYZ(&DataPoll[CursPoll]);
#ifdef TRACE_RTC_SAMPLE
		DataPoll[CursPoll].rtc_after = RTC->CNT;
#endif
		/*
    tfp_printf("%d*X=%d|Y=%d|Z=%d\r\n", sampleVar, DataPoll[CursPoll].x,
      DataPoll[CursPoll].y, DataPoll[CursPoll].z, CursPoll);
    tfp_printf("%d*S=%d|IRQ=1\r\n", sampleVar, CursPoll);
		 */
		sampleVar++;
		batch = SampleBatch;
		if (CursPoll >= (batch - 1)) {
			AcceleroIrqData	= true;
			AcceleroStampCB	= RTC->CNT;
			CursPoll = 0;
			TD_WakeMainLoop();
		} else {
			CursPoll++;
		}
	} else {
		//tfp_printf("D %d\r\n", rtc_debug);
		TD_Trap(TRAP_ACCELERO_MODE,
				(((RTC->CNT-TD_Accelero.allow_spi_rtc) & 0xFFFFFF) << 8) |
				(TD_Accelero.allow_spi_tag << 4) | 10);
	}
}

/***************************************************************************//**
 * @brief
 *   Sample Accelerometer in Polling mode.
 ******************************************************************************/
void TD_ACCELERO_PollingSample(void)
{
	if (TD_Accelero.allow_spi) {
		rtc_mem = (RTC->CNT - rtc) & 0xFFFFFF;
		rtc = RTC->CNT;
		if ((rtc_mem < ACC_PERIOD_MIN || rtc_mem > ACC_PERIOD_MAX) &&
				rtc_mem != RTC->CNT) {
			AcceleroDrift |= DRIFT_POLL_ERR;
			AcceleroStampIrq = rtc_mem;
		}
		if (AcceleroIrqData) {
			AcceleroDrift |= OVERRUN_ERR;
		}

#ifdef TRACE_RTC_SAMPLE
		DataPoll[CursPoll].rtc_before = RTC->CNT;
#endif
		TD_ACCELERO_RawSampleXYZ();
	} else {
		//tfp_printf("D %d\r\n", rtc_debug);
		TD_Trap(TRAP_ACCELERO_MODE,
				(((RTC->CNT-TD_Accelero.allow_spi_rtc) & 0xFFFFFF) << 8) |
				(TD_Accelero.allow_spi_tag << 4) | 9);
	}
}

/***************************************************************************//**
 * @brief
 *   LETIMER IRQ initialization for Accelero polling mode
 ******************************************************************************/
void TD_ACCELERO_InitLETIMER(void)
{
	LETIMER_Init_TypeDef letimerInit = LETIMER_INIT_DEFAULT;

	CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);
	CMU_ClockEnable(cmuClock_LETIMER0, true);

	letimerInit.enable = false;
	letimerInit.comp0Top = true;
	letimerInit.repMode = letimerRepeatFree;
	LETIMER_Init(LETIMER0, &letimerInit);
	LETIMER_CompareSet(LETIMER0, 0, 1310);
	LETIMER_CompareSet(LETIMER0, 1, 1211);
	LETIMER_RepeatSet(LETIMER0, 0, 0x01);
	LETIMER_RepeatSet(LETIMER0, 1, 0x01);
	LETIMER_IntEnable(LETIMER0, (LETIMER_IF_COMP0 | LETIMER_IF_COMP1));
	LETIMER_Enable (LETIMER0, true);
	//	tfp_printf("LETIMER init\r\n");
	NVIC_EnableIRQ (LETIMER0_IRQn);
}



/***************************************************************************//**
 * @brief
 *   LETIMER IRQ stop for Accelero polling mode
 ******************************************************************************/
void TD_ACCELERO_StopLETIMER(void)
{
	NVIC_DisableIRQ (LETIMER0_IRQn);
	LETIMER_IntClear (LETIMER0, LETIMER_IFC_COMP0);
	LETIMER_IntClear (LETIMER0, LETIMER_IFC_COMP1);
	LETIMER_Enable (LETIMER0, false);
	LETIMER_Reset (LETIMER0);

}

/***************************************************************************//**
 * @brief
 *   Set SPI locking status
 *
 ******************************************************************************/
void TD_ACCELERO_SetLockingStatus(bool state, uint8_t tag)
{
	TD_Accelero.allow_spi = state;
	TD_Accelero.allow_spi_tag = tag;
	TD_Accelero.allow_spi_rtc = RTC->CNT;
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

	for (i = 0x0; i <= 0x3F; i++) {
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

/** @} (end addtogroup TD_ACCELERO) */
