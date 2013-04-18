/******************************************************************************
 * @file
 * @brief Sensor Device Application Example
 * @author Telecom Design S.A.
 * @version 2.0.0
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2012-2013 Telecom Design S.A., http://www.telecom-design.com</b>
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
#include <efm32.h>
#include <td_core.h>
#include <td_flash.h>
#include <td_rtc.h>
#define USE_PRINTF
#include <td_printf.h>
#include <td_uart.h>
#include <td_scheduler.h>
#include <td_sensor.h>
#include <td_sensor_lan.h>
#include <td_sensor_device.h>
#include <sensor_event.h>
#include <sensor_register.h>
#include <sensor_data.h>


/*******************************************************************************
 **************************   DEFINES   ****************************************
 ******************************************************************************/

/** LED port */
#define LED_PORT TIM2_PORT

/** LED bit */
#define LED_BIT	TIM2_BIT

/** Define your current variable version */
#define VARIABLES_VERSION 0x1

/** Define your own device class */
#define DEVICE_CLASS 0x0001


/*******************************************************************************
 **************************   PRIVATE VARIABLES   ******************************
 ******************************************************************************/

/** LED timer id*/
static uint8_t LedTimer=0xFF;

/** Boot flash variable*/
static bool FirstBoot=false;


/*******************************************************************************
 **************************   GLOBAL FUNCTIONS   *******************************
 ******************************************************************************/

/***************************************************************************//**
 * @brief
 *  LED blinking. This function is being called by the Scheduler.
 *
 * @param[in] arg
 *  Argument passed by Scheduler. Not used here.
 *
 ******************************************************************************/
static void led_blink(uint32_t arg, uint8_t repetition)
{
	GPIO_PinOutToggle(LED_PORT, LED_BIT);
}



/***************************************************************************//**
 * @brief
 *   User setup function.
 ******************************************************************************/
void TD_USER_Setup(void)
{
	//Set variable version
	TD_FLASH_SetVariablesVersion(VARIABLES_VERSION);

	//Init serial interface
	init_printf(TD_UART_Init(9600, true, false),
		    		TD_UART_Putc,
		    		TD_UART_Start,
		    		TD_UART_Stop);

	//Init LED
	GPIO_PinModeSet(TIM2_PORT, TIM2_BIT, gpioModePushPull, 0);

	//Initialize the device as a transmitter with no local RF configuration.
	TD_SENSOR_Init(SENSOR_DEVICE,869312500,14);

	//Register only once on Sensor using persistent flash variable
	if(!TD_FLASH_DeclareVariable((uint8_t *)&FirstBoot, 1, 0))
	{
		//Set the device class
		TD_SENSOR_SetDeviceClass(DEVICE_CLASS);

		//Register on the Gateway
		while(TD_SENSOR_DEVICE_Register() != ACK_OK)
		{
			TD_RTC_Delay(T10S);
		}

		//the gateway is most likely still in registration mode
		TD_RTC_Delay(T20S);

		//Register on Sensor.
		while(TD_SENSOR_SendRegister() != ACK_OK)
		{
			TD_RTC_Delay(T10S);
		}

		//Blink three times at 0.5s to let us know registration went fine.
		LedTimer=TD_SCHEDULER_Append(0, 16384, 0, 6, led_blink, 1);

		//Enable boot monitoring
		TD_SENSOR_MonitorBoot(true,0);

		//Save on flash
		FirstBoot=true;
		TD_FLASH_WriteVariables();

		//Process events if any
		TD_SENSOR_Process();
	}
}

/***************************************************************************//**
 * @brief
 *   User loop function.
 ******************************************************************************/
void TD_USER_Loop(void)
{
	TD_SENSOR_Process();
}
