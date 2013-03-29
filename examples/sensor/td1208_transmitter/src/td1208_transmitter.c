/******************************************************************************
 * @file transmitter.c
 * @brief Sensor Transmitter Application Example
 * @author Telecom Design S.A.
 * @version 1.0.0
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2012 Telecom Design S.A., http://www.telecom-design.com</b>
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

#include <stdint.h>
#include <stdbool.h>
#include <efm32.h>
#include <em_gpio.h>
#include <td_scheduler.h>
#include <td_uart.h>
#include <td_measure.h>
#include <td_flash.h>
#define USE_PRINTF
#include <td_printf.h>
#include <td_module.h>
#include <td_gpio.h>
#include <td_sensor.h>
#include <sensor_event.h>
#include <sensor_register.h>

#define DEVICE_CLASS 0x0001 //define your own device class

#define BATTERY_LEVEL_LOW 2600 //battery low level in mV
#define BATTERY_LEVEL_OK 3100 //battery low level in mV

#define TEMPERATURE_LEVEL_LOW 50 //temperature low level in 1/10 degrees
#define TEMPERATURE_LEVEL_HIGH 550 //temperature high level in 1/10 degrees
#define TEMPERATURE_CHECKING_INTERVAL 600 //temperature checking interval in seconds

#define KEEPALIVE_INTERVAL 3600*24 //keepalive interval in seconds (24 hours)


static uint8_t led_timer=0xFF;
static bool FirstBoot=false;

/*******************************************************************************
 **************************   GLOBAL FUNCTIONS   *******************************
 ******************************************************************************/


/***************************************************************************//**
 * @brief
 *  Switch LED ON/OFF
 *
 * @param[in] state
 *  If true, switch ON the LED. If false switch it OFF.
 ******************************************************************************/
static void set_led(bool state)
{
	if(state)
	{
		//set LED on
		GPIO_PinOutSet(TIM2_PORT, TIM2_BIT);
	}
	else
	{
		//set LED off
		 GPIO_PinOutClear(TIM2_PORT, TIM2_BIT);
	}
}

/***************************************************************************//**
 * @brief
 *  LED blinking. This function is being called by the Scheduler.
 *
 * @param[in] arg
 *  Argument passed by Scheduler. Used to toggle the led.
 ******************************************************************************/
static void led_blink(uint32_t arg, uint8_t repetition)
{
	set_led((bool)(arg&0x1));
	TD_SCHEDULER_SetArg(led_timer,!(arg&0x1));
}

/***************************************************************************//**
 * @brief
 *   Battery Callback. Called on battery Event. *
 *   Switch on blinking the blue LED if battery state is low.
 *   Switch off blinking if battery state goes back to OK.
 ******************************************************************************/
static bool battery_user_callback(bool state, uint16_t level)
{

	tfp_printf("Battery user callback\r\n");

	//if battery level is low
	if(!state)
	{
		//setup blink every 2 seconds
		led_timer=TD_SCHEDULER_Append(1, 0, 0, 0xFF, led_blink, 1);
	}
	else
	{
		//if a timer was started
		if(led_timer!=0xFF)
		{
			//stop it
			TD_SCHEDULER_Remove(led_timer);
			led_timer=0xFF;
		}

		//set LED off
		set_led(0);
	}

	//send sigfox
	return true;
}

/***************************************************************************//**
 * @brief
 * Temperature callback. Called on Temperature Events.
 ******************************************************************************/
static bool temperature_user_callback(TemperatureState state, int16_t level)
{
	tfp_printf("Temperature user callback\r\n");
	return true;
}

/***************************************************************************//**
 * @brief
 * Switch callback. Called on switch Events.
 ******************************************************************************/
bool switch_user_callback(GPIO_Port_TypeDef port, unsigned int bit, bool state)
{
	tfp_printf("Switch user callback, port: %d - bit: %d - state: %d\r\n",port, bit, state);
	return true;
}

/***************************************************************************//**
 * @brief
 *   User setup function.
 ******************************************************************************/
void TD_USER_Setup(void)
{

	// Initialize the UART
	init_printf(TD_UART_Init(9600, true, false),
	    		TD_UART_Putc,
	    		TD_UART_Start,
	    		TD_UART_Stop);

	//Print out current temperature
	tfp_printf("Temperature: %d.C\r\n",TD_MEASURE_VoltageTemperatureExtended(true)/10);

	//Init LED
	GPIO_PinModeSet(TIM2_PORT, TIM2_BIT, gpioModePushPull, 0);

	//Initialize the device as a transmitter with no local RF configuration.
	TD_SENSOR_Init(SENSOR_TRANSMITTER,0,0);

	//Register on Sensor and activate monitoring only once using persistent flash variable
	if(!TD_FLASH_DeclareVariable((uint8_t *)&FirstBoot, 1, 0))
	{
		//Set the device class
		TD_SENSOR_SetDeviceClass(DEVICE_CLASS);

		//Register on Sensor
		TD_SENSOR_SendRegister();

		//Enable battery monitoring and setup a callback on event
		TD_SENSOR_MonitorBattery(true, BATTERY_LEVEL_LOW, BATTERY_LEVEL_OK,battery_user_callback);

		//Enable temperature monitoring and setup a callback on event
		TD_SENSOR_MonitorTemperature(true, TEMPERATURE_CHECKING_INTERVAL, TEMPERATURE_LEVEL_LOW, TEMPERATURE_LEVEL_HIGH, temperature_user_callback);

		//Enable Switch Monitoring and setup a callback on event
		TD_SENSOR_MonitorSwitch(true, USR0_PORT, USR0_BIT, true, true, true, true, switch_user_callback);

		//Enable KeepAlive Monitoring very 24 hours
		TD_SENSOR_MonitorKeepAlive(true, KEEPALIVE_INTERVAL);

		//Save on flash
		FirstBoot=true;
		TD_FLASH_WriteVariables();
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
