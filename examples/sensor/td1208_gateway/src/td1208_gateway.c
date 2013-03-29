/******************************************************************************
 * @file
 * @brief Sensor Gateway Application example
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
#include <td_rtc.h>
#define USE_PRINTF
#include <td_printf.h>
#include <td_uart.h>
#include <td_flash.h>
#include <td_scheduler.h>
#include <td_sensor.h>
#include <td_sensor_gateway.h>
#include <sensor_register.h>


#define DEVICE_CLASS 0x0001 //define your own device class
#define KEEPALIVE_INTERVAL 3600*24 //keep-alive every day

static bool FirstBoot=0; //is the device already registered on sensor?
static uint8_t led_timer=0xFF;

/*******************************************************************************
 **************************   GLOBAL FUNCTIONS   *******************************
 ******************************************************************************/


/***************************************************************************//**
 * @brief
 *  Switch LED ON/OFF
 *
 * @param[in] state
 *  If true, switch ON the LED. If Flase switch it OFF.
 *
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
 *  Argument passed by Scheduler.
 *
 ******************************************************************************/
static void led_blink(uint32_t arg, uint8_t repetition)
{
	set_led((bool)(arg&0x1));
	TD_SCHEDULER_SetArg(led_timer,!(arg&0x1));
}


/***************************************************************************//**
 * @brief
 *  Stop registration. Called by timer.
 *
 ******************************************************************************/
static void StopRegistration(uint32_t arg, uint8_t repetition)
{
	TD_SENSOR_GATEWAY_StopRegistration();
	set_led(false);
}


static void OnRegistration(uint32_t lan_address, uint32_t sigfox_id)
{
	tfp_printf("Device registered %02x - %02x \r\n", lan_address, sigfox_id);

	//Blink 5 times at 0.5s to let us know about registration.
	led_timer=TD_SCHEDULER_Append(0, 8192 , 0, 5, led_blink, 1);
}

/***************************************************************************//**
 * @brief
 *   User setup function.
 ******************************************************************************/
void TD_USER_Setup(void)
{
	//Init serial interface
	init_printf(TD_UART_Init(9600, true, false),
		    		TD_UART_Putc,
		    		TD_UART_Start,
		    		TD_UART_Stop);

	//Init LED
	GPIO_PinModeSet(TIM2_PORT, TIM2_BIT, gpioModePushPull, 0);

	//Initialize the device as a transmitter with no local RF configuration.
	TD_SENSOR_Init(SENSOR_GATEWAY,869312500,14);

	//Register only once on Sensor using persistent flash variable
	if(!TD_FLASH_DeclareVariable((uint8_t *)&FirstBoot, 1, 0))
	{
		//Set the device class
		TD_SENSOR_SetDeviceClass(DEVICE_CLASS);

		//Register on Sensor
		TD_SENSOR_SendRegister();

		//Enable boot monitoring
		TD_SENSOR_MonitorBoot(true,0);

		//Enable keepalive monitoring every day
		TD_SENSOR_MonitorKeepAlive(true,KEEPALIVE_INTERVAL);

		//Save on flash
		FirstBoot=true;
		TD_FLASH_WriteVariables();
	}

	//Open registration
	TD_SENSOR_GATEWAY_StartRegistration(OnRegistration);

	//Set led on while registration is opened
	set_led(true);

	//Close it in a 20 seconds. One shot timer.
	TD_SCHEDULER_Append(30, 0, 0, 1, StopRegistration, 0);


}

/***************************************************************************//**
 * @brief
 *   User loop function.
 ******************************************************************************/
void TD_USER_Loop(void)
{
	TD_LAN_Process();
	TD_SENSOR_Process();
}
