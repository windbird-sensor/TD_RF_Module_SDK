/******************************************************************************
 * @file
 * @brief template file for TDxxxx RF modules.
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
#include <td_sensor.h>
#include <sensor_register.h>


/*******************************************************************************
 **************************   DEFINES   ****************************************
 ******************************************************************************/

/** Define your current variable version */
#define VARIABLES_VERSION 0x1

/** Define your own device class */
#define DEVICE_CLASS 0x0001


/*******************************************************************************
 **************************   PRIVATE VARIABLES   ******************************
 ******************************************************************************/

/** Boot flash variable */
static bool FirstBoot=false;


/*******************************************************************************
 **************************   GLOBAL FUNCTIONS   *******************************
 ******************************************************************************/

/***************************************************************************//**
 * @brief
 *   User setup function.
 ******************************************************************************/
void TD_USER_Setup(void)
{
	//Set variable version
	TD_FLASH_SetVariablesVersion(VARIABLES_VERSION);

	//Initialize the device as a transmitter with no local RF configuration.
	TD_SENSOR_Init(SENSOR_TRANSMITTER,0,0);


	//Register only once on Sensor using persistent flash variable
	if(!TD_FLASH_DeclareVariable((uint8_t *)&FirstBoot, 1, 0))
	{
		//Set the device class
		TD_SENSOR_SetDeviceClass(DEVICE_CLASS);

		//Register on Sensor
		TD_SENSOR_SendRegister();

		//Append monitoring here


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
