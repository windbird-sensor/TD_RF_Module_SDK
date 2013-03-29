/***************************************************************************//**
 * @file td_sensor.h
 * @brief Sensor Monitoring
 * @author Telecom Design S.A.
 * @version 1.0.0
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2013 Telecom Design S.A., http://www.telecom-design.com</b>
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

#ifndef TD_SENSOR_H_
#define TD_SENSOR_H_

#include <stdbool.h>
#include <stdint.h>
#include <em_gpio.h>
#include "sensor_private.h"
#include "sensor_config.h"

/***************************************************************************//**
 * @addtogroup TD_SENSOR Sensor Monitoring
 * @{ */

/*******************************************************************************
 **************************  TYPEDEFS   ****************************************
 ******************************************************************************/

/** @addtogroup TD_SENSOR_TYPEDEFS Typedefs
 * @{ */

/***************************************************************************//**
 * @brief
 *   Temperature Monitoring state
 ******************************************************************************/
typedef enum {
	SENSOR_DEVICE, SENSOR_GATEWAY, SENSOR_TRANSMITTER

} ModuleType;

typedef enum {
	TEMPERATURE_LOW, TEMPERATURE_OK, TEMPERATURE_HIGH,

} TemperatureState;

typedef struct {
	bool (*user_callback)(GPIO_Port_TypeDef port, unsigned int bit,	bool state);

	bool monitor :1;
	GPIO_Port_TypeDef port :8;
	uint8_t bit :8;
	bool falling :1;
	bool rising :1;
	bool pull :1;
	bool pull_state :1;
	bool state :1;

}__PACKED SwitchConfiguration;

typedef struct {
	bool (*user_callback)(bool state, uint16_t level);

	bool monitor :1;
	bool state :1;
	uint8_t level :8;
	uint16_t level_low :16;
	uint16_t level_ok :16;

}__PACKED BatteryConfiguration;

typedef struct {
	bool (*user_callback)(TemperatureState state, int16_t level);

	bool monitor :1;
	TemperatureState state :4;
	uint8_t timer :8;
	int16_t level_low :16;
	int16_t level_high :16;
	int16_t level :16;
	uint32_t interval :32;

}__PACKED TemperatureConfiguration;

typedef struct {
	bool monitor :1;
	uint32_t interval :32;
	uint8_t timer :8;

}__PACKED ConnectionConfiguration;

typedef struct {
	bool monitor :1;
	uint32_t interval :32;
	uint8_t timer :8;

}__PACKED SensorKeepaliveConfiguration;

typedef struct {
	bool monitor :1;
	int8_t level_low :8;
	int8_t level_ok :8;

}__PACKED RSSIConfiguration;

typedef struct {
	bool (*user_callback)(void);
	bool monitor :1;

}__PACKED BootConfiguration;

typedef uint16_t DeviceClass;

/***************************************************************************//**
 * @brief
 *   Module configuration
 ******************************************************************************/
typedef struct {
	ModuleType type;									///< Module type (Device, Gateway, Transmitter)

	DeviceClass class;									///< Device Class

	SwitchConfiguration switches[SENSOR_MAX_SWITCH];	///<Switch list
	uint8_t switch_count;								///<Switch count
	uint32_t switch_mask;								///<Switch irq mask

	BatteryConfiguration battery;						///<Battery config
	BootConfiguration boot;								///<Boot config
	TemperatureConfiguration temperature;				///<Temperature config
	ConnectionConfiguration connection;					///<Connection config
	RSSIConfiguration rssi;								///<RSSI config
	SensorKeepaliveConfiguration keepalive;				///<Sensor Keep-alive config

} ModuleConfiguration;

/** @} */

extern uint32_t SigfoxID;

/*******************************************************************************
 *************************   PROTOTYPES   **************************************
 ******************************************************************************/

/** @addtogroup TD_SENSOR_PUBLIC_FUNCTIONS Public Functions
 * @{ */
/** @addtogroup TD_SENSOR_PROTOTYPES Prototypes
 * @{ */

bool TD_SENSOR_Init(ModuleType type, uint32_t lan_frequency, int16_t lan_power_level);
void TD_SENSOR_SetDeviceClass(uint16_t class);
void TD_SENSOR_Reset();

void TD_SENSOR_MonitorBattery(bool enable, uint16_t level_low, uint16_t level_ok, bool (*callback)(bool state, uint16_t level));
void TD_SENSOR_MonitorBoot(bool enable, bool (*callback)(void));
bool TD_SENSOR_MonitorConnection(bool enable, uint32_t interval);
bool TD_SENSOR_MonitorKeepAlive(bool enable, uint32_t interval);
bool TD_SENSOR_MonitorRSSI(bool enable, int8_t level_low, int8_t level_ok);
bool TD_SENSOR_MonitorSwitch(bool enable, GPIO_Port_TypeDef port, unsigned int bit, bool falling, bool rising, bool pull, bool state, bool (*switch_callback)(GPIO_Port_TypeDef port, unsigned int bit,	bool state));
void TD_SENSOR_MonitorTemperature(bool enable, uint32_t interval, int16_t level_low, int16_t level_high, bool (*callback)(TemperatureState state, int16_t level));

ModuleType TD_SENSOR_GetModuleType();
ModuleConfiguration * TD_SENSOR_GetModuleConfiguration();

void TD_SENSOR_Process();


/** @} */
/** @} */
/** @} (end addtogroup TD_SENSOR) */

#endif /* TD_SENSOR_H_ */
