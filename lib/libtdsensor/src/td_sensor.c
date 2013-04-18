/***************************************************************************//**
 * @file td_sensor.c
 * @brief Sensor Monitoring
 * @author Telecom Design S.A.
 * @version 1.1.0
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

#include <stdint.h>
#include <stdbool.h>

#include <td_flash.h>
#include <td_measure.h>
#include <td_gpio.h>
#include <td_scheduler.h>

#include "sensor_private.h"
#include "sensor_config.h"
#include "sensor_keepalive.h"
#include "sensor_event.h"

#include "td_sensor_lan.h"
#include "td_sensor_device.h"
#include "td_sensor_gateway.h"
#include "td_sensor_transmitter.h"

#include "td_sensor.h"

/***************************************************************************//**
 * @addtogroup TD_SENSOR Sensor Monitoring
 * @brief Sensor initialization and monitoring functions.
 *
 * @{
 ******************************************************************************/

/*******************************************************************************
 **************************  DEFINES   ****************************************
 ******************************************************************************/

/** @addtogroup TD_SENSOR_DEFINES Defines
 * @{ */

/*Device only. Time to wait between two events.*/
#define SIGFOX_TRANSMISSION_TIME 7

/*Device only. Time to wait after a collision.*/
#define COLLISION_WAIT_TIME 10

/*Device only. Time to wait after all retries ended-up in collisions.*/
#define NACK_WAIT_INTERVAL 3600

/*Local retries count for each event*/
#define EVENT_BATTERY_RETRIES 2
#define EVENT_BOOT_RETRIES 3
#define EVENT_LOCAL_KEEPALIVE_RETRIES 3
#define EVENT_SENSOR_KEEPALIVE_RETRIES 3
#define EVENT_SWITCH_RETRIES 2
#define EVENT_TEMPERATURE_RETRIES 2

/** @} */

/*******************************************************************************
 **************************  TYPEDEFS   ****************************************
 ******************************************************************************/

/** @addtogroup TD_SENSOR_TYPEDEFS Typedefs
 * @{ */

/** Sensor Event */
typedef enum {
	SENSOR_SEND_BATTERY,
	SENSOR_SEND_LOCAL_KEEPALIVE,
	SENSOR_SEND_SENSOR_KEEPALIVE,
	SENSOR_SEND_TEMPERATURE,
	SENSOR_SEND_SWITCH,
	SENSOR_SEND_BOOT
} SensorEvent;


 /**   Sensor Machine State Event*/
typedef struct {
	SensorEvent event :8;	///<event
	uint8_t arg :8; 		///<opt. arg
	uint8_t retry :8; 		///<collision retry
}__PACKED SensorMachineStateEvent;

/** @} */

/*******************************************************************************
 **************************  PRIVATE VARIABLES   *******************************
 ******************************************************************************/

/** @addtogroup TD_SENSOR_Private_Variables Private Variables
 * @{ */


/**Local Module configuration*/
static ModuleConfiguration Module;

/**Sensor machine state events queue, count and index*/
static SensorMachineStateEvent EventQueue[MAX_SENSOR_EVENT];
static uint8_t EventCount = 0;
static uint8_t EventIndex = 0;

/**Device only. Estimation of gateway business*/
static bool GatewayBusy = false;

/**Sigfox ID*/
uint32_t SigfoxID;

/** @} */


/*******************************************************************************
 **************************  PRIVATE FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup TD_SENSOR_PRIVATE_FUNCTIONS Private Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Module local variables initialization
 *
 ******************************************************************************/
static void TD_SENSOR_PrivateInit()
{
	int i;
	for (i = 0; i < SENSOR_MAX_SWITCH; i++) {
		Module.switches[i].monitor = false;
	}
	Module.switch_count = 0;
	Module.switch_mask = 0;

	Module.battery.state = true;
	Module.temperature.state = TEMPERATURE_OK;
	Module.rssi.monitor = false;
	Module.temperature.monitor = false;
	Module.connection.monitor = false;
	Module.battery.monitor = false;
	Module.boot.monitor = false;
	Module.keepalive.monitor = false;

	Module.class = DEFAULT_DEVICE_CLASS;

}


/***************************************************************************//**
 * @brief
 *   Append an event to the queue
 *
 * @param[in] event
 *	Event to be processed.
 *
 * @param[in] arg
 *	Optionnal arguments.
 *
 * @param[in] retry
 *	How many retries if the Gateway does not ack.
 ******************************************************************************/
static void TD_SENSOR_AppendEvent(SensorEvent event, uint8_t arg, uint8_t retry)
{
	uint8_t index;

	if (EventCount <= MAX_SENSOR_EVENT) {
		//circular buffer
		index = EventIndex + EventCount;
		if (index >= MAX_SENSOR_EVENT) {
			index -= MAX_SENSOR_EVENT;
		}

		EventQueue[index].event = event;
		EventQueue[index].arg = arg;
		EventQueue[index].retry = retry;

		EventCount++;
	}
}

/***************************************************************************//**
 * @brief
 *   Collision handler, called by Scheduler
 *
 * @param[in] arg
 *	Timer argument. Contains information about event
 *
 * @param[in] repetitions
 *	Timer argument. Not used.
 ******************************************************************************/
static void TD_SENSOR_CollisionHandler(uint32_t arg, uint8_t repetitions)
{
	TD_SENSOR_AppendEvent((SensorEvent) (arg & 0xFF), (arg >> 8) & 0xFF, (arg >> 16) & 0xFF);
}

/***************************************************************************//**
 * @brief
 *   Gateway busy Handler, called by Scheduler
 *
 * @param[in] arg
 *	Timer argument. Not used.
 *
 * @param[in] repetitions
 *	Timer argument. Not used.
 ******************************************************************************/
static void TD_SENSOR_GatewayBusyHandler(uint32_t arg, uint8_t repetitions)
{
	GatewayBusy = false;
}

/***************************************************************************//**
 * @brief
 *   Keepalive callback, called by Scheduler
 *
 * @param[in] arg
 *	Timer argument. Not used.
 *
 * @param[in] repetitions
 *	Timer argument. Not used.
 ******************************************************************************/
static void TD_SENSOR_KeepAliveCallback(uint32_t arg, uint8_t repetitions)
{
	TD_SENSOR_AppendEvent(SENSOR_SEND_SENSOR_KEEPALIVE, 0, EVENT_SENSOR_KEEPALIVE_RETRIES);
}

/***************************************************************************//**
 * @brief
 *   Connection callback, called by Scheduler
 *
 * @param[in] arg
 *	Timer argument. Not used.
 *
 * @param[in] repetitions
 *	Timer argument. Not used.
 ******************************************************************************/
static void TD_SENSOR_ConnectionCallBack(uint32_t arg, uint8_t repetitions)
{
	TD_SENSOR_AppendEvent(SENSOR_SEND_LOCAL_KEEPALIVE, 0, EVENT_LOCAL_KEEPALIVE_RETRIES);
}

/***************************************************************************//**
 * @brief
 *   Battery callback called by ADC
 ******************************************************************************/
static void TD_SENSOR_BatteryCallBack(void)
{
	//toggle state
	Module.battery.state = !Module.battery.state;

	TD_SENSOR_AppendEvent(SENSOR_SEND_BATTERY, 0, EVENT_BATTERY_RETRIES);

	//measure voltage if state is high or to provide it to the callback
	if (Module.battery.state || Module.battery.user_callback != 0) {
		Module.battery.level = TD_MEASURE_Voltage();
	}

	//toggle battery monitoring for hysteresis
	if (Module.battery.state) {
		TD_MEASURE_EnableBatteryLevelMonitoring(true, Module.battery.level_low, TD_SENSOR_BatteryCallBack);
	} else {
		TD_MEASURE_EnableBatteryLevelMonitoring(false, Module.battery.level_ok, TD_SENSOR_BatteryCallBack);
	}

}

/***************************************************************************//**
 * @brief
 *   Temperature Callback called by Scheduler
 *
 * @param[in] arg
 *	Timer argument. Not used.
 *
 * @param[in] repetitions
 *	Timer argument. Not used.
 ******************************************************************************/
static void TD_SENSOR_TemperatureCallBack(uint32_t arg, uint8_t repetitions)
{
	uint8_t prev_state;

	//measure temperature and save previous state
	Module.temperature.level = TD_MEASURE_TemperatureExtended();
	prev_state = Module.temperature.state;

	//check state
	if (Module.temperature.level <= Module.temperature.level_low) {
		Module.temperature.state = TEMPERATURE_LOW;
	} else if (Module.temperature.level >= Module.temperature.level_high) {
		Module.temperature.state = TEMPERATURE_HIGH;
	} else {
		Module.temperature.state = TEMPERATURE_OK;
	}

	//if state changed add an event
	if (Module.temperature.state != prev_state) {
		TD_SENSOR_AppendEvent(SENSOR_SEND_TEMPERATURE, 0, EVENT_TEMPERATURE_RETRIES);
	}
}

/***************************************************************************//**
 * @brief
 *   Switch Callback called on GPIO IRQ
 ******************************************************************************/
static void TD_SENSOR_SwitchCallBack()
{
	int i;
	int done = 0;
	uint32_t temp_mask = 0;
	uint32_t mask = GPIO_IntGet();

	//check irq for all switches
	for (i = 0; i < SENSOR_MAX_SWITCH; i++) {
		if (done >= Module.switch_count) {
			break;
		}

		if (Module.switches[i].bit != 0xFF && Module.switches[i].port != 0xFF) {
			done++;

			temp_mask = 1 << Module.switches[i].bit;

			//TODO: debounce
			if (temp_mask & mask) {
				bool state = (GPIO_PinInGet(Module.switches[i].port, Module.switches[i].bit)) ? true : false;

				TD_SENSOR_AppendEvent(SENSOR_SEND_SWITCH, (state<<7) | (i&0x7F), EVENT_SWITCH_RETRIES);

			}
		}
	}
}

/***************************************************************************//**
 * @brief
 *   Set callback and irq for a switch
 *
 ******************************************************************************/
static void TD_SENSOR_SetSwitchMonitoring(GPIO_Port_TypeDef port, unsigned int bit, bool falling, bool rising, bool pull, bool pull_state,
		bool (*switch_callback)(GPIO_Port_TypeDef port, unsigned int bit, bool state))
{

	if (((1<<bit) & 0xAAAAAAAA) != 0) {
		//odd
		TD_GPIO_SetCallback(TD_GPIO_USER_ODD, TD_SENSOR_SwitchCallBack,	Module.switch_mask & 0xAAAAAAAA);
	} else {
		//even
		TD_GPIO_SetCallback(TD_GPIO_USER_EVEN, TD_SENSOR_SwitchCallBack, Module.switch_mask & 0x55555555);
	}

	Module.switch_count++;

	if (pull) {
		GPIO_PinModeSet(port, bit, gpioModeInputPullFilter, pull_state);
	} else {
		GPIO_PinModeSet(port, bit, gpioModeInput, 0);
	}

	//enable irq
	GPIO_IntConfig(port, bit, rising, falling, true);

}

/***************************************************************************//**
 * @brief
 *   Process any State event
 *
 * @param[in] event
 *	Event to be processed
 *
 * @return
 *  True if event was properly processed (ie the frame was acked by the Gateway)
 *  False if the Gateway enver acked the frame.
 ******************************************************************************/
static bool TD_SENSOR_EventProcess(SensorMachineStateEvent event)
{
	bool send_sigfox = true;
	bool acked = false;
	uint8_t switch_index;
	bool switch_state;

	switch (event.event) {
	case SENSOR_SEND_BATTERY:

		if (Module.battery.user_callback != 0) {
			send_sigfox = (*Module.battery.user_callback)(Module.battery.state, Module.battery.level);
		}

		if (send_sigfox) {
			acked = TD_SENSOR_SendEventBattery(Module.battery.state, Module.battery.level);
		}

		break;

	case SENSOR_SEND_BOOT:

		if (Module.boot.user_callback != 0) {
			send_sigfox = (*Module.boot.user_callback)();
		}

		if (send_sigfox) {
			acked = TD_SENSOR_SendEventBoot();
		}

		break;

	case SENSOR_SEND_TEMPERATURE:

		if (Module.temperature.user_callback != 0) {
			send_sigfox = (*Module.temperature.user_callback)(Module.temperature.state, Module.temperature.level);
		}

		if (send_sigfox) {
			acked = TD_SENSOR_SendEventTemperature(Module.temperature.state);
		}

		break;

		//device only
	case SENSOR_SEND_LOCAL_KEEPALIVE:
		acked = TD_SENSOR_DEVICE_KeepAlive(true, Module.connection.interval, Module.rssi.monitor, Module.rssi.level_low, Module.rssi.level_ok);
		break;

	case SENSOR_SEND_SENSOR_KEEPALIVE:
		acked = TD_SENSOR_SendKeepAlive();
		break;

	case SENSOR_SEND_SWITCH:

		switch_index = (event.arg&0x7F);
		switch_state = (event.arg>>7);
		if (Module.switches[switch_index].user_callback != 0) {
			send_sigfox = (*Module.switches[switch_index].user_callback)(Module.switches[switch_index].port, Module.switches[switch_index].bit,
					switch_state);
		}
		if (send_sigfox) {
			acked = TD_SENSOR_SendEventSwitch(Module.switches[switch_index].port, Module.switches[switch_index].bit, switch_state);
		}
		break;

	default:
		break;
	}

	return acked;
}

/** @} */

/*******************************************************************************
 **************************  PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup TD_SENSOR_PUBLIC_FUNCTIONS Public Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *	Configure Switch Monitoring on the Module.
 *
 * @param[in] enable
 *	Enable (true) or disable (false) switch monitoring
 *
 * @param[in] port
 *	Port on which the module is connected.
 *
 * @param[in] bit
 *	Bit on which the module is connected.
 *
 * @param[in] falling
 *	If true an irq will be triggered on falling edge
 *
 * @param[in] rising
 *	If true an irq will be triggered on rising edge
 *
 * @param[in] pull
 *	Enable pull-up or pull-down on the specified port,bit.
 *
 * @param[in] pull_state
 *	If pull is set to true, pull_state determines if the pin is pulled high (true) or pull-down (false).
 *	Otherwise it has no effect.
 *
 * @param[in] switch_callback
 *	Callback to call when an irq for this specific switch occurs.
 *
 * @details
 *
 *	A Switch ON Sensor message will be sent on a falling edge and a Switch OFF message will be sent on a rising edge. It
 *	is therefore highly recommended to connect one side of your switch to the ground and activate pull-up on the GPIO.
 *
 *
 ******************************************************************************/
bool TD_SENSOR_MonitorSwitch(bool enable, GPIO_Port_TypeDef port, unsigned int bit, bool falling, bool rising, bool pull, bool pull_state,
		bool (*switch_callback)(GPIO_Port_TypeDef port, unsigned int bit, bool state))
{
	int i;

	//if enable, append the GPIO in the list
	if (enable) {
		if (Module.switch_count >= SENSOR_MAX_SWITCH) {
			return false;
		}

		for (i = 0; i < SENSOR_MAX_SWITCH; i++) {
			if (Module.switches[i].monitor == false) {

				Module.switches[i].monitor = true;
				Module.switches[i].bit = bit;
				Module.switches[i].port = port;
				Module.switches[i].falling = falling;
				Module.switches[i].rising = rising;
				Module.switches[i].pull = pull;
				Module.switches[i].pull_state = pull_state;
				Module.switches[i].state = GPIO_PinInGet(port, bit);
				Module.switches[i].user_callback = switch_callback;

				Module.switch_mask |= 1 << bit;

				TD_SENSOR_SetSwitchMonitoring(port,bit,falling,rising, pull, pull_state, switch_callback);

				return true;
			}
		}
		return false;
	} else //if disable remove the GPIO from the list
	{
		for (i = 0; i < SENSOR_MAX_SWITCH; i++) {
			if (Module.switches[i].bit == bit && Module.switches[i].port == port) {
				uint32_t mask = 1 << bit;
				mask = ~mask;
				Module.switch_mask &= mask;

				if ((bit & 0xAAAAAAAA) != 0) {
					//odd IRQ
					TD_GPIO_SetCallback(TD_GPIO_USER_ODD, TD_SENSOR_SwitchCallBack, Module.switch_mask & 0xAAAAAAAA);
				} else {
					//even IRQ
					TD_GPIO_SetCallback(TD_GPIO_USER_EVEN, TD_SENSOR_SwitchCallBack, Module.switch_mask & 0x55555555);
				}

				//disable irq
				GPIO_IntConfig(port, bit, false, false, false);

				Module.switches[i].monitor = false;

				Module.switches[i].bit = 0xFF;
				Module.switches[i].port = (GPIO_Port_TypeDef) 0xFF;
				Module.switches[i].user_callback = 0;
				Module.switch_count--;

				return true;
			}
		}

	}
	return false;
}

/***************************************************************************//**
 * @brief
 *	Configure Temperature Monitoring on the Module.
 *
 * @param[in] enable
 *	Enable (true) or disable (false) temperature monitoring
 *
 * @param[in] interval
 *	Interval in seconds at which the temperature should be checked
 *
 * @param[in] level_low
 *	Temperature level in 1/10 Celsius Degrees at which a Temperature Low Event is being sent to Sensor
 *
 * @param[in] level_high
 *	Temperature level in 1/10 Celsius Degrees at which a Temperature High Event is being sent to Sensor
 *
 * @param[in] callback
 *  	Function being called on any Temperature Event.
 *
 * @details
 *	A Temperature OK Event is sent once on Sensor if the current temperature is
 *	within the allowed range and if a Temperature High/Low Event has been previously sent.
 ******************************************************************************/
void TD_SENSOR_MonitorTemperature(bool enable, uint32_t interval, int16_t level_low, int16_t level_high,
		bool (*callback)(TemperatureState state, int16_t level))
{

	if (enable) {
		if (!Module.temperature.monitor) {
			//start timer
			Module.temperature.timer = TD_SCHEDULER_Append(interval, 0, 0, 0xFF, TD_SENSOR_TemperatureCallBack, 0);

			if (Module.temperature.timer != 0xFF) {
				Module.temperature.monitor = true;
				Module.temperature.interval = interval;
				Module.temperature.level_low = level_low;
				Module.temperature.level_high = level_high;
				Module.temperature.user_callback = callback;

				//check temp right now
				TD_SENSOR_TemperatureCallBack(0, 0);

			}
		} else //modify settings
		{
			TD_SCHEDULER_SetInterval(Module.temperature.timer, interval, 0, 0);

			Module.temperature.interval = interval;
			Module.temperature.level_low = level_low;
			Module.temperature.level_high = level_high;
			Module.temperature.user_callback = callback;

			//check temp right now
			TD_SENSOR_TemperatureCallBack(0, 0);
		}
	} else if (!enable && Module.temperature.monitor) {
		TD_SCHEDULER_Remove(Module.temperature.timer);
		Module.temperature.timer = 0;
		Module.temperature.monitor = false;
		;
	}

}

/***************************************************************************//**
 * @brief
 *   Configure RSSI Monitoring on gateway side. Keep alive monitoring
 *   must be enabled to enable/disable RSSI.
 *
 *  @param[in] enable
 *  	Enable (true) or disable (false) RSSI monitoring
 *
 *  @param[in] level_low
 *  	RSSI level at which a RSSI Low Event is being sent to Sensor
 *
 *  @param[in] level_ok
 *  	RSSI level at which a RSSI OK Event is being sent to Sensor (this
 *  	only happens once after a RSSI Low Event occurred).
 *  @return
 *  	True if the Module is a Device, keepalive is enabled and if the gateway acked the keepalive frame.
 *
 ******************************************************************************/
bool TD_SENSOR_MonitorRSSI(bool enable, int8_t level_low, int8_t level_ok)
{
	//Only a Device can get RSSI monitored
	if (Module.type == SENSOR_DEVICE) {
		if (Module.connection.monitor) {
			if (enable) {
				//if there is something to update
				if (!Module.rssi.monitor || Module.rssi.level_low != level_low || Module.rssi.level_ok != level_ok) {
					if (TD_SENSOR_DEVICE_KeepAlive(true, Module.connection.interval, true, level_low, level_ok) == ACK_OK) {
						Module.rssi.level_low = level_low;
						Module.rssi.level_ok = level_ok;
						Module.rssi.monitor = true;
						return true;
					}
				}
			} else if (Module.rssi.monitor) {
				if (TD_SENSOR_DEVICE_KeepAlive(true, Module.connection.interval, false, 0, 0) == ACK_OK) {
					Module.rssi.monitor = false;
					return true;
				}
			}
		}

	}

	return false;
}

/***************************************************************************//**
 * @brief
 *   Configure Battery Monitoring
 *
 *  @param[in] enable
 *  	Enable (true) or disable (false) Battery monitoring
 *
 *  @param[in] level_low
 *  	Battery level at which a Battery Low Event is being sent to Sensor
 *
 *  @param[in] level_ok
 *  	Battery level at which a Battery OK Event is being sent to Sensor (this
 *  	only happens once after a Battery Low Event occurred).
 *
 *  @param[in] callback
 *  	Function being called on any Battery Event.
 *
 ******************************************************************************/
void TD_SENSOR_MonitorBattery(bool enable, uint16_t level_low, uint16_t level_ok, bool (*callback)(bool state, uint16_t level))
{
	Module.battery.monitor = enable;

	if (enable) {
		uint16_t voltage_mv;
		voltage_mv = TD_MEASURE_VoltageExtended();

		if (voltage_mv > level_low) {
			Module.battery.state = true;
		} else {
			Module.battery.state = false;
			EventQueue[EventCount].event = SENSOR_SEND_BATTERY;
			EventCount++;
		}

		Module.battery.level_low = level_low;
		Module.battery.level_ok = level_ok;
		Module.battery.user_callback = callback;

		if (Module.battery.state) {
			TD_MEASURE_EnableBatteryLevelMonitoring(true, Module.battery.level_low, TD_SENSOR_BatteryCallBack);
		} else {
			TD_MEASURE_EnableBatteryLevelMonitoring(false, Module.battery.level_ok, TD_SENSOR_BatteryCallBack);
		}
	} else {
		TD_MEASURE_DisableBatteryLevelMonitoring();
	}

}

/***************************************************************************//**
 * @brief
 *   Configure Boot Monitoring.
 *
 *  @param[in] enable
 *  	Enable (true) or disable (false) boot monitoring
 *
 *  @param[in] callback
 *  	Function being called on Boot Event.
 ******************************************************************************/
void TD_SENSOR_MonitorBoot(bool enable, bool (*callback)(void))
{
	Module.boot.monitor = enable;
	Module.boot.user_callback = callback;
}

/***************************************************************************//**
 * @brief
 *   Configure Connection Monitoring on both gateway and module side. Configure
 *   the module to emit a local keep-alive frame on a given interval and configure
 *   the gateway to look for this keep-alive. A Connection Lost event
 *   will be sent to Sensor if the keep-alive was not received by the gateway. Also one
 *   Connection OK event will be emitted to Sensor if a keep-alive is being received after
 *   a Connection Lost event.
 *
 *  @param[in] enable
 *  	Enable (true) or disable (false) connection monitoring
 *
 *  @param[in] interval
 *  	Interval in seconds at which connection is being checked.
 *
 * @return
 *   True if the Module is a Device and if a timer has been set.
 *   False otherwise.
 ******************************************************************************/
bool TD_SENSOR_MonitorConnection(bool enable, uint32_t interval)
{
	if (Module.type == SENSOR_DEVICE) {
		if (enable) {
			//if not already activated
			if (!Module.connection.monitor) {
				//activate keep alive monitoring even if gateway does not reply!
				Module.connection.timer = TD_SCHEDULER_Append(interval, 0, 0, 0xFF, TD_SENSOR_ConnectionCallBack, 0);

				if (Module.connection.timer != 0xFF) {
					TD_SENSOR_DEVICE_KeepAlive(true, interval, Module.rssi.monitor, Module.rssi.level_low, Module.rssi.level_ok);
					Module.connection.monitor = enable;
					Module.connection.interval = interval;
					return true;
				}

			} else //just change interval
			{
				TD_SCHEDULER_SetInterval(Module.connection.timer, interval, 0, 0);
				Module.connection.interval = interval;
				return true;
			}

		} else if (!enable && Module.connection.monitor) {
			if (TD_SENSOR_DEVICE_KeepAlive(false, 0, false, 0, 0) == ACK_OK) {
				TD_SCHEDULER_Remove(Module.connection.timer);
				Module.connection.monitor = false;
				Module.rssi.monitor = false;
				Module.connection.interval = 0;
				Module.connection.timer = 0xFF;
				return true;
			}
		}

		return true;
	}
	return false;
}

/***************************************************************************//**
 * @brief
 *  Sensor Keepalive Monitoring
 *
 *  @param[in] enable
 *  	Enable (true) or disable (false) keep-alive monitoring
 *
 *  @param[in] interval_hour
 *  	Interval in hours at which keep-alive should be sent.
 *
 ******************************************************************************/
bool TD_SENSOR_MonitorKeepAlive(bool enable, uint8_t interval_hour)
{
	if (enable) {
		//activate timer is not already activated
		if (!Module.keepalive.monitor) {

			//send it once to let Sensor know about interval
			TD_SENSOR_KeepAliveCallback(0,0);
			Module.keepalive.timer = TD_SCHEDULER_Append(interval_hour*3600, 0, 0, 0xFF, TD_SENSOR_KeepAliveCallback, 0);

			if (Module.keepalive.timer != 0xFF) {
				Module.keepalive.monitor = true;
				Module.keepalive.interval = interval_hour;
				return true;
			}
		}
		else //or only change interval
		{
			TD_SCHEDULER_SetInterval(Module.keepalive.timer, interval_hour*3600, 0, 0);
			return true;
		}

	} else if (!enable && Module.keepalive.monitor) {
		//else remove timer
		Module.keepalive.monitor = false;
		Module.keepalive.interval = 0;
		TD_SENSOR_KeepAliveCallback(0,0);
		TD_SCHEDULER_Remove(Module.keepalive.timer);
		return true;
	}

	return false;
}

/***************************************************************************//**
 * @brief
 *   Sensor State Machine processing. to be called in User Loop
 ******************************************************************************/
void TD_SENSOR_Process()
{
	bool acked = false;

	while (EventCount > 0) {
		if (Module.type != SENSOR_DEVICE || (Module.type == SENSOR_DEVICE && !GatewayBusy)) {

			SensorMachineStateEvent event;
			event.event = EventQueue[EventIndex].event;
			event.arg = EventQueue[EventIndex].arg;
			event.retry = EventQueue[EventIndex].retry;

			EventCount--;
			EventIndex++;

			if (EventIndex == MAX_SENSOR_EVENT) {
				EventIndex = 0;
			}

			acked = TD_SENSOR_EventProcess(event);

			//if the module is a device, events can't be processed at once
			if (Module.type == SENSOR_DEVICE) {
				//if frame not acked and there is still replies to process
				if (!acked && event.retry > 0) {
					uint32_t arg = 0;
					event.retry--;
					arg = ((event.retry << 16) & 0xFF) | ((event.arg << 8) & 0xFF) | (event.event & 0xFF);

					//start one shot timer in 8 seconds
					TD_SCHEDULER_Append(COLLISION_WAIT_TIME, 0, 0, 1, TD_SENSOR_CollisionHandler, arg);

					//if there is no timer available the Event is lost
				}
				//if retries are not enough we want our events to be processed when the Gateway is back (except for local keepalive)
				else if (!acked && event.retry == 0 && event.event != SENSOR_SEND_LOCAL_KEEPALIVE) {
					uint32_t arg = 0;
					arg = ((event.retry << 16) & 0xFF) | ((event.arg << 8) & 0xFF) | (event.event & 0xFF);

					//start one shot timer in 3600 seconds
					TD_SCHEDULER_Append(NACK_WAIT_INTERVAL, 0, 0, 1, TD_SENSOR_CollisionHandler, arg);
				} else if (acked) {
					//the gateway is now busy with sigfox, wait end of transmission
					if (TD_SCHEDULER_Append(SIGFOX_TRANSMISSION_TIME, 0, 0, 1, TD_SENSOR_GatewayBusyHandler, 0) != 0xFF) {
						GatewayBusy = true;
					}
				}

				//in all cases break
				break;
			}

		} else {
			break;
		}

	}

	if (Module.type == SENSOR_GATEWAY || Module.type == SENSOR_TRANSMITTER) {
			TD_SENSOR_TRANSMITTER_Process();
		}

}

/***************************************************************************//**
 * @brief
 *   Module reset. All monitoring and LAN configuration or devices are deleted.
 *    Only Module type is not altered.
 *
 ******************************************************************************/
void TD_SENSOR_Reset()
{
	int i;

	// Remove all active timers
	if (Module.keepalive.timer != 0xFF) {
		TD_SCHEDULER_Remove(Module.keepalive.timer);
		Module.keepalive.timer = 0xFF;
	}

	if (Module.connection.timer != 0xFF) {
		TD_SCHEDULER_Remove(Module.connection.timer);
		Module.connection.timer = 0xFF;
	}

	if (Module.temperature.timer != 0xFF) {
		TD_SCHEDULER_Remove(Module.temperature.timer);
		Module.temperature.timer = 0xFF;
	}

	// Stop battery monitoring
	TD_MEASURE_DisableBatteryLevelMonitoring();

	// Stop switch monitoring
	for (i = 0; i < SENSOR_MAX_SWITCH; i++) {
		if (Module.switches[i].monitor) {
			//disable irq
			GPIO_IntConfig(Module.switches[i].port, Module.switches[i].bit, false, false, false);
		}
	}

	TD_GPIO_SetCallback(TD_GPIO_USER_ODD, TD_SENSOR_SwitchCallBack, 0);
	TD_GPIO_SetCallback(TD_GPIO_USER_EVEN, TD_SENSOR_SwitchCallBack, 0);

	// Lan Reset
	if (Module.type != SENSOR_TRANSMITTER) {
		if (Module.type == SENSOR_GATEWAY) {
			TD_SENSOR_GATEWAY_DeleteAllDevices();
		} else if (Module.type == SENSOR_DEVICE) {
			TD_SENSOR_DEVICE_Reset();
		}
	}

	// Init local variables
	TD_SENSOR_PrivateInit();
}

/***************************************************************************//**
 * @brief
 *   Set the Device class
 *
 *  @param[in] class
 *  	New Device Class
 ******************************************************************************/
void TD_SENSOR_SetDeviceClass(uint16_t class)
{
	Module.class = class;
}

/***************************************************************************//**
 * @brief
 *   Return the Module type
 *
 * @return
 *  Module type (Device, Gateway or Transmitter)
 ******************************************************************************/
ModuleType TD_SENSOR_GetModuleType()
{
	return Module.type;
}


/***************************************************************************//**
 * @brief
 *  Returns Module's configuration.
 *
 ******************************************************************************/
ModuleConfiguration * TD_SENSOR_GetModuleConfiguration()
{
	return &Module;
}


/***************************************************************************//**
 * @brief
 *  Returns Module's Sigfox ID
 * @return
 *  Sigfox ID
 ******************************************************************************/
uint32_t  TD_SENSOR_GetSigfoxID()
{
	return SigfoxID;
}


/***************************************************************************//**
 * @brief
 *   Init module as a gateway or as a device with a given configuration or load information
 *   from flash.
 *
 * @param[in] type
 *	Module type
 *
 * @param[in] lan_frequency
 *	LAN frequency in Hz. Can be within 868000000..869700000 range.
 *
 * @param[in] lan_power_level
 *	LAN power level in dBm. Range is -35..14.
 *
 * @return
 *   True if a valid SigfoxID could be found in flash.
 *   False otherwise.
 ******************************************************************************/
bool TD_SENSOR_Init(ModuleType type, uint32_t lan_frequency, int16_t lan_power_level)
{
	TD_DEVICE device;
	int i;

	//read sigfox ID
	if (TD_FLASH_DeviceRead(&device)) {
		SigfoxID = device.Serial;

	} else
		return false;

	//Init config if not in flash
	if (!TD_FLASH_DeclareVariable((uint8_t *) &Module, sizeof(Module), 0)) {
		TD_SENSOR_PrivateInit();
	}

	//set module type
	Module.type = type;

	//clear all pending irqs
	NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
	NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
	NVIC_EnableIRQ(GPIO_EVEN_IRQn);
	NVIC_EnableIRQ(GPIO_ODD_IRQn);

	//init depending on module type
	if (Module.type == SENSOR_DEVICE) {
		TD_SENSOR_LAN_Init(false, lan_frequency, lan_power_level);
	} else if (Module.type == SENSOR_GATEWAY) {
		TD_SENSOR_LAN_Init(true, lan_frequency, lan_power_level);
	}

	if (Module.type == SENSOR_GATEWAY || type == SENSOR_TRANSMITTER) {
		TD_SENSOR_TRANSMITTER_Init();
	}

	//enable all monitoring according to flash values

	if (Module.boot.monitor) {
		TD_SENSOR_AppendEvent(SENSOR_SEND_BOOT, 0, EVENT_BOOT_RETRIES);
	}

	if (Module.battery.monitor) {
		TD_SENSOR_MonitorBattery(1, Module.battery.level_low, Module.battery.level_ok, Module.battery.user_callback);
	}

	if (Module.temperature.monitor) {
		Module.temperature.monitor = false;
		TD_SENSOR_MonitorTemperature(1, Module.temperature.interval, Module.temperature.level_low, Module.temperature.level_high,
				Module.temperature.user_callback);
	}

	if (Module.connection.monitor) {
		Module.connection.monitor = false;
		TD_SENSOR_MonitorConnection(true, Module.connection.interval);
	}

	if (Module.keepalive.monitor) {
		Module.keepalive.monitor = false;
		TD_SENSOR_MonitorKeepAlive(true, Module.keepalive.interval);
	}

	for (i = 0; i < SENSOR_MAX_SWITCH; i++) {
		if (Module.switches[i].monitor) {
			TD_SENSOR_SetSwitchMonitoring(Module.switches[i].port, Module.switches[i].bit, Module.switches[i].falling, Module.switches[i].rising,
					Module.switches[i].pull, Module.switches[i].pull_state, Module.switches[i].user_callback);
		}
	}

	//process in case of event
	TD_SENSOR_Process();

	return true;
}

/** @} */

/** @} */
