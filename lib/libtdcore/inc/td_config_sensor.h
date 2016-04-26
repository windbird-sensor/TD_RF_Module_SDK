/** @cond TD_CONFIG */
/***************************************************************************//**
 * @file
 * @brief Configuration file for the TDxxxx RF modules Sensor.
 * @author Telecom Design S.A.
 * @version 1.0.0
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2013-2015 Telecom Design S.A., http://www.telecomdesign.fr</b>
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
#ifndef __TD_CONFIG_SENSOR_H
#define __TD_CONFIG_SENSOR_H

#include <stdint.h>
#include <stdbool.h>

#include <td_config_ext.h>

#ifdef __cplusplus
extern "C" {
#endif

#if IS_EMPTY(TD_SENSOR_USE_CODE)
#error("TD_SENSOR_USE_CODE must be set to 0 or 1, or removed")
#endif

/* For all chips, by default we use sensor */
#ifndef	TD_SENSOR_USE_CODE
#define TD_SENSOR_USE_CODE		1
#endif

#if TD_SENSOR_USE_CODE

#include <td_sensor.h>
#include <td_sensor_lan.h>
#include <td_sensor_transmitter.h>
#include <td_sensor_device.h>
#include <td_sensor_gateway.h>

#ifndef TD_SENSOR_TRANSMITTER_MAX_RETRANSMIT

#ifdef EFM32TG210F32
#define TD_SENSOR_TRANSMITTER_MAX_RETRANSMIT 10
#else
#define TD_SENSOR_TRANSMITTER_MAX_RETRANSMIT 10
#endif

#endif // TD_SENSOR_TRANSMITTER_MAX_RETRANSMIT

#ifndef TD_SENSOR_TRANSMITTER_MAX_TRANSMIT
#define TD_SENSOR_TRANSMITTER_MAX_TRANSMIT 1
#endif

#if TD_SENSOR_TRANSMITTER_MAX_RETRANSMIT > 0

    /** Transmitter retransmit queue */
    uint8_t const CONFIG_TD_SENSOR_TRANSMITTER_MAX_RETRANSMIT =
    TD_SENSOR_TRANSMITTER_MAX_RETRANSMIT;
    static TD_SENSOR_TRANSMITTER_Retransmission_t
    TransmitterRetransmissionList[TD_SENSOR_TRANSMITTER_MAX_RETRANSMIT];
    TD_SENSOR_TRANSMITTER_Retransmission_t
    *TD_SENSOR_TRANSMITTER_RetransmissionList = TransmitterRetransmissionList;
#else

    /** Transmitter retransmit queue */
    uint8_t const CONFIG_TD_SENSOR_TRANSMITTER_MAX_RETRANSMIT = 0;
    TD_SENSOR_TRANSMITTER_Retransmission_t
    *TD_SENSOR_TRANSMITTER_RetransmissionList = NULL;

#endif

#if TD_SENSOR_TRANSMITTER_MAX_TRANSMIT > 0

    /** Transmitter transmit queue */
    uint8_t const CONFIG_TD_SENSOR_TRANSMITTER_MAX_TRANSMIT =
    TD_SENSOR_TRANSMITTER_MAX_TRANSMIT;
    static TD_SENSOR_TRANSMITTER_Transmission_t
    TransmitterTransmissionQueue[TD_SENSOR_TRANSMITTER_MAX_TRANSMIT];
    TD_SENSOR_TRANSMITTER_Transmission_t *TD_SENSOR_TRANSMITTER_TransmissionQueue
    = TransmitterTransmissionQueue;

#else

    /** Transmitter transmit queue */
    uint8_t const CONFIG_TD_SENSOR_TRANSMITTER_MAX_TRANSMIT = 0;
    TD_SENSOR_TRANSMITTER_Transmission_t *TD_SENSOR_TRANSMITTER_TransmissionQueue
    = NULL;

#endif

#ifndef TD_SENSOR_DEVICE_QUEUE_MAX

#ifdef EFM32TG210F32
#define TD_SENSOR_DEVICE_QUEUE_MAX 10
#else
#define TD_SENSOR_DEVICE_QUEUE_MAX 10
#endif

#endif // TD_SENSOR_DEVICE_QUEUE_MAX

#if TD_SENSOR_DEVICE_QUEUE_MAX > 0

    /** Device queue */
    uint8_t const CONFIG_TD_SENSOR_DEVICE_QUEUE_MAX = TD_SENSOR_DEVICE_QUEUE_MAX;
    static TD_SENSOR_DEVICE_DeviceQueueFrame_t
    DeviceQueueList[TD_SENSOR_DEVICE_QUEUE_MAX];
    TD_SENSOR_DEVICE_DeviceQueueFrame_t * DeviceQueue = DeviceQueueList;

#else

    /** Device queue */
    uint8_t const CONFIG_TD_SENSOR_DEVICE_QUEUE_MAX = 0;
    TD_SENSOR_DEVICE_DeviceQueueFrame_t * DeviceQueue = NULL;

#endif

#ifndef TD_SENSOR_GATEWAY_MAX_DEVICE
#define TD_SENSOR_GATEWAY_MAX_DEVICE 15
#endif

#if (TD_SENSOR_GATEWAY_MAX_DEVICE > 0) && !defined(TD_SENSOR_GATEWAY_REMOVE_CODE)

    /**Registered Devices List and count*/
    uint8_t const CONFIG_TD_SENSOR_GATEWAY_MAX_DEVICE =
    TD_SENSOR_GATEWAY_MAX_DEVICE + 1;
    static TD_SENSOR_GATEWAY_Device_t
    GatewayDeviceList[TD_SENSOR_GATEWAY_MAX_DEVICE + 1];
    TD_SENSOR_GATEWAY_Device_t * DeviceList = GatewayDeviceList;

#else

    /**Registered Devices List and count*/
    uint8_t const CONFIG_TD_SENSOR_GATEWAY_MAX_DEVICE =
    TD_SENSOR_GATEWAY_MAX_DEVICE + 1;
    TD_SENSOR_GATEWAY_Device_t * DeviceList = NULL;

#endif

#ifndef TD_SENSOR_MAX_SWITCH
#define TD_SENSOR_MAX_SWITCH 	8
#endif

#ifndef TD_SENSOR_MAX_SWITCH_EVENT
#define TD_SENSOR_MAX_SWITCH_EVENT 5
#endif

#if TD_SENSOR_MAX_SWITCH > 0

    /** Switch monitoring */
    uint8_t const CONFIG_TD_SENSOR_MAX_SWITCH = TD_SENSOR_MAX_SWITCH;
    static TD_SENSOR_SwitchConfiguration_t
    SensorSwitchConfig[TD_SENSOR_MAX_SWITCH];
    TD_SENSOR_SwitchConfiguration_t *TD_SENSOR_SwitchConfig = SensorSwitchConfig;
    /** Switch monitoring - non irq processing (add 1 for empty buffer stage needed) */
    uint8_t const CONFIG_TD_SENSOR_MAX_SWITCH_EVENT =
    TD_SENSOR_MAX_SWITCH_EVENT + 1;
    static TD_SENSOR_SwitchState_t
    SensorSwitchStateList[TD_SENSOR_MAX_SWITCH_EVENT];
    TD_SENSOR_SwitchState_t * TD_SENSOR_SwitchStateList = SensorSwitchStateList;

#else

    /** Switch monitoring */
    uint8_t const CONFIG_TD_SENSOR_MAX_SWITCH = TD_SENSOR_MAX_SWITCH;
    TD_SENSOR_SwitchConfiguration_t * TD_SENSOR_SwitchConfig = NULL;
    /** Switch monitoring - non irq processing */
    uint8_t const CONFIG_TD_SENSOR_MAX_SWITCH_EVENT = TD_SENSOR_MAX_SWITCH_EVENT;
    TD_SENSOR_SwitchState_t * TD_SENSOR_SwitchStateList = NULL;

#endif

// Default config : use all sensor monitoring
#ifndef TD_SENSOR_MONITOR_CONFIG
#define TD_SENSOR_MONITOR_CONFIG	TD_SENSOR_MONITOR_BOOT | \
    TD_SENSOR_MONITOR_BATT | TD_SENSOR_MONITOR_TEMP | TD_SENSOR_MONITOR_CONNECT \
    |TD_SENSOR_MONITOR_KEEPALIVE | TD_SENSOR_MONITOR_SWITCH
#endif

#if (TD_SENSOR_MONITOR_CONFIG) & TD_SENSOR_MONITOR_BOOT
#define TD_SENSOR_MONITOR_BOOT_val	\
    {TD_SENSOR_ApplyBootMonitoringConfiguration},
#else
#define TD_SENSOR_MONITOR_BOOT_val
#endif

#if (TD_SENSOR_MONITOR_CONFIG) & TD_SENSOR_MONITOR_BATT
#define TD_SENSOR_MONITOR_BATT_val \
    {TD_SENSOR_ApplyBatteryMonitoringConfiguration},
#else
#define TD_SENSOR_MONITOR_BATT_val
#endif

#if (TD_SENSOR_MONITOR_CONFIG) & TD_SENSOR_MONITOR_TEMP
#define TD_SENSOR_MONITOR_TEMP_val \
    {TD_SENSOR_ApplyTemperatureMonitoringConfiguration},
#else
#define TD_SENSOR_MONITOR_TEMP_val
#endif

#if (TD_SENSOR_MONITOR_CONFIG) & TD_SENSOR_MONITOR_CONNECT
#define TD_SENSOR_MONITOR_CONNECT_val \
    {TD_SENSOR_ApplyConnectionMonitoringConfiguration},
#else
#define TD_SENSOR_MONITOR_CONNECT_val
#endif

#if (TD_SENSOR_MONITOR_CONFIG) & TD_SENSOR_MONITOR_KEEPALIVE
#define TD_SENSOR_MONITOR_KEEPALIVE_val \
    {TD_SENSOR_ApplyKeepaliveMonitoringConfiguration},
#else
#define TD_SENSOR_MONITOR_KEEPALIVE_val
#endif

#if (TD_SENSOR_MONITOR_CONFIG) & TD_SENSOR_MONITOR_SWITCH
#define TD_SENSOR_MONITOR_SWITCH_val \
    {TD_SENSOR_ApplySwitchMonitoringConfiguration},
#else
#define TD_SENSOR_MONITOR_SWITCH_val
#endif

TD_SENSOR_MONITOR_Callback_t TD_SENSOR_MONITOR_Callback_List[] = {
    TD_SENSOR_MONITOR_BOOT_val
    TD_SENSOR_MONITOR_BATT_val
    TD_SENSOR_MONITOR_TEMP_val
    TD_SENSOR_MONITOR_CONNECT_val
    TD_SENSOR_MONITOR_KEEPALIVE_val
    TD_SENSOR_MONITOR_SWITCH_val
    {NULL}
};

#ifdef TD_SENSOR_GATEWAY_REMOVE_CODE
NOP_DYNAMIC(TD_SENSOR_GATEWAY_Init);
NOP_DYNAMIC(TD_SENSOR_GATEWAY_FrameReceived);
#else
INIT_DYNAMIC(TD_SENSOR_GATEWAY_Init);
INIT_DYNAMIC(TD_SENSOR_GATEWAY_FrameReceived);
#endif

#ifdef TD_SENSOR_MONITOR_REMOVE_CODE
#define TD_SENSOR_MONITOR_EVENT_REMOVE_CODE
#define TD_SENSOR_MONITOR_SWITCH_REMOVE_CODE
NOP_DYNAMIC(TD_SENSOR_MonitorInit);
#else
INIT_DYNAMIC(TD_SENSOR_MonitorInit);
#endif

#ifdef TD_SENSOR_MONITOR_EVENT_REMOVE_CODE
NULL_DYNAMIC(TD_SENSOR_EventProcess);
#else
INIT_DYNAMIC(TD_SENSOR_EventProcess);
#endif

#ifdef TD_SENSOR_MONITOR_SWITCH_REMOVE_CODE
NOP_DYNAMIC(TD_SENSOR_SwitchEventProcess);
#else
INIT_DYNAMIC(TD_SENSOR_SwitchEventProcess);
#endif

#ifdef TD_SENSOR_TRANSMITTER_REMOVE_CODE
NOP_DYNAMIC(TD_SENSOR_TRANSMITTER_Init);
NOP_DYNAMIC(TD_SENSOR_TRANSMITTER_Process);
NOP_DYNAMIC(TD_SENSOR_TRANSMITTER_SendSigfox);
#else
INIT_DYNAMIC(TD_SENSOR_TRANSMITTER_Init);
INIT_DYNAMIC(TD_SENSOR_TRANSMITTER_Process);
INIT_DYNAMIC(TD_SENSOR_TRANSMITTER_SendSigfox);
#endif

#ifdef TD_SENSOR_TRANSMITTER_REMOVE_DUTY_CYCLE_CODE
NULL_DYNAMIC(TD_SENSOR_TRANSMITTER_IsTxAllowed);
#else
INIT_DYNAMIC(TD_SENSOR_TRANSMITTER_IsTxAllowed);
#endif

#ifdef TD_SENSOR_DEVICE_REMOVE_CODE
NOP_DYNAMIC(TD_SENSOR_DEVICE_FrameReceived);
NOP_DYNAMIC(TD_SENSOR_DEVICE_Process);
NOP_DYNAMIC(TD_SENSOR_DEVICE_Forward);
#else
INIT_DYNAMIC(TD_SENSOR_DEVICE_FrameReceived);
INIT_DYNAMIC(TD_SENSOR_DEVICE_Process);
INIT_DYNAMIC(TD_SENSOR_DEVICE_Forward);
#endif

#ifdef TD_SENSOR_LAN_REMOVE_CODE
NOP_DYNAMIC(TD_SENSOR_LAN_Init);
#else
INIT_DYNAMIC(TD_SENSOR_LAN_Init);
#endif

#endif // TD_SENSOR_USE_CODE

#ifdef __cplusplus
}
#endif

#endif // __TD_CONFIG_SENSOR_H
/** @endcond */
