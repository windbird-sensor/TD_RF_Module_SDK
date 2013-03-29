/***************************************************************************//**
 * @file td_sensor_gateway.c
 * @brief Sensor Gateway
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

#include <stdbool.h>
#include <stdint.h>
#include <td_lan.h>
#include <td_flash.h>
#include <td_rtc.h>
#include <td_sigfox.h>
#include "sensor_private.h"
#include "td_sensor.h"
#include "td_sensor_lan.h"
#include "td_sensor_device.h"
#include "sensor_event.h"
#include "td_scheduler.h"
#include "sensor_config.h"
#include "sensor_send_private.h"
#include "td_sensor_transmitter.h"
#include "td_sensor_gateway.h"

/***************************************************************************//**
 * @addtogroup TD_SENSOR_GATEWAY Sensor LAN Gateway
 * @{
 ******************************************************************************/

/*******************************************************************************
 **************************  DEFINES   ****************************************
 ******************************************************************************/

/** @addtogroup TD_SENSOR_GATEWAY_DEFINES Defines
 * @{ */

#define MAX_GATEWAY_EVENT 5

/** @} */

/*******************************************************************************
 **************************  TYPEDEFS   ****************************************
 ******************************************************************************/

/** @addtogroup TD_SENSOR_TYPEDEFS Typedefs
 * @{ */

/***************************************************************************//**
 * @brief
 *   Sensor Device types
 ******************************************************************************/
typedef enum {
	TD1202

} DeviceType;

/***************************************************************************//**
 * @brief
 *   Gateway Events
 ******************************************************************************/
typedef enum {
	CONNECTION_LOST, CONNECTION_OK, RSSI_LOW, RSSI_OK,

} GatewayEvent;

/***************************************************************************//**
 * @brief
 *   Distant device keep-alive configuration
 ******************************************************************************/
typedef struct {
	uint32_t interval :32;	///<keep alive checking interval
	uint8_t timer :8;		///<keep alive timer id
	bool monitor :1;		///<monitoring enabled?
	bool status :1;		///<keep alive status (lost/ok)
	bool validated :1;		///<keep alive validated?

}__PACKED GatewayKeepaliveConfig;

/***************************************************************************//**
 * @brief
 *   Distant RSSI keep-alive configuration
 ******************************************************************************/
typedef struct {
	uint8_t level_ok :8;	///<rssi level ok
	uint8_t level_low :8;	///<rssi level low
	bool monitor :1;	///<monitoring enabled?
	bool status :1;	///<current status (low/ok)

}__PACKED GatewayRSSIConfig;


/***************************************************************************//**
 * @brief
 *   Distant device configuration
 ******************************************************************************/
typedef struct {
	GatewayRSSIConfig rssi;				///<RSSI config
	GatewayKeepaliveConfig keepalive;	///<Keepalive config

} DeviceConfig;

/***************************************************************************//**
 * @brief
 *   Distant device data
 ******************************************************************************/
typedef struct {
	DeviceConfig config;
	uint32_t sigfox_id :32; ///<Device's Sigfox ID
	uint32_t lan_address :24; ///<Device's Lan Address
	DeviceType type :8;  ///<Device Type

} Device;

typedef struct {
	GatewayEvent event; 	///<Gateway Event
	uint32_t arg; 		//<Optionnal argument

} GatewayMachineStateEvent;

/** @} */

/*******************************************************************************
 **************************  PUBLIC VARIABLES   *******************************
 ******************************************************************************/

extern uint32_t SigfoxID;

/*******************************************************************************
 **************************  PRIVATE VARIABLES   *******************************
 ******************************************************************************/

/** @addtogroup TD_SENSOR_GATEWAY_PRIVATE_VARIABLES Private Variables
 * @{ */

/** Frame received buffer*/
static TD_LAN_frame_t gateway_RX;

/** Computed Gateway address*/
static uint32_t GatewayAddress = 0;

/**Registered Devices List and count*/
static Device DeviceList[MAX_DEVICE];
static uint32_t DeviceCount = 1; //0 is gateway

/**Registration and pairing status*/
static bool RegistrationEnabled = false;
static bool PairingEnabled = false;

/** LAN Data received callback*/
static uint8_t (*DataCallback)(uint8_t * data, uint8_t len, uint8_t * reply) = 0;

/** On Registration callback*/
static void (*RegistrationCallback)(uint32_t lan_address, uint32_t sigfox_id) =
0;

/** @} */

/*******************************************************************************
 **************************  PRIVATE FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup TD_SENSOR_GATEWAY_PRIVATE_FUNCTIONS Private Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *  Compute a 2 bytes address out of 4 bytes
 *
 * @param[in] address
 *  32 bits address
 *
 * @return
 *  16 bit address
 *
 ******************************************************************************/
static uint16_t TD_SENSOR_LAN_ComputeAddressTo16bits(uint32_t address)
{
	uint8_t xor_even = 0, xor_odd = 0;

	xor_even = ((address >> 24) & 0xFF) ^ ((address >> 8) & 0xFF);
	xor_odd = ((address >> 16) & 0xFF) ^ ((address >> 0) & 0xFF);

	return (xor_even << 8) | xor_odd;
}

/***************************************************************************//**
 * @brief
 *  Compute a 1 bytes address out of 4 bytes
 *
 * @param[in] address
 *  32 bits address
 *
 * @return
 *  8 bit address
 *
 ******************************************************************************/
static uint8_t TD_SENSOR_LAN_ComputeAddressTo8bits(uint32_t address)
{
	int i;
	uint8_t xor = 0;
	for (i = 0; i < 4; i++) {
		xor ^= (address >> (8 * i));
	}
	return xor;
}

/***************************************************************************//**
 * @brief
 *   Keep-Alive handler, called by Scheduler. Make sure the connection monitored Device
 *   has emitted its keep-alive within schedule.
 *
 * @param[in] arg
 *	Timer argument. Contains entry_id
 *
 * @param[in] repetitions
 *	Timer argument. Not used.
 ******************************************************************************/
static void TD_SENSOR_GATEWAY_KeepAliveHandler(uint32_t arg, uint8_t repetitions)
{

	uint8_t prev_status;
	uint8_t entry_id = arg;

	//save previous status
	prev_status = DeviceList[entry_id].config.keepalive.status;
	DeviceList[entry_id].config.keepalive.status = DeviceList[entry_id].config.keepalive.validated;

	//compare status with previous
	if (prev_status != (uint8_t) DeviceList[entry_id].config.keepalive.status) {
		if (!DeviceList[entry_id].config.keepalive.status) {
			//send connection lost
			TD_SENSOR_SendEventConnection(0, entry_id);
		}
	}

	//reset keep alive received indicator
	DeviceList[entry_id].config.keepalive.validated = false;

}

/***************************************************************************//**
 * @brief
 *   Append an Device to the registration List.
 *
 * @param[in] device
 *   Pointer to the Device to append
 *
 * @return
 *   EntryId if the device has been added
 *   0XFF otherwise
 ******************************************************************************/

static uint8_t TD_SENSOR_GATEWAY_AppendDevice(Device * device)
{
	uint8_t address;
	int i = 0;
	//no more than 16 devices around
	if (DeviceCount > MAX_DEVICE) {
		return 0xFF;
	}

	//if device already registered before then give it his address back
	for (i = 0; i < MAX_DEVICE; i++) {
		if (device->sigfox_id == DeviceList[i].sigfox_id) {
			return DeviceList[i].lan_address;
		}
	}

	//otherwise compute a new address
	address = TD_SENSOR_LAN_ComputeAddressTo8bits(device->sigfox_id);

	bool ok;

	//check if address not already in use
	do {
		ok = true;
		for (i = 1; i < DeviceCount; i++) {
			if (DeviceList[i].lan_address == address) {
				address++;
				if (address > DeviceCount)
					address = 1;
				ok = false;
				break;
			}
		}

	} while (!ok);

	//affect address
	DeviceList[DeviceCount].lan_address = address;
	DeviceList[DeviceCount].sigfox_id = device->sigfox_id;

	DeviceCount++;

	//update flash
	TD_FLASH_WriteVariables();

	return address;
}

/***************************************************************************//**
 * @brief
 *   Returns a pointer to the device according to a SigfoxId
 *
 * @param[in] lan_address
 *   Device's lan_address
 *
 * @return
 *   Pointer to the Access Point.
 *   0 if the SigfoxId was not found in the registration list
 ******************************************************************************/

static uint8_t TD_SENSOR_GATEWAY_GetDevice(uint8_t lan_address)
{
	int i = 1;
	//tfp_printf("look for address : %02x \r\n",lan_address);
	while (i < DeviceCount) {
		//tfp_printf("address number %d : %02x \r\n",i,DeviceList[i].lan_address);
		if (DeviceList[i].lan_address == lan_address)
			return i;
		i++;
	}

	return 0xFF;
}

/***************************************************************************//**
 * @brief
 *   Send an ack on LAN to a given device.
 *
 * @param[in] rx_frame
 * 	Frame to be acked.
 *
 * @param[in] code
 *  Ack code to be replied.
 *
 * @param[in] data
 *  Optional data to be sent back. If a null pointer is provided, received data but last byte is being
 *  copied into payload.
 *
 * @param[in] count
 *  Size of data.
 *
 * @return

 ******************************************************************************/
static void TD_SENSOR_GATEWAY_Ack(TD_LAN_frame_t *rx_frame, AckCode code, uint8_t * data, uint8_t count)
{
	int i;
	TD_LAN_frame_t tx_frame;

	// Reply frame to sender with acknowledge flag set
	tx_frame.header = rx_frame->header;
	SET_ACK(tx_frame.header, TD_LAN_ACK);

	//first byte is ack code
	tx_frame.payload[0] = code;

	//if no data or too many bytes then recopy everything but last byte from rx frame
	if (data == 0 || count == 0 || count > TD_LAN_PAYLOAD_SIZE - 1) {
		for (i = 0; i < TD_LAN_PAYLOAD_SIZE - 1; i++) {
			tx_frame.payload[i + 1] = rx_frame->payload[i];
		}
	} else //otherwise send data
	{
		for (i = 0; i < count; i++) {
			tx_frame.payload[i + 1] = data[i];
		}

		//fill with 0s
		for (; i < TD_LAN_PAYLOAD_SIZE - 1; i++) {
			tx_frame.payload[i + 1] = 0;
		}
	}

	//Send Ack frame
	TD_LAN_SendFrame(1, &tx_frame, rx_frame);

}

/***************************************************************************//**
 * @brief
 *   RSSI level checking for each received message if monitoring enabled.
 *
 * @param[in] entry_id
 * 	entry_id of device
 *
 ******************************************************************************/
static void TD_SENSOR_GATEWAY_CheckRSSI(uint8_t entry_id)
{
	//RSSI checking
	if (DeviceList[entry_id].config.rssi.monitor) {
		int8_t rssi;
		uint8_t old_status;
		rssi = TD_LAN_ReadLatchedRSSI();

		old_status = DeviceList[entry_id].config.rssi.status;

		if (rssi > DeviceList[entry_id].config.rssi.level_ok) {
			DeviceList[entry_id].config.rssi.status = true;
		} else if (rssi < DeviceList[entry_id].config.rssi.level_low) {
			DeviceList[entry_id].config.rssi.status = false;
		}

		if (old_status != (uint8_t) DeviceList[entry_id].config.rssi.status) {
			if (DeviceList[entry_id].config.rssi.status) {
				TD_SENSOR_SendEventRSSI(1, entry_id);
			} else {
				TD_SENSOR_SendEventRSSI(0, entry_id);
			}
		}

	}
}

/***************************************************************************//**
 * @brief
 *  Process data from a keepalive frame to configure rssi/connection monitoring of the
 *  concerned device.
 *
 * @param[in] entry
 * 	EntryID of the device which emitted the keepalive.
 *
 * @param[in] frame
 * 	Pointer to the LocalKeepAlive frame.
 *
 ******************************************************************************/
static void TD_SENSOR_GATEWAY_ConfigureDeviceMonitoring(uint8_t entry, LocalKeepAliveFrame * frame)
{

	if (frame->keepalive == false && DeviceList[entry].config.keepalive.monitor == true) {
		//disable keep alive
		DeviceList[entry].config.keepalive.monitor = false;
		TD_SCHEDULER_Remove(DeviceList[entry].config.keepalive.timer);
	} else if (frame->keepalive == true) {
		if (DeviceList[entry].config.keepalive.monitor == false) {
			DeviceList[entry].config.keepalive.timer = TD_SCHEDULER_Append(frame->interval, 0, frame->interval / 2, 0xFF,
					TD_SENSOR_GATEWAY_KeepAliveHandler, entry);
		} else {
			if (frame->interval != DeviceList[entry].config.keepalive.interval) {
				TD_SCHEDULER_SetInterval(DeviceList[entry].config.keepalive.timer, frame->interval, 0, frame->interval / 2);
			}
		}

		DeviceList[entry].config.keepalive.monitor = true;
		DeviceList[entry].config.keepalive.interval = frame->interval;

		if (!frame->rssi) {
			DeviceList[entry].config.rssi.monitor = false;
		} else {
			DeviceList[entry].config.rssi.monitor = true;
			DeviceList[entry].config.rssi.status = true;
			DeviceList[entry].config.rssi.level_low = frame->level_low;
			DeviceList[entry].config.rssi.level_ok = frame->level_ok;
		}
	}
}

/***************************************************************************//**
 * @brief
 *   Gateway RX frame handler
 *
 * @param[in] rx_frame
 *   Received frame
 *
 * @param[in] tx_frame
 *   Transmit frame
 *
 * @return
 * 	Always return 0;
 ******************************************************************************/
static int TD_SENSOR_GATEWAY_FrameReceived(TD_LAN_frame_t *tx_frame, TD_LAN_frame_t *rx_frame)
{
	int i;
	LocalSensorFrame * frame;
	uint8_t lan_address;
	uint8_t frame_count;
	uint8_t entry;

	/**********************************************************************************************************************************/
	/*BE CAREFUL HERE: ACK must be sent quickly, adding trace can slow it down very much and ACK won't be received in time*************/
	/**********************************************************************************************************************************/

	//TODO improve to get it fine for different network
	//get device address considering network mask
	lan_address = GET_ADDRESS(gateway_RX.header) & ((~NETWORK_MASK));

	//if address=0 then that's a broadcast
	if (lan_address == 0) {
		entry = 0;
	} else //otherwise find corresponding entryID
	{
		entry = TD_SENSOR_GATEWAY_GetDevice(lan_address);
	}

	frame = (LocalSensorFrame*) gateway_RX.payload;
	frame_count = frame->count;

	//data count is on 4 bits but we need to handle 0-16 lenght
	if (frame_count == 0) {
		//if one of the byte is !=0 then  count=16 otherwise count=0
		for (i = 0; i < TD_LAN_PAYLOAD_SIZE - 1; i++) {
			if (frame->data[i] != 0) {
				frame_count = 16;
				break;
			}
		}

	}

	if (entry != 0xFF) //if device is found or frame is broadcast
			{
		if (entry != 0) //if device is registered
				{

			if (entry < DeviceCount) {
				switch (frame->type) {
				case LOCAL_FORWARD: {
					LocalForwardFrame * forward_frame;
					TD_SENSOR_GATEWAY_Ack(&gateway_RX, ACK_OK, 0, 0);
					forward_frame = (LocalForwardFrame *) (frame->data);

					TD_SENSOR_TRANSMITTER_SendSigfox((SensorFrame *) forward_frame->sigfox,
							frame_count - sizeof(TransmitProfile) - sizeof(SensorFrameHeader), entry, &forward_frame->profile);

				}
					break;

				case LOCAL_KEEPALIVE: {
					LocalKeepAliveFrame * keepalive_frame;
					TD_SENSOR_GATEWAY_Ack(&gateway_RX, ACK_OK, 0, 0);
					keepalive_frame = (LocalKeepAliveFrame *) (frame->data);

					TD_SENSOR_GATEWAY_ConfigureDeviceMonitoring(entry, keepalive_frame);

					if (DeviceList[entry].config.keepalive.monitor) {
						//if keepalive status was connection lost (changing battery for ex.) then we need to resynchronise
						//ie restart the timer on keep alive reception with very same parameters
						if (DeviceList[entry].config.keepalive.status == false) {
							TD_SCHEDULER_SetInterval(DeviceList[entry].config.keepalive.timer, DeviceList[entry].config.keepalive.interval, 0,
									DeviceList[entry].config.keepalive.interval / 2);
							TD_SENSOR_SendEventConnection(1, entry);
							DeviceList[entry].config.keepalive.status = true;
						}

						TD_SENSOR_GATEWAY_Ack(&gateway_RX, ACK_OK, 0, 0);
						DeviceList[entry].config.keepalive.validated = true;
					} else {
						TD_SENSOR_GATEWAY_Ack(&gateway_RX, ACK_ERROR, 0, 0);
					}

				}
					break;

				case LOCAL_REGISTER:
					TD_SENSOR_GATEWAY_Ack(&gateway_RX, ACK_ERROR, 0, 0);
					break;

				case LOCAL_DATA:

					if (DataCallback != 0) {
						uint8_t reply[15];
						uint8_t reply_count;

						reply_count = (*DataCallback)(frame->data, frame_count, reply);
						TD_SENSOR_GATEWAY_Ack(&gateway_RX, ACK_OK, reply, reply_count);
					} else {
						//echo
						TD_SENSOR_GATEWAY_Ack(&gateway_RX, ACK_OK, frame->data, frame_count);
					}

					break;

				}

				TD_SENSOR_GATEWAY_CheckRSSI(entry);

			}
		} else if (frame->count > 0) //if broadcast and frame count >0
				{
			//only the register local frame is allowed if a device is not already paired
			if (frame->type == LOCAL_REGISTER) {
				/********DON'T ADD TOO MANY THINGS HERE BEFORE ACKING*****************/

				if (RegistrationEnabled || PairingEnabled) {
					//TODO: handle pairing

					Device new_device;
					LocalRegisterFrame *reg_frame;
					LocalKeepAliveFrame *keep_frame;
					uint8_t new_lan_address;

					reg_frame = (LocalRegisterFrame *) (frame->data);
					keep_frame = (LocalKeepAliveFrame*) (&frame->data[sizeof(LocalRegisterFrame)]);

					//to avoid lucky registration first byte is also checked
					if (reg_frame->checker == LOCAL_REGISTER_CHECK) {
						new_device.sigfox_id = reg_frame->SigfoxID;

						//get an address for the device
						new_lan_address = TD_SENSOR_GATEWAY_AppendDevice(&new_device);

						if (new_lan_address == 0xFF) {
							TD_SENSOR_GATEWAY_Ack(&gateway_RX, ACK_ERROR, 0, 0);
						} else //if an address could be given to the device
						{

							LanAddress newAdress;
							newAdress.address = new_lan_address | GatewayAddress;
							newAdress.mask = NETWORK_MASK;

							/******************ACK MUST BE SENT QUCIKLY*************/

							TD_SENSOR_GATEWAY_Ack(&gateway_RX, ACK_OK, (uint8_t *) &newAdress, 8);

							TD_SENSOR_GATEWAY_ConfigureDeviceMonitoring(entry, keep_frame);

							if (RegistrationCallback != 0) {
								(*RegistrationCallback)(newAdress.address, new_device.sigfox_id);
							}

						}
					}

				}

			}

		}
	}

	TD_SENSOR_GATEWAY_StartReception();
	return 0;

}

/** @} */

/*******************************************************************************
 **************************  PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup TD_SENSOR_GATEWAY_PUBLIC_FUNCTIONS Public Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *  Delete all registered devices
 *
 ******************************************************************************/
void TD_SENSOR_GATEWAY_DeleteAllDevices()
{
	int i;

	for (i = 1; i < MAX_DEVICE; i++) {
		//stop device timer
		if (DeviceList[i].config.keepalive.monitor) {
			TD_SCHEDULER_Remove(DeviceList[i].config.keepalive.timer);
		}

		DeviceList[i].sigfox_id = 0;
		DeviceList[i].config.keepalive.monitor = 0;
		DeviceList[i].config.rssi.monitor = 0;
		DeviceList[i].type = TD1202;
	}

	DeviceCount = 1;
}

/***************************************************************************//**
 * @brief
 *  Delete one registered device
 *
 * @param [in] lan_address
 *
 ******************************************************************************/

void TD_SENSOR_GATEWAY_DeleteDevice(uint32_t lan_address)
{
	int i;
	uint8_t entry;

	entry = TD_SENSOR_GATEWAY_GetDevice(lan_address);

	//stop device timer
	if (DeviceList[entry].config.keepalive.monitor) {
		TD_SCHEDULER_Remove(DeviceList[entry].config.keepalive.timer);
	}

	if (entry < DeviceCount) {
		for (i = entry; i < DeviceCount; i++) {
			DeviceList[i].sigfox_id = DeviceList[i + 1].sigfox_id;
			DeviceList[i].lan_address = DeviceList[i + 1].lan_address;
			DeviceList[i].type = DeviceList[i + 1].type;
			DeviceList[i].config.keepalive = DeviceList[i + 1].config.keepalive;
			DeviceList[i].config.rssi = DeviceList[i + 1].config.rssi;
		}

		DeviceList[DeviceCount].sigfox_id = 0;
		DeviceCount--;
	}
}

/***************************************************************************//**
 * @brief
 *  Set a callback on Local Data Frame Reception
 *
 * @param[in] user_data_callback
 *   CallBack should return reply len which must be <=15
 *
 ******************************************************************************/
void TD_SENSOR_GATEWAY_SetDataCallback(uint8_t (*user_data_callback)(uint8_t * data, uint8_t len, uint8_t * reply))
{
	DataCallback = user_data_callback;
}

/***************************************************************************//**
 * @brief
 *  Start listening to local devices
 ******************************************************************************/
void TD_SENSOR_GATEWAY_StartReception()
{
	if (!TD_LAN_ReceiveFrame(T1S, 0, &gateway_RX)) {
		TD_LAN_Stop(RESULT_ABORT);
	}
}

/***************************************************************************//**
 * @brief
 *  Stop listening to local devices
 ******************************************************************************/
void TD_SENSOR_GATEWAY_StopReception()
{
	TD_LAN_Stop(RESULT_ABORT);
}

/***************************************************************************//**
 * @brief
 *   Open Device registration
 ******************************************************************************/
void TD_SENSOR_GATEWAY_StartRegistration(void (*callback)(uint32_t lan_address, uint32_t sigfox_id))
{
	RegistrationCallback = callback;
	TD_SENSOR_LAN_setLanAddress(BROADCAST_ADDRESS, BROADCAST_MASK);
	RegistrationEnabled = true;
	TD_SENSOR_GATEWAY_StartReception();
}

/***************************************************************************//**
 * @brief
 *  Close Device registration
 ******************************************************************************/
void TD_SENSOR_GATEWAY_StopRegistration()
{
	RegistrationEnabled = false;
	TD_SENSOR_LAN_setLanAddress(GatewayAddress, NETWORK_MASK);
	TD_SENSOR_GATEWAY_StartReception();
}

/***************************************************************************//**
 * @brief
 *  Allow pairing
 ******************************************************************************/
void TD_SENSOR_GATEWAY_StartPairing()
{
	TD_SENSOR_LAN_setLanAddress(BROADCAST_ADDRESS, NETWORK_MASK);
	PairingEnabled = true;
	TD_SENSOR_GATEWAY_StartReception();
}

/***************************************************************************//**
 * @brief
 * Forbid pairing
 ******************************************************************************/
void TD_SENSOR_GATEWAY_StopPairing()
{
	PairingEnabled = false;
	TD_SENSOR_LAN_setLanAddress(GatewayAddress, NETWORK_MASK);
	TD_SENSOR_GATEWAY_StartReception();
}

/***************************************************************************//**
 * @brief
 *  Gateway Init
 ******************************************************************************/
void TD_SENSOR_GATEWAY_Init()
{
	int i;

	GatewayAddress = (TD_SENSOR_LAN_ComputeAddressTo16bits(SigfoxID)) << 8;

	if (!TD_FLASH_DeclareVariable((uint8_t *) &DeviceCount, sizeof(uint32_t), 0)) {
		DeviceCount = 1;
	}

	if (!TD_FLASH_DeclareVariable((uint8_t *) &DeviceList, sizeof(Device) * MAX_DEVICE, 0)) {
		DeviceList[0].sigfox_id = SigfoxID;
		DeviceList[0].lan_address = GatewayAddress;

		for (i = 1; i < MAX_DEVICE; i++) {
			DeviceList[i].sigfox_id = 0;
		}
	}

	//reset keepalive and rssi for all concerned device
	for (i = 1; i < MAX_DEVICE; i++) {
		DeviceList[i].config.keepalive.monitor = false;
		DeviceList[i].config.keepalive.status = false;

		DeviceList[i].config.rssi.status = false;
		DeviceList[i].config.rssi.monitor = false;

	}

	TD_LAN_SetUserCallback(TD_SENSOR_GATEWAY_FrameReceived);

	TD_SENSOR_LAN_setLanAddress(GatewayAddress, NETWORK_MASK);
}

/** @} */

/** @} */

