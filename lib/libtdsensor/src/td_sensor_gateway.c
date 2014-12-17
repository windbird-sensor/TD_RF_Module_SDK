/***************************************************************************//**
 * @file
 * @brief Sensor Gateway
 * @author Telecom Design S.A.
 * @version 1.3.0
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

#include <td_lan.h>
#include <td_flash.h>
#include <td_rtc.h>
#include <td_scheduler.h>
#include <td_printf.h>
#include <td_config_ext.h>
#include <td_sigfox.h>

#include "sensor_event.h"
#include "td_sensor.h"
#include "td_sensor_lan.h"
#include "td_sensor_device.h"
#include "td_sensor_transmitter.h"
#include "td_sensor_gateway.h"
#include "td_sensor_utils.h"

/***************************************************************************//**
 * @addtogroup TD_SENSOR_GATEWAY Sensor LAN Gateway
 * @{
 ******************************************************************************/

/*******************************************************************************
 **************************  DEFINES   ****************************************
 ******************************************************************************/

/** @addtogroup TD_SENSOR_GATEWAY_DEFINES Defines
 * @{ */

/** Conditional printf macro */
#define DEBUG_PRINTF(...) //tfp_printf(__VA_ARGS__)

/* Time in seconds to wait after a forward */
#define DEBUG_DUMP(t,d,s) //tfp_dump(t,d,s)

/** @} */

/*******************************************************************************
 **************************  PRIVATE VARIABLES   *******************************
 ******************************************************************************/

/** @addtogroup TD_SENSOR_GATEWAY_LOCAL_VARIABLES Local Variables
 * @{ */

/** Frame received buffer */
static TD_LAN_frame_t gateway_RX;

/** Let to known if an ack frame is expected */
static bool AckExpected = false;

/** Computed Gateway address */
static uint32_t GatewayAddress = 0;

/** Registered device list */
extern TD_SENSOR_GATEWAY_Device_t *DeviceList;

/** Registered device count */
static uint32_t DeviceCount = 1;

/** Registration enable flag */
static bool RegistrationEnabled = false;

/** Pairing enable flag */
static bool PairingEnabled = false;

/** Reception enable flag */
static bool ReceptionEnabled  = false;

/** Reception activated by the registration flag */
static bool ReceptionEnByReg = false;

/** LAN Data received callback */
static int8_t (*DataCallback)(uint8_t *data, uint8_t len, uint8_t *reply) = 0;

/** LAN Data received callback extended */
static int8_t (*DataCallbackExt)(uint8_t entry_id, uint8_t *data, uint8_t len, uint8_t *reply) = 0;

/** Connection callback */
static void (*ConnectionCallback)(bool state, uint8_t id) = 0;

/** Registration callback */
static void (*RegistrationCallback)(uint32_t lan_address, uint32_t sigfox_id) = 0;

/** Registration type */
static TD_SENSOR_GATEWAY_Registration_t RegistrationType = EN_TYPE_REG_NONE;

/** User callback to execute before and after LAN transmission */
static void (*UserCallback)(void) = 0;

/** For code removal purposes */
extern TD_LAN_callback_t const TD_SENSOR_GATEWAY_FrameReceived_;

/** Rx period */
static uint32_t RxPeriod = 0;

/** @} */

/*******************************************************************************
 **************************  PRIVATE FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup TD_SENSOR_GATEWAY_LOCAL_FUNCTIONS Local Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Keep-Alive handler, called by Scheduler.
 *
 * @details
 *   This function makes sure that the device with its connection monitored
 *   has emitted its keep-alive frame within schedule.
 *
 * @param[in] arg
 *	Timer argument. Contains the device entry ID.
 *
 * @param[in] repetitions
 *	Timer argument. Not used.
 ******************************************************************************/
static void TD_SENSOR_GATEWAY_KeepAliveHandler(uint32_t arg, uint8_t repetitions)
{
	uint8_t prev_status;
	uint8_t entry_id = arg;

	// Save previous status
	prev_status = DeviceList[entry_id].config.keepalive.status;
	DeviceList[entry_id].config.keepalive.status =
		DeviceList[entry_id].config.keepalive.validated;

	// Compare status with previous
	if (prev_status != (uint8_t) DeviceList[entry_id].config.keepalive.status) {
		if (!DeviceList[entry_id].config.keepalive.status) {

			// Send connection lost
			if (ConnectionCallback != 0) {
				(*ConnectionCallback)(0, entry_id);
			} else {
				TD_SENSOR_SendEventConnection(0, entry_id);
			}
		}
	}

	// Reset keep alive received indicator
	DeviceList[entry_id].config.keepalive.validated = false;
}

/***************************************************************************//**
 * @brief
 *   Append an Device to the registration list.
 *
 * @param[in] device
 *   Pointer to the device to append. Set its lan_address field to zero to
 *   generate an address according to the SIGFOX ID, or set it to 1..15 to force
 *   a device address.
 *
 * @return
 *   Returns an entry ID if the device has been added, 255 otherwise.
 ******************************************************************************/
static uint32_t TD_SENSOR_GATEWAY_AppendDevicePrivate(
	TD_SENSOR_GATEWAY_Device_t *device)
{
	uint32_t address;
	int i = 0;

	if (DeviceCount >= CONFIG_TD_SENSOR_GATEWAY_MAX_DEVICE) {
		return 255;
	}
	if (device->lan_address == 0) {

		// If device is already registered, give it back its previous address
		for (i = 1; i < CONFIG_TD_SENSOR_GATEWAY_MAX_DEVICE; i++) {
			if (device->sigfox_id == DeviceList[i].sigfox_id) {
				return DeviceList[i].lan_address;
			}
		}

		// Otherwise compute a new address
		address = TD_SENSOR_LAN_ComputeAddressTo8bits(device->sigfox_id);

		// Check if address is not already in use
		for (i = 1; i < DeviceCount; i++) {
			if (DeviceList[i].lan_address == address) {

				// Address 0 and 255 are reserved
				if (++address == 255) {
					address = 1;
				}

				// Re-scan all devices again
				i = 1;
			}
		}
		address |= (GatewayAddress & NETWORK_MASK);
	} else {
		address = device->lan_address;
	}

	// Affect address
	DeviceList[DeviceCount].lan_address = address ;
	DeviceList[DeviceCount].sigfox_id = device->sigfox_id;
	DeviceList[DeviceCount].device_class = device->device_class;
	DeviceCount++;
	return address;
}

/***************************************************************************//**
 * @brief
 *   Returns a device index in device list array according to a SIGFOX ID.
 *
 * @param[in] lan_address
 *   The device LAN address.
 *
 * @return
 *   Returns the index of the device in the device list array, or 255 if the
 *   SIGFOX ID was not found.
 ******************************************************************************/
static uint8_t TD_SENSOR_GATEWAY_GetDevice(uint8_t lan_address)
{
	int i;

	for (i = 1; i < DeviceCount; i++) {
		if ((DeviceList[i].lan_address & 0xFF) == lan_address) {
			return i;
		}
	}
	return 255;
}

/***************************************************************************//**
 * @brief
 *   Send an acknowledgment frame on LAN to a given device.
 *
 * @param[in] rx_frame
 * 	Frame to be acknowledged.
 *
 * @param[in] code
 *  Acknowledgment code to be replied.
 *
 * @param[in] data
 *  Optional data to be sent back. If a null pointer is provided, everything but
 *  the received data last byte is sent back.
 *
 * @param[in] count
 *  Length of the data in bytes.
 ******************************************************************************/
static void TD_SENSOR_GATEWAY_Ack(TD_LAN_frame_t *rx_frame,
	TD_SENSOR_LAN_AckCode_t code, uint8_t *data, uint8_t count)
{
	int i;
	TD_LAN_frame_t tx_frame;

	// Reply frame to sender with acknowledge flag set
	tx_frame.header = rx_frame->header;
	SET_ACK(tx_frame.header, TD_LAN_ACK);

	// First byte is acknowledgment code
	tx_frame.payload[0] = code;

	// If no data provided or too many bytes, then copy everything but the last
	// byte from received frame (header not included)
	if (data == 0 || count == 0 || count > TD_LAN_PAYLOAD_SIZE - 1) {
		for (i = 0; i < TD_LAN_PAYLOAD_SIZE - 1; i++) {
			tx_frame.payload[i + 1] = rx_frame->payload[i + 1];
		}
	} else {

		// Otherwise, just send the data
		for (i = 0; i < count; i++) {
			tx_frame.payload[i + 1] = data[i];
		}

		// Fill with 0s
		for (; i < TD_LAN_PAYLOAD_SIZE - 1; i++) {
			tx_frame.payload[i + 1] = 0;
		}
	}

	if (UserCallback != 0) {
		(*UserCallback)();
	}

	// Send the acknowlegment frame
	if (!TD_LAN_SendFrame(1, &tx_frame, rx_frame)) {
		DEBUG_PRINTF("LAN Error %d \r\n", TD_LAN_LastError());
	}

	if (UserCallback != 0) {
		(*UserCallback)();
	}
}

/***************************************************************************//**
 * @brief
 *   Send an RSSI event for a given device.
 *
 * @param[in] entry_id
 * 	The device entry ID to check for.
 ******************************************************************************/
static void TD_SENSOR_GATEWAY_CheckRSSI(uint8_t entry_id)
{
	int8_t rssi;
	uint8_t old_status;

	if (DeviceList[entry_id].config.rssi.monitor) {
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
 *  Process data from a keep-alive frame to configure RSSI/connection monitoring
 *  for the given deivce.
 *
 * @param[in] entry
 * 	The device entry ID to configure.
 *
 * @param[in] frame
 * 	Pointer to the keep-alive frame.
 ******************************************************************************/
static void TD_SENSOR_GATEWAY_ConfigureDeviceMonitoring(uint8_t entry,
	TD_SENSOR_LAN_KeepAliveFrame_t *frame)
{
	if (frame->keepalive == false &&
		DeviceList[entry].config.keepalive.monitor == true) {
		DeviceList[entry].config.keepalive.monitor = false;
		TD_SCHEDULER_Remove(DeviceList[entry].config.keepalive.timer);
	} else if (frame->keepalive == true) {
		if (DeviceList[entry].config.keepalive.monitor == false) {

			// Add a keep-alive timer for this device
			DeviceList[entry].config.keepalive.timer = TD_SCHEDULER_Append(
				frame->interval,
				0,
				frame->interval / 2,
				0xFF,
				TD_SENSOR_GATEWAY_KeepAliveHandler,
				entry);
		} else {
			if (frame->interval != DeviceList[entry].config.keepalive.interval) {

				// Update an existing keep-alive timer
				TD_SCHEDULER_SetInterval(DeviceList[entry].config.keepalive.timer,
					frame->interval,
					0,
					frame->interval / 2);
			}
		}
		DeviceList[entry].config.keepalive.monitor = true;
		DeviceList[entry].config.keepalive.interval = frame->interval;
		DeviceList[entry].config.keepalive.voltage_level = frame->voltage_level;
		DeviceList[entry].config.keepalive.temperature_level = frame->temp_level;
		DeviceList[entry].config.keepalive.battery = frame->battery;
		if (!frame->rssi) {
			DeviceList[entry].config.rssi.monitor = false;
		} else {
			DeviceList[entry].config.rssi.monitor = true;
			DeviceList[entry].config.rssi.status = true;
			DeviceList[entry].config.rssi.level_low = frame->level_low;
			DeviceList[entry].config.rssi.level_ok = frame->level_ok;
		}
	}

	// Save configuration changes in Flash memory
	TD_FLASH_WriteVariables();
}

/***************************************************************************//**
 * @brief
 *   Gateway received frame handler.
 *
 * @param[in] rx_frame
 *   Pointer to a buffer containing the eceived frame.
 *
 * @param[out] tx_frame
 *   Pointer to a buffer that will receive the frame to transmit.
 *
 * @return
 * 	Always returns 1;
 ******************************************************************************/
int DYNAMIC(TD_SENSOR_GATEWAY_FrameReceived)(TD_LAN_frame_t *tx_frame,
	TD_LAN_frame_t *rx_frame)
{
	int i;
	TD_SENSOR_LAN_Frame_t frame;
	TD_SENSOR_LAN_ForwardFrame_t forward_frame;
	TD_SENSOR_TransmitProfile_t profile;
	uint8_t lan_address;
	uint8_t frame_count;
	uint8_t entry;
	TD_SENSOR_LAN_FrameType_t frame_type;
	uint8_t reply[15];
	int8_t reply_count;
	uint32_t new_lan_address;
	uint8_t msgSize;
	TD_SENSOR_LAN_KeepAliveFrame_t *keepalive_frame;
	TD_SENSOR_GATEWAY_Device_t new_device;
	TD_SENSOR_LAN_RegisterFrame_t *reg_frame;
	TD_SENSOR_LAN_PairingParams_t newPairingParams;

	/* Acknowledgment reply is time-critical, adding traces may slow down a lot
	 * and may prevent acknowledgments from being received on time
	 */
	// We don't want to handle frame with acknowledge flag here
	if (AckExpected == true) {

		// We don't want to handle acknowledgements here
		if (GET_ACK(rx_frame->header)) {
			return 1;
		} else {
			return -1;
		}
	}

	//TODO: Improve to get it fine for different network
	// Get device address filtered with the network mask
	lan_address = GET_ADDRESS(gateway_RX.header) & ((~NETWORK_MASK));

	//TODO: THIS IS WRONG (about broadcast)
	// If address is 0, then it is a broadcast
	if (lan_address == 0) {
		entry = 0;
	} else {

		// Otherwise find corresponding entryID
		entry = TD_SENSOR_GATEWAY_GetDevice(lan_address);
	}
	DEBUG_PRINTF("E %d\r\n", entry);
	DEBUG_DUMP("RX: ", gateway_RX.payload, TD_LAN_PAYLOAD_SIZE);
	memcpy(&frame, gateway_RX.payload, TD_LAN_PAYLOAD_SIZE);
	frame_count = GET_LOCALSENSORFRAME_COUNT(frame.header);
	frame_type = (TD_SENSOR_LAN_FrameType_t) GET_LOCALSENSORFRAME_TYPE(
		frame.header);
	DEBUG_DUMP("Frame: ", &frame, 16);

	// Data count is on 4 bits but we need to handle 0-16 length
	if (frame_count == 0) {

		// If one of the byte is != 0, then  count = 16 otherwise count = 0
		for (i = 0; i < TD_LAN_PAYLOAD_SIZE - 1; i++) {
			if (frame.data[i] != 0) {
				frame_count = 16;
				break;
			}
		}
	}
	DEBUG_PRINTF("RXT %d %d\r\n", frame_type, frame_count);
	if (entry != 255) {

		// If device is found or frame is broadcast
		if (entry != 0) {

			// If device is registered
			if (entry < DeviceCount) {
				switch (frame_type) {
				case LOCAL_FORWARD: {
					TD_SENSOR_GATEWAY_Ack(&gateway_RX, ACK_OK, 0, 0);
					memcpy((uint8_t *)&forward_frame, frame.data, frame_count);
					profile.interval = GET_LOCALFORWARDFRAME_INTERVAL(
						forward_frame.profile);
					profile.repetition = GET_LOCALFORWARDFRAME_REPETITION(
						forward_frame.profile);
					DEBUG_DUMP("Forward: ", &forward_frame, frame_count);
					DEBUG_PRINTF(" \r\nProfile:%d %d\r\n", profile.interval,
						profile.repetition);

					//TODO: Replace SIGFOX frame and profile decoding!
					TD_SENSOR_TRANSMITTER_SendSigfox(forward_frame.sigfox,
						frame_count - sizeof (forward_frame.profile),
						entry,
						&profile);
				}
				break;

				case LOCAL_KEEPALIVE:
					keepalive_frame = (TD_SENSOR_LAN_KeepAliveFrame_t *)
						(frame.data);
					keepalive_frame->interval = NTOHL(keepalive_frame->interval);
					TD_SENSOR_GATEWAY_ConfigureDeviceMonitoring(entry,
						keepalive_frame);
					if (DeviceList[entry].config.keepalive.monitor) {
						TD_SENSOR_GATEWAY_Ack(&gateway_RX, ACK_OK, 0, 0);

						// If keep-alive status indicates a connection lost
						// (battery was replaced for example), then we need to
						// resynchronize, i.e. restart the timer on keep-alive
						// reception with very same parameters
						if (DeviceList[entry].config.keepalive.status == false) {

							TD_SCHEDULER_SetInterval(
								DeviceList[entry].config.keepalive.timer,
								DeviceList[entry].config.keepalive.interval,
								0,
								DeviceList[entry].config.keepalive.interval / 2);

							// Send connection ok
							if (ConnectionCallback != 0) {
								(*ConnectionCallback)(1, entry);
							} else {
								TD_SENSOR_SendEventConnection(1, entry);
							}
							DeviceList[entry].config.keepalive.status = true;
						}
						DeviceList[entry].config.keepalive.validated = true;
					} else {
						TD_SENSOR_GATEWAY_Ack(&gateway_RX, ACK_ERROR, 0, 0);
					}
					break;

				case LOCAL_REGISTER:
					TD_SENSOR_GATEWAY_Ack(&gateway_RX, ACK_ERROR, 0, 0);
					break;


				case LOCAL_DATA:
					if ((DataCallback != 0) || (DataCallbackExt != 0)) {
						if (DataCallback != 0) {
							reply_count = (*DataCallback)(frame.data,
								frame_count, reply);
						} else {
							reply_count = (*DataCallbackExt)(entry, frame.data,
								frame_count, reply);
						}
						if (reply_count > 0) {

							// Custom acknowledgment
							TD_SENSOR_GATEWAY_Ack(&gateway_RX, ACK_OK, reply,
								reply_count);
						} else if (reply_count == 0) {

							// Simple-copy acknowledgment
							TD_SENSOR_GATEWAY_Ack(&gateway_RX, ACK_OK,
								frame.data, frame_count);
						} else {

							// No acknowledgment
						}
					} else {

						// Simple-copy acknowledgment
						TD_SENSOR_GATEWAY_Ack(&gateway_RX, ACK_OK, frame.data,
							frame_count);
					}
					break;

				default:
					break;
				}
				TD_SENSOR_GATEWAY_CheckRSSI(entry);
			}
		} else if (frame_count > 0) {

			// Only a LAN registration frame is allowed if a device is not
			// already paired
			if (frame_type == LOCAL_REGISTER) {
				if (RegistrationEnabled || PairingEnabled) {

					//TODO: handle pairing
					reg_frame = (TD_SENSOR_LAN_RegisterFrame_t *) (frame.data);
					new_device.sigfox_id = reg_frame->SigfoxID;
					new_device.device_class = reg_frame->device_class;
					new_device.lan_address = 0;

					// Get an address for the device
					new_lan_address = TD_SENSOR_GATEWAY_AppendDevicePrivate(
						&new_device);
					if (new_lan_address == 255) {

						// If no address could be assigned to the device
						TD_SENSOR_GATEWAY_Ack(&gateway_RX, ACK_ERROR, 0, 0);
					} else {

						// If an address could be given to the device
						TD_SENSOR_LAN_Address_t newAdress;
						newAdress.address = new_lan_address;
						newAdress.mask = NETWORK_MASK;
						newPairingParams.lanAddress = newAdress;
						newPairingParams.pairingType = (uint8_t) RegistrationType;
						if (RegistrationType != EN_TYPE_REG_NONE) {
							msgSize = sizeof (TD_SENSOR_LAN_PairingParams_t);
						} else {
							msgSize = sizeof (TD_SENSOR_LAN_Address_t);
						}

						/******************ACK MUST BE SENT QUCIKLY*************/
						TD_SENSOR_GATEWAY_Ack(&gateway_RX, ACK_OK,
							(uint8_t *) &newPairingParams, msgSize);

						if (RegistrationCallback != 0) {
							(*RegistrationCallback)(newAdress.address,
								new_device.sigfox_id);
						}

						// Save configuration changes in Flash memory
						TD_FLASH_WriteVariables();
					}
				}
			}
		}
	}

	// Restart reception if it was not stopped within function
	if (ReceptionEnabled) {
		TD_SENSOR_GATEWAY_StartReception();
	}
	return 1;
}

/** @} */

/*******************************************************************************
 **************************  PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup TD_SENSOR_GATEWAY_GLOBAL_FUNCTIONS Global Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *  Gateway Initialization.
 *
 * @return
 *  Returns false if LAN initialization failed, true otherwise.
 ******************************************************************************/
bool DYNAMIC(TD_SENSOR_GATEWAY_Init)(void)
{
	int i;
	const TD_SIGFOX_Proxy_t *pProxy;
	const TD_SIGFOX_ProxyStatus_t *pProxyStatus;
	const TD_SIGFOX_ProxyConfig_t *pProxyConfig;

	bool booster_found = false, booster_exist = false;
	GatewayAddress = (TD_SENSOR_LAN_ComputeAddressTo16bits(SigfoxID)) << 8;

	// Reset keep-alive and RSSI for all devices
	for (i = 1; i < CONFIG_TD_SENSOR_GATEWAY_MAX_DEVICE; i++) {
		DeviceList[i].config.keepalive.monitor = false;
		DeviceList[i].config.keepalive.status = false;
		DeviceList[i].config.rssi.status = false;
		DeviceList[i].config.rssi.monitor = false;
	}

	// If a booster is paired, add it in DeviceList
	pProxy = TD_SIGFOX_PROXY_Get();
	pProxyStatus = &pProxy->status;
	pProxyConfig = &pProxy->config;
	if (pProxyStatus->paired == true)	{

		// Check if the booster is already added to DeviceList
		for (i = 1; i < CONFIG_TD_SENSOR_GATEWAY_MAX_DEVICE; i++) {

			// If Booster Sigfox id is found
			if (DeviceList[i].sigfox_id == pProxyStatus->id) {
				booster_found = true;
				break;
			}
		}

		// It's a new booster
		if (booster_found == false) {

			// Check if a booster already exist in pairing table
			for (i = 1; i < CONFIG_TD_SENSOR_GATEWAY_MAX_DEVICE; i++) {

				// If Booster class is found
				if (DeviceList[i].device_class == 0x0009) {
					booster_exist = true;
					break;
				}
			}

			// If there is no booster in pairing table
			if (booster_exist == false)	{

				// If there is any place in device table
				if (DeviceCount <= CONFIG_TD_SENSOR_GATEWAY_MAX_DEVICE) {

					// Set to 0, not used for booster
					memset(&DeviceList[DeviceCount].config.keepalive, 0,
						sizeof (DeviceList[DeviceCount].config.keepalive));
					memset(&DeviceList[DeviceCount].config.rssi, 0,
						sizeof (DeviceList[DeviceCount].config.rssi));

					// Set booster infos
					DeviceList[DeviceCount].sigfox_id = pProxyStatus->id;
					DeviceList[DeviceCount].lan_address = pProxyStatus->address;
					DeviceList[DeviceCount].type = SENSOR_GATEWAY;
					DeviceList[DeviceCount].device_class = pProxyConfig->class;
					DeviceCount++;
				} else {

					// There is no place in pairing table
					TD_SIGFOX_PROXY_ClearPairing();
					return false;
				}
			} else {

				// Previous booster exist in pairing table, replace it
				DeviceList[i].sigfox_id = pProxyStatus->id;
				DeviceList[i].lan_address = pProxyStatus->address;
				DeviceList[i].type = SENSOR_GATEWAY;
				DeviceList[i].device_class = pProxyConfig->class;
			}
		}
	}
	return TD_SENSOR_LAN_setLanAddress(GatewayAddress, NETWORK_MASK);
}

/***************************************************************************//**
 * @brief
 *  Declare flash variables.
 *
 * @return
 *   Nothing.
 ******************************************************************************/
void TD_SENSOR_GATEWAY_DeclareFlashVariable(void)
{
	uint8_t i;
	uint32_t gatewayAddress = 0;

	TD_SENSOR_Configuration_t *config;
	config = TD_SENSOR_GetModuleConfiguration();
	gatewayAddress = (TD_SENSOR_LAN_ComputeAddressTo16bits(SigfoxID)) << 8;
	if (!TD_FLASH_DeclareVariable((uint8_t *) &DeviceCount, sizeof (uint32_t), 0)) {
		DeviceCount = 1;
	}
	if (!TD_FLASH_DeclareVariable((uint8_t *) DeviceList,
		sizeof (TD_SENSOR_GATEWAY_Device_t) * CONFIG_TD_SENSOR_GATEWAY_MAX_DEVICE,
		0)) {
		DeviceList[0].sigfox_id = SigfoxID;
		DeviceList[0].lan_address = gatewayAddress;
		DeviceList[0].type = config->type;
		DeviceList[0].device_class = config->class;
		for (i = 1; i < CONFIG_TD_SENSOR_GATEWAY_MAX_DEVICE; i++) {
			DeviceList[i].sigfox_id = 0;
		}
	}
}

/** @} */

/** @addtogroup TD_SENSOR_GATEWAY_USER_FUNCTIONS User Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *  Send a data frame to a specific device given by its address.
 *
 * @param[in] address
 *  The address to which the frame should be sent.
 *
 * @param[in] data
 *  Pointer to the buffer containing the data to be sent.
 *
 * @param[in] length
 *  Size in bytes of data to be sent.
 *
 * @param[out] data_rx
 *  Pointer to a buffer that will received the reply.
 ******************************************************************************/
TD_SENSOR_LAN_AckCode_t TD_SENSOR_GATEWAY_SendDataByAddress(uint32_t address,
	uint8_t *data, uint8_t length, uint8_t *data_rx)
{
	TD_SENSOR_LAN_AckCode_t ret;

	AckExpected = true;
	if (UserCallback != 0) {
		UserCallback();
	}
	ret = TD_SENSOR_LAN_SendFrameTo(address, LOCAL_DATA, data, length, data_rx);
	if (UserCallback != 0) {
		UserCallback();
	}
	AckExpected = false;
	return ret;
}

/***************************************************************************//**
 * @brief
 *  Broadcast a data frame to all devices.
 *
 * @param[in] data
 *  Pointer to the buffer containing the data to be broadcasted.
 *
 * @param length
 *  Size in bytes of data to be sent.
 ******************************************************************************/
TD_SENSOR_LAN_AckCode_t TD_SENSOR_GATEWAY_SendDataBroadcast(uint8_t *data, uint8_t length)
{
	TD_SENSOR_LAN_AckCode_t ret;
	uint8_t temp = TD_SENSOR_LAN_GetFrameRetry();

	// No acknowledgment as this is a broadcast
	AckExpected = true;
	TD_SENSOR_LAN_SetFrameRetry(0);
	if (UserCallback != 0) {
		UserCallback();
	}
	ret = TD_SENSOR_LAN_SendFrameTo(GatewayAddress | 0xFF, LOCAL_DATA, data, length,
			0);
	if (UserCallback != 0) {
		UserCallback();
	}
	TD_SENSOR_LAN_SetFrameRetry(temp);
	AckExpected = false;
	return ret;
}

/***************************************************************************//**
 * @brief
 *  Broadcast a data frame to all devices without waiting Ack.
 *
 * @param[in] data
 *  Pointer to the buffer containing the data to be broadcasted.
 *
 * @param length
 *  Size in bytes of data to be sent.
 ******************************************************************************/
TD_SENSOR_LAN_AckCode_t TD_SENSOR_GATEWAY_SendDataBroadcastWithoutAck(uint8_t *data, uint8_t length)
{
	TD_SENSOR_LAN_AckCode_t ret;
	uint8_t temp = TD_SENSOR_LAN_GetFrameRetry();

	// No acknowledgment as this is a broadcast
	AckExpected = false;
	TD_SENSOR_LAN_SetFrameRetry(0);
	if (UserCallback != 0) {
		UserCallback();
	}
	ret = TD_SENSOR_LAN_SendFrameTo(GatewayAddress | 0xFF, LOCAL_DATA, data, length, 0);
	if (UserCallback != 0) {
		UserCallback();
	}
	TD_SENSOR_LAN_SetFrameRetry(temp);
	return ret;
}

/***************************************************************************//**
 * @brief
 *  Broadcast a keep-alive frame to all devices.
 *
 * @param[in] enabled
 *  Reserved for future use.
 *
 * @param[in] interval
 *  Interval in seconds at which the keep-alive frame is sent.
 *
 * @param[in] data
 *   Reserved for future use.
 *
 * @param[in] length
 *   Reserved for future use.
 ******************************************************************************/
void TD_SENSOR_GATEWAY_SendKeepAliveBroadcast(bool enabled, uint8_t interval,
	uint8_t *data, uint8_t length)
{
	TD_SENSOR_LAN_KeepAliveFrame_t frame;
	uint8_t payload[7] = {0};
	uint8_t *pPayload = NULL;
	//uint8_t keepalive_rssi = 0;
	uint8_t temp = TD_SENSOR_LAN_GetFrameRetry();

	// No acknowledgment as this is a broadcast
	TD_SENSOR_LAN_SetFrameRetry(0);
	memset(&frame, 0, sizeof (frame));
	frame.interval = interval;
	frame.level_low = 0;
	frame.level_ok = 0;
	frame.keepalive = true;
	frame.rssi = true;
	AckExpected = true;
	pPayload = payload;

	/* Build interval */
	frame.interval = HTONL(frame.interval);
	memcpy(pPayload, &frame.interval, sizeof (frame.interval));
	pPayload += sizeof (interval);

	/* Build level low */
	*pPayload = frame.level_low;
	pPayload += sizeof (frame.level_low);

	/* Build level ok */
	*pPayload = frame.level_ok;
	pPayload += sizeof (frame.level_ok);

	/* Build keepalive and rssi */
	*pPayload = (frame.keepalive == true ? 0x01 : 0x00) |
		(frame.rssi == true ? 0x02 : 0x00);
	//pPayload += sizeof(keepalive_rssi);
	pPayload += sizeof (uint8_t);
	if (UserCallback != 0) {
		UserCallback();
	}
	TD_SENSOR_LAN_SendFrameTo(GatewayAddress | 0xFF,
		LOCAL_KEEPALIVE,
		payload,
		pPayload - payload,
		0);
	if (UserCallback != 0) {
		UserCallback();
	}
	TD_SENSOR_LAN_SetFrameRetry(temp);
	AckExpected = false;
}

/***************************************************************************//**
 * @brief
 *   Returns device index in device list array if any.
 *
 * @param[in] device_class
 *   The device class to match.
 *
 * @param[out] device_list
 *   Pointer to an array that will receive the matching indices.
 *
 * @return
 *   Returns the number of devices with the device_class, max is 5 devices
 ******************************************************************************/
uint8_t TD_SENSOR_GATEWAY_GetDeviceIndex(uint16_t device_class,
	uint8_t *device_list)
{
	uint8_t index, indexDev = 0;

	for (index = 1; index < DeviceCount; index++) {
		if (DeviceList[index].device_class == device_class) {
			if (indexDev < 5) {
				device_list[indexDev++] = index;
			}
		}
	}
	return indexDev;
}

/***************************************************************************//**
 * @brief
 *  Get a pointer to the Device list including gateway itself at index 0.
 *
 * @return
 *  Returns a pointer to the device list.
 ******************************************************************************/
const TD_SENSOR_GATEWAY_Device_t *TD_SENSOR_GATEWAY_GetDeviceList(void)
{
	return DeviceList;
}

/***************************************************************************//**
 * @brief
 *  Get the count of registered devices.
 *
 * @return
 *  Returns the count of currently registered devices.
 ******************************************************************************/
uint8_t TD_SENSOR_GATEWAY_GetDeviceCount(void)
{

	// Device count includes gateway itself
	return DeviceCount;
}

/***************************************************************************//**
 * @brief
 *  Delete all registered devices.
 ******************************************************************************/
void TD_SENSOR_GATEWAY_DeleteAllDevices(void)
{
	int i;

	for (i = 1; i < CONFIG_TD_SENSOR_GATEWAY_MAX_DEVICE; i++) {
		if (DeviceList[i].config.keepalive.monitor) {
			TD_SCHEDULER_Remove(DeviceList[i].config.keepalive.timer);
		}
		DeviceList[i].sigfox_id = 0;
		DeviceList[i].config.keepalive.monitor = 0;
		DeviceList[i].config.rssi.monitor = 0;
		DeviceList[i].type = 0;
	}

	// The only device left is the gateway itself
	DeviceCount = 1;
}

/***************************************************************************//**
 * @brief
 *  Delete one registered device from the device list.
 *
 * @param [in] lan_address
 *  The device LAN address to delete.
 ******************************************************************************/
void TD_SENSOR_GATEWAY_DeleteDevice(uint32_t lan_address)
{
	int i;
	uint8_t entry;

	entry = TD_SENSOR_GATEWAY_GetDevice(lan_address);

	// Stop device keep-alive timer if any
	if (DeviceList[entry].config.keepalive.monitor) {
		TD_SCHEDULER_Remove(DeviceList[entry].config.keepalive.timer);
	}

	//TODO: replace with memcpy
	if (entry < DeviceCount) {
		for (i = entry; i < DeviceCount; i++) {
			DeviceList[i].sigfox_id = DeviceList[i + 1].sigfox_id;
			DeviceList[i].lan_address = DeviceList[i + 1].lan_address;
			DeviceList[i].type = DeviceList[i + 1].type;
			DeviceList[i].config.keepalive = DeviceList[i + 1].config.keepalive;
			DeviceList[i].config.rssi = DeviceList[i + 1].config.rssi;
			DeviceList[i].device_class = DeviceList[i + 1].device_class;
		}
		DeviceList[DeviceCount].sigfox_id = 0;
		DeviceCount--;
	}
}

/***************************************************************************//**
 * @brief
 *  Delete one registered device from the device list by using its entry id.
 *
 * @param [in] entry_id
 *  The device entry id to delete.
 ******************************************************************************/
void TD_SENSOR_GATEWAY_DeleteDeviceByEntryId(uint8_t entry_id)
{
	int index;

	// Stop device keep-alive timer if any
	if (DeviceList[entry_id].config.keepalive.monitor) {
		TD_SCHEDULER_Remove(DeviceList[entry_id].config.keepalive.timer);
	}
	if (entry_id < DeviceCount) {
		for (index = entry_id; index < DeviceCount; index++) {
			memcpy((char *)&DeviceList[index], (char *)&DeviceList[index + 1], sizeof(TD_SENSOR_GATEWAY_Device_t));
		}

		// Reset device on the top of the table
		memset((char *)&DeviceList[DeviceCount], 0, sizeof (TD_SENSOR_GATEWAY_Device_t));
		DeviceCount--;
	}
}

/***************************************************************************//**
 * @brief
 *  Set a callback on Local Data Frame Reception
 *
 * @param[in] user_data_callback
 *   Pointer to a callback function that will be called upon local data frame
 *   reception with the following arguments:
 *     - data ill be a pointer to a buffer containing the received payload
 *     - length will be the size in bytes of the received payload
 *     - reply will be a pointer to the buffer to be filled with reply if used
 *   The callback function should return the reply length which must be <=15, or
 *   < 0 if no acknowledgment is required.
 *   This function must return as quickly as possible in the case where an
 *   acknowledgment should be sent.
 ******************************************************************************/
void TD_SENSOR_GATEWAY_SetDataCallback(int8_t (*user_data_callback)(
		uint8_t *data, uint8_t length, uint8_t *reply))
{
	DataCallback = user_data_callback;
}


/***************************************************************************//**
 * @brief
 *  Set a callback on Local Data Frame Reception
 *
 * @param[in] user_data_callback
 *   Pointer to a callback function that will be called upon local data frame
 *   reception with the following arguments:
 *     - data ill be a pointer to a buffer containing the received payload
 *     - length will be the size in bytes of the received payload
 *     - reply will be a pointer to the buffer to be filled with reply if used
 *   The callback function should return the reply length which must be <=15, or
 *   < 0 if no acknowledgment is required.
 *   This function must return as quickly as possible in the case where an
 *   acknowledgment should be sent.
 ******************************************************************************/
void TD_SENSOR_GATEWAY_SetDataCallbackExt(int8_t (*user_data_callback)(
		uint8_t entry_id, uint8_t *data, uint8_t length, uint8_t *reply))
{
	DataCallbackExt = user_data_callback;
}


/***************************************************************************//**
 * @brief
 *  Set a callback to execute when connection with a device is lost or
 *  connected
 *
 * @param[in] user_connection_callback
 *   Pointer to a callback function that will be called upon connection is lost
 *   or retrieved with the following arguments:
 *     - state will be connection state (0 = lost, 1 = connected)
 *     - id will be the device id in pairing table
 *   The callback function should return nothing.
 *   This function must return as quickly as possible in the case where an
 *   acknowledgment should be sent.
 ******************************************************************************/
void TD_SENSOR_GATEWAY_SetConnectionCallback(void (*user_connection_callback)
	(bool state, uint8_t id))
{
	ConnectionCallback = user_connection_callback;
}

/***************************************************************************//**
 * @brief
 *  Start listening for local devices.
 ******************************************************************************/
void TD_SENSOR_GATEWAY_StartReception(void)
{
	ReceptionEnabled = true;
	TD_LAN_SetUserCallback(TD_SENSOR_GATEWAY_FrameReceived);
	//tfp_printf("TD_SENSOR_GATEWAY_StartReception:0x%08X\r\n",&gateway_RX);
	if (RxPeriod == 0) {
		RxPeriod = CONFIG_LAN_PERIOD;
	}
	TD_LAN_ReceiveFrame(RxPeriod, 0, &gateway_RX);
}

/***************************************************************************//**
 * @brief
 *  Stop listening for local devices.
 ******************************************************************************/
void TD_SENSOR_GATEWAY_StopReception(void)
{
	ReceptionEnabled = false;
	TD_LAN_Abort();
}

/***************************************************************************//**
 * @brief
 *  Get the current reception state.
 *
 * @return
 *  Returns true if reception enabled. false otherwise.
 ******************************************************************************/
bool TD_SENSOR_GATEWAY_IsReceptionEnabled(void)
{
	return ReceptionEnabled;
}

/***************************************************************************//**
 * @brief
 *   Start Device registration.
 *
 * @param[in] callback
 *   Pointer to the user callback function which will be called when a
 *   registration occurs, with the following arguments:
 *     - lan_address will be the LAN address of the newly registered device
 *     - sigfox_id will be the corresponding SIGFOX ID of he device
 ******************************************************************************/
void TD_SENSOR_GATEWAY_StartRegistration(void (*callback)(uint32_t lan_address,
	uint32_t sigfox_id))
{
	TD_SENSOR_GATEWAY_StartRegistrationExt(callback, EN_TYPE_REG_NONE);
}

/***************************************************************************//**
 * @brief
 *   Open Device registration
 *
 * @param[in] callback
 *  User callback which will be called when a registration occur
 *  with computed lan_address and sigfox id of the newly registered device as
 *  parameters.
 *
 * @param[in] type
 *  Registration type to start.
 ******************************************************************************/
void TD_SENSOR_GATEWAY_StartRegistrationExt(void (*callback)(
		uint32_t lan_address, uint32_t sigfox_id),
	TD_SENSOR_GATEWAY_Registration_t type)
{
	bool rx_en = ReceptionEnabled;

	// Stop reception if activated
	if (rx_en) {
		ReceptionEnByReg = false;
		TD_SENSOR_GATEWAY_StopReception();
	} else {
		ReceptionEnByReg = true;
	}
	RegistrationCallback = callback;
	RegistrationType = type;
	TD_SENSOR_LAN_setLanAddress(BROADCAST_ADDRESS, BROADCAST_MASK);
	RegistrationEnabled = true;

	// Always start reception
	TD_SENSOR_GATEWAY_StartReception();
}

/***************************************************************************//**
 * @brief
 *  Close Device registration
 ******************************************************************************/
void TD_SENSOR_GATEWAY_StopRegistration(void)
{
	TD_SENSOR_GATEWAY_StopReception();
	RegistrationEnabled = false;
	RegistrationCallback = 0;
	TD_SENSOR_LAN_setLanAddress(GatewayAddress, NETWORK_MASK);

	// If reception was not started by registration, restart it
	if (!ReceptionEnByReg) {
		TD_SENSOR_GATEWAY_StartReception();
	}
}

/***************************************************************************//**
 * @brief
 *   Manually append a device.
 *
 * @param[in] sigfox_id
 *   The SIGFOX ID for the device to add.
 *
 * @param[in] class
 *   The class to use for the device to add.
 *
 * @param[in] lan_address
 *   The LAN address to assign to the device to add. Set to zero to generate an
 *   address according to the SIGFOX ID, or set it to 1..15 to force a device
 *   address.
 *
 * @return
 *   Returns an entry ID if the device has been added, 255 otherwise.
 ******************************************************************************/
uint32_t TD_SENSOR_GATEWAY_AppendDevice(uint32_t sigfox_id, uint16_t class,
	uint32_t lan_address)
{
	TD_SENSOR_GATEWAY_Device_t device;
	device.sigfox_id = sigfox_id;
	device.device_class = class;
	device.lan_address = lan_address;
	return (TD_SENSOR_GATEWAY_AppendDevicePrivate(&device) |
		(GatewayAddress & NETWORK_MASK));
}

/***************************************************************************//**
 * @brief
 *  Start pairing
 ******************************************************************************/
void TD_SENSOR_GATEWAY_StartPairing(void)
{
	TD_SENSOR_LAN_setLanAddress(BROADCAST_ADDRESS, NETWORK_MASK);
	PairingEnabled = true;
	TD_SENSOR_GATEWAY_StartReception();
}

/***************************************************************************//**
 * @brief
 *  Stop pairing
 ******************************************************************************/
void TD_SENSOR_GATEWAY_StopPairing(void)
{
	PairingEnabled = false;
	TD_SENSOR_LAN_setLanAddress(GatewayAddress, NETWORK_MASK);
	TD_SENSOR_GATEWAY_StartReception();
}

/***************************************************************************//**
 * @brief
 *   Callback to call before and after Lan/Sigfox transmission
 ******************************************************************************/
void TD_SENSOR_GATEWAY_SetUserCallback(void (*user_callback)(void))
{
	UserCallback = user_callback;
}

/***************************************************************************//**
 * @brief
 *   Callback to set tx and rx reception period
 ******************************************************************************/
void TD_SENSOR_GATEWAY_SetTxRxPeriod(uint32_t tx_period, uint32_t rx_period)
{
	RxPeriod = rx_period;
	TD_SENSOR_LAN_SetTxPeriod(tx_period);
}

/***************************************************************************//**
 * @brief
 *  Get the current Ack expected state.
 *
 * @return
 *  Returns the current Ack expected state.
 ******************************************************************************/
bool TD_SENSOR_GATEWAY_GetAckExpectedState(void)
{
	return AckExpected;
}

/** @} */

/** @} (end addtogroup TD_SENSOR_GATEWAY) */
