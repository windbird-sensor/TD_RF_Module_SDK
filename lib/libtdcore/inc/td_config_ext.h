/** @cond TD_CONFIG */
/***************************************************************************//**
 * @file
 * @brief Configuration file for the TDxxxx RF modules.
 * @author Telecom Design S.A.
 * @version 1.1.0
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

#ifndef __TD_CONFIG_EXT_H
#define __TD_CONFIG_EXT_H

#include <stdint.h>
#include <stdbool.h>

#include <em_gpio.h>
#include <td_gpio.h>

#define TD_SENSOR_MONITOR_BOOT			0x1
#define TD_SENSOR_MONITOR_BATT			0x2
#define TD_SENSOR_MONITOR_TEMP			0x4
#define TD_SENSOR_MONITOR_CONNECT		0x8
#define TD_SENSOR_MONITOR_KEEPALIVE		0x10
#define TD_SENSOR_MONITOR_SWITCH		0x20

/** Macro to declare a dynamic function */
#define DYNAMIC(function) \
	function##__DYNAMIC

/** Macro to declare a link-time binding for a dynamic function */
#define DECLARE_DYNAMIC(result, function, ...) \
	typedef result (*function##_t) (__VA_ARGS__); \
	extern function##_t const function; \
	result function##__DYNAMIC(__VA_ARGS__);

/** Macro to declare an alternate link-time binding for a dynamic function */
#define DECLARE_DYNAMIC_ALT(result, function, function_alt, ...) \
	extern function##_t const function_alt; \
	result function_alt##__DYNAMIC(__VA_ARGS__);

/** Dynamic function NULL binding */
#define NULL_DYNAMIC(function) \
	function##_t const function = (function##_t) NULL;

/** Dynamic function TRAP_HERE binding */
#define TRAP_DYNAMIC(function) \
	function##_t const function = (function##_t) TD_TRAP_HERE;

/** Dynamic function binding to actual implementation code */
#define INIT_DYNAMIC(function) \
	function##_t const function = function##__DYNAMIC;

/** Dynamic alternate function binding to actual implementation code */
#define INIT_DYNAMIC_ALT(function,function_alt) \
	function##_t const function = function_alt##__DYNAMIC;

/** Dynamic function NOP_HERE binding */
#define NOP_DYNAMIC(function) \
	function##_t const function = (function##_t) TD_NOP_HERE;

/* To define custom board, td_config.h default value must be overloaded
 * Here we defined useful helpers to do this overloading
 * See td_config.h documentation for more in depth informations
 */

#define SIZEOF(x) ((char*)(&(x) + 1) - (char*)&(x))

#ifndef NULL
#define NULL ((void*)0)
#endif

#define IS_EMPTY(x)		1-x-1==2

/* Product init data macros and define
 * This small command subsystem is used for initial pin setup during bootloading
 * stage
 */

/* Product init data global command */
#define PRODUCT_DATA_INIT_LOW		0
#define PRODUCT_DATA_INIT_HIGH		1
#define PRODUCT_DATA_INIT_STRENGTH	2
#define PRODUCT_DATA_INIT_ROUTE		3

/* Product init data / IO setup mode */
#define PI_DISABLED		0
#define PI_INPUT		1
#define PI_INPUT_PULL	2
#define PI_OUTPUT		4

/* Product init data route options / IO route (debug) mode */
#define PRODUCT_DATA_INIT_ROUTE_DISABLE		0

/* Product init data / macros */
#define PIP(p,b,m,v) 	(((p) << 12) | ((b) << 8) | ((v) << 4) | (m))
#define PIS(p,s) 		(((p) << 12) | ((s) <<8 ) | ((PRODUCT_DATA_INIT_STRENGTH) << 4))
#define PIR(s) 			(((s) << 8 ) | ((PRODUCT_DATA_INIT_ROUTE) << 4))
#define PINULL 			(((0xF) << 4))

/* Theses are global constants value that can be used in projects */

/*
	 *    #define                                 | Level        | Type      |      Usage     | Description
	 *    :---------------------------------------|:------------:|:---------:|:--------------:|:--------------------------------------------------------------------
	 *    MANUFACTURER                            | Applicative  | string    |  AT commands   | Manufacturer name
	 *    HARDWARE_VERSION                        | Applicative  | string    |  AT commands   | Hardware version
	 *    SOFTWARE_VERSION                        | Applicative  | string    |  AT commands   | Software version
	 *    RELEASE_DATE                            | Applicative  | string    |  AT commands   | Release date
	 *    SERIAL_NUMBER                           | Applicative  | string    |  AT commands   | Serial number
	 *    PRODUCT                                 | Applicative  | string    |  AT commands   | Product name
*/
//#define PRE_SDK4_COMPATIBILITY
#ifdef PRE_SDK4_COMPATIBILITY
#define ProductType 							CONFIG_PRODUCT_TYPE
#define ProductLedPolarity						CONFIG_PRODUCT_LED_POLARITY
#define ProductLedPort							CONFIG_PRODUCT_LED_PORT
#define ProductLedBit							CONFIG_PRODUCT_LED_BIT
#define ProductLedBlink							CONFIG_PRODUCT_LED_BLINK
#define ProductBootloaderChannel				CONFIG_PRODUCT_BOOTLOADER_CHANNEL
#define ProductBootloaderSkip					CONFIG_PRODUCT_BOOTLOADER_SKIP
#define ProductUartBootloaderSkip				CONFIG_PRODUCT_UART_BOOTLOADER_SKIP
#define ProductInitData							CONFIG_PRODUCT_INIT_DATA
#define ProductInitDataSize						CONFIG_PRODUCT_INIT_DATA_SIZE
#define TD_SPI_MaxConf							CONFIG_MAX_SPI_ID
#define TD_FLASH_MaxDataPointer					CONFIG_TD_FLASH_MAX_DATA_POINTER
#define TD_BSP_StackSize						CONFIG_STACK_SIZE
#define TD_SENSOR_TRANSMITTER_MaxTransmission	CONFIG_TD_SENSOR_TRANSMITTER_MAX_TRANSMIT
#define TD_SENSOR_TRANSMITTER_MaxRetransmission	CONFIG_TD_SENSOR_TRANSMITTER_MAX_RETRANSMIT
#define TD_SENSOR_GATEWAY_MaxDevice				CONFIG_TD_SENSOR_GATEWAY_MAX_DEVICE
#define TD_SENSOR_MaxSwitch						CONFIG_TD_SENSOR_MAX_SWITCH
#define TD_SENSOR_MaxSwitchEvent				CONFIG_TD_SENSOR_MAX_SWITCH_EVENT
#define TD_SCHEDULER_MaxTimer					CONFIG_TD_SCHEDULER_MAX_TIMER
#define TD_SCHEDULER_MaxQueue					CONFIG_TD_SCHEDULER_MAX_QUEUE
#define LanPeriod								CONFIG_LAN_PERIOD
#define LanThreshold							CONFIG_LAN_THRESHOLD
#define TD_BSP_GPS_CS_Port						CONFIG_GPS_CS_PORT
#define TD_BSP_GPS_CS_Bit						CONFIG_GPS_CS_BIT
#define TD_BSP_GPS_IRQ_Port						CONFIG_GPS_IRQ_PORT
#define TD_BSP_GPS_IRQ_Bit						CONFIG_GPS_IRQ_BIT
#define TD_BSP_GPS_RESET_Port					CONFIG_GPS_RESET_PORT
#define TD_BSP_GPS_RESET_Bit					CONFIG_GPS_RESET_BIT
#define TD_BSP_GPS_VBCKP_Port					CONFIG_GPS_VBCKP_PORT
#define TD_BSP_GPS_VBCKP_Bit					CONFIG_GPS_VBCKP_BIT
#define TD_BSP_GPS_VIO_Port						CONFIG_GPS_VIO_PORT
#define TD_BSP_GPS_VIO_Bit						CONFIG_GPS_VIO_BIT
#define TD_BSP_PowerCrystalPort					CONFIG_POWER_CRYSTAL_PORT
#define TD_BSP_PowerCrystalBit					CONFIG_POWER_CRYSTAL_BIT
#define TD_BSP_ShutdownPort						CONFIG_SHTD_PORT
#define TD_BSP_ShutdownBit						CONFIG_SHTD_BIT
#define TD_BSP_RadioInfoPin						CONFIG_RADIO_INFO_PIN
#define TD_BSP_ForceRadioReset					CONFIG_FORCE_RADIO_RESET
#define TD_BSP_RadioPAPower						CONFIG_RADIO_PA_POWER
#define RadioInitData							CONFIG_RADIO_INIT_DATA
#endif

#define AT_manufacturer							CONFIG_MANUFACTURER
#define AT_hardwareRevision						CONFIG_HARDWARE_VERSION
#define AT_softwareRevision						CONFIG_SOFTWARE_VERSION
#define AT_releaseDate							CONFIG_RELEASE_DATE
#define AT_serial								CONFIG_SERIAL_NUMBER

/***************************
 *  Application definition
 ****************************/
extern char const *CONFIG_MANUFACTURER;
extern char const *CONFIG_HARDWARE_VERSION;
extern char const *CONFIG_SOFTWARE_VERSION;
extern char const *CONFIG_RELEASE_DATE;
extern char *CONFIG_SERIAL_NUMBER;

/***************************
 *  Customer Board definition
 ****************************/
extern char const CONFIG_PRODUCT_TYPE;
extern char const CONFIG_PRODUCT_LED_POLARITY;
extern GPIO_Port_TypeDef const CONFIG_PRODUCT_LED_PORT;
extern char const CONFIG_PRODUCT_LED_BIT;
extern char const CONFIG_PRODUCT_LED_BLINK;
extern unsigned char const CONFIG_PRODUCT_BOOTLOADER_CHANNEL;
extern unsigned char const CONFIG_PRODUCT_BOOTLOADER_SKIP;
extern uint8_t const CONFIG_PRODUCT_UART_BOOTLOADER_SKIP;
extern unsigned short const CONFIG_PRODUCT_INIT_DATA[];
extern unsigned char const CONFIG_PRODUCT_INIT_DATA_SIZE;
extern uint8_t const ProductBootloaderP1[];
extern uint32_t const CONFIG_LEUART_LOCATION;
extern GPIO_Port_TypeDef const CONFIG_LEUART_TX_PORT;
extern uint8_t	const CONFIG_LEUART_TX_BIT;
extern GPIO_Port_TypeDef const CONFIG_LEUART_RX_PORT;
extern uint8_t	const CONFIG_LEUART_RX_BIT;
extern uint8_t const CONFIG_TD_UART_COUNT;
extern uint8_t CONFIG_TD_UI_Id[];

/***************************
 *  SYSTEM and LIBRARY definition
 ****************************/
extern unsigned char const CONFIG_MAX_SPI_ID;
extern uint32_t const CONFIG_TD_FLASH_VARIABLES_VERSION;
extern uint8_t const CONFIG_TD_FLASH_MAX_DATA_POINTER;
extern uint8_t const CONFIG_TD_FLASH_USER_PAGE;
extern uint16_t const CONFIG_STACK_SIZE;
extern uint16_t const CONFIG_AT_PERSIST_SIZE;

/* TD_SENSOR limits */
extern uint8_t const CONFIG_TD_SENSOR_TRANSMITTER_MAX_TRANSMIT;
extern uint8_t const CONFIG_TD_SENSOR_TRANSMITTER_MAX_RETRANSMIT;
extern uint8_t const CONFIG_TD_SENSOR_DEVICE_QUEUE_MAX;
extern uint8_t const CONFIG_TD_SENSOR_GATEWAY_MAX_DEVICE;
extern uint8_t const CONFIG_TD_SENSOR_MAX_SWITCH;
extern uint8_t const CONFIG_TD_SENSOR_MAX_SWITCH_EVENT;
extern uint8_t const CONFIG_TD_SCHEDULER_MAX_TIMER;
extern uint8_t const CONFIG_TD_SCHEDULER_MAX_QUEUE;
extern bool const CONFIG_TD_SCHEDULER_DONT_OVF_QUEUE;
extern unsigned char CONFIG_LAN_THRESHOLD;
extern unsigned long const CONFIG_LAN_PERIOD;
extern unsigned char const CONFIG_LAN_ADDRESS_SIZE;

/* TD_GEOLOC limits */
extern bool const CONFIG_TD_GEOLOC_RAW_OUTPUT;
/***************************
 *  TD12xxx Module definition
 ****************************/
/* GPS Chip ports definitions on EFM32 */
extern TD_GPIO_Port_TypeDef const CONFIG_GPS_CS_PORT;
extern uint8_t const CONFIG_GPS_CS_BIT;
extern GPIO_Port_TypeDef const CONFIG_GPS_IRQ_PORT;
extern uint8_t const CONFIG_GPS_IRQ_BIT;
extern TD_GPIO_Port_TypeDef const CONFIG_GPS_RESET_PORT;
extern uint8_t const CONFIG_GPS_RESET_BIT;
extern TD_GPIO_Port_TypeDef const CONFIG_GPS_VBCKP_PORT;
extern uint8_t const CONFIG_GPS_VBCKP_BIT;
extern TD_GPIO_Port_TypeDef const CONFIG_GPS_VIO_PORT;
extern uint8_t const CONFIG_GPS_VIO_BIT;

/* ACCELERO Chip ports definitions on EFM32 */
extern TD_GPIO_Port_TypeDef const CONFIG_ACCELERO_CS_PORT;
extern uint8_t const CONFIG_ACCELERO_CS_BIT;
extern GPIO_Port_TypeDef const CONFIG_ACCELERO_IRQ_PORT;
extern uint8_t const CONFIG_ACCELERO_IRQ_BIT;
extern uint16_t const CONFIG_ACCELERO_SPI_MODE;
extern uint8_t const CONFIG_ACCELERO_SPI_BUS;

/* MAGNETO Chip ports definitions on EFM32 */
extern TD_GPIO_Port_TypeDef const CONFIG_MAGNETO_CS_PORT;
extern uint8_t const CONFIG_MAGNETO_CS_BIT;
extern uint16_t const CONFIG_MAGNETO_SPI_MODE;
extern uint8_t const CONFIG_MAGNETO_SPI_BUS;

/* PRESSURE Chip ports definitions on EFM32 */
extern TD_GPIO_Port_TypeDef const CONFIG_PRESSURE_CS_PORT;
extern uint8_t const CONFIG_PRESSURE_CS_BIT;
extern uint16_t const CONFIG_PRESSURE_SPI_MODE;
extern uint8_t const CONFIG_PRESSURE_SPI_BUS;

/* RF chip port definitions on EFM32 CPU side */
extern TD_GPIO_Port_TypeDef const CONFIG_POWER_CRYSTAL_PORT;
extern uint8_t const CONFIG_POWER_CRYSTAL_BIT;
extern TD_GPIO_Port_TypeDef const CONFIG_SHTD_PORT;
extern uint8_t const CONFIG_SHTD_BIT;

/* RF chip port definitions on RF Chip side */
extern uint8_t const CONFIG_RADIO_INFO_PIN;

/* RF chip ports behavior */
extern uint8_t const CONFIG_FORCE_RADIO_RESET;
extern uint8_t const CONFIG_RADIO_PA_POWER;
extern unsigned char const CONFIG_RADIO_INIT_DATA[];
extern uint8_t const CONFIG_RADIO_USE_TXCO;
extern GPIO_Port_TypeDef const CONFIG_RADIO_INFO_PORT;
extern uint8_t const CONFIG_RADIO_INFO_BIT;
extern TD_GPIO_Port_TypeDef const CONFIG_RADIO_POWER_PORT;
extern uint8_t const CONFIG_RADIO_POWER_BIT;

/* RF Proxy class config */
extern uint8_t const CONFIG_TD_SIGFOX_PROXY_CLASS;

/** I/O mode for RF chip GPIO pin */
typedef enum {
	TD_RF_GPIO_DISABLED = 0,
	TD_RF_GPIO_INPUT,
	TD_RF_GPIO_OUTPUT,
	TD_RF_GPIO_CUSTOM,
	TD_RF_GPIO_READ,
} TD_RF_gpio_mode_t;

bool TD_RF_GPIO_PinConfigure(uint8_t pin, TD_RF_gpio_mode_t mode,
	uint8_t *value);
#define GPIO_NOP						0
#define GPIO_TRISTATE					1
#define GPIO_RX_RAW_DATA_OUT			21
#define GPIO_DRIVE0						2
#define GPIO_DRIVE1						3

typedef void (*TD_MAIN_Init_t)(void);
extern TD_MAIN_Init_t const CONFIG_EARLY_DEBUG;
extern bool const CONFIG_PRODUCT_EARLY_DEBUG;
void main_init();
void main_init_debug();

/* RF constant power calculation */
typedef uint8_t (*RFPowerCalculation_t)(uint8_t level, uint32_t voltage);
extern RFPowerCalculation_t const CONFIG_TD_RF_POWER_CALCULATION;
extern int8_t const CONFIG_TD_RF_POWER_CALCULATION_COEF[];

/* Si4461 PA Mode */
extern uint8_t const CONFIG_TD_RF_SI4461_CLASS_E;

#include <td_trap.h>

typedef void (*TD_Dump_Func_t)(void);

/* Dynamic dump function */
extern uint8_t const TD_TRAP_MaxSystemDump;
extern TD_Dump_Func_t const TD_TRAP_SystemDumpFunc[];

#endif // __TD_CONFIG_EXT_H
/** @endcond */
