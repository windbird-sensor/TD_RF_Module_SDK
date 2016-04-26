/** @cond TD_CONFIG */
/***************************************************************************//**
 * @file
 * @brief Configuration file for the TDxxxx RF modules.
 * @author Telecom Design S.A.
 * @version 1.5.0
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2013-2016 Telecom Design S.A., http://www.telecomdesign.fr</b>
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

/** Macro to declare a link-time binding for a dynamic function */
#define DECLARE_DYNAMIC_LIGHT(function) \
	typedef void (*function##_t) (void);

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

#define SIZEOF(x) ((char *)(&(x) + 1) - (char *) &(x))

#ifndef NULL
#define NULL ((void *) 0)
#endif

#define IS_EMPTY(x)		1 - x - 1 == 2

/* Product init data macros and define
 * This small command subsystem is used for initial pin setup during bootloading
 * stage
 */

/* Product init data global command */
#define PRODUCT_DATA_INIT_LOW		0
#define PRODUCT_DATA_INIT_HIGH		1
#define PRODUCT_DATA_INIT_STRENGTH	2
#define PRODUCT_DATA_INIT_ROUTE		3
#define PRODUCT_DATA_INIT_PRS_ASYNC	4

/* Product init data / IO setup mode */
#define PI_DISABLED		0
#define PI_INPUT		1
#define PI_INPUT_PULL	2
#define PI_OUTPUT		4

/* Product init data route options / IO route (debug) mode */
#define PRODUCT_DATA_INIT_ROUTE_DISABLE		0

/* Product init data / macros */
#define PIP(p,b,m,v) 	(((p) << 12) | ((b) << 8) | ((v) << 4) | (m))
#define PIS(p,s) 		(((p) << 12) | ((s) << 8 ) | \
	((PRODUCT_DATA_INIT_STRENGTH) << 4))
#define PIR(s) 			(((s) << 8 ) | ((PRODUCT_DATA_INIT_ROUTE) << 4))
#define PIA(c,src,sig)	(((src) >> 6) | ((sig & 0x6) << 7) | \
	((PRODUCT_DATA_INIT_PRS_ASYNC) << 4) | ((sig & 0x1) << 3) | (c))
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
extern int8_t const CONFIG_PRODUCT_TYPE;
extern int8_t const CONFIG_PRODUCT_LED_POLARITY;
extern GPIO_Port_TypeDef const CONFIG_PRODUCT_LED_PORT;
extern int8_t const CONFIG_PRODUCT_LED_BIT;
extern int8_t const CONFIG_PRODUCT_LED_BLINK;
extern GPIO_DriveMode_TypeDef const CONFIG_PRODUCT_LED_DRIVE;
extern uint8_t const CONFIG_PRODUCT_BOOTLOADER_CHANNEL;
extern uint16_t const CONFIG_PRODUCT_INIT_DATA[];
extern uint8_t const CONFIG_PRODUCT_INIT_DATA_SIZE;
extern uint8_t const ProductBootloaderP1[];
extern uint32_t const CONFIG_LEUART_LOCATION;
extern uint32_t const CONFIG_LEUART_SPEED;
extern void * const CONFIG_LEUART_DEVICE;
extern GPIO_Port_TypeDef const CONFIG_LEUART_TX_PORT;
extern uint8_t	const CONFIG_LEUART_TX_BIT;
extern GPIO_Port_TypeDef const CONFIG_LEUART_RX_PORT;
extern uint8_t	const CONFIG_LEUART_RX_BIT;
extern uint8_t const CONFIG_TD_UART_COUNT;
extern uint8_t const CONFIG_TD_STREAM_COUNT;
extern int const CONFIG_TD_STREAM_FIFOSIZE;
extern uint8_t CONFIG_TD_UI_Id[];

/* CAPSENSE application definitions on EFM32 */
extern ACMP_TypeDef * const CONFIG_ACMP_CAPSENSE;
extern uint8_t const CONFIG_ACMP_CAPSENSE_CHANNEL;
extern uint32_t const CONFIG_ACMP_CAPSENSE_CLKEN;
extern uint32_t const CONFIG_PRS_CH_CTRL_SOURCESEL_ACMP_CAPSENSE;
extern uint32_t const CONFIG_PRS_CH_CTRL_SIGSEL_ACMPOUT_CAMPSENSE;
extern int32_t const CONFIG_CAPSENSE_DELTA_CHANGE_THRESHOLD;
extern uint8_t const CONFIG_CAPSENSE_HISTORY_DEPTH ;
extern uint32_t * const TD_CAPSENSE_ValueHistory;

/***************************
 *  SYSTEM and LIBRARY definition
 ****************************/
extern unsigned char const CONFIG_MAX_SPI_ID;
extern uint32_t const CONFIG_TD_FLASH_VARIABLES_VERSION;
extern uint8_t const CONFIG_TD_FLASH_MAX_DATA_POINTER;
extern uint8_t const CONFIG_TD_FLASH_USER_PAGE;
extern uint16_t const CONFIG_STACK_SIZE;
extern uint16_t const CONFIG_AT_PERSIST_SIZE;
extern bool const CONFIG_AT_LEUART_PERSIST;
extern bool const CONFIG_AT_ARG_CASE_SENSITIVE;
extern uint8_t const CONFIG_EFM_ASSERT_NOT_TRAPPED;
extern uint32_t const CONFIG_FLASH_LAYOUT[];
extern char* const CONFIG_FLASH_NAME_LAYOUT[];
extern bool const CONFIG_ALLOW_LEUART_LOW_BAUDRATE_ON_HFRCO;
extern uint32_t const CONFIG_LIMIT_FLASH_SIZE;
extern bool const CONFIG_PROTECTED_LOADER_FLASH_SIZE;
extern bool const CONFIG_PREVENT_FREE_BACKGROUND_CYCLE;
extern const void * CONFIG_PRINTF_ULONG;
extern const void * CONFIG_PRINTF_LONG;
extern const uint8_t CONFIG_PRINTF_INT64_SUPPORT_ON;
extern bool const CONFIG_RDEBUG_FULL_PKT;
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
extern uint8_t const CONFIG_LAN_THRESHOLD;
extern uint8_t const CONFIG_LAN_THRESHOLD_RX;
extern uint8_t const CONFIG_LAN_LBT_COUNT_MAX;
extern uint32_t const CONFIG_LAN_PERIOD;
extern uint8_t const CONFIG_LAN_ADDRESS_SIZE;
extern uint32_t const CONFIG_LAN_CHECK_CALLBACK_TIME;

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
extern  uint8_t const CONFIG_GPS_SPI_BUS;
extern void * const CONFIG_GPS_INTERFACE;

/* ACCELERO Chip ports definitions on EFM32 */
extern TD_GPIO_Port_TypeDef const CONFIG_ACCELERO_POWER_PORT;
extern uint8_t const CONFIG_ACCELERO_POWER_BIT;
extern TD_GPIO_Port_TypeDef const CONFIG_ACCELERO_CS_PORT;
extern uint8_t const CONFIG_ACCELERO_CS_BIT;
extern GPIO_Port_TypeDef const CONFIG_ACCELERO_IRQ_PORT;
extern uint8_t const CONFIG_ACCELERO_IRQ_BIT;
extern uint16_t const CONFIG_ACCELERO_SPI_MODE;
extern uint8_t const CONFIG_ACCELERO_SPI_BUS;
extern uint32_t const CONFIG_ACCELERO_SPI_SPEED;

/* FLASH SPI Chip ports definitions on EFM32 */
extern TD_GPIO_Port_TypeDef const CONFIG_FLASH_POWER_PORT;
extern uint8_t const CONFIG_FLASH_POWER_BIT;
extern TD_GPIO_Port_TypeDef const CONFIG_FLASH_CS_PORT;
extern uint8_t const CONFIG_FLASH_CS_BIT;
extern GPIO_Port_TypeDef const CONFIG_FLASH_IRQ_PORT;
extern uint8_t const CONFIG_FLASH_IRQ_BIT;
extern uint16_t const CONFIG_FLASH_SPI_MODE;
extern uint8_t const CONFIG_FLASH_SPI_BUS;

/* MAGNETO Chip ports definitions on EFM32 */
extern TD_GPIO_Port_TypeDef const CONFIG_MAGNETO_POWER_PORT;
extern uint8_t const CONFIG_MAGNETO_POWER_BIT;
extern TD_GPIO_Port_TypeDef const CONFIG_MAGNETO_CS_PORT;
extern uint8_t const CONFIG_MAGNETO_CS_BIT;
extern TD_GPIO_Port_TypeDef const CONFIG_MAGNETO_IRQ_PORT;
extern uint8_t const CONFIG_MAGNETO_IRQ_BIT;
extern uint16_t const CONFIG_MAGNETO_SPI_MODE;
extern uint8_t const CONFIG_MAGNETO_SPI_BUS;

/* PRESSURE Chip ports definitions on EFM32 */
extern TD_GPIO_Port_TypeDef const CONFIG_PRESSURE_POWER_PORT;
extern uint8_t const CONFIG_PRESSURE_POWER_BIT;
extern TD_GPIO_Port_TypeDef const CONFIG_PRESSURE_CS_PORT;
extern uint8_t const CONFIG_PRESSURE_CS_BIT;
extern uint16_t const CONFIG_PRESSURE_SPI_MODE;
extern uint8_t const CONFIG_PRESSURE_SPI_BUS;
extern uint32_t const CONFIG_PRESSURE_SPI_SPEED;

/* RF chip port definitions on EFM32 CPU side */
extern TD_GPIO_Port_TypeDef const CONFIG_POWER_CRYSTAL_PORT;
extern uint8_t const CONFIG_POWER_CRYSTAL_BIT;
extern TD_GPIO_Port_TypeDef const CONFIG_SHTD_PORT;
extern uint8_t const CONFIG_SHTD_BIT;
extern GPIO_Port_TypeDef const CONFIG_RADIO_INFO_PORT;
extern uint8_t const CONFIG_RADIO_INFO_BIT;
extern TD_GPIO_Port_TypeDef const CONFIG_RADIO_POWER_PORT;
extern uint8_t const CONFIG_RADIO_POWER_BIT;
extern GPIO_Port_TypeDef const CONFIG_RADIO_CS_PORT;
extern uint8_t const CONFIG_RADIO_CS_BIT;
extern GPIO_Port_TypeDef const CONFIG_RADIO_IRQ_PORT;
extern uint8_t const CONFIG_RADIO_IRQ_BIT;
extern TD_GPIO_Port_TypeDef const CONFIG_RADIO_INT_PA_TX_PORT;
extern uint8_t const CONFIG_RADIO_INT_PA_TX_BIT;
extern TD_GPIO_Port_TypeDef const CONFIG_RADIO_PA_TX_PORT;
extern uint8_t const CONFIG_RADIO_PA_TX_BIT;
extern TD_GPIO_Port_TypeDef const CONFIG_RADIO_PA_RX_LNA_PORT;
extern uint8_t const CONFIG_RADIO_PA_RX_LNA_BIT;
extern TD_GPIO_Port_TypeDef const CONFIG_RADIO_PA_ON_PORT;
extern uint8_t const CONFIG_RADIO_PA_ON_BIT;

/* RF chip port definitions on RF Chip side */
extern uint8_t const CONFIG_RADIO_INFO_PIN;
extern uint8_t const CONFIG_RADIO_INT_PA_TX_PIN;

/* RF chip ports behavior */
extern uint8_t const CONFIG_FORCE_RADIO_RESET;
extern uint8_t const CONFIG_RADIO_PA_POWER;
extern uint8_t const CONFIG_RADIO_PA_MAX;
extern uint8_t const CONFIG_RADIO_XO_TUNE;
extern unsigned char const CONFIG_RADIO_INIT_DATA[];
extern uint8_t const CONFIG_RADIO_USE_TCXO;
extern uint8_t const CONFIG_RADIO_BOOT_RF_SHUTDOWN;
extern TIMER_TypeDef * const CONFIG_RADIO_POWER_TIMER;
extern uint8_t const CONFIG_RADIO_POWER_TIMER_CC;
extern uint8_t const *CONFIG_RADIO_PATCH_BLOB;
extern int const CONFIG_RADIO_PATCH_SIZE;

extern bool const CONFIG_USE_FREQUENCY_CAL;
extern bool const CONFIG_ACCELERO_SKIP_INIT;
extern bool const CONFIG_ACCELERO_POLLING_MODE;
extern bool const CONFIG_CHIP_EZR;
extern uint8_t const CONFIG_ITU_ISM_REGION;
extern uint32_t const CONFIG_SIGFOX_FCC_MACRO_CHANNEL_BITMASK_MSB;
extern uint32_t const CONFIG_SIGFOX_FCC_MACRO_CHANNEL_BITMASK_MID;
extern uint32_t const CONFIG_SIGFOX_FCC_MACRO_CHANNEL_BITMASK_LSB;
extern uint16_t const CONFIG_SIGFOX_FCC_DEFAULT_MACRO_CHANNEL;

/* RF Proxy class config */
extern uint8_t const CONFIG_TD_SIGFOX_PROXY_CLASS;

extern uint32_t const CONFIG_LEUART_LOCATION;

/* BLE config */
extern TD_GPIO_Port_TypeDef const CONFIG_BLE_POWER_PORT;
extern uint8_t const CONFIG_BLE_POWER_BIT;
extern TD_GPIO_Port_TypeDef const CONFIG_BLE_RESET_PORT;
extern uint8_t const CONFIG_BLE_RESET_BIT;
extern uint8_t const CONFIG_BLE_UART_CONF;
extern uint32_t const CONFIG_BLE_UART_SPEED;
extern uint32_t const CONFIG_BLE_UART_PRELOAD_SPEED;
extern void const *CONFIG_BLE_UART;


// Gpio config values
#define GPIO_NOP						0
#define GPIO_TRISTATE					1
#define GPIO_TX_DATA_CLK_OUT			16
#define GPIO_RX_RAW_DATA_OUT			21
#define GPIO_IN_TX_STATE				32
#define GPIO_IN_RX_STATE				33
#define GPIO_DRIVE0						2
#define GPIO_DRIVE1						3

typedef bool (*TD_SIGFOX_SendV1_t)(uint8_t mode,  bool value, uint8_t *message,
	uint8_t size, uint8_t retry, bool ack, bool reserved);
extern TD_SIGFOX_SendV1_t const CONFIG_TD_SIGFOX_SEND_V1;
typedef void (*TD_MAIN_Init_t)(void);
extern TD_MAIN_Init_t const CONFIG_EARLY_DEBUG;
extern TD_MAIN_Init_t const CONFIG_EARLY_DEBUG_CUSTOM;
extern const uint8_t CONFIG_TD_UI_COUNT;
extern bool const CONFIG_PRODUCT_EARLY_DEBUG;
void main_init();
void main_init_debug();

/* RF constant power calculation */
typedef uint8_t (*RFPowerCalculation_t)(int level, uint32_t voltage);
extern RFPowerCalculation_t const CONFIG_TD_RF_POWER_CALCULATION;
extern int8_t const CONFIG_TD_RF_POWER_CALCULATION_COEF[];

/* Si4461 PA Mode */
extern uint8_t const CONFIG_TD_RF_SI4461_CLASS_E;

/* Custom PA Mode */
extern uint8_t const CONFIG_TD_RF_PA_CUSTOM;

#include <td_trap.h>

typedef void (*TD_Dump_Func_t)(void);

/* Dynamic dump function */
extern uint8_t const TD_TRAP_MaxSystemDump;
extern TD_Dump_Func_t const TD_TRAP_SystemDumpFunc[];

#endif // __TD_CONFIG_EXT_H
/** @endcond */
