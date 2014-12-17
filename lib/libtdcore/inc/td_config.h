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
#ifndef __TD_CONFIG_H
#define __TD_CONFIG_H

#include <stdint.h>

/* Note : this file td_config.h is always included in USER code, so .h file are
 * library include '<xxx.h>' */
#include <td_config_ext.h>
#include <td_trap.h>
#include <td_rtc.h>
#include <td_scheduler.h>
#include <td_spi.h>
#include <td_gpio.h>
#include <td_utils.h>
#include <td_flash.h>
#include <td_boot.h>

#ifdef __cplusplus
extern "C" {
#endif

  /***********************************************************************//**
   * @addtogroup DYNAMIC_OPTIONS Dynamic Options
   * @brief Link-time dynamic options for binary inter-modules/libraries
   * parameters.
   * @{
   * @details
   *
   *   # Introduction
   *
   *   The Telecom Design RF SDK API is organized into subsystems, providing a
   *   consistent interface to both hardware and software components.
   *
   *   Related subsystems are grouped into different static libraries , each
   *   library
   *   taking care of a given layer in the overall Telecom Design RF SDK API.
   *
   *   However, there is often the need for an API user to provide build-time
   *   options to these libraries, and from within a library, to be able to
   *   access the build-time chosen option values.
   *
   *   Unfortunately, it is not possible to use standard preprocessor
   *   definitions, as some of the libraries (such as the libtdrf) or just
   *   some of the modules within a library (such as the UBLOX module within
   *   the libtgeoloc library) are only provided in binary object format,
   *   mainly for Intellectual Property and/or patent issues.
   *
   *   But despite the fact that you do not have the sources for these
   *   objects, the DYNAMIC_OPTION module provides a simple mechanism allowing
   *   you to:
   *
   *     - define user option values once at link-time
   *     - reference these user option values from anywhere within the
   *       firmware at run-time, including within the binary-only static
   *       libraries
   *
   *   # Usage
   *
   *   In your project main source file (where TD_USER_Setup() is located)
   *   <b>and only here</b>, you must:
   *   - optionally overload the definitions of library parameters (see
   *   <i>Parameters</i> below)
   *   - include <td_config.h> at the end of your include list.
   *
   *   # Example
   * @code
   *   #include "config.h"
   *   #include <efm32.h>
   *   #include <td_core.h>
   *   #include <td_rtc.h>
   *   [...]
   *
   *   // Define 'manufacturer' for AT command
   *   #define MANUFACTURER "Telecom Design"
   *
   *   // Define 'product name' for AT command
   *   #define PRODUCT "Sample include"
   *
   *   // We need 14 timer for our application
   *   #define TD_SCHEDULER_MAX_TIMER 14
   *
   *   // But we will never have more than 5 timers ready at a time
   *   #define TD_SCHEDULER_MAX_QUEUE 5
   *
   *   #include <td_config.h>
   *
   * @endcode
   *
   * @addtogroup EVB_MODULES Standard EVB Modules
   * @brief Standard EVB Module definitions
   * @{
   * @details
   *  # Standard EVB Modules
   *
   *   When you are using standard EVB TD MODULE, only one define is mandatory
   *
   * @code
   *	#define MODULE_REVISION REVISION_TD1202
   *	#define MODULE_REVISION REVISION_TD1204
   *	#define MODULE_REVISION REVISION_TD1208
   * @endcode
   *
   *  It will define all needed parameters. All of theses can be overloaded by
   *  custom defines if needed.
   *
   * @} (end addtogroup EVB_MODULES)
   *
   * @addtogroup CUSTOM_BOARD Custom Board
   * @brief Custom Board with standard TD Chips
   * @{
   * @details
   *	# Custom Board with standard TD Chips
   *
   *	If your are using a custom board with a standard TD12xx chip, you can
   *	used these #defineto automatically setup configuration for your chip,
   *	and only add configuration needed for your board
   *	(PRODUCT_INIT_DATA,PRODUCT_LED_PORT, ...)
   *
   * @code
   *	#define CHIP_TD1202
   *	#define CHIP_TD1204
   *	#define CHIP_TD1208
   * @endcode
   * @} (end addtogroup CUSTOM_BOARD)
   *
   * @addtogroup PARAMETERS Dynamic Parameters
   * @brief Dynamic parameters that can be overloaded for libraries
   * @{
   * @details
   *
   *   # Parameters
   *
   *   You can define each of these parameters before including <td_config.h>
   *
   *   Level can be:
   *   - Chip       : these parameters control internal Chip configuration.
   *   				Don't modify them if you use standard TDxxx chip/module
   *   				products
   *   - Board      : these parameters control Board configuration. Don't
   *   				modify them if you use standard TD EVB board. If you use
   *   				your own custom board, you must customize them
   *   - Applicative: these parameters are merely software related ones (AT
   *   				Configuration) and can be modified to fit your needs
   *
   *   Anywhere in your application all of theses parameters are accessible
   *   (read only) via constant CONFIG_xxxx
   *
   *   If you want to force a parameter in your main source file:
   *
   * @code
   *   #define TD_SCHEDULER_MAX_TIMER 12
   *   #include <td_config.h>
   * @endcode
   *
   *   On all other sources files, all parameters are available :
   *   tfp_printf("Max timer : %d, MaxQueue : %d\r\n",CONFIG_TD_SCHEDULER_MAX_TIMER,CONFIGTD_SCHEDULER_MAX_QUEUE);
   *
   *    Define                                 | Level        | Type      |      Usage     | Description
   *    :---------------------------------------|:------------:|:---------:|:--------------:|:--------------------------------------------------------------------
   *    MANUFACTURER                            | Applicative  | string    |  AT commands   | Manufacturer name
   *    PRODUCT                                 | Applicative  | string    |  AT commands   | Product name
   *    HARDWARE_VERSION                        | Applicative  | string    |  AT commands   | Hardware version
   *    SOFTWARE_VERSION                        | Applicative  | string    |  AT commands   | Software version
   *    RELEASE_DATE                            | Applicative  | string    |  AT commands   | Release date
   *    SERIAL_NUMBER                           | Applicative  | string    |  AT commands   | Serial number
   *    PRODUCT_LED_BLINK		 	              | Applicative  | boolean   |  Bootloader    | Set to true, blink led during flash of each sector (default : false)
   *    PRODUCT_BOOTLOADER_CHANNEL	 	      | Applicative  | integer   |  Bootloader    | Bootloader channel (def : 255), other not actually handled by TD_Loader
   *    PRODUCT_BOOTLOADER_SKIP  	 	          | Applicative  | integer   |  Bootloader    | Set to true, skip bootloader
   *    PRODUCT_UART_BOOTLOADER_SKIP  	 	  | Applicative  | integer   |  Bootloader    | Set to true, skip uart bootloader
   *    PRODUCT_EARLY_DEBUG  	 	          	  | Applicative  | integer   |  Startup       | Set to true, activate early debug (in main @115200,8N1)
   *    TD_SCHEDULER_MAX_TIMER                  | Applicative  | integer   |  Scheduler     | Total number of Scheduler Timers
   *    TD_SCHEDULER_MAX_QUEUE                  | Applicative  | integer   |  Scheduler     | Total number of timer call-back (not IRQ) in pending queue
   *    TD_SENSOR_TRANSMITTER_MAX_TRANSMIT      | Applicative  | integer   |  Sensor        | Max pending SIGFOX message transmission count
   *    TD_SENSOR_TRANSMITTER_MAX_RETRANSMIT    | Applicative  | integer   |  Sensor        | Max pending SIGFOX message retransmission count
   *    TD_SENSOR_GATEWAY_MAX_DEVICE            | Applicative  | integer   |  Sensor        | Max attached devices to Gateway (Gateway itself count for 1 device)
   *    TD_SENSOR_MAX_SWITCH                    | Applicative  | integer   |  Sensor        | Max number of handled switches
   *    TD_SENSOR_MAX_SWITCH_EVENT			  | Applicative  | integer   |  Sensor        | Max number of switches event queue (for non IRQ processing)
   *    TD_FLASH_MAX_DATA_POINTER               | Applicative  | integer   |  Flash Variable| Max number of flash variable handled
   *    TD_FLASH_USER_PAGE					  | Applicative  | integer   |  Flash Variable| Defines how many pages should be used for storing flash variables. Use must make sure not to overwrite code.
   *    TD_UART_COUNT                  		  | Applicative  | integer   |  Uart          | UART used. Experimental only ! use of USART1 need always 2 UART
   *    AT_PERSIST_SIZE                         | Applicative  | integer   |  Flash Variable| Max persist size for AT commands
   *    STACK_SIZE                              | Applicative  | integer   |  system wide   | System stack size in bytes
   *	TD_RF_POWER_CALCULATION                 | Applicative  | function  |  RF			  | Callback to defined to overload PA indice calculation. Don't use if you don't know what you're doing!
   *    TD_RF_POWER_CALCULATION_COEF            | Applicative  | s integer |  RF			  | Voltage to PA indice table. To be defined according to measurement in order to ensure constant dB output.
   *    TD_STACK_PROTECT                        | Applicative  | boolean   |  system wide   | Enable stack protection at startup
   *    TD_RF_SI4461_CLASS_E 					  | Applicative  | boolean   |  RF			  | Define to 1 to use square wave mode (class E - 16dB) on si4461
   *    TD_GEOLOC_RAW_OUTPUT					  | Applicative  | boolean   |  GPS			  | Define to true to obtain raw gps chip outpu value. Can be combine to Ubx7Parse function to control gps via u-center.
   *    MODULE_REVISION                         | Board        | symbol    |  system wide   | REVISION_TD1202, REVISION_TD1204, REVISION_TD1208 (often defined in project defines)
   *    PRODUCT_LED_POLARITY		              | Board        | integer   |  Bootloader    | 0:led lit when TIM2 set to VDD, 1:led lit when TIM2 set to GND
   *    PRODUCT_LED_PORT		 	              | Board        | gpioPortx |  Bootloader    | Set port of Led (default : TIM2_PORT), 0xFF : no led
   *    PRODUCT_LED_BIT		 	              | Board        | integer   |  Bootloader    | Set bit of Led (default : TIM2_BIT)
   *    PRODUCT_INIT_DATA 	 	              | Board        | array     |  Bootloader    | Others port to initialize (see Init Data)
   *    MAX_USER_SPI_ID						  | Board        | integer   |  Spi           | Max number of SPI users
   *    LEUART_LOCATION						  | Board        | const     |  LEUart        | Set LEUART pin location (LEUART_ROUTE_LOCATION_LOCx)
   *    PRODUCT_TYPE		                      | Chip         | integer   |  Bootloader    | 0:TD1202, 8:TD1204, 9:TD1205, 10:TD1208 (automatically set if not set and MODULE_REVISION defined)
   *    DUMP_PRODUCT_CONFIG                     | Applicative  | integer   |  CompileTime   | If set, do a dump with #info of system config used
   *
   *   These parameters are RF SYSTEM parameters do not MESS with them!
   *
   *   Define                                 | Level        | Type      |      Usage          | Description
   *   :---------------------------------------|:------------:|:---------:|:-------------------:|:------------------------------------------------------------------------------------
   *   TD_LOADER_TRANSMITTER				     | Applicative  | define    |  Bootloader         | DEPRECATED : contact TD if needed
   *   POWER_CRYSTAL_PORT		 	             | Chip         | gpioPortx |  Bootloader         | Set port of RF TCXO
   *   POWER_CRYSTAL_BIT		                 | Chip         | integer   |  Bootloader         | Set bit of RF TCXO
   *   SHTD_PORT		 	                     | Chip         | gpioPortx |  Bootloader         | Set port of RF Shutdown
   *   SHTD_BIT		               			 | Chip         | integer   |  Bootloader         | Set bit of RF Shutdown
   *   RADIO_INIT_DATA					     | Chip         | array     |  Bootloader         | Initial configuration of RF PIN
   *   RADIO_INFO_PIN					         | Chip         | integer   |  td_rf              | RF PIN number connected to EFM32, used for LAN CCA.
   *   RADIO_INFO_PORT						 | Chip         | integer   |  td_rf              | EFM32 Port number connected to Silabs, used for LAN RSSI.
   *   RADIO_INFO_BIT 						 | Chip         | integer   |  td_rf              | EFM32 Bit number connected to Silabs, used for LAN RSSI.
   *   RADIO_POWER_PORT						 | Chip         | integer   |  td_rf              | External Radio Power handling
   *   RADIO_POWER_BIT 						 | Chip         | integer   |  td_rf              | External Radio Power handling
   *   FORCE_RADIO_RESET					     | Chip         | integer   |  td_rf              | Force RF reset at each reset
   *   RADIO_USE_TCXO 						 | Chip			| boolean   |  td_rf			  | Use quartz instead of tcxo
   *   RADIO_PA_POWER					         | Applicative  | integer   |  td_sigfox          | Default RF power (dBm)
   *   LAN_PERIOD						         | Applicative  | integer   |  td_lan             | Receive sampling period (timer unit)
   *   LAN_THRESHOLD						     | Applicative  | integer   |  td_lan  			  | RSSI Threshold in LAN reception mode in 0.5dB step from -126dB max sensitivity (default : 32 aka -110dB)
   *   LAN_ADDRESS_SIZE					     | Applicative  | integer   |  td_lan  			  | LAN address size (in bits default 24)
   *
   *   These parameters are GPS SYSTEM parameters do not MESS with them!
   *
   *   Define                                 | Level        | Type      |      Usage     | Description
   *   :---------------------------------------|:------------:|:---------:|:--------------:|-------------------------------
   *   GPS_CS_PORT		 	                 | Chip         | gpioPortx |  Bootloader    | set port of GPS CS
   *   GPS_CS_BIT		                         | Chip         | number    |  Bootloader    | set bit of GPS CS
   *   GPS_IRQ_PORT		 	                 | Chip         | gpioPortx |  Bootloader    | set port of GPS IRQ
   *   GPS_IRQ_BIT		                     | Chip         | number    |  Bootloader    | set bit of GPS IRQ
   *   GPS_RESET_PORT		 	                 | Chip         | gpioPortx |  Bootloader    | set port of GPS RESET
   *   GPS_RESET_BIT		                     | Chip         | number    |  Bootloader    | set bit of GPS RESET
   *   GPS_VBCKP_PORT		 	                 | Chip         | gpioPortx |  Bootloader    | set port of GPS VBCKP
   *   GPS_VBCKP_BIT		                     | Chip         | number    |  Bootloader    | set bit of GPS VBCKP
   *   GPS_VIO_PORT		 	                 | Chip         | gpioPortx |  Bootloader    | set port of GPS VIO
   *   GPS_VIO_BIT		                     | Chip         | number    |  Bootloader    | set bit of GPS VIO
   *
   *   These parameters are for General code use. It will force to include
   *   (or remove) blocks of code.
   *   In general use case theses parameters are not useful, they are just here
   *   for special use case
   *
   *   Define                                 | Description
   *   :---------------------------------------|-------------------------------
   *   TD_GEOLOC_USE_CODE		 	 			 |Force use/not use the libtdgeoloc dynamic data
   *   TD_SENSOR_USE_CODE		 	 			 |Force use/not use the libtdsensor dynamic data
   *   TD_SIGFOX_USE_TEST_CARRIER_CODE      	 |Add SIGFOX test carrier code
   *   TD_SIGFOX_TRANSMIT_DEFAULT              |Default Sigfox transmit mode (can be ommited), use booster if found/paired, local transmit if no booster paired
   *   TD_SIGFOX_TRANSMIT_FORWARD_ONLY         |Use forward mode only. Local Sigfox ID module will never be used, only booster ID will be used
   *   TD_SIGFOX_TRANSMIT_LOCAL_ONLY           |Prevent use of any proxy. Use only local ID
   *   TD_SIGFOX_TRANSMIT_NONE                 |Don't use Sigfox transmit code at all (remove all transmit & receive code)
   *   TD_SIGFOX_RECEIVE_NONE					 |Don't use Sigfox receive code at all (remove all receive code)
   *   TD_SIGFOX_USE_RECEIVE_TEST_CODE		 |Use Sigfox receive test code
   *
   *   These parameters are Code Size reduction parameters. If you remove code
   *   and use it after that, it will throw a Trap
   *
   *   Define                                 | Description
   *   :---------------------------------------|-------------------------------
   *   TD_SENSOR_GATEWAY_REMOVE_CODE		 	 |Remove Sensor Gateway code
   *   TD_SENSOR_TRANSMITTER_REMOVE_CODE	 	 |Remove Sensor Transmitter code
   *   TD_SENSOR_TRANSMITTER_DUTY_CYCLE_CODE 	 |Remove Sensor Transmitter duty cycle monitor code
   *   TD_SENSOR_DEVICE_REMOVE_CODE        	 |Remove Sensor Device code
   *   TD_SENSOR_LAN_REMOVE_CODE          	 |Remove Lan code
   *   TD_SENSOR_MONITOR_REMOVE_CODE		 	 |Remove Sensor Switch Monitor code (include EVENT and SWITCH)
   *   TD_SENSOR_MONITOR_EVENT_REMOVE_CODE	 |Remove Sensor Switch Event Monitor code
   *   TD_SENSOR_MONITOR_SWITCH_REMOVE_CODE	 |Remove Sensor Switch Event Switch code
   *   TD_SENSOR_MONITOR_CONFIG				 |Get a list of 'ored' value in (TD_SENSOR_MONITOR_BOOT,TD_SENSOR_MONITOR_BATT,TD_SENSOR_MONITOR_TEMP,TD_SENSOR_MONITOR_CONNECT,TD_SENSOR_MONITOR_KEEPALIVE,TD_SENSOR_MONITOR_SWITCH) of monitor to activate. To remove all, use TD_SENSOR_MONITOR_REMOVE_CODE
   *   TD_GEOLOC_LOGGER_REMOVE_CODE		     |Remove Geolocation Flash logger code
   *   TD_SIGFOX_REMOVE_CODE		 	         |[Deprecated, use SIGFOX_TRANSMIT_xxx] Remove all SIGFOX code
   *   TD_SIGFOX_PROXY_REMOVE_CODE			 |[Deprecated, use SIGFOX_TRANSMIT_xxx] Remove Proxy handling in Sigfox transmission
   *   TD_SIGFOX_PROXY_USE_FORWARD			 |[Deprecated, use SIGFOX_TRANSMIT_xxx] Proxy use forward (proxy sid) instead of relay (module sid).
   *   TD_TRAP_RESET_CODE		                 |Reset only trap
   *   TD_TRAP_MINI_CODE		                 |Remove standard printf trap handler, replace with minimal trap
   *   TD_TRAP_PRINTF_CODE		 	         |Standard printf trap handler
   *   TD_TRAP_FLASH_CODE		                 |Remove standard printf trap handler, replace with flash trap
   *   TD_TRAP_FLASH_PRINTF_CODE		 	     |Add flash handler with standard printf handler.
   *   TD_ALL_DUMP_REMOVE_CODE		 	     |Remove all xxx_DUMP code (same as enable one by one)
   *   TD_GPIO_DUMP_REMOVE_CODE		 	     |Remove TD_GPIO_Dump() code
   *   TD_RF_DUMP_REMOVE_CODE		 	 	     |Remove TD_RF_Dump() code
   *   TD_IRQ_DUMP_REMOVE_CODE		 	 	 |Remove TD_GPIO_Dump() code
   *   TD_SCHEDULER_DUMP_REMOVE_CODE		 	 |Remove TD_SCHEDULER_Dump() code
   *   TD_UBX7_DUMP_REMOVE_CODE		 	 	 |Remove TD_UBX7_Dump() code
   *   TD_SPILOCK_DUMP_REMOVE_CODE		 	 |Remove TD_SPILock_Dump() code
   *   TD_SCHEDULER_REMOVE_CODE		 	 	 |Remove TD_SCHEDULER code
   *   TD_REMOVE_UART_LOADER                   |Remove UART Loader (on LEUART, location 0)
   *
   *   Note : for flash trap handler, remember to call TD_TRAP_DirectToFlash
   *   function in your code don't forget to remove TD_TRAP_DirectToFlash to
   *   reclaim space if you not use it
   *
   * @} (end addtogroup PARAMETERS)
   *
   * @addtogroup INIT_DATA Initialization Data
   * @brief Initialization Data
   * @{
   * @details
   *   # Initialization Data
   *
   *   Data is an array of unsigned shorts, each entry set one pin state
   *   For each entry, use
   *
   *   PIP(p,b,m,v)	: initialize one port
   *      p : port (gpioPortx)
   *      b : bit number (0=>15)
   *      m : port mode (PI_DISABLED:disabled, PI_INPUT:input, PI_INPUT_PULL:input, PI_OUTPUT:push pull)
   *      v : init value (PRODUCT_DATA_INIT_LOW (0):low/pull-down, PRODUCT_DATA_INIT_HIGH (1):high/pull-up)
   *
   *   Note :
   *   	PI_DISABLED, initial value of 0 : no pull-down, initial value of 1 : pull-up
   *   	PI_INPUT, no pull-up/pull-down
   *   	PI_INPUT_PULL, initial value of 0 : pull-down, initial value of 1 : pull-up
   *
   *   PIS(p,s)	: initial port strength
   *      p : port (gpioPortx)
   *	  s : drive mode set (gpioDriveModexx)
   *
   *   PIR(s)	: gpio route config. Will disable F0
   *	  s : bit 0 : enable SWCLK
   *	  s : bit 1 : enable SWDIO
   *	  s : bit 2 : enable SWOPEN
   *
   *    example:
   * @code
   *    #define PRODUCT_INIT_DATA {PIP(gpioPortB, 1, PI_OUTPUT, 0), PIP(gpioPortF, 7, PI_OUTPUT, 1)}
   * @endcode
   *		set Port B1 to 0 and Port F7 to 1
   *
   *	PINULL : dummy entry, Do nothing. Initialization array length must be
   *	> 0 so, used for empty array
   *
   *	Note : PRODUCT_LED_PORT, PRODUCT_LED_BIT are initialized before Init
   *	Data.
   *	If PRODUCT_LED_PORT is different from 0xFF, it set this port to push pull
   *
   * @} (end addtogroup INIT_DATA)
   * @} (end addtogroup DYNAMIC_OPTIONS)
   *
   ******************************************************************************/

  /** @cond TD_CONFIG */

  /* Cook standard AT variables */
  /* If MANUFACTURER is not defined, we suppose no AT command are included */
#ifdef MANUFACTURER
#ifndef MANUFACTURER
#error("To use AT command, MANUFACTURER must be defined")
#endif
#ifndef PRODUCT
#error("To use AT command, PRODUCT must be defined")
#endif
#ifndef HARDWARE_VERSION
#error("To use AT command, HARDWARE_VERSION must be defined")
#endif
#ifndef SOFTWARE_VERSION
#error("To use AT command, SOFTWARE_VERSION must be defined")
#endif
#ifndef RELEASE_DATE
#error("To use AT command, RELEASE_DATE must be defined")
#endif
#ifndef SERIAL_NUMBER
#error("To use AT command, SERIAL_NUMBER must be defined")
#endif
#define TD_STDLIB_DATA_AT	\
  /** AT device Manufacturer */\
  char const *CONFIG_MANUFACTURER = MANUFACTURER" "PRODUCT;\
  \
  /** AT hardware revision */\
  char const *CONFIG_HARDWARE_VERSION = HARDWARE_VERSION;\
  \
  /** AT software revision */\
  char const *CONFIG_SOFTWARE_VERSION = SOFTWARE_VERSION;\
  \
  /** AT firmware release date */\
  char const *CONFIG_RELEASE_DATE = RELEASE_DATE;\
  \
  /** AT device serial number */
  char *CONFIG_SERIAL_NUMBER = SERIAL_NUMBER;
#else /*MANUFACTURER*/
#define TD_STDLIB_DATA_AT
#endif /*MANUFACTURER*/

  /* Check MANDATORY variables and set others to default values */
#ifdef MODULE_REVISION	/* Do we have standard board? */
#ifndef PRODUCT_TYPE	/* Do we want to define for us? */

#if MODULE_REVISION == REVISION_TD1202
#define PRODUCT_TYPE			0
#define CHIP_TD1202
#endif

#if MODULE_REVISION == REVISION_TD1204
#define PRODUCT_TYPE			8
#define CHIP_TD1204
#endif

#if MODULE_REVISION == REVISION_TD1205
#define PRODUCT_TYPE			9
#define PRODUCT_LED_POLARITY 	1
#define CHIP_TD1205
#endif

#if MODULE_REVISION == REVISION_TD1208
#define PRODUCT_TYPE			10
#define CHIP_TD1208
#endif

#endif /* PRODUCT_TYPE*/
#endif /* MODULE_REVISION*/

  /* Definition of standard TD Chips */
#ifdef CHIP_TD1202
#define POWER_CRYSTAL_PORT      gpioPortF           /**< Power RF Crystal port */
#define POWER_CRYSTAL_BIT       2					/**< Power RF Crystal bit */
#define	SHTD_PORT               gpioPortC           /**< RF GPIO 1 port */
#define	SHTD_BIT                13					/**< RF GPIO 1 bit */
#define RADIO_INIT_DATA			GPIO_TRISTATE, GPIO_NOP, GPIO_TRISTATE, \
	GPIO_RX_RAW_DATA_OUT, GPIO_NOP
#define RADIO_INFO_PIN			2
#define RADIO_INFO_PORT 		gpioPortB
#define RADIO_INFO_BIT 			14
#define TD_GEOLOC_LOGGER_REMOVE_CODE
#ifndef	TD_GEOLOC_USE_CODE
#define TD_GEOLOC_USE_CODE		0
#endif
#define RADIO_USE_TCXO 			1
#define TARGET_COMPILER_IAR
#endif

#ifdef CHIP_TD1204
#define POWER_CRYSTAL_PORT      gpioPortF           /**< Power RF Crystal port */
#define POWER_CRYSTAL_BIT       2					/**< Power RF Crystal bit */
#define	SHTD_PORT               gpioPortC           /**< RF GPIO 1 port */
#define	SHTD_BIT                13					/**< RF GPIO 1 bit */
#define GPS_CS_PORT 			gpioPortB
#define GPS_CS_BIT 				13
#define GPS_IRQ_PORT 			gpioPortC
#define GPS_IRQ_BIT 			1
#define GPS_RESET_PORT 			TD_GPIO_PortRadio
#define GPS_RESET_BIT 			2
#define GPS_VBCKP_PORT 			TD_GPIO_PortRadio 	/**< port 0x10 is Silabs */
#define GPS_VBCKP_BIT 			3
#define GPS_VIO_PORT 			USR2_PORT
#define GPS_VIO_BIT 			USR2_BIT
#define ACCELERO_CS_PORT		USR1_PORT
#define ACCELERO_CS_BIT			USR1_BIT
#define ACCELERO_IRQ_PORT		USR4_PORT
#define ACCELERO_IRQ_BIT		USR4_BIT
#define RADIO_INFO_PIN			0
#define RADIO_INFO_PORT 		gpioPortB
#define RADIO_INFO_BIT 			14
#define RADIO_USE_TCXO 			1

#ifndef PRODUCT_INIT_DATA
#define PRODUCT_INIT_DATA\
  {PIP(gpioPortB,13,PI_DISABLED,1),	/* CS pull-up to 1 for GPS and accelerometer */ \
  PIP(gpioPortC,15,PI_OUTPUT,1)}
#define RADIO_INIT_DATA			GPIO_TRISTATE, GPIO_RX_RAW_DATA_OUT, \
	GPIO_TRISTATE, GPIO_DRIVE0, GPIO_NOP
#endif
#ifndef	TD_GEOLOC_USE_CODE
#define TD_GEOLOC_USE_CODE		1
#endif
#ifndef TD_STACK_PROTECT
#define TD_STACK_PROTECT 1
#endif
#define TARGET_COMPILER_GCC
#endif /* CHIPTD1204 */

#ifdef CHIP_TD1205
#define POWER_CRYSTAL_PORT      gpioPortA           /**< Power RF Crystal port */
#define POWER_CRYSTAL_BIT       0					/**< Power RF Crystal bit */
#define	SHTD_PORT               gpioPortC           /**< RF GPIO 1 port */
#define	SHTD_BIT                13					/**< RF GPIO 1 bit */
#define GPS_CS_PORT 			USR0_PORT
#define GPS_CS_BIT 				USR0_BIT
#define GPS_IRQ_PORT 			USR3_PORT
#define GPS_IRQ_BIT 			USR3_BIT
#define GPS_RESET_PORT 			gpioPortB
#define GPS_RESET_BIT 			11
#define GPS_VBCKP_PORT 			gpioPortF
#define GPS_VBCKP_BIT 			2
#define GPS_VIO_PORT 			USR2_PORT
#define GPS_VIO_BIT 			USR2_BIT
#define ACCELERO_CS_PORT		USR1_PORT
#define ACCELERO_CS_BIT			USR1_BIT
#define ACCELERO_IRQ_PORT		USR4_PORT
#define ACCELERO_IRQ_BIT		USR4_BIT
#define RADIO_INFO_PIN			1
#define RADIO_INFO_PORT 		gpioPortB
#define RADIO_INFO_BIT 			14
#define RADIO_USE_TCXO 			1

#ifndef PRODUCT_INIT_DATA
#define PRODUCT_INIT_DATA\
  {PIP(gpioPortB,13,PI_DISABLED, 1),	/* CS pull-up to 1 for GPS and accelerometer */ \
  PIP(gpioPortC,15,PI_DISABLED, 1)}
#endif
#define RADIO_INIT_DATA			GPIO_IN_RX_STATE, GPIO_TRISTATE, \
	GPIO_IN_TX_STATE, GPIO_TRISTATE, GPIO_NOP
#define RADIO_PA_POWER			20
#ifndef	TD_GEOLOC_USE_CODE
#define TD_GEOLOC_USE_CODE		1
#endif
#ifndef TD_STACK_PROTECT
#define TD_STACK_PROTECT 1
#endif
#define TARGET_COMPILER_GCC
#endif /* CHIPTD1205 */

#ifdef CHIP_TD1208
#define POWER_CRYSTAL_PORT      gpioPortF           /**< Power RF Crystal port */
#define POWER_CRYSTAL_BIT       2					/**< Power RF Crystal bit */
#define	SHTD_PORT               gpioPortC           /**< RF GPIO 1 port */
#define	SHTD_BIT                13					/**< RF GPIO 1 bit */
#define RADIO_INIT_DATA			GPIO_TRISTATE, GPIO_NOP, GPIO_RX_RAW_DATA_OUT, \
	GPIO_NOP, GPIO_NOP
#define RADIO_INFO_PIN			2
#define RADIO_INFO_PORT 		gpioPortB
#define RADIO_INFO_BIT 			14
#define TD_GEOLOC_LOGGER_REMOVE_CODE
#ifndef	TD_GEOLOC_USE_CODE
#define TD_GEOLOC_USE_CODE		0
#endif

#define RADIO_USE_TCXO 			1

#ifndef TD_STACK_PROTECT
#define TD_STACK_PROTECT 1
#endif
#define TARGET_COMPILER_GCC
#endif /* CHIPTD1208 */

#if IS_EMPTY(TD_SENSOR_USE_CODE)
#error("TD_SENSOR_USE_CODE must be set to 0 or 1, or removed")
#endif

#if IS_EMPTY(TD_GEOLOC_USE_CODE)
#error("TD_GEOLOC_USE_CODE must be set to 0 or 1, or removed")
#endif

  /* For all chip, by default we would use sensor */
#ifndef	TD_SENSOR_USE_CODE
#define TD_SENSOR_USE_CODE		1
#endif

  /* PRODUCT_TYPE is mandatory, see error message for standard value or ask Telecom Design for custom design
   * This type is used by the integrated RF loader to prevent bricking products with wrong firmware updates.
   */
#ifndef PRODUCT_TYPE
#error("You must specify a product type : PRODUCT_TYPE=x. TD1202_EVB:0 TD1204_EVB:8 TD1205_EVB:9 TD1208_EVB:10")
#endif

  /* PRODUCT_LED_POLARITY : value to set on PIN TIM2(or
   * PRODUCT_LED_PORT/PRODUCT_LED_BIT) to turn OFF information led (used by the
   * integrated bootloader)
   * default value : led active high
   */
#ifndef PRODUCT_LED_POLARITY
#define PRODUCT_LED_POLARITY 	0
#endif

#if IS_EMPTY(PRODUCT_LED_POLARITY)
#error("PRODUCT_LED_POLARITY is empty, must be set to 0 or 1")
#else

#if ((PRODUCT_LED_POLARITY != 0) && (PRODUCT_LED_POLARITY != 1))
#error("PRODUCT_LED_POLARITY must be set to 0 or 1")
#endif

#endif /* PRODUCT_LED_POLARITY */

  /* PRODUCT_LED_PORT : port used to turn on information led (used by the
   * integrated bootloader)
   * default value : TIM2_PORT (TDxxxx modules)
   */
#ifndef PRODUCT_LED_PORT
#define PRODUCT_LED_PORT TIM2_PORT
#endif

  /* PRODUCT_LED_BIT : bit used to turn on information led (used by the
   * integrated bootloader)
   * default value : TIM2_BIT (TDxxxx modules)
   */
#ifndef PRODUCT_LED_BIT
#define PRODUCT_LED_BIT TIM2_BIT
#endif

  /* PRODUCT_LED_BLINK : set to 1, blink led during each flash sector write
   * default value : false
   * Note : remember if you want to change this default value for TDxxxx module
   * to update ALL user bootloader documentations ...
   */
#ifndef PRODUCT_LED_BLINK
#define PRODUCT_LED_BLINK 		0
#endif

#if (IS_EMPTY(PRODUCT_LED_BLINK))
#error("PRODUCT_LED_BLINK is empty, must be set to 0 or 1")
#else

#if ((PRODUCT_LED_BLINK != 0) && (PRODUCT_LED_BLINK != 1))
#error("PRODUCT_LED_BLINK must be set to 0 or 1")
#endif

#endif /* PRODUCT_LED_BLINK */

    /* LEUART_LOCATION : location of UART pin
     * default value : LEUART_ROUTE_LOCATION_LOC0
     */
#ifndef LEUART_LOCATION
#define LEUART_LOCATION LEUART_ROUTE_LOCATION_LOC0
#endif

#if LEUART_LOCATION == LEUART_ROUTE_LOCATION_LOC1
  #define LEUART_TX_PORT		gpioPortB
  #define LEUART_TX_BIT			13
  #define LEUART_RX_PORT		gpioPortB
  #define LEUART_RX_BIT			14
#endif
#if LEUART_LOCATION == LEUART_ROUTE_LOCATION_LOC0
  #define LEUART_TX_PORT		gpioPortD
  #define LEUART_TX_BIT			4
  #define LEUART_RX_PORT		gpioPortD
  #define LEUART_RX_BIT			5
#endif
#ifndef LEUART_TX_PORT
#error LEUART location is defined to un unhandled confirguration, see config.h for more information
#endif

#ifndef TD_UART_COUNT
#define TD_UART_COUNT			1
#endif

#include <td_uart.h>
uint8_t const CONFIG_TD_UART_COUNT = TD_UART_COUNT;
TD_UART_port_t TD_UART[TD_UART_COUNT];

  /* PRODUCT_BOOTLOADER_CHANNEL : channel to use for bootloader feature
   * default value : 255
   * Note :
   * 	channel 25 (868.4MHz) and 26 (869.80MHz) are allowed in France
   * 	channel 254 is on 856.798185
   * 	channel 255 is on 855.173185
   */
#ifndef PRODUCT_BOOTLOADER_CHANNEL
#define PRODUCT_BOOTLOADER_CHANNEL	255
#endif

  /* PRODUCT_BOOTLOADER_SKIP : set to true, do not execute bootloader
   * default value : 0
   */
#ifndef PRODUCT_BOOTLOADER_SKIP
#define PRODUCT_BOOTLOADER_SKIP	0
#endif


#if IS_EMPTY(PRODUCT_BOOTLOADER_SKIP)
#error("PRODUCT_BOOTLOADER_SKIP is empty, must be set to 0 or 1")
#endif

	/* PRODUCT_BOOTLOADER_SKIP_UART : set to true, do not execute UART
	 * bootloader
	 * default value : 0
	 */
#ifndef PRODUCT_UART_BOOTLOADER_SKIP
#define PRODUCT_UART_BOOTLOADER_SKIP	0
#endif

#if IS_EMPTY(PRODUCT_UART_BOOTLOADER_SKIP)
#error("PRODUCT_UART_BOOTLOADER_SKIP is empty, must be set to 0 or 1")
#endif

  /* RADIO_INIT_DATA : RF IO port initialized in bootloader
  * default value : empty
  */
#ifndef RADIO_INIT_DATA
#error("RADIO_INIT_DATA must be defined !\r\n"\
"Please remove your PRODUCT_TYPE definition and keep only MODULE_REVISION definition.\r\n"\
"if PRODUCT_TYPE is externally defined, all SYSTEM define must also occurs")
#endif

  /* PRODUCT_INIT_DATA : CPU IO port optionally initialized in bootloader
   * default value : 0x00F0. This value is a "do nothing" value
   */
#ifndef PRODUCT_INIT_DATA
#define PRODUCT_INIT_DATA {PINULL}
#endif

  /* RADIO_INFO_PIN : RF IO port link to EFM32 CPU
   * default value : empty
   */
#ifndef RADIO_INFO_PIN
#error("RADIO_INFO_PIN must be defined ! (if PRODUCT_TYPE is externally defined all SYSTEM define must also occurs)")
#endif

  /* RADIO_INFO_PORT : EFM32 CPU port link to RF IO
   * default value : empty
   */
#ifndef RADIO_INFO_PORT
#error("RADIO_INFO_PORT must be defined ! (if PRODUCT_TYPE is externally defined all SYSTEM define must also occurs)")
#endif
  /* RADIO_INFO_PIN : EFM32 CPU bit link to RF IO
   * default value : empty
   */
#ifndef RADIO_INFO_BIT
#error("RADIO_INFO_BIT must be defined ! (if PRODUCT_TYPE is externally defined all SYSTEM define must also occurs)")
#endif

  /* RADIO_USE_TCXO : Radio config
   * default value : empty
   */
#ifndef RADIO_USE_TCXO
#error("RADIO_USE_TCXO must be defined ! (if PRODUCT_TYPE is externally defined all SYSTEM define must also occurs)")
#endif

  /* FORCE_RADIO_RESET : force RF reset at each reset
   * default value : 0
   */
#ifndef FORCE_RADIO_RESET
#define FORCE_RADIO_RESET		0
#endif

  /* RADIO_PA_POWER : default RF PA power for SIGFOX
   * default value : 14
   */
#ifndef RADIO_PA_POWER
#define RADIO_PA_POWER			14
#endif

#ifndef MAX_USER_SPI_ID
#define MAX_SPI_ID MAX_SYSTEM_SPI_ID
#else
#define MAX_SPI_ID (MAX_SYSTEM_SPI_ID + MAX_USER_SPI_ID)
#endif

#ifdef GPS_CS_PORT
#ifndef GPS_CS_BIT
#error("GPS_CS_BIT is undefined")
#endif
#ifndef GPS_IRQ_PORT
#error("GPS_IRQ_PORT is undefined")
#endif
#ifndef GPS_IRQ_BIT
#error("GPS_IRQ_BIT is undefined")
#endif
#ifndef GPS_VIO_PORT
#error("GPS_VIO_PORT is undefined")
#endif
#ifndef GPS_RESET_PORT
#error("GPS_RESET_PORT is undefined")
#endif
#ifndef GPS_RESET_BIT
#error("GPS_RESET_BIT is undefined")
#endif
#ifndef GPS_VBCKP_PORT
#error("GPS_VBCKP_PORT is undefined")
#endif
#ifndef GPS_VBCKP_BIT
#error("GPS_VBCKP_BIT is undefined")
#endif
#endif /* GPS_CS_PORT (Do we want GPS?) */

  /* Board Support Package definition */
#ifdef GPS_CS_PORT
#define TD_BSP_GPS\
  TD_GPIO_Port_TypeDef const CONFIG_GPS_CS_PORT = GPS_CS_PORT;\
  uint8_t const CONFIG_GPS_CS_BIT = GPS_CS_BIT;\
  GPIO_Port_TypeDef const CONFIG_GPS_IRQ_PORT = GPS_IRQ_PORT;\
  uint8_t const CONFIG_GPS_IRQ_BIT = GPS_IRQ_BIT;\
  TD_GPIO_Port_TypeDef const CONFIG_GPS_RESET_PORT = GPS_RESET_PORT;\
  uint8_t const CONFIG_GPS_RESET_BIT = GPS_RESET_BIT;\
  TD_GPIO_Port_TypeDef const CONFIG_GPS_VBCKP_PORT = GPS_VBCKP_PORT;\
  uint8_t const CONFIG_GPS_VBCKP_BIT = GPS_VBCKP_BIT;\
  TD_GPIO_Port_TypeDef const CONFIG_GPS_VIO_PORT = GPS_VIO_PORT;\
  uint8_t const CONFIG_GPS_VIO_BIT = GPS_VIO_BIT;
#else
#define TD_BSP_GPS
#endif

#ifdef ACCELERO_CS_PORT
#ifndef ACCELERO_CS_BIT
#error("ACCELERO_CS_BIT is undefined")
#endif
#ifndef ACCELERO_IRQ_PORT
#error("ACCELERO_IRQ_PORT is undefined")
#endif
#ifndef ACCELERO_IRQ_BIT
#error("ACCELERO_IRQ_BIT is undefined")
#endif
#endif /* ACCELERO_CS_PORT (Do we want ACCELERO ?) */

#ifndef ACCELERO_SPI_MODE
#define ACCELERO_SPI_MODE usartClockMode3
#endif
#ifndef ACCELERO_SPI_BUS
#define ACCELERO_SPI_BUS 		0
#endif
  /* Board Support Package definition */
#ifdef ACCELERO_CS_PORT
#define TD_BSP_ACCELERO\
  TD_GPIO_Port_TypeDef const CONFIG_ACCELERO_CS_PORT = (TD_GPIO_Port_TypeDef) \
  ACCELERO_CS_PORT;\
  uint8_t const CONFIG_ACCELERO_CS_BIT = ACCELERO_CS_BIT;\
  GPIO_Port_TypeDef const CONFIG_ACCELERO_IRQ_PORT = ACCELERO_IRQ_PORT;\
  uint8_t const CONFIG_ACCELERO_IRQ_BIT = ACCELERO_IRQ_BIT;
	uint16_t const CONFIG_ACCELERO_SPI_MODE = ACCELERO_SPI_MODE;
	uint8_t const CONFIG_ACCELERO_SPI_BUS = ACCELERO_SPI_BUS;
#else
#define TD_BSP_ACCELERO
#endif

	/* Board Support Package definition */
#ifdef MAGNETO_CS_PORT
#define TD_BSP_MAGNETO\
	TD_GPIO_Port_TypeDef const CONFIG_MAGNETO_CS_PORT = (TD_GPIO_Port_TypeDef) \
	MAGNETO_CS_PORT;\
	uint8_t const CONFIG_MAGNETO_CS_BIT = MAGNETO_CS_BIT;
	uint16_t const CONFIG_MAGNETO_SPI_MODE = MAGNETO_SPI_MODE;
	uint8_t const CONFIG_MAGNETO_SPI_BUS = MAGNETO_SPI_BUS;
#else
#define TD_BSP_MAGNETO
#endif

#ifndef PRESSURE_SPI_MODE
#define PRESSURE_SPI_MODE usartClockMode3
#endif
#ifndef PRESSURE_SPI_BUS
#define PRESSURE_SPI_BUS 		0
#endif
#ifdef PRESSURE_CS_PORT
#define TD_BSP_PRESSURE\
	TD_GPIO_Port_TypeDef const CONFIG_PRESSURE_CS_PORT = (TD_GPIO_Port_TypeDef) \
	PRESSURE_CS_PORT;\
	uint8_t const CONFIG_PRESSURE_CS_BIT = PRESSURE_CS_BIT;
	uint16_t const CONFIG_PRESSURE_SPI_MODE = PRESSURE_SPI_MODE;
	uint8_t const CONFIG_PRESSURE_SPI_BUS = PRESSURE_SPI_BUS;
#else
#define TD_BSP_PRESSURE
#endif

#if !defined(POWER_CRYSTAL_PORT) && IS_EMPTY(POWER_CRYSTAL_PORT)
#error("PRODUCT_CRYSTAL_PORT must be set (if PRODUCT_TYPE is externally defined all SYSTEM define must also occurs)")
#endif

#if !defined(POWER_CRYSTAL_BIT) && IS_EMPTY(POWER_CRYSTAL_BIT)
#error("PRODUCT_CRYSTAL_BIT must be set (if PRODUCT_TYPE is externally defined all SYSTEM define must also occurs)")
#endif

#if !defined(SHTD_PORT) && IS_EMPTY(SHTD_PORT)
#error("SHTD_PORT must be set (if PRODUCT_TYPE is externally defined all SYSTEM define must also occurs)")
#endif

#if !defined(SHTD_BIT) && IS_EMPTY(SHTD_BIT)
#error("SHTD_BIT must be set (if PRODUCT_TYPE is externally defined all SYSTEM define must also occurs)")
#endif

#ifdef RADIO_POWER_PORT
	TD_GPIO_Port_TypeDef const CONFIG_RADIO_POWER_PORT = RADIO_POWER_PORT;
	uint8_t const CONFIG_RADIO_POWER_BIT = RADIO_POWER_BIT;
#else
	TD_GPIO_Port_TypeDef const CONFIG_RADIO_POWER_PORT = TD_GPIO_PortNull;
	uint8_t const CONFIG_RADIO_POWER_BIT = 0;
#endif

#define TD_BSP_RADIO\
  TD_GPIO_Port_TypeDef const CONFIG_POWER_CRYSTAL_PORT = (TD_GPIO_Port_TypeDef) \
  POWER_CRYSTAL_PORT;\
  uint8_t const CONFIG_POWER_CRYSTAL_BIT = POWER_CRYSTAL_BIT;\
  TD_GPIO_Port_TypeDef const CONFIG_SHTD_PORT = (TD_GPIO_Port_TypeDef) SHTD_PORT;\
  uint8_t const CONFIG_SHTD_BIT = SHTD_BIT;\
  uint8_t const CONFIG_RADIO_INFO_PIN = RADIO_INFO_PIN;\
  GPIO_Port_TypeDef const CONFIG_RADIO_INFO_PORT = (GPIO_Port_TypeDef) \
  RADIO_INFO_PORT;\
  uint8_t const CONFIG_RADIO_INFO_BIT = RADIO_INFO_BIT;\
  uint8_t const CONFIG_FORCE_RADIO_RESET = FORCE_RADIO_RESET;\
  uint8_t const CONFIG_RADIO_PA_POWER = RADIO_PA_POWER;\
  uint8_t const CONFIG_RADIO_USE_TXCO = RADIO_USE_TCXO;

#ifdef PRODUCT_UI
		const uint8_t TD_UI[]={PRODUCT_UI,0xFF};
		uint8_t CONFIG_TD_UI_Id[PRODUCT_UI_COUNT+1];
		#define PRODUCT_UI_INIT	TD_TOOLS_Full_Init(TD_UI);
#else
		#define PRODUCT_UI_INIT
#endif


  /* TD_SCHEDULER_MAX_TIMER : total number of Scheduler Timer. Cost gain :
   * ram usage, system performance.
   * Be careful, standard libraries use some timers ...
   * default value : TD1202 = 10 others = 100 */
#ifndef TD_SCHEDULER_MAX_TIMER
#ifdef EFM32TG210F32
#define TD_SCHEDULER_MAX_TIMER	10
#else
#define TD_SCHEDULER_MAX_TIMER	100
#endif
#endif /* TD_SCHEDULER_MAX_TIMER */

  /* TD_SCHEDULER_MAX_QUEUE : size of timer execution queue. Cost gain : ram
   * usage.
   * Be careful, if timer queue is too short, some timer events will get lost!
   * default value : TD1202 = 10 others = 100 */
#ifndef TD_SCHEDULER_MAX_QUEUE
#ifdef EFM32TG210F32
#define TD_SCHEDULER_MAX_QUEUE	10
#else
#define TD_SCHEDULER_MAX_QUEUE	100
#endif
#endif /* TD_SCHEDULER_MAX_QUEUE */

#if TD_SENSOR_USE_CODE

#include <td_sensor_transmitter.h>

#ifndef TD_SENSOR_TRANSMITTER_MAX_RETRANSMIT
#ifdef EFM32TG210F32
#define TD_SENSOR_TRANSMITTER_MAX_RETRANSMIT 10
#else
#define TD_SENSOR_TRANSMITTER_MAX_RETRANSMIT 10
#endif
#endif

#ifndef TD_SENSOR_TRANSMITTER_MAX_TRANSMIT
#define TD_SENSOR_TRANSMITTER_MAX_TRANSMIT 1
#endif

#if TD_SENSOR_TRANSMITTER_MAX_RETRANSMIT > 0
#define TD_STDLIB_DATA_SENSOR_TRANSMITTER_RETRANSMISSION\
  /** Transmitter retransmit queue */\
  uint8_t const CONFIG_TD_SENSOR_TRANSMITTER_MAX_RETRANSMIT = \
  TD_SENSOR_TRANSMITTER_MAX_RETRANSMIT;\
  static TD_SENSOR_TRANSMITTER_Retransmission_t \
  TransmitterRetransmissionList[TD_SENSOR_TRANSMITTER_MAX_RETRANSMIT];\
  TD_SENSOR_TRANSMITTER_Retransmission_t \
  *TD_SENSOR_TRANSMITTER_RetransmissionList = TransmitterRetransmissionList;
#else
#define TD_STDLIB_DATA_SENSOR_TRANSMITTER_RETRANSMISSION\
  uint8_t const CONFIG_TD_SENSOR_TRANSMITTER_MAX_RETRANSMIT = 0;\
  TD_SENSOR_TRANSMITTER_Retransmission_t \
  *TD_SENSOR_TRANSMITTER_RetransmissionList = NULL;
#endif

#if TD_SENSOR_TRANSMITTER_MAX_TRANSMIT > 0
#define TD_STDLIB_DATA_SENSOR_TRANSMITTER_TRANSMISSION\
  /** Transmitter transmit queue */\
  uint8_t const CONFIG_TD_SENSOR_TRANSMITTER_MAX_TRANSMIT = \
  TD_SENSOR_TRANSMITTER_MAX_TRANSMIT;\
  static TD_SENSOR_TRANSMITTER_Transmission_t \
  TransmitterTransmissionQueue[TD_SENSOR_TRANSMITTER_MAX_TRANSMIT];\
  TD_SENSOR_TRANSMITTER_Transmission_t *TD_SENSOR_TRANSMITTER_TransmissionQueue \
  = TransmitterTransmissionQueue;
#else
#define TD_STDLIB_DATA_SENSOR_TRANSMITTER_TRANSMISSION\
  uint8_t const CONFIG_TD_SENSOR_TRANSMITTER_MAX_TRANSMIT = 0;\
  TD_SENSOR_TRANSMITTER_Transmission_t *TD_SENSOR_TRANSMITTER_TransmissionQueue \
  = NULL;
#endif

#define TD_STDLIB_DATA_SENSOR_TRANSMITTER\
  TD_STDLIB_DATA_SENSOR_TRANSMITTER_RETRANSMISSION\
  TD_STDLIB_DATA_SENSOR_TRANSMITTER_TRANSMISSION
#else	/* No td_sensor_transmitter.h included */
#define TD_STDLIB_DATA_SENSOR_TRANSMITTER
#endif /* TD_SENSOR_USE_CODE */

#if TD_SENSOR_USE_CODE

#include <td_sensor_device.h>

#ifndef TD_SENSOR_DEVICE_QUEUE_MAX
#ifdef EFM32TG210F32
#define TD_SENSOR_DEVICE_QUEUE_MAX 10
#else
#define TD_SENSOR_DEVICE_QUEUE_MAX 10
#endif
#endif

#if TD_SENSOR_DEVICE_QUEUE_MAX > 0
#define TD_STDLIB_DATA_SENSOR_DEVICE\
  /** Device queue */\
  uint8_t const CONFIG_TD_SENSOR_DEVICE_QUEUE_MAX = TD_SENSOR_DEVICE_QUEUE_MAX;\
  static TD_SENSOR_DEVICE_DeviceQueueFrame_t \
  DeviceQueueList[TD_SENSOR_DEVICE_QUEUE_MAX];\
  TD_SENSOR_DEVICE_DeviceQueueFrame_t * DeviceQueue = DeviceQueueList;
#else
#define TD_STDLIB_DATA_SENSOR_DEVICE\
  uint8_t const CONFIG_TD_SENSOR_DEVICE_QUEUE_MAX = 0;\
  TD_SENSOR_DEVICE_DeviceQueueFrame_t * DeviceQueue = NULL;
#endif

#else
#define TD_STDLIB_DATA_SENSOR_DEVICE
#endif /* TD_SENSOR_USE_CODE */

#if TD_SENSOR_USE_CODE

#include <td_sensor_gateway.h>

#ifndef TD_SENSOR_GATEWAY_MAX_DEVICE
#define TD_SENSOR_GATEWAY_MAX_DEVICE 15
#endif

#if (TD_SENSOR_GATEWAY_MAX_DEVICE > 0) && !defined(TD_SENSOR_GATEWAY_REMOVE_CODE)
#define TD_STDLIB_DATA_SENSOR_GATEWAY\
  /**Registered Devices List and count*/\
  uint8_t const CONFIG_TD_SENSOR_GATEWAY_MAX_DEVICE = \
  TD_SENSOR_GATEWAY_MAX_DEVICE + 1;\
  static TD_SENSOR_GATEWAY_Device_t \
  GatewayDeviceList[TD_SENSOR_GATEWAY_MAX_DEVICE + 1];\
  TD_SENSOR_GATEWAY_Device_t * DeviceList = GatewayDeviceList;
#else
#define TD_STDLIB_DATA_SENSOR_GATEWAY\
  /**Registered Devices List and count*/\
  uint8_t const CONFIG_TD_SENSOR_GATEWAY_MAX_DEVICE = \
  TD_SENSOR_GATEWAY_MAX_DEVICE + 1;\
  TD_SENSOR_GATEWAY_Device_t * DeviceList = NULL;
#endif

#else	/* No td_sensor_gateway.h included */
#define TD_STDLIB_DATA_SENSOR_GATEWAY
#endif /* TD_SENSOR_USE_CODE */

  /* If we have td_data_sensor.h included, implement transmitter data */
#if TD_SENSOR_USE_CODE
#include <td_sensor.h>

#ifndef TD_SENSOR_MAX_SWITCH
#define TD_SENSOR_MAX_SWITCH 	8
#endif

#ifndef TD_SENSOR_MAX_SWITCH_EVENT
#define TD_SENSOR_MAX_SWITCH_EVENT 5
#endif

#if TD_SENSOR_MAX_SWITCH > 0
#define TD_STDLIB_DATA_SENSOR_SWITCH\
  /** Switch monitoring */\
  uint8_t const CONFIG_TD_SENSOR_MAX_SWITCH = TD_SENSOR_MAX_SWITCH;\
  static TD_SENSOR_SwitchConfiguration_t \
  SensorSwitchConfig[TD_SENSOR_MAX_SWITCH];\
  TD_SENSOR_SwitchConfiguration_t *TD_SENSOR_SwitchConfig = SensorSwitchConfig;\
  /** Switch monitoring - non irq processing (add 1 for empty buffer stage needed) */\
  uint8_t const CONFIG_TD_SENSOR_MAX_SWITCH_EVENT = \
  TD_SENSOR_MAX_SWITCH_EVENT + 1;\
  static TD_SENSOR_SwitchState_t \
  SensorSwitchStateList[TD_SENSOR_MAX_SWITCH_EVENT];\
  TD_SENSOR_SwitchState_t * TD_SENSOR_SwitchStateList = SensorSwitchStateList;
#else
#define TD_STDLIB_DATA_SENSOR_SWITCH\
  /** Switch monitoring */\
  uint8_t const CONFIG_TD_SENSOR_MAX_SWITCH = TD_SENSOR_MAX_SWITCH;\
  TD_SENSOR_SwitchConfiguration_t * TD_SENSOR_SwitchConfig = NULL;\
  /** Switch monitoring - non irq processing */\
  uint8_t const CONFIG_TD_SENSOR_MAX_SWITCH_EVENT = TD_SENSOR_MAX_SWITCH_EVENT;\
  TD_SENSOR_SwitchState_t * TD_SENSOR_SwitchStateList = NULL;
#endif
#else	/* No td_data_sensor.h included */
#define TD_STDLIB_DATA_SENSOR_SWITCH
#endif /* TD_SENSOR_USE_CODE */

#if TD_SENSOR_USE_CODE
#include <td_sensor.h>
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
#endif /* TD_SENSOR_USE_CODE */

#if PRODUCT_BOOTLOADER_SKIP
  TD_BOOT_Handler_t const TD_BOOT_Handler = NULL;
#endif

uint8_t const CONFIG_PRODUCT_UART_BOOTLOADER_SKIP = PRODUCT_UART_BOOTLOADER_SKIP;


#if TD_STACK_PROTECT==1
  TD_StackProtect_t const TD_StackProtect = TD_TRAP_StackProtect;
#else
  TD_StackProtect_t const TD_StackProtect = NULL;
#endif

#ifndef LAN_PERIOD
#define LAN_PERIOD 				T1S
#endif

#ifndef LAN_THRESHOLD
#define LAN_THRESHOLD 			40
#endif

#ifndef LAN_ADDRESS_SIZE
#define LAN_ADDRESS_SIZE 		24
#endif

#if ((LAN_PERIOD / T26_6MS) >= (1 << (31 - LAN_ADDRESS_SIZE)))
#error("LAN_PERIOD/LAN_ADDRESS_SIZE error. LAN_PERIOD or LAN_ADDRESS_SIZE is too large ")
#endif

#if ((LAN_PERIOD / T26_6MS) >= 253)
#error("LAN_PERIOD too high. For the time no more than 253 packets can be send. This limitation is to be removed")
#endif

#define TD_STDLIB_SCHEDULER\
/** Timer list */\
uint8_t const CONFIG_TD_SCHEDULER_MAX_TIMER = TD_SCHEDULER_MAX_TIMER;\
static TD_SCHEDULER_timer_t SchedulerTimer[TD_SCHEDULER_MAX_TIMER];\
TD_SCHEDULER_timer_t * TD_SCHEDULER_Timer = SchedulerTimer;\
\
/** Scheduler callback queue */\
uint8_t const CONFIG_TD_SCHEDULER_MAX_QUEUE = TD_SCHEDULER_MAX_QUEUE;\
static TD_SCHEDULER_callback_t SchedulerCallbackQueue[TD_SCHEDULER_MAX_QUEUE];\
TD_SCHEDULER_callback_t * TD_SCHEDULER_CallbackQueue  = SchedulerCallbackQueue;

/** Scheduler don't trap on queue overflow flag */
#ifdef TD_SCHEDULER_DONT_OVF_QUEUE
bool const CONFIG_TD_SCHEDULER_DONT_OVF_QUEUE = true;
#else
bool const CONFIG_TD_SCHEDULER_DONT_OVF_QUEUE = false;
#endif

#ifdef PRODUCT_EARLY_DEBUG
const TD_MAIN_Init_t CONFIG_EARLY_DEBUG = main_init_debug;
bool const CONFIG_PRODUCT_EARLY_DEBUG=true;
#else
const TD_MAIN_Init_t CONFIG_EARLY_DEBUG = main_init;
bool const CONFIG_PRODUCT_EARLY_DEBUG=false;
#endif

#if PRODUCT_BOOTLOADER_CHANNEL == 255
#define	PRODUCT_BOOTLOADER_CHANNEL_VALUE	BOOT_FREQ_CHAN255
#endif
#if PRODUCT_BOOTLOADER_CHANNEL == 254
#define	PRODUCT_BOOTLOADER_CHANNEL_VALUE	BOOT_FREQ_CHAN254
#endif
#if PRODUCT_BOOTLOADER_CHANNEL == 25
#define	PRODUCT_BOOTLOADER_CHANNEL_VALUE	BOOT_FREQ_CHAN25
#endif
#if PRODUCT_BOOTLOADER_CHANNEL == 26
#define	PRODUCT_BOOTLOADER_CHANNEL_VALUE	BOOT_FREQ_CHAN26
#endif
#ifndef PRODUCT_BOOTLOADER_CHANNEL_VALUE
#error("PRODUCT_BOOTLOADER_CHANNEL is set to an invalid value")
#endif

#ifndef __TD_FLASH_H
#include <td_flash.h>
#endif

#ifdef TD_FLASH_MAX_DATA_POINTER
#define MAX_FLASH_DATA_POINTER TD_FLASH_MAX_DATA_POINTER
#else
  /** Maximum number of data pointers */
#ifdef EFM32TG210F32
#define MAX_FLASH_DATA_POINTER	10
#else
#define MAX_FLASH_DATA_POINTER	25
#endif
#endif /* TD_FLASH_MAX_DATA_POINTER */

#ifndef	TD_FLASH_USER_PAGE
#define TD_FLASH_USER_PAGE		1
#endif

uint8_t const CONFIG_TD_FLASH_USER_PAGE = TD_FLASH_USER_PAGE;

#ifndef	AT_PERSIST_SIZE
  #ifdef EFM32TG210F32
    /** AT persist buffer size */
  #define AT_PERSIST_SIZE		36
  #else
    /** AT persist buffer size */
  #define AT_PERSIST_SIZE		192
  #endif
#endif

uint16_t const CONFIG_AT_PERSIST_SIZE = AT_PERSIST_SIZE;
/** AT persist buffer */
uint8_t AT_persist_buffer_buffer[AT_PERSIST_SIZE];
uint8_t *AT_persist_buffer=AT_persist_buffer_buffer;

#ifndef FLASH_VARIABLES_VERSION
  #define FLASH_VARIABLES_VERSION 0
#endif

uint32_t const CONFIG_TD_FLASH_VARIABLES_VERSION = FLASH_VARIABLES_VERSION;

uint8_t const CONFIG_TD_FLASH_MAX_DATA_POINTER = MAX_FLASH_DATA_POINTER;
static TD_FLASH_variable_t FlashDataList[MAX_FLASH_DATA_POINTER];
TD_FLASH_variable_t *TD_FLASH_DataList = FlashDataList;

#define _PRODUCT_TYPE PRODUCT_TYPE

	/* These definitions are the core "dynamic" library data
	 * They must be called outside all function on only one file of each project
	 * PRODUCT_TYPE must always be defined
	 */
	TD_STDLIB_DATA_AT

	/** Product type */
	char const CONFIG_PRODUCT_TYPE = PRODUCT_TYPE;

	/** Bootloader Led Polarity */
	char const CONFIG_PRODUCT_LED_POLARITY = PRODUCT_LED_POLARITY;

	/** Bootloader Led Port */
	GPIO_Port_TypeDef const CONFIG_PRODUCT_LED_PORT = (GPIO_Port_TypeDef) \
		PRODUCT_LED_PORT;

	/** Bootloader Led Bit */
	char const CONFIG_PRODUCT_LED_BIT = PRODUCT_LED_BIT;

	/** Bootloader Led Blink */
	char const CONFIG_PRODUCT_LED_BLINK = PRODUCT_LED_BLINK;

	/** Bootloader Channel */
	unsigned char const CONFIG_PRODUCT_BOOTLOADER_CHANNEL = \
		PRODUCT_BOOTLOADER_CHANNEL;

	/** Bootloader Channel (private parameter)*/
	uint8_t const ProductBootloaderP1[] = {
		8, 0x11, 0x40, 4, 0x00,
		PRODUCT_BOOTLOADER_CHANNEL_VALUE & 0xFF,
		(PRODUCT_BOOTLOADER_CHANNEL_VALUE >> 24) & 0xFF,
		(PRODUCT_BOOTLOADER_CHANNEL_VALUE >> 16) & 0xFF,
		(PRODUCT_BOOTLOADER_CHANNEL_VALUE >> 8) & 0xFF,
		0
	};

	/** Bootloader Init Data */
	unsigned short const CONFIG_PRODUCT_INIT_DATA[] = PRODUCT_INIT_DATA;

	/** LEUART location */
	uint32_t const CONFIG_LEUART_LOCATION = LEUART_LOCATION;
	GPIO_Port_TypeDef const CONFIG_LEUART_TX_PORT =
		(GPIO_Port_TypeDef)LEUART_TX_PORT;
	uint8_t	const CONFIG_LEUART_TX_BIT = LEUART_TX_BIT;
	GPIO_Port_TypeDef const CONFIG_LEUART_RX_PORT =
		(GPIO_Port_TypeDef)LEUART_RX_PORT;
	uint8_t	const CONFIG_LEUART_RX_BIT = LEUART_RX_BIT;

	/** RF Init Data */
	unsigned char const CONFIG_RADIO_INIT_DATA[] = {
		6, 0x13, RADIO_INIT_DATA, 0
	};

	/** LAN sampling period */
	unsigned long const CONFIG_LAN_PERIOD = LAN_PERIOD;

	/** LAN address size */
	unsigned char const CONFIG_LAN_ADDRESS_SIZE = LAN_ADDRESS_SIZE;

	/** LAN threshold level */
	unsigned char CONFIG_LAN_THRESHOLD = LAN_THRESHOLD;

	/** SPI Max user and data */
	unsigned char const CONFIG_MAX_SPI_ID = MAX_SPI_ID;
	TD_SPI_Conf_t __BOOTVARIABLE TD_SPI_Conf_[MAX_SPI_ID + 1];
	TD_SPI_Conf_t *TD_SPI_Conf = TD_SPI_Conf_;

	unsigned char const CONFIG_PRODUCT_INIT_DATA_SIZE =
		sizeof (CONFIG_PRODUCT_INIT_DATA) / sizeof (unsigned short);
	TD_STDLIB_SCHEDULER
	TD_STDLIB_DATA_SENSOR_TRANSMITTER
	TD_STDLIB_DATA_SENSOR_DEVICE
	TD_STDLIB_DATA_SENSOR_GATEWAY
	TD_STDLIB_DATA_SENSOR_SWITCH
	TD_BSP_GPS
	TD_BSP_ACCELERO
	TD_BSP_MAGNETO
	TD_BSP_PRESSURE
	TD_BSP_RADIO
	/* This line must remain empty - end of definition */

  /* Handle code reduction system */
#if defined(TD_TRAP_MINI_CODE)
	TD_TRAP_callback_t TD_Trap_Callback = (TD_TRAP_callback_t)
		TD_TRAP_Mini_Callback;
	TD_TRAP_callback_t const TD_Trap_Callback2 =
		(TD_TRAP_callback_t)TD_TRAP_Mini_Callback;
#elif defined(TD_TRAP_FLASH_CODE)
	TD_TRAP_callback_t TD_Trap_Callback = (TD_TRAP_callback_t)
		TD_TRAP_Flash_Callback;
	TD_TRAP_callback_t const TD_Trap_Callback2 =
		(TD_TRAP_callback_t)TD_TRAP_Mini_Callback;
#elif defined(TD_TRAP_FLASH_PRINTF_CODE)
	TD_TRAP_callback_t TD_Trap_Callback = (TD_TRAP_callback_t)
		TD_TRAP_Mini_Callback;
	TD_TRAP_callback_t const TD_Trap_Callback2 = (TD_TRAP_callback_t)
		TD_TRAP_Mini_Callback;
#elif defined(TD_TRAP_PRINTF_CODE)
	TD_TRAP_callback_t TD_Trap_Callback = TD_TRAP_Printf_Callback;
	TD_TRAP_callback_t const TD_Trap_Callback2 = TD_TRAP_Printf_Callback;
#elif defined(TD_TRAP_RESET_CODE)
	TD_TRAP_callback_t TD_Trap_Callback = TD_TRAP_Reset_Callback;
	TD_TRAP_callback_t const TD_Trap_Callback2 = TD_TRAP_Reset_Callback;
#else
  TD_TRAP_callback_t TD_Trap_Callback = TD_TRAP_Printf_Callback;
  TD_TRAP_callback_t const TD_Trap_Callback2 = TD_TRAP_Printf_Callback;
#endif

#if TD_SENSOR_USE_CODE
#include <td_sensor.h>

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

#include <td_sensor_transmitter.h>
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

#include <td_sensor_device.h>
#ifdef TD_SENSOR_DEVICE_REMOVE_CODE
	NOP_DYNAMIC(TD_SENSOR_DEVICE_FrameReceived);
	NOP_DYNAMIC(TD_SENSOR_DEVICE_Process);
	NOP_DYNAMIC(TD_SENSOR_DEVICE_Forward);
#else
	INIT_DYNAMIC(TD_SENSOR_DEVICE_FrameReceived);
	INIT_DYNAMIC(TD_SENSOR_DEVICE_Process);
	INIT_DYNAMIC(TD_SENSOR_DEVICE_Forward);
#endif

#include <td_sensor_lan.h>
#ifdef TD_SENSOR_LAN_REMOVE_CODE
	NOP_DYNAMIC(TD_SENSOR_LAN_Init);
#else
	INIT_DYNAMIC(TD_SENSOR_LAN_Init);
#endif

#endif /* TD_SENSOR_USE_CODE */

#if TD_GEOLOC_USE_CODE
#include <td_geoloc.h>
#include <ubx7.h>

#ifndef TD_GEOLOC_RAW_OUTPUT
#define TD_GEOLOC_RAW_OUTPUT false
#endif

	bool const CONFIG_TD_GEOLOC_RAW_OUTPUT = TD_GEOLOC_RAW_OUTPUT;

#ifdef TD_GEOLOC_LOGGER_REMOVE_CODE
	TRAP_DYNAMIC(TD_FLASH_InitLogger);
	TRAP_DYNAMIC(TD_GEOLOC_Log);
#else
	INIT_DYNAMIC(TD_FLASH_InitLogger);
	INIT_DYNAMIC(TD_GEOLOC_Log);
#endif
#endif /* TD_GEOLOC_USE_CODE */

#include <td_sigfox.h>

/* Si4461 PA mode - switch current 0 or ndef, square wave 1 */
#ifndef TD_RF_SI4461_CLASS_E
#define TD_RF_SI4461_CLASS_E	0
#endif

	/* Si4461 PA Mode */
	uint8_t const CONFIG_TD_RF_SI4461_CLASS_E = TD_RF_SI4461_CLASS_E;

/* Proxy class */
#ifndef TD_SIGFOX_PROXY_CLASS
#define TD_SIGFOX_PROXY_CLASS	9
#endif

	/* RF Proxy class config */
	uint8_t const CONFIG_TD_SIGFOX_PROXY_CLASS = TD_SIGFOX_PROXY_CLASS;

/* RF Power calculation defines, define to use user callback */
#ifndef TD_RF_POWER_CALCULATION
#define TD_RF_POWER_CALCULATION	NULL
#endif

	/* RF Power calculation callback */
	RFPowerCalculation_t const CONFIG_TD_RF_POWER_CALCULATION =
		TD_RF_POWER_CALCULATION;

	/* RF Power calculation coef, [0] is dB Value, [1] is max_voltage_mv / 100;
	 * [2] is power for max_voltage;
	 * [3] is power for max_voltage - 100mV; etc
	 * indice = 127 means max for all next values
	 * indice = -1 means end of table
	 * indice = -2 means end
	 *
	 * ex :
	 *
	 * TD_RF_POWER_CALCULATION_COEF {14,36,100,101,102,103,104,106,108,112,120,
	 * 127,-1,10,36,127,-1,-2}
	 *
	 * -> 14dB, start level 3.6V
	 *
	 * 3.6 -> 100
	 * 3.5 -> 101
	 * 3.4 -> 102
	 * 3.3 -> 103
	 * 3.2 -> 104
	 * 3.1 -> 106
	 * 3.0 -> 108
	 * 2.9 -> 112
	 * 2.8 -> 120
	 * <=2.7 -> 127
	 *
	 * -> 10dB, start level 3.6V
	 * <=3.6 -> 127
	 *
	 * intermediate voltage values will use linear approximation
	 *
	 *    */
#ifndef TD_RF_POWER_CALCULATION_COEF
	int8_t const CONFIG_TD_RF_POWER_CALCULATION_COEF[] = {-2};
#else
	int8_t const CONFIG_TD_RF_POWER_CALCULATION_COEF[] =
		TD_RF_POWER_CALCULATION_COEF;
#endif

/* Sigfox handling system
 *
 *   +=============+               +=======+
 *   |local module |   \/\/\/\---> |booster|
 *   +=============+               +=======+
 *
 *
 *                     | Use local emit | Use forward   |   Use proxy    |   Define                         |   Remarks
 *   :-----------------|:--------------:|:-------------:|:--------------:|:--------------------------------:|:----------------
 *                     | ID module      | ID booster    | ID module      |                                  |
 *   :-----------------|:--------------:|:-------------:|:--------------:|:--------------------------------:|:----------------
 *   default           |     USE        |      -        |    USE         | TD_SIGFOX_TRANSMIT_DEFAULT       | no define
 *   forward only      |      -         |     USE       |     -          | TD_SIGFOX_TRANSMIT_FORWARD_ONLY  | SIGFOX_REMOVE_CODE and SIGFOX_PROXY_USE_FORWARD
 *   local only        |     USE        |      -        |     -          | TD_SIGFOX_TRANSMIT_LOCAL_ONLY    | SIGFOX_PROXY_REMOVE_CODE
 *   none              |      -         |      -        |     -          | TD_SIGFOX_TRANSMIT_NONE          | SIGFOX_REMOVE_CODE and SIGFOX_PROXY_REMOVE_CODE
 *   :-----------------|:--------------:|:-------------:|:--------------:|:--------------------------------:|:----------------
 *   SIGFOX_REMOVE_CODE|    remove      |      -        |   remove       |                                  |
 *   PROXY_REMOVE_CODE |      -         |      -        |   remove       |                                  |
 *   PROXY_USE_FORWARD |      -         |     USE       |                |                                  |
 *
 */

#ifdef TD_SIGFOX_TRANSMIT_DEFAULT
#define SIGFOX_TRANSMIT_DEFINED
#endif

#ifdef TD_SIGFOX_TRANSMIT_FORWARD_ONLY
#ifdef SIGFOX_TRANSMIT_DEFINED
#error Only one SIGFOX_TRANSMIT_xxxx can be used at once !
#endif
#define SIGFOX_TRANSMIT_DEFINED
#define TD_SIGFOX_REMOVE_CODE
#define TD_SIGFOX_PROXY_USE_FORWARD
#endif

#ifdef TD_SIGFOX_TRANSMIT_LOCAL_ONLY
#ifdef SIGFOX_TRANSMIT_DEFINED
#error Only one SIGFOX_TRANSMIT_xxxx can be used at once !
#endif
#define SIGFOX_TRANSMIT_DEFINED
#define TD_SIGFOX_PROXY_REMOVE_CODE
#endif

#ifdef TD_SIGFOX_TRANSMIT_NONE
#ifdef SIGFOX_TRANSMIT_DEFINED
#error Only one SIGFOX_TRANSMIT_xxxx can be used at once !
#endif
#define SIGFOX_TRANSMIT_DEFINED
#define TD_SIGFOX_REMOVE_CODE
#define TD_SIGFOX_PROXY_REMOVE_CODE
/* If we cannot transmit, we cannot receive */
#define TD_SIGFOX_DOWNLINK_REMOVE_CODE
#else
#ifdef TD_SIGFOX_RECEIVE_NONE
#define TD_SIGFOX_DOWNLINK_REMOVE_CODE
#endif
#endif

#if defined(TD_SIGFOX_PROXY_REMOVE_CODE) && defined(TD_SIGFOX_PROXY_USE_FORWARD)
#error ("TD_SIGFOX_PROXY_REMOVE_CODE and TD_SIGFOX_PROXY_USE_FORWARD can't be used together. TD_SIGFOX_PROXY_USE_FORWARD Requires Proxy")
#endif

#if defined(TD_SIGFOX_REMOVE_CODE) &&!defined(TD_SIGFOX_PROXY_REMOVE_CODE) && !defined(TD_SIGFOX_PROXY_USE_FORWARD)
#error ("If you don't want to use proxy please use TD_SIGFOX_PROXY_REMOVE_CODE. Otherwise don't use TD_SIGFOX_REMOVE_CODE")
#endif

#if (defined(TD_SIGFOX_REMOVE_CODE) && defined(TD_SIGFOX_PROXY_REMOVE_CODE))
	/* We want to remove all Sigfox possibilities */
	NOP_DYNAMIC(TD_SIGFOX_Init);
	NULL_DYNAMIC(TD_SIGFOX_SendRaw);
	NULL_DYNAMIC(TD_SIGFOX_DirectTransmission);
	NOP_DYNAMIC(TD_SIGFOX_ComputeFrameByMode);
#else
  #if defined(TD_SIGFOX_PROXY_USE_FORWARD) && !defined(TD_SIGFOX_PROXY_REMOVE_CODE)
	/* We are in forward mode only -> no sigfox capabilities */
	INIT_DYNAMIC_ALT(TD_SIGFOX_Init, TD_SIGFOX_InitForward);
	NULL_DYNAMIC(TD_SIGFOX_SendRaw);
	NULL_DYNAMIC(TD_SIGFOX_DirectTransmission);
	NOP_DYNAMIC(TD_SIGFOX_ComputeFrameByMode);
  #else
	/* We are in full mode */
	INIT_DYNAMIC(TD_SIGFOX_Init);
	INIT_DYNAMIC(TD_SIGFOX_SendRaw);
	INIT_DYNAMIC(TD_SIGFOX_DirectTransmission);
	INIT_DYNAMIC(TD_SIGFOX_ComputeFrameByMode);
  #endif

#endif

#if (defined(TD_SIGFOX_REMOVE_CODE) && !defined(TD_SIGFOX_PROXY_USE_FORWARD)) || \
	defined(TD_SIGFOX_PROXY_REMOVE_CODE)
	/* We remove all code and don't forward, or we want only local transmit */
	NOP_DYNAMIC(TD_SIGFOX_PROXY_Discover);
	NOP_DYNAMIC(TD_SIGFOX_PROXY_Init);
	NOP_DYNAMIC(TD_SIGFOX_PROXY_CompleteInit);
#else
	/* We would use proxy or forward */
	INIT_DYNAMIC(TD_SIGFOX_PROXY_Discover);
	INIT_DYNAMIC(TD_SIGFOX_PROXY_Init);
	INIT_DYNAMIC(TD_SIGFOX_PROXY_CompleteInit);
#endif

#if defined(TD_SIGFOX_REMOVE_CODE) && defined(TD_SIGFOX_PROXY_REMOVE_CODE)
	NOP_DYNAMIC(TD_SIGFOX_Send);
#else
	INIT_DYNAMIC(TD_SIGFOX_Send);
#endif

#ifdef TD_SIGFOX_PROXY_REMOVE_CODE
	/* We don't use proxy at all */
	NULL_DYNAMIC(TD_SIGFOX_PROXY_Transmission);
	NOP_DYNAMIC(TD_SIGFOX_PROXY_Send);
	NULL_DYNAMIC(TD_SIGFOX_PROXY_ForwardTransmission);
	NULL_DYNAMIC(TD_SIGFOX_PROXY_Process);
#else
  #ifdef TD_SIGFOX_PROXY_USE_FORWARD
	/* We use proxy in forward mode */
	NULL_DYNAMIC(TD_SIGFOX_PROXY_Transmission);
	INIT_DYNAMIC(TD_SIGFOX_PROXY_Send);
	INIT_DYNAMIC(TD_SIGFOX_PROXY_ForwardTransmission);
  #else
	/* We use proxy in transparent mode */
	INIT_DYNAMIC(TD_SIGFOX_PROXY_Transmission);
	INIT_DYNAMIC(TD_SIGFOX_PROXY_Send);
	NULL_DYNAMIC(TD_SIGFOX_PROXY_ForwardTransmission);
  #endif
  #ifdef TD_SIGFOX_DOWNLINK_REMOVE_CODE
	/* We use only proxy for transmit not receive */
	NULL_DYNAMIC(TD_SIGFOX_PROXY_Process);
  #else
	/* We use proxy for transmit and receive */
	INIT_DYNAMIC(TD_SIGFOX_PROXY_Process);
  #endif
#endif

#if defined(TD_SIGFOX_REMOVE_CODE) || defined(TD_SIGFOX_DOWNLINK_REMOVE_CODE)
	/* We don't use Sigfox receive */
	NULL_DYNAMIC(TD_SIGFOX_DOWNLINK_DirectProcess);
#else
	/* We use Sigfox receive */
	INIT_DYNAMIC(TD_SIGFOX_DOWNLINK_DirectProcess);
#endif

#ifdef TD_REMOVE_UART_LOADER
	NOP_DYNAMIC(TD_BOOT_UartLoader);
#else
	INIT_DYNAMIC(TD_BOOT_UartLoader);
#endif

#ifdef TD_SIGFOX_USE_TEST_CARRIER_CODE
	INIT_DYNAMIC(TD_SIGFOX_TestCarrier);
#else
	NULL_DYNAMIC(TD_SIGFOX_TestCarrier);
#endif
#ifdef TD_SIGFOX_USE_DOWNLINK_TEST_CODE
	INIT_DYNAMIC(TD_SIGFOX_DOWNLINK_ReceiveTest);
#else
	NULL_DYNAMIC(TD_SIGFOX_DOWNLINK_ReceiveTest);
#endif

#ifdef TD_SCHEDULER_REMOVE_CODE
	NULL_DYNAMIC(TD_SCHEDULER_Init);
	NULL_DYNAMIC(TD_SCHEDULER_Process);
#else
	INIT_DYNAMIC(TD_SCHEDULER_Init);
	INIT_DYNAMIC(TD_SCHEDULER_Process);
#endif

#include "td_trap.h"
#ifdef 	TD_ALL_DUMP_REMOVE_CODE
	//TD_SystemDump_t const TD_SystemDump = (TD_SystemDump_t) NULL;
	NULL_DYNAMIC(TD_SystemDump);
#define TD_GPIO_DUMP_REMOVE_CODE
#define TD_RF_DUMP_REMOVE_CODE
#define TD_IRQ_DUMP_REMOVE_CODE
#define TD_SCHEDULER_DUMP_REMOVE_CODE
#define TD_GEOLOC_DUMP_REMOVE_CODE
#define TD_SPILOCK_DUMP_REMOVE_CODE
#define TD_ACCELERO_DUMP_REMOVE_CODE
#define TD_SENSOR_DUMP_REMOVE_CODE
#else
	INIT_DYNAMIC(TD_SystemDump);
#endif

#if TD_GEOLOC_USE_CODE
/* If there is a compilation error at this point
 * -if you don't want to use libtdgeoloc : add #define TD_GEOLOC_USE_CODE 0
 * - if you want to use libtdgeoloc : add libtdgeoloc/inc include path and the
 *    path to the libtdgeoloc library corresponding to your build configuration
 *    in your project settings
 */
#include <td_accelero.h>

#endif

	void TD_RF_Dump(void);
	TD_Dump_Func_t const TD_TRAP_SystemDumpFunc [] = {
#ifdef TD_GPIO_DUMP_REMOVE_CODE
		NULL,
#else
		TD_GPIO_Dump,
#endif
#if defined(TD_RF_DUMP_REMOVE_CODE)
		NULL,
#else
		TD_RF_Dump,
#endif
#ifdef TD_IRQ_DUMP_REMOVE_CODE
		NULL,
#else
		TD_IRQ_Dump,
#endif
#ifdef TD_SCHEDULER_DUMP_REMOVE_CODE
		NULL,
#else
		TD_SCHEDULER_Dump,
#endif
#if defined(TD_GEOLOC_DUMP_REMOVE_CODE) || TD_GEOLOC_USE_CODE==0
		NULL,
#else
		TD_GEOLOC_Dump,
#endif
#ifdef TD_SPILOCK_DUMP_REMOVE_CODE
		NULL,
#else
		TD_SPI_LockDump,
#endif
#if defined(TD_ACCELERO_DUMP_REMOVE_CODE) || TD_GEOLOC_USE_CODE==0
		NULL,
#else
		TD_ACCELERO_Dump,
#endif
#if defined(TD_ACCELERO_DUMP_REMOVE_CODE) || TD_SENSOR_USE_CODE==0
		NULL,
#else
	  TD_SENSOR_Dump
#endif

	};

	uint8_t const TD_TRAP_MaxSystemDump = sizeof (TD_TRAP_SystemDumpFunc) /
		sizeof (TD_Dump_Func_t);

#if defined(__ICCARM__)
#ifdef STACK_SIZE
#error("Stack size must be modified directly in .ICF file with IAR. If you want TD1202/1208 source code compatibility use, #if defined(__GNUC__) around #define STACK_SIZE")
#endif
#endif
#if defined(__GNUC__)
#ifndef STACK_SIZE
#define STACK_SIZE				0x800
#endif

  /* Create a custom stack with name SYMBOL, aligned to ALIGNMENT bytes, sized
   * by SIZE bytes, but possibly shortened such that the initial stack pointer
   * (symbol __cs3_stack) that points to the block's last extent is aligned to
   * ALIGNMENT bytes, too.
   * HACK : we must add 'used' attributed to not have stack removed by gentle
   * GCC compiler in its size optimization process ...
   */
#define TD_CS3_STACK_SYMBOL(symbol, size, alignment) \
static char __attribute__ ((section(".stack"),aligned (alignment),used)) \
	symbol[(size - ((size) % (alignment)))]; \
asm (".global __cs3_stack"); \
asm ("__cs3_stack = " #symbol " + (" #size ") - (" #alignment ")" \
		" - ((" #size ") % (" #alignment "))")
#define TD_STACK(size)\
	TD_CS3_STACK_SYMBOL(__cs3_stack_block, (size), 32);
	TD_STACK(STACK_SIZE)
#endif

/* Real stack size can be lowered for alignment issue. Here we take it in account, not optimized way
* by removing alignment size on stack size
*/
#ifndef STACK_SIZE
	uint16_t const CONFIG_STACK_SIZE = 0x400 - 0x20;
#else
	uint16_t const CONFIG_STACK_SIZE = STACK_SIZE - 0x20;
#endif

#ifdef DUMP_PRODUCT_CONFIG

// Definition to expand macro then apply to pragma message
#define VALUE_TO_STRING(x)		#x
#define VALUE(x)				VALUE_TO_STRING(x)
#define VAR_NAME_VALUE(var)		#var "=" VALUE(var)

#pragma message("=========================")
#pragma message("=      Board level      =")
#pragma message("=========================")
#pragma message(VAR_NAME_VALUE(PRODUCT_TYPE))
#pragma message(VAR_NAME_VALUE(PRODUCT_LED_PORT))
#pragma message(VAR_NAME_VALUE(PRODUCT_LED_BIT))
#pragma message(VAR_NAME_VALUE(PRODUCT_LED_BLINK))
#pragma message("=========================")
#pragma message("=       Chip level      =")
#pragma message("=========================")
#pragma message(VAR_NAME_VALUE(MODULE_REVISION))
#pragma message(VAR_NAME_VALUE(POWER_CRYSTAL_PORT))
#pragma message(VAR_NAME_VALUE(POWER_CRYSTAL_BIT))
#pragma message(VAR_NAME_VALUE(SHTD_PORT))
#pragma message(VAR_NAME_VALUE(SHTD_BIT))
#pragma message("-------------------------")

#endif

#ifdef __cplusplus
}
#endif

#else
#error("td_config.h MUST NOT be included multiple time")
#endif // __TD_CONFIG_H
/** @endcond */
