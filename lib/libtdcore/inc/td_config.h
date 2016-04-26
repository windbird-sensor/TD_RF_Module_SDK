/***************************************************************************//**
 * @file
 * @brief Configuration file for the TDxxxx RF modules.
 * @author Telecom Design S.A.
 * @version 1.2.0
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
#ifndef __TD_CONFIG_H
#define __TD_CONFIG_H

#include <stdint.h>
#include <stdbool.h>

#include <td_config_ext.h>
#include <td_trap.h>
#include <td_rtc.h>
#include <td_scheduler.h>
#include <td_spi.h>
#include <td_gpio.h>
#include <td_utils.h>
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
 * @addtogroup STANDARD_BOARDS Standard TD EVB Boards
 * @brief Standard EVB Board definitions
 * @{
 * @details
 *  # Standard EVB Boards
 *
 *   When you are using a standard EVB TD board, only one define is mandatory
 *
 * @code
 *	#define MODULE_REVISION REVISION_TD1202
 *	#define MODULE_REVISION REVISION_TD1204
 *	#define MODULE_REVISION REVISION_TD1208
 *	#define MODULE_REVISION REVISION_TD1508
 * @endcode
 *
 *  It will define all required parameters. All of theses can be overloaded by
 *  using custom defines if needed.
 *
 * @} (end addtogroup STANDARD_BOARDS)
 *
 * @addtogroup STANDARD_CHIPS Standard TD Chips
 * @brief Standard TD Chips with custom board
 * @{
 * @details
 *	# Standard TD Chips with custom board
 *
 *	If your are using a standard TD12xx chip/module on a custom board, you
 *	can used these #define to automatically setup configuration for your chip,
 *	and only add the configuration required for your board
 *	(PRODUCT_INIT_DATA,PRODUCT_LED_PORT, ...)
 *
 * @code
 *	#define CHIP_TD1202
 *	#define CHIP_TD1204
 *	#define CHIP_TD1208
 *	#define CHIP_TD1508
 * @endcode
 * @} (end addtogroup STANDARD_CHIPS)
 *
 * @addtogroup PARAMETERS Dynamic Parameters
 * @brief Dynamic parameters that can be overloaded for libraries
 * @{
 * @details
 *
 *   # Parameters
 *
 *   You can define any of these parameters as preprocessor macros before
 *   including <td_config.h> once within your project, they will be turned into
 *   corresponding "CONFIG_xxxx" global constants defined at link time and
 *   available everywhere in your project and in all linked static libraries.
 *
 *   Level can be:
 *   - Chip       : These parameters control internal Chip/Module
 *                  configuration.
 *   				Don't modify them if you use standard TDxxx chip/module
 *   				products.
 *   				Define these parameters if you implement the TD RF
 *   				reference design onto your board.
 *   - Board      : These parameters control Board configuration.
 *                  Don't modify them if you use standard TD EVB boards.
 *                  Define these parameters If you use your own custom board.
 *   - Applicative: These parameters are merely software subsystems related
 *                  ones and can be modified to fit your needs.
 *
 *   If you want to force a parameter in your main source file:
 *
 * @code
 *   #define TD_SCHEDULER_MAX_TIMER 12
 *   #include <td_config.h>
 * @endcode
 *
 *   In all other sources files, this parameter will be available like below:
 *
 * @code
 *   tfp_printf("Max timer : %d, MaxQueue : %d\r\n", CONFIG_TD_SCHEDULER_MAX_TIMER, CONFIG_TD_SCHEDULER_MAX_QUEUE);
 * @endcode
 *
 *    The following parameters are used for setting MCU features and memory configuration:
 *
 *    Define                                  | Level        | Type      |      Usage     | Description
 *    :---------------------------------------|:------------:|:---------:|:--------------:|:--------------------------------------------------------------------
 *    FLASH_VARIABLES_VERSION                 | Applicative  | integer   |  Flash Variable| Version of stored flash variables
 *    TD_FLASH_MAX_DATA_POINTER               | Applicative  | integer   |  Flash Variable| Max number of flash variable handled
 *    TD_FLASH_USER_PAGE                      | Applicative  | integer   |  Flash Variable| Defines how many pages should be used for storing flash variables. Use must make sure not to overwrite code.
 *    PROTECTED_LOADER_FLASH_SIZE             | Applicative  | boolean   |  Flash Layout  | forbid bypass of check size before loading (legacy loader)
 *    LIMIT_FLASH_SIZE                        | Applicative  | integer   |  Flash Layout  | Override definition of system flash size (for a value lesser than used for compile)
 *    FLASH_LAYOUTx                           | Applicative  | symbol    |  Flash Layout  | Declare flash layout section x can be 1 to 4 (see FLASH_LAYOUT)
 *    FLASH_LAYOUTx_SIZE                      | Applicative  | symbol    |  Flash Layout  | Declare flash layout section x can be 1 to 4 (see FLASH_LAYOUT)
 *    STACK_SIZE                              | Applicative  | integer   |  Stack         | System stack size in bytes
 *    TD_STACK_PROTECT                        | Applicative  | boolean   |  Stack         | Enable stack protection at startup
 *    TD_TRAP_RESET_CODE                      | Applicative  | boolean   |  Trap          | Reset only trap
 *    TD_TRAP_MINI_CODE                       | Applicative  | boolean   |  Trap          | Remove standard printf trap handler, replace with minimal trap
 *    TD_TRAP_PRINTF_CODE                     | Applicative  | boolean   |  Trap          | Standard printf trap handler
 *    TD_TRAP_FLASH_CODE                      | Applicative  | boolean   |  Trap          | Remove standard printf trap handler, replace with flash trap
 *    TD_TRAP_FLASH_PRINTF_CODE               | Applicative  | boolean   |  Trap          | Add flash handler with standard printf handler.
 *    TD_ALL_DUMP_REMOVE_CODE                 | Applicative  | boolean   |  Dump          | Remove all xxx_DUMP code (same as enable one by one)
 *    TD_GPIO_DUMP_REMOVE_CODE                | Applicative  | boolean   |  Dump          | Remove TD_GPIO_Dump() code
 *    TD_RF_DUMP_REMOVE_CODE                  | Applicative  | boolean   |  Dump          | Remove TD_RF_Dump() code
 *    TD_IRQ_DUMP_REMOVE_CODE                 | Applicative  | boolean   |  Dump          | Remove TD_GPIO_Dump() code
 *    TD_SCHEDULER_DUMP_REMOVE_CODE           | Applicative  | boolean   |  Dump          | Remove TD_SCHEDULER_Dump() code
 *    TD_GEOLOC_DUMP_REMOVE_CODE              | Applicative  | boolean   |  Dump          | Remove TD_GEOLOC_Dump() code
 *    TD_SPILOCK_DUMP_REMOVE_CODE             | Applicative  | boolean   |  Dump          | Remove TD_SPILock_Dump() code
 *    TD_ACCELERO_DUMP_REMOVE_CODE            | Applicative  | boolean   |  Dump          | Remove TD_ACCELERO_Dump() code
 *    TD_SENSOR_DUMP_REMOVE_CODE              | Applicative  | boolean   |  Dump          | Remove TD_SE?SOR_Dump() code
 *
 *    The following parameters are used for setting peripheral device configuration:
 *
 *    Define                                  | Level        | Type      |      Usage     | Description
 *    :---------------------------------------|:------------:|:---------:|:--------------:|:--------------------------------------------------------------------
 *    ACCELERO_CS_PORT                        | Chip         | gpioPortx |  Accelerometer | Set port for accelerometer CS
 *    ACCELERO_CS_BIT                         | Chip         | number    |  Accelerometer | Set bit for accelerometer CS
 *    ACCELERO_POWER_PORT                     | Chip         | gpioPortx |  Accelerometer | Set port for accelerometer power
 *    ACCELERO_POWER_BIT                      | Chip         | number    |  Accelerometer | Set bit for accelerometer power
 *    ACCELERO_IRQ_PORT                       | Chip         | gpioPortx |  Accelerometer | Set port for accelerometer IRQ
 *    ACCELERO_IRQ_BIT                        | Chip         | number    |  Accelerometer | Set bit for accelerometer IRQ
 *    ACCELERO_SKIP_INIT                      | Applicative  | boolean   |  Accelerometer | Skip whole accelerometer initialization
 *    ACCELERO_POLLING_MODE                   | Applicative  | boolean   |  Accelerometer | Force using polling mode (for processing data)
 *    ACCELERO_SPI_SPEED                      | Applicative  | integer   |  Accelerometer | Accelerometer SPI bus speed (in Hz)
 *    ACCELERO_SPI_MODE                       | Applicative  | integer   |  Accelerometer | Accelerometer SPI bus clock mode
 *    ACCELERO_SPI_BUS                        | Applicative  | integer   |  Accelerometer | Accelerometer SPI bus number
 *    BLE_POWER_PORT                          | Chip         | gpioPortx |  BLE           | Set port for BLE power
 *    BLE_POWER_BIT                           | Chip         | number    |  BLE           | Set bit for BLE power
 *    BLE_RESET_PORT                          | Chip         | gpioPortx |  BLE           | Set port for BLE RESET
 *    BLE_RESET_BIT                           | Chip         | number    |  BLE           | Set bit for BLE RESET
 *    BLE_UART                                | Chip         | const     |  BLE           | Set UART device to use for BLE
 *    BLE_UART_CONF                           | Chip         | integer   |  BLE           | Set UART device position for BLE
 *    BLE_UART_PRELOAD_SPEED                  | Applicative  | integer   |  BLE           | Set UART speed for preload synchronizing
 *    BLE_UART_SPEED                          | Applicative  | integer   |  BLE           | Set UART speed for loading the firmware
 *    FLASH_CS_PORT                           | Chip         | gpioPortx |  Flash         | Set port for external Flash CS
 *    FLASH_CS_BIT                            | Chip         | number    |  Flash         | Set bit for external Flash CS
 *    FLASH_POWER_PORT                        | Chip         | gpioPortx |  Flash         | Set port for external Flash Flash power
 *    FLASH_POWER_BIT                         | Chip         | number    |  Flash         | Set bit for external Flash power
 *    FLASH_IRQ_PORT                          | Chip         | gpioPortx |  Flash         | Set port for external Flash IRQ
 *    FLASH_IRQ_BIT                           | Chip         | number    |  Flash         | Set bit for external Flash IRQ
 *    FLASH_SPI_MODE                          | Applicative  | integer   |  Flash         | Flash SPI bus clock mode
 *    FLASH_SPI_BUS                           | Applicative  | integer   |  Flash         | Flash SPI bus number
 *    GPS_CS_PORT                             | Chip         | gpioPortx |  GPS      r    | Set port for GPS CS
 *    GPS_CS_BIT                              | Chip         | number    |  GPS           | Set bit for GPS CS
 *    GPS_IRQ_PORT                            | Chip         | gpioPortx |  GPS           | Set port for GPS IRQ
 *    GPS_IRQ_BIT                             | Chip         | number    |  GPS           | Set bit for GPS IRQ
 *    GPS_RESET_PORT                          | Chip         | gpioPortx |  GPS           | Set port for GPS RESET
 *    GPS_RESET_BIT                           | Chip         | number    |  GPS           | Set bit for GPS RESET
 *    GPS_VBCKP_PORT                          | Chip         | gpioPortx |  GPS           | Set port for GPS VBCKP
 *    GPS_VBCKP_BIT                           | Chip         | number    |  GPS           | Set bit for GPS VBCKP
 *    GPS_VIO_PORT                            | Chip         | gpioPortx |  GPS           | Set port for GPS VIO
 *    GPS_VIO_BIT                             | Chip         | number    |  GPS           | Set bit for GPS VIO
 *    GPS_SPI_BUS                             | Applicative  | integer   |  GPS           | GPS SPI bus number
 *    MAGNETO_CS_PORT                         | Chip         | gpioPortx |  Magnetometer  | Set port for magnetometer CS
 *    MAGNETO_CS_BIT                          | Chip         | number    |  Magnetometer  | Set bit for magnetometer CS
 *    MAGNETO_POWER_PORT                      | Chip         | gpioPortx |  Magnetometer  | Set port for magnetometer Flash power
 *    MAGNETO_POWER_BIT                       | Chip         | number    |  Magnetometer  | Set bit for magnetometer power
 *    MAGNETO_IRQ_PORT                        | Chip         | gpioPortx |  Magnetometer  | Set port for magnetometer IRQ
 *    MAGNETO_IRQ_BIT                         | Chip         | number    |  Magnetometer  | Set bit for magnetometer IRQ
 *    MAGNETO_SPI_MODE                        | Applicative  | integer   |  Magnetometer  | Magnetometer SPI bus clock mode
 *    MAGNETO_SPI_BUS                         | Applicative  | integer   |  Magnetometer  | Magnetometer SPI bus number
 *    PRESSURE_CS_PORT                        | Chip         | gpioPortx |  Pressure      | Set port for pressure sensor CS
 *    PRESSURE_CS_BIT                         | Chip         | number    |  Pressure      | Set bit for pressure sensor CS
 *    PRESSURE_POWER_PORT                     | Chip         | gpioPortx |  Pressure      | Set port for pressure sensor Flash power
 *    PRESSURE_POWER_BIT                      | Chip         | number    |  Pressure      | Set bit for pressure sensor power
 *    PRESSURE_IRQ_PORT                       | Chip         | gpioPortx |  Pressure      | Set port for pressure sensor IRQ
 *    PRESSURE_IRQ_BIT                        | Chip         | number    |  Pressure      | Set bit for pressure sensor IRQ
 *    PRESSURE_SPI_MODE                       | Applicative  | integer   |  Pressure      | Pressure sensor SPI bus clock mode
 *    PRESSURE_SPI_BUS                        | Applicative  | integer   |  Pressure      | Pressure sensor SPI bus number
 *    RADIO_CS_PORT                           | Chip         | gpioPortx |  RF            | Set port for RF CS.
 *    RADIO_CS_BIT                            | Chip         | integer   |  RF            | Set bit for RF CS
 *    RADIO_IRQ_PORT                          | Chip         | gpioPortx |  RF            | Set port for RF IRQ
 *    RADIO_IRQ_BIT                           | Chip         | integer   |  RF            | Set bit for RF IRQ
 *    RADIO_INFO_PIN                          | Chip         | integer   |  RF            | Set RF chip GPIO number for RF INFO
 *    RADIO_INFO_PORT                         | Chip         | gpioPortx |  RF            | Set port for RFINFO
 *    RADIO_INFO_BIT                          | Chip         | integer   |  RF            | Set bit for RF INFO
 *    POWER_CRYSTAL_PORT                      | Chip         | gpioPortx |  RF            | Set port for RF TCXO
 *    POWER_CRYSTAL_BIT                       | Chip         | integer   |  RF            | Set bit for RF TCXO
 *    SHTD_PORT                               | Chip         | gpioPortx |  RF            | Set port for RF Shutdown
 *    SHTD_BIT                                | Chip         | integer   |  RF            | Set bit for RF Shutdown
 *    RADIO_POWER_PORT                        | Chip         | gpioPortx |  RF            | Set port for RF power
 *    RADIO_POWER_BIT                         | Chip         | integer   |  RF            | Set bit for RF power
 *    RADIO_POWER_TIMER                       | Chip         | integer   |  RF            | Set RF power timer
 *    RADIO_POWER_TIMER_CC                    | Chip         | integer   |  RF            | Set RF power timer capture channel
 *    RADIO_INT_PA_TX_PORT                    | Chip         | gpioPortx |  RF            | Set port for internal PA TX input from RF chip
 *    RADIO_INT_PA_TX_BIT                     | Chip         | integer   |  RF            | Set bit for internal PA TX input from RF chip
 *    RADIO_PA_TX_PORT                        | Chip         | gpioPortx |  RF            | Set port for external PA TX output
 *    RADIO_PA_TX_BIT                         | Chip         | integer   |  RF            | Set bit for external PA TX output
 *    RADIO_PA_RX_LNA_PORT                    | Chip         | gpioPortx |  RF            | Set port for external LNA RX output
 *    RADIO_PA_RX_LNA_BIT                     | Chip         | integer   |  RF            | Set bit for external LNA RX output
 *    RADIO_PA_ON_PORT                        | Chip         | gpioPortx |  RF            | Set port for external PA power output
 *    RADIO_PA_ON_BIT                         | Chip         | integer   |  RF            | Set bit for external PA power output
 *    RADIO_BOOT_RF_SHUTDOWN                  | Applicative  | integer   |  RF            | Control if radio is totally shutdown at boot end, or just sleeped (default : do shutdown)
 *    FORCE_RADIO_RESET                       | Applicative  | boolean   |  RF            | Force RF reset at each reset
 *    RADIO_USE_TCXO                          | Chip         | boolean   |  RF            | Use crystal instead of TCXO
 *    USE_FREQUENCY_CAL                       | Applicative  | boolean   |  RF            | Use system frequency correction for TCXO
 *    RADIO_PA_POWER                          | Applicative  | integer   |  RF            | Default RF power (dBm)
 *    RADIO_PA_MAX                            | Applicative  | integer   |  RF            | Maximum RF PA value (raw)
 *    RADIO_XO_TUNE                           | Chip         | integer   |  RF            | Tune value for crystal capacitor
 *    TD_RF_SI4461_CLASS_E                    | Applicative  | boolean   |  RF            | Define to 1 to use square wave mode (class E - 16dB) on si4461
 *    TD_RF_PA_CUSTOM                         | Applicative  | integer   |  RF            | Custom PA mode (raw)
 *    ITU_ISM_REGION                          | Applicative  | integer   |  RF            | Set ITU ISM region (1: ETSI, 2: FCC)
 *    TD_RF_POWER_CALCULATION                 | Applicative  | function  |  RF            | Callback to defined to overload PA index calculation. Don't use if you don't know what you're doing!
 *    TD_RF_POWER_CALCULATION_COEF            | Applicative  | integer   |  RF            | Voltage to PA index table. To be defined according to measurement in order to ensure constant dB output.
 *    RADIO_PATCH_BLOB                        | Applicative  | array     |  RF            | Array containing RF chip patch blob
 *    RADIO_PATCH_SIZE                        | Applicative  | integer   |  RF            | Size of RF chip patch in bytes
 *    RADIO_INIT_DATA                         | Chip         | array     |  RF            | Initial configuration for RF chip GPIO pins
 *
 *    The following parameters are used for setting character I/O configuration:
 *
 *    Define                                  | Level        | Type      |      Usage     | Description
 *    :---------------------------------------|:------------:|:---------:|:--------------:|:--------------------------------------------------------------------
 *    TD_UART_COUNT                           | Applicative  | integer   |  UART          | UART used. Use of USART1 always requires 2 UARTs
 *    LEUART_DEVICE                           | Board        | const     |  UART          | Set LEUART device to use
 *    LEUART_LOCATION                         | Board        | integer   |  UART          | Set LEUART pin location (LEUART_ROUTE_LOCATION_LOCx)
 *    LEUART_SPEED                            | Board        | integer   |  UART          | Set LEUART speed
 *    ALLOW_LEUART_LOW_BAUDRATE_ON_HFRCO      | Applicative  | boolean   |  UART          | If set, permit to use LEUART with low baudrate (< 9600) event if an other LEUART is at baudrate>9600 (will use HFRCO, and not be compatible in reception with EM2 mode without break)
 *    TD_STREAM_COUNT                         | Applicative  | integer   |  Stream        | Number of Stream to use.
 *    TD_STREAM_FIFO_SIZE                     | Applicative  | integer   |  Stream        | Size of fifo (read) in each stream
 *
 *    The following parameters are used for setting software module configuration:
 *
 *    Define                                  | Level        | Type      |      Usage     | Description
 *    :---------------------------------------|:------------:|:---------:|:--------------:|:--------------------------------------------------------------------
 *    MANUFACTURER                            | Applicative  | string    |  AT parser     | Manufacturer name
 *    PRODUCT                                 | Applicative  | string    |  AT parser     | Product name
 *    HARDWARE_VERSION                        | Applicative  | string    |  AT parser     | Hardware version
 *    SOFTWARE_VERSION                        | Applicative  | string    |  AT parser     | Software version
 *    RELEASE_DATE                            | Applicative  | string    |  AT parser     | Release date
 *    SERIAL_NUMBER                           | Applicative  | string    |  AT parser     | Serial number
 *    AT_LEUART_PERSIST                       | Applicative  | boolean   |  AT parser     | Set to true, permit to AT_CORE to restore System UART settings
 *    AT_ARG_CASE_SENSITIVE                   | Applicative  | boolean   |  AT parser     | Set to true, permit AT_Tokenize to return a case sensitive buffer ('A' to 'z')
 *    AT_PERSIST_SIZE                         | Applicative  | integer   |  AT parser     | Max persist size for AT commands
 *    PRODUCT_EARLY_DEBUG                     | Applicative  | integer   |  Startup       | Set to true, activate early debug (in main @115200,8N1)
 *    PRODUCT_EARLY_DEBUG_CUSTOM              | Applicative  | integer   |  Startup       | Function that will be called in early debug mode, just after stack protect and bootloader
 *    PRODUCT_BOOTLOADER_SKIP                 | Applicative  | integer   |  Bootloader    | Set to true, skip bootloader (keep IO Init, etc)
 *    PRODUCT_BOOTLOADER_FULL_SKIP            | Applicative  | integer   |  Bootloader    | Set to true, skip bootloader (remove all initializations, user code is responsible for all initializations : clock, GPIO, etc)
 *    PRODUCT_UART_BOOTLOADER_SKIP            | Applicative  | integer   |  Bootloader    | Set to true, skip UART bootloader
 *    PRODUCT_TYPE                            | Chip         | integer   |  Bootloader    | 0:TD1202, 8:TD1204, 9:TD1205, 10:TD1208 (automatically set if not set and MODULE_REVISION defined)
 *    PRODUCT_INIT_DATA                       | Board        | array     |  Bootloader    | Others port to initialize (see Init Data)
 *    PRODUCT_LED_POLARITY                    | Board        | integer   |  Bootloader    | 0:led lit when TIM2 set to VDD, 1:led lit when TIM2 set to GND
 *    PRODUCT_LED_PORT                        | Board        | gpioPortx |  Bootloader    | Set port of LED (default : TIM2_PORT), 0xFF : no led
 *    PRODUCT_LED_BIT                         | Board        | integer   |  Bootloader    | Set bit of LED (default : TIM2_BIT)
 *    PRODUCT_LED_DRIVE                       | Board        | const     |  Bootloader    | Set LED drive current
 *    PRODUCT_LED_BLINK                       | Applicative  | boolean   |  Bootloader    | Set to true, blink LED during flash of each sector (default : false)
 *    PRODUCT_BOOTLOADER_CHANNEL              | Applicative  | integer   |  Bootloader    | Bootloader channel (def : 255), other not actually handled by TD_Loader
 *    TD_LOADER_TRANSMITTER                   | Applicative  | define    |  Bootloader    | DEPRECIATED : contact TD if needed
 *    TD_GEOLOC_RAW_OUTPUT                    | Applicative  | boolean   |  GPS           | Define to true to obtain raw GPS chip output value. Can be combine to Ubx7Parse function to control gps via u-center.
 *    LAN_PERIOD                              | Applicative  | integer   |  LAN           | Receive sampling period (timer unit)
 *    LAN_THRESHOLD                           | Applicative  | integer   |  LAN           | RSSI Threshold in LAN transmission mode (Listen Before Talk) in 0.5dB step from -126dB max sensitivity (default : 32 aka -110dB)
 *    LAN_THRESHOLD_RX                        | Applicative  | integer   |  LAN           | RSSI Threshold in windowed LAN reception mode in 0.5dB step from -126dB max sensitivity (default : 32 aka -110dB)
 *    LAN_LBT_COUNT_MAX                       | Applicative  | integer   |  LAN           | Number of free LBT period before sending
 *    LAN_ADDRESS_SIZE                        | Applicative  | integer   |  LAN           | LAN address size (in bits default 24)
 *    LAN_CHECK_CALLBACK_TIME                 | Applicative  | integer   |  LAN           | LAN callback timeout check (in 32 kHz ticks default 175 ms)
 *    PRINTF_INT64_SUPPORT                    | Applicative  | boolean   |  Printf        | If defined, handle int64 support with %ld, %lX, %ud else will be legacy 32 bit support (size optimization)
 *    TD_SCHEDULER_MAX_TIMER                  | Applicative  | integer   |  Scheduler     | Total number of Scheduler Timers
 *    TD_SCHEDULER_MAX_QUEUE                  | Applicative  | integer   |  Scheduler     | Total number of timer call-back (not IRQ) in pending queue
 *    TD_SENSOR_TRANSMITTER_MAX_TRANSMIT      | Applicative  | integer   |  Sensor        | Max pending SIGFOX message transmission count
 *    TD_SENSOR_TRANSMITTER_MAX_RETRANSMIT    | Applicative  | integer   |  Sensor        | Max pending SIGFOX message retransmission count
 *    TD_SENSOR_GATEWAY_MAX_DEVICE            | Applicative  | integer   |  Sensor        | Max attached devices to Gateway (Gateway itself count for 1 device)
 *    TD_SENSOR_MAX_SWITCH                    | Applicative  | integer   |  Sensor        | Max number of handled switches
 *    TD_SENSOR_MAX_SWITCH_EVENT              | Applicative  | integer   |  Sensor        | Max number of switches event queue (for non IRQ processing)
 *
 *    The following parameters are used for setting miscellaneous configuration:
 *
 *    Define                                  | Level        | Type      |      Usage     | Description
 *    :---------------------------------------|:------------:|:---------:|:--------------:|:--------------------------------------------------------------------
 *    PREVENT_FREE_BACKGROUND_CYCLE           | Applicative  | boolean   |  System        | Use at your own risk! Do not launch free background cycle when not explicitly asked by BackgroundRoundWanted (less power, but more error prone!)
 *    EFM_ASSERT_NOT_TRAPPED                  | Applicative  | boolean   |  EFM32         | Do not trap on failed EFM32 ASSERT (to use the EFM32 assert system, DEBUG_EFM_USER or DEBUG_EFM must be defined when compiling emlib)
 *    MAX_USER_SPI_ID                         | Board        | integer   |  SPI           | Max number of SPI devices
 *    RDEBUG_FULL_PKT                         | Applicative  | bool      |  RadioDebug    | Only get full packets (if FIFO is not available, do not store)
 *    DUMP_PRODUCT_CONFIG                     | Applicative  | integer   |  CompileTime   | If set, do a dump with #info of system configuration used
 *
 *    These parameters are for General code use. It will force to include
 *    (or remove) blocks of code.
 *    In general use case theses parameters are not useful, they are just here
 *    for special use case
 *
 *    Define                                  | Description
 *    :---------------------------------------|-------------------------------
 *    TD_GEOLOC_USE_CODE                      | Force to use/not use the libtdgeoloc dynamic data
 *    TD_SENSOR_USE_CODE                      | Force to use/not use the libtdsensor dynamic data
 *    TD_SIGFOX_USE_TEST_CARRIER_CODE         | Add the Sigfox RF test carrier code
 *    TD_SIGFOX_TRANSMIT_DEFAULT              | Default Sigfox transmit mode (can be omitted), use booster if found/paired, local transmit if no booster paired
 *    TD_SIGFOX_TRANSMIT_FORWARD_ONLY         | Use forward mode only. Local Sigfox ID module will never be used, only booster ID will be used
 *    TD_SIGFOX_TRANSMIT_LOCAL_ONLY           | Prevent use of any proxy. Use only local Sigfox ID
 *    TD_SIGFOX_TRANSMIT_NONE                 | Don't use Sigfox transmit code at all (remove all transmit & receive code)
 *    TD_SIGFOX_RECEIVE_NONE                  | Don't use Sigfox receive code at all (remove all receive code)
 *    TD_SIGFOX_USE_RECEIVE_TEST_CODE         | Use Sigfox receive test code
 *
 *    The following parameters are useful for reducing code size. If you remove
 *    a feature and use it after that, it will throw a trap.
 *
 *    Define                                  | Description
 *    :---------------------------------------|-------------------------------
 *    TD_SENSOR_LAN_REMOVE_CODE               | Remove Sensor LAN code
 *    TD_SENSOR_GATEWAY_REMOVE_CODE           | Remove Sensor Gateway code
 *    TD_SENSOR_TRANSMITTER_REMOVE_CODE       | Remove Sensor Transmitter code
 *    TD_SENSOR_DEVICE_REMOVE_CODE            | Remove Sensor Device code
 *    TD_SENSOR_TRANSMITTER_DUTY_CYCLE_CODE   | Remove Sensor Transmitter ETSI EN 300-220 duty cycle monitor code
 *    TD_SENSOR_MONITOR_REMOVE_CODE           | Remove Sensor Switch Monitor code (include EVENT and SWITCH)
 *    TD_SENSOR_MONITOR_EVENT_REMOVE_CODE     | Remove Sensor Switch Event Monitor code
 *    TD_SENSOR_MONITOR_SWITCH_REMOVE_CODE    | Remove Sensor Switch Event Switch code
 *    TD_SENSOR_MONITOR_CONFIG                | Get a list of 'ored' value in (TD_SENSOR_MONITOR_BOOT,TD_SENSOR_MONITOR_BATT,TD_SENSOR_MONITOR_TEMP,TD_SENSOR_MONITOR_CONNECT,TD_SENSOR_MONITOR_KEEPALIVE,TD_SENSOR_MONITOR_SWITCH) of monitor to activate. To remove all, use TD_SENSOR_MONITOR_REMOVE_CODE
 *    TD_GEOLOC_LOGGER_REMOVE_CODE            | Remove Geolocation Flash logger code
 *    TD_SIGFOX_REMOVE_CODE                   | [Depreciated, use SIGFOX_TRANSMIT_xxx] Remove all SIGFOX code
 *    TD_SIGFOX_PROXY_REMOVE_CODE             | [Depreciated, use SIGFOX_TRANSMIT_xxx] Remove Proxy handling in Sigfox transmission
 *    TD_SIGFOX_PROXY_USE_FORWARD             | [Depreciated, use SIGFOX_TRANSMIT_xxx] Proxy use forward (proxy sid) instead of relay (module sid).
 *    TD_TRAP_RESET_CODE                      | Reset only trap
 *    TD_TRAP_MINI_CODE                       | Remove standard printf trap handler, replace with minimal trap
 *    TD_TRAP_PRINTF_CODE                     | Standard printf trap handler
 *    TD_TRAP_FLASH_CODE                      | Remove standard printf trap handler, replace with flash trap
 *    TD_TRAP_FLASH_PRINTF_CODE               | Add flash handler with standard printf handler.
 *    TD_ALL_DUMP_REMOVE_CODE                 | Remove all xxx_DUMP code (same as enable one by one)
 *    TD_GPIO_DUMP_REMOVE_CODE                | Remove TD_GPIO_Dump() code
 *    TD_RF_DUMP_REMOVE_CODE                  | Remove TD_RF_Dump() code
 *    TD_IRQ_DUMP_REMOVE_CODE                 | Remove TD_GPIO_Dump() code
 *    TD_SCHEDULER_DUMP_REMOVE_CODE           | Remove TD_SCHEDULER_Dump() code
 *    TD_GEOLOC_DUMP_REMOVE_CODE              | Remove TD_GEOLOC_Dump() code
 *    TD_SPILOCK_DUMP_REMOVE_CODE             | Remove TD_SPILock_Dump() code
 *    TD_ACCELERO_DUMP_REMOVE_CODE            | Remove TD_ACCELERO_Dump() code
 *    TD_SENSOR_DUMP_REMOVE_CODE              | Remove TD_SE?SOR_Dump() code
 *    TD_SCHEDULER_REMOVE_CODE                | Remove TD_SCHEDULER code
 *    TD_REMOVE_UART_LOADER                   | Remove UART Loader (on LEUART, location 0)
 *
 *    Note : for flash trap handler, remember to call TD_TRAP_DirectToFlash()
 *    function in your code.
 *    Don't forget to remove TD_TRAP_DirectToFlash() to reclaim space if you do
 *    not use it.
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
 *   PIP(p,b,m,v): initialize one port
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
 *   PIA(c,src,sig) : Asynchronous PRS route config
 *      c : PRS channel
 *      src : PRS source
 *      sig : PRS signal
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

// Board configuration
#include "td_config_board.h"

// Chip configuration, must come after chip configuration
#include "td_config_chip.h"

// MCU features and memory configuration
#include "td_config_variables.h"
#include "td_config_flash_layout.h"
#include "td_config_stack.h"
#include "td_config_trap.h"
#include "td_config_dump.h"

// Peripheral device configuration
#include "td_config_accelerometer.h"
#include "td_config_ble.h"
#include "td_config_flash.h"
#include "td_config_gps.h"
#include "td_config_magnetometer.h"
#include "td_config_pressure.h"
#include "td_config_radio.h"

// Character I/O configuration
#include "td_config_uart.h"
#include "td_config_stream.h"

// Software module configuration
#include "td_config_at_parser.h"
#include "td_config_bootloader.h"
#include "td_config_geoloc.h"
#include "td_config_lan.h"
#include "td_config_printf.h"
#include "td_config_scheduler.h"
#include "td_config_sensor.h"
#include "td_config_sigfox.h"

/**************************************************************************//**
 * Miscellaneous configuration
 ******************************************************************************/

#ifndef PREVENT_FREE_BACKGROUND_CYCLE
#define PREVENT_FREE_BACKGROUND_CYCLE	0
#endif

#ifdef PRODUCT_UI
const uint8_t TD_UI[] = {PRODUCT_UI, 0xFF};
uint8_t CONFIG_TD_UI_Id[PRODUCT_UI_COUNT + 1];
const uint8_t CONFIG_TD_UI_COUNT = PRODUCT_UI_COUNT + 1;
#define PRODUCT_UI_INIT	TD_TOOLS_Full_Init(TD_UI);
#else
#define PRODUCT_UI_INIT
#endif

#ifndef EFM_ASSERT_NOT_TRAPPED
#define EFM_ASSERT_NOT_TRAPPED	0
#endif

#ifndef MAX_USER_SPI_ID
#define MAX_SPI_ID				MAX_SYSTEM_SPI_ID
#else
#define MAX_SPI_ID				(MAX_SYSTEM_SPI_ID + MAX_USER_SPI_ID)
#endif

	bool const CONFIG_PREVENT_FREE_BACKGROUND_CYCLE = PREVENT_FREE_BACKGROUND_CYCLE;
	uint8_t const CONFIG_EFM_ASSERT_NOT_TRAPPED = EFM_ASSERT_NOT_TRAPPED;
	unsigned char const CONFIG_MAX_SPI_ID = MAX_SPI_ID;
	TD_SPI_Conf_t __BOOTVARIABLE TD_SPI_Conf_[MAX_SPI_ID + 1];
	TD_SPI_Conf_t *TD_SPI_Conf = TD_SPI_Conf_;


#ifndef RDEBUG_FULL_PKT
#define RDEBUG_FULL_PKT	0
#endif

	bool const CONFIG_RDEBUG_FULL_PKT = RDEBUG_FULL_PKT;

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
#pragma message(VAR_NAME_VALUE(PRODUCT_BOOTLOADER_CHANNEL_VALUE))
#pragma message("-------------------------")
#endif

#ifdef __cplusplus
}
#endif

#else
#error("td_config.h MUST NOT be included multiple times")
#endif // __TD_CONFIG_H
/** @endcond */

