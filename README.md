Welcome to TD RF Module SDK (powered by TD next, a division of Telecom Design)
=======================================================

The TD RF Module SDK provides a complete Rapid Embedded Development Environment for the TD RF modules and reference
designs.

The TD RF Module SDK contains all the Eclipse projects, static libraries, examples with source codes
and auto-generated API documentation to develop for the TD RF modules and reference designs.

Further details on the TD RF Module SDK can be found at <http://rfmodules.td-next.com/>

We hope you enjoy using the TD RF Module SDK!

_The TD next Team_

Getting the Sources
===================

The SDK source code and examples are no longer distributed with the TD RF Module SDK Tools zip file but
are available through the <https://github.com/Telecom-Design/TD_RF_Module_SDK> Github repository
once you register your Telecom Design Evaluation Board (EVB) by following the steps described on
<http://rfmodules.td-next.com/sdk/>.

The stand-alone Eclipse Package which should be used to compile the source can be found here:
<https://developers.insgroup.fr/releases/2014/02/25/sdk-400-version-available/index.html>

The following steps detail how to download the source code and import all projects into the Eclipse
environment.

  1. Navigate to the `"C:\TD\TD_RF_Module_SDK-v6.0.0\eclipse"` folder and double-click on the
     `"eclipse.exe"` icon
  2. Open the `"File"` menu and select the `"Import..."` item.
  3. In the `"Import"` dialog, unfold the `"Git"` folder by clicking on the `"+"` sign left to it,
     select the `"Projects from Git"` item and click on the `"Next >"` button
  4. In the `"Import Projects from Git"`, select the `"URI"` icon ad click on the `"Next >"` button
  5. Enter The Github repository URL `"https://github.com/Telecom-Design/TD_RF_Module_SDK.git"`
     in the `"URI"` field
  6. Enter your Github username and password in the `"User:"` and `"Password:"` fields,
     respectively and click on the `"Next >"` button
  7. Check the `"Master"` branch box and click on the `"Next >"` button
  8. Enter `"C:\TD\TD_RF_Module_SDK-v6.0.0\Github\TD_RF_Module_SDK"` in the `"Directory:"` field and
     click on the `"Next >"` button
  9. Check the `"Import existing projects"` radio button and click on the `"Next >"` button
  10. Click on the `"Finish"` button. The Git import will take place, this may take a while

All the available libraries and examples should now be available in the Project Explorer panel.

Organizing the Sources
======================

By default, all projects are presented in Eclipse at the same level without particular organization
except that they are sorted alphabetically.

In order to have a more logical organization, we must import an Eclipse "Working Set" that will
provide a grouping of projects by categories.

To do so, launch Eclipse by navigating to the `"C:\TD\TD_RF_Module_SDK-v6.0.0\eclipse"` folder and
double-click on the `"eclipse.exe"` icon (if not already done) and:

  1. Open the `"File"` menu and select the `"Import..."` item.
  2. In the `"Import"` dialog, unfold the `"General"` folder by clicking on the `"+"` sign left to
     it, select the `"Working Sets"` item and click on the `"Next >"` button
  3. Enter `"C:\TD\TD_RF_Module_SDK-v6.0.0\Github\TD_RF_Module_SDK\TD_RF_Module_SDK.wst"` in the
     `"Browse..."` field, check all working sets and click on the `"Finish"` button
  4. Click on the small downwards arrow in the top-right corner of the `"Project Explorer"` panel
     and select the `"Select Working Sets..."` item
  5. In the `"Select Working Sets"` dialog, click on the `"Select All"`, then on the `"OK"` button
  6. Click on the small downwards arrow in the top-right corner of the `"Project Explorer"` panel
     again and select the `"Top Level Elements > Working Sets"` item

All the available libraries and examples should now be better organized in the Project Explorer panel.

Compiling the Sources
=====================

With the exception of the binary-only static libraries in the `"libtddrivers"` and `"libtdrf"`
projects, all deliverables are presented in source form only, and must be compiled to obtain an
executable firmware. To compile properly you must add the gcc path into your Eclipse environment variables
by opening: Window -> Preferences -> C/C++ -> Environment -> Add...

Please add the PATH variable with the following value:

C:\TD\TD_RF_Module_SDK-v6.0.0\gnu\bin

Then if we take the `"blink"` project as an example, here are the steps required to compile it:

  1. In order to avoid unnecessary rebuilds of the common libraries, it is best to set the right
     build configuration for all these libraries: unfold the `"Common_Libraries"` working set in the
     Project Explorer panel on the left side, then select all the projects by clicking on the first one
     in the list, pressing the `"SHIFT"`key, then clicking on the last one in the list
  2. Click on the small downwards arrow right next to the `"Hammer"` icon in the top menu bar and
     select the right build configuration corresponding to your board: `"TD1204"` , `"TD1205"`
     `"TD1208"` or `"TD1508"`, and `"Debug"` for building an executable firmware with integrated symbols
     suitable for source code debugging, or `"Release"` for a stripped down firmware
  3. Unfold the `"Core_Examples"` working set in the Project Explorer panel on the left side, then
     select the `"blink"` project
  4. Click again on the small downwards arrow right next to the `"Hammer"` icon in the top menu bar and
     select the same build configuration as for the libraries above
  5. Compilation of the `"blink"`project and all the required dependencies will take place, which can
     be monitored in the `"Console"` tab of the bottom panel

The same procedure can be used for all the example projects when required.

Flashing / Debugging
====================

Unlike some dedicated embedded Interactive Development Environments (IDEs), Eclipse does not come with
fixed Flash/Debug commands or menu buttons: they need to be added explicitly on a project by project basis.

Fortunately, these Flash/Debug `"Launchers"` can be imported or duplicated to other projects easily:

  1. Open the `"File"` menu and select the `"Import..."` item.
  2. In the `"Import"` dialog, unfold the `"Run/Debug"` folder by clicking on the `"+"` sign left to it,
     select the `"Launch Configurations"` item and click on the `"Next >"` button
  3. Enter `"C:/TD/TD_RF_Module_SDK-v6.0.0/Github/TD_RF_Module_SDK/Eclipse Launchers"` in the `"Browse..."`
     field, check the box in front of the `"Eclipse Launchers"` item and click on the `"Finish"` button
  4. All the default Flash/Debug `"Launchers"` will be added to the top menu `"Bug"` and `"Green Circle
     with With Right Arrow and Briefcase"` icons
  5. If required, you can edit these `"Launchers"` by choosing the "Debug Configurations..."` or
     `"External Tools Configurations..." entries in the menu obtained by the small downwards arrow right
     next to the corresponding `"Bug"` or `"Green Circle with With Right Arrow and Briefcase"` icon
  6. In the dialog windows that opens, you can edit a given configuration directly and click on the
     `"Apply"` and `"Close"` buttons, or you an duplicate it by right-clicking on it and selecting the
     `"Duplicate"` entry in the contextual menu that pops up

Then, to flash a firmware to the TD12xx/TD15xx board:

  1. Select the desired project in the Project Explorer panel on the left side
  2. Right-click on the project and select the `"Build Configurations > Set Active >"` and the desired
     build configuration as explained above
  3. Optionally click on the `"Hammer"` icon in the top menu bar to build the project
  4. Click on the  the small downwards arrow right next to the `"Green Circle with With Right Arrow and
     Briefcase"` icon and select `"Flash Selected Project"` in the contextual menu

Similarly, to debug a firmware on the TD12xx/TD15xx board:

  1. Select the desired project in the Project Explorer panel on the left side
  2. Right-click on the project and select the `"Build Configurations > Set Active >"` and the desired
     build configuration as explained above, this must be a `"Debug"`one
  3. If this is the first time you debug this project, you must edit/add a `"Launcher"` by choosing the
     "Debug Configurations..."` entry in the menu obtained by the small downwards arrow right next to the
     `"Bug"` icon
  4. In the dialog windows that opens, you can edit a given configuration directly (you only need to
     change the `"Project:"` and `"C/C++ Application:"` values) and click on the `"Apply"` and `"Close"`
     buttons, or you an duplicate an existing one by right-clicking on it and selecting the `"Duplicate"`
     entry in the contextual menu that pops up, then changing these same values
  5. Click on the  the small downwards arrow right next to the `"Bug"` icon and select the
     `"Debug TD12xx"` or `"Debug TD15xx"` entry in the contextual menu that corresponds to your board
  6. The first time you launch a debug session, Eclipse will prompt you if you want to switch to a new
     `"Debug Perspective"` (a window panel organization purposed for debugging): click on `"Yes"`
  7. The firmware will optionally be built and flashed to the module, the execution will stop early in
     the program execution, giving you an opportunity to single-step, continue, explore variables as
      expected from a source-level debugger
  8. When finished, you can cleanly disconnect from the running application by right-clicking on the
     `"Debug TD12xx"` or `"Debug TD15xx"` entry in the tree under the "Debug" panel, select the
     `"Terminate/Disconnect All"` entry in the contextual menu, clicking on the `"XX"` icon at the top
     of this panel, then clinking on the `"C/C++"` button located at the top right of the main window
     to return to the normal C/C++ edit panel organization

Release Notes
=============
v6.3.2 (April 2016)
-------------------
### Release Notes ###

#### Major Updates ####

#### **libtdcore** ####
  * Added Td_HallEffect module
  * Up AT___PERSIST_SIZE to 220 bytes
  * Added sigfox channel managment error

#### SIGFOX: ####
  * Corrected of spectral quality in EZR FCC configuration
  * Add generic Sigfox test mode
  * Add CW test mode
  * Add macro channel bitmask
  * Macro channels Harmonization according to Sigfox requirement.
  * Library certified by SIGFOX

v6.2.0 (February 2016)
-------------------
Bug Fixes
#### SIGFOX: ####
  - Corrected a bug that may create an irregular ramp down duration
  - Corrected a bug that may not set LAN frequency after a Sigfox transmission
  - Corrected a bug that may disable GPIO interrupts after a Sigfox transmission

v6.1.0 (January 2016)
-------------------
  - This version is a major update of TD RF Module SDK.

### Release Notes ###

#### Bug Fixes ####

#### SIGFOX: ####

v6.0.0 (December 2015)
-----------------
  - This version is a major update of TD RF Module SDK.

### Release Notes ###

#### Bug Fixes ####

#### SIGFOX: ####

  * Corrected a bug that may crash the device after 21285 Sigfox transmissions

#### **libtdcore** ####

  * Corrected drift in voltage measurement

#### Major Updates ####

#### **libtdcore** ####

  * Split link time configuration into multiple files
  * Added a link-time Flash layout mechanism
  * Added stream support
  * Added long long support

#### **libtdrf** ####

  * Added support for TD1508 (FCC)
  * Added support for reference design (both ETSI and FCC)

v5.0.1 (December 2014)
-----------------

  - This release is a minor update of TD RF Module SDK.

### Release Notes ###

#### Bug Fixes ####

#### SIGFOX: ####

  * Corrected a problem causing loss of SIGFOX sequence number after un upgrade from previous SDKs

v5.0.0 (December 2014)
-----------------

  - This release is a major update of TD RF Module SDK.

### Release Notes ###

#### Major Updates ####

#### **libtdcore** ####

  * Added the CAPSENSE module for capacitive touch buttons
  * UART module now handles both LEUART and standard UART

##### **libtdrf** #####

  * Bidirectional (down-link) communication
  * Transparent (use sender's SIGFOX ID) or non-transparent (use proxy's SIGFOX ID) SIGFOX proxy
  * Fast LAN mode: once synchronized, transmitter and receiver can exchange data at maximum speed

##### **libtdtools** #####

  * New library for managing switches and LEDs

#### Minor Updates ####

#### **libtdcore** ####

  * Increased maximum number of AT extensions to 16
  * Now handling the different EFM32 power modes
  * Unified the RF and standard GPIO management

##### **libtdrf** #####

  * Custom transmit (user callback function to write into FIFO) and receive (user callback function to read from FIFO and stop RX)
  * Changed the TD LAN windowed listen period to variable up to 6.7 s instead of fixed 1 s, using the `LAN_PERIOD` and `LAN_ADDRESS_SIZE` configuration parameters

##### **libtdsensor** #####

  * Corresponds to UDM v2.3

##### **libtdgeoloc** #####

  * Now uses the UBlox 7 raw format instead of NMEA by default
  * No longer use Sensor switch monitoring for accelerometer interrupts, have the driver handle it directly for efficiency's sake
  * Added click monitoring
  * Added power mode usage
  * Added GPS autonomous computation handling
  * Added ephemeris management

#### Bug Fixes ####

#### **libtdcore** ####

  * Handles UART overflow gracefully

##### **libtdrf** #####

  - Corrected a timing problem on first frame detected with new base station software upgrade (already corrected in v4.1.0)

##### **libtdsensor** #####

  * Do not set encoded voltage length if no length pointer provided to `TD_SENSOR_EncodeLocalVoltage()` or `TD_SENSOR_EncodeLocalTemperature()`

##### **libtdgeoloc** #####

  * Skipped first meaningless accelerometer data values in callback
  * Fixed a bug in AT$AR= (read accelerometer register)
  * Initialized the first byte in Sensor position data frame
  * Fixed non-volatile accelerometer IRQ flag

v4.1.0 (December 2014)
-----------------

  - This release is a hotfix of the TD RF Module SDK. __Please upgrade all firmwares generated with the SDK 4.0.0 to SDK 4.1.0 as it contains an important fix for compatibility with the SIGFOX network__

### Release Notes ###

#### Minor Updates ####

##### **libtdcore** #####

  - Changed the definition of `T1S`

#### Additions ####

##### **libtdcore** #####

  - Added the function `TD_RTC_SignedTimeDiff()`

##### **libtdrf** #####

###### LAN: ######

  - Changed the TD LAN windowed listen period to variable up to 6.7 s instead of fixed 1 s, using the `LAN_PERIOD` and `LAN_ADDRESS_SIZE` configuration parameters

#### Bug Fixes ####

##### **libtdrf** #####

###### SIGFOX: ######

  - Corrected a timing problem on first frame detected with new base station software upgrade

v4.0.0 (February 2014)
-----------------

  - This release is a major update of TD RF Module SDK.

### Release Notes ###

#### Major Updates ####

  - Supports the TD1204 products with GPS and 3D accelerometer functions
  - Supports modules with SIGFOX ID >= 0x10000 (still retaining compatibility with smaller 16 bit IDs)
  - Now supports SIGFOX protocol V1 by default

#### Minor Updates ####

##### **libtdcore** #####

  - Battery level monitoring (with IRQs) available
  - Optional printf int64 support
  - Stdlib atoll support

#### Additions ####

##### **libtdcore** #####

  - Added watchdog module
  - Added the `AT CORE` module which implements basic "system" AT command
  - Added the `SPI` module to manage sharing of the SPI bus (multiple peripherals on the same SPI bus)
  - Added a link-time `OPTION` mechanism to customize the behavior and/or limits of binary-only objects/libraries
  - Added a fully configurable `TRAP` system to handle System/User code faults
  - Added system dumps debugging tools

##### **libtdsensor** #####

  - Added a retransmission profile system
  - Added a custom BootEvent
  - Added queue handling for message transmission
  - Added a *forward* option (optional gateway)
  - Extension of the **Device** and **Gateway** APIs
  - Extension of the set of Sensor Events

##### **libtdgeoloc** #####

  - First release

##### **libtddrivers** #####

  - First release

#### Bug Fixes ####

##### **libtdcore** #####

###### AT Parser: ######

  - Upon startup, if a parser extension module has no persistent data stored, its store function will be called immediately to fix it
  - Correction on numeric parameter read in AT command (function AT_atoll) on value with leading spaces characters eg. " `123`": the first digit was incorrectly dropped and result set to 23

###### Flash: ######

  - Corrections on "user variables" handling

###### GPIO: ######

  - Modification of behavior in the IRQ handling. List of I/O that actually trigged interrupt (mask) must no longer be read by `GPIO_IntGet()` but is directly passed to the to Hook callback registered for I/O interrupt handling

###### System: ######

  - A fully deterministic, guaranted main loop processing (including the `TD_USER_Loop` function) can be trigged by calling the `TD_WakeMainLoop()` function, all others systems (while(flags) processing) must be removed. `TD_WakeMainLoop()` must be used in user interrupt handlers

###### RTC: ######

  - Major correction on low level function to allow calls in all possible context (background task, IRQs), with all special RTC cases handled
  - Know limitation : a call to `TD_RTC_Delay()` while in IRQ handling will result in an active wait (not energy efficient). However, calling a wait function like `TD_RTC_Delay()` in IRQ context is a bad practice

###### Scheduler: ######

  - Adding a Timer is now possible in IRQ context
  - Major rewrite of the **SCHEDULER** processing to remove some race, full strengthening on `TD_SCHEDULER_AppendIrq()`, `TD_SCHEDULER_Append()`, `TD_SCHEDULER_Remove()` from anywhere in the user code, in any context

###### stdlib: ######

  - atoi now handles correctly signed value

##### **libtdrf** #####

###### RF: ######

  - Bulletproof handling of RF chip mode (sleep/Idle/Receive/Transmit) transitions

###### LAN: ######

  - Major rewrite of LAN functions, it is heavily recommended to not mix devices with a `libtdrf` library older than v4.0.0
  - Better error handling and reporting

###### BOOTLOADER: ######

  - Extension of the bootloader to provide customization parameters: LED status pin location, optional update blink indication, RF channel selection, etc.

v3.0.0 (May 2013)
-----------------

 - First public release

### Release Notes ###

New functionality contained in the TD RF Module SDK 3.0.0 include:

  - The IDE is now based on the Eclipse ?Juno? SR2 release.
  - The compilation tools are now based on GCC 4.5.1.
  - Support for CMSIS v2.0 (Cortex Microcontroller Software Interface Standard), including DSPLib