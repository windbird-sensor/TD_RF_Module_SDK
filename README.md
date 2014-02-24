Welcome to TD RF Module SDK (powered by Telecom Design)
=======================================================

The TD RF Module SDK provides a complete Rapid Embedded Development Environment for the TD RF modules.

The TD RF Module SDK contains all the Eclipse projects, static libraries, exampels with soruce codes
and auto-generated API documentation to develop for the TD RF modules.

Further details on the TD RF Module SDK can be found at <https://developers.insgroup.fr/>

We hope you enjoy using the TD RF Module SDK!

_The Telecom Design Team_

Getting the Sources
===================

The SDK source code and examples are no longer distributed with the TD RF Module SDK Tools zip file but
are available through the <https://github.com/Telecom-Design/TD_RF_Module_SDK> Github repository
once you register your Telecom Design Evaluation Board (EVB) by following the steps described on <https://developers.insgroup.fr/>.

The following steps detail how to download the source code and import all projects into the Eclipse environment.

  - Navigate to the `"C:\TD\TD_RF_Module_SDK-v4.0.0\eclipse"` folder and double-click on the `"eclipse.exe"` icon
  - Open the `"File"` menu and select the `"Import..."` item.
  - In the `"Import"` dialog, unfold the `"Git"` folder by clicking on the `"+"` sign left to it, select the `"Projects from Git"` item and click on the `"Next >"` button
  - In the `"Import Projects from Git"`, select the `"URI"` icon ad click on the `"Next >"` button
  - Enter `The Github repository URL "https://github.com/Telecom-Design/TD_RF_Module_SDK.git"`in the `"URI"` field
  - Enter your Github username and password in the `"User:"` and `"Password:"` fields, respectively and click on the `"Next >"` button
  - Check the `"Master"` branch box and click on the `"Next >"` button
  - Enter `"C:\TD\TD_RF_Module_SDK-v4.0.0\Github\TD_RF_Module_SDK"` in the `"Directory:"` field and click on the `"Next >"` button
  - Check the `"Import existing projects"` radio button and click on the `"Next >"` button
  - Click on the `"Finish"` button. The Git import will take place, this may take a while

All the available libraries and examples should now be available in the Project Explorer panel.

Release Notes
=============

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

  - The IDE is now based on the Eclipse “Juno” SR2 release.
  - The compilation tools are now based on GCC 4.5.1.
  - Support for CMSIS v2.0 (Cortex Microcontroller Software Interface Standard), including DSPLib