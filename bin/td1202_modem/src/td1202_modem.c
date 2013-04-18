/******************************************************************************
 * @file td1202.c
 * @brief User Application implementation for TD1202 module.
 * @author Telecom Design S.A.
 * @version 1.0.0
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2012 Telecom Design S.A., http://www.telecom-design.fr</b>
 ******************************************************************************
 *
 * This source code is the property of Telecom Design S.A.
 *
 * This copyright notice may not be removed from the source code nor changed.
 *
 *****************************************************************************/

#include "config.h"
#include <efm32.h>
#include <td_core.h>
#include <td_rtc.h>
#include <td_uart.h>
#include <td_printf.h>
#include <at_parse.h>
#if AT_MFG_TEST
#include <at_mfg_test.h>
#include <rf_mfg_test.h>
#elif RF_ANTENNA_MATCHING
#include <at_mfg_test.h>
#include <rf_cert_test.h>
#elif RF_CEM_RX_TEST || RF_CEM_TX_TEST
#include <at_mfg_test.h>
#include <rf_cem_test.h>
#endif
#if AT_LAN_RF
#include <at_lan.h>
#include <td_lan.h>
#endif
#if AT_RADIO_INFO
#include <at_radio.h>
#endif
#if AT_SIGFOX
#include <at_sigfox.h>
#endif


/*******************************************************************************
 ******************************   DEFINES   ************************************
 ******************************************************************************/

#if RF_ANTENNA_MATCHING
/** Default RF parameters for antenna matching */
TD_RF_param_t RF_Params = {
	868500000,	// Frequency 868.50 MHz
	2400,		// Baudrate  2400 bps
	3000,		// Deviation 3 kHz
	TD_RF_CW,	// Modulation type is Continuous Wave
	14,			// Power level 14 dBm
	TD_RF_LOW	// Data provider continuous low level
};
#endif

/*******************************************************************************
 ******************************  CONSTANTS  ************************************
 ******************************************************************************/

/** AT device Manufacturer */
char const *AT_manufacturer = MANUFACTURER" "PRODUCT;

/** AT hardware revision */
char const *AT_hardwareRevision = HARDWARE_VERSION;

/** AT software revision */
char const *AT_softwareRevision = SOFTWARE_VERSION;

/** AT firmware release date */
char const *AT_releaseDate = RELEASE_DATE;

/** AT device serial number */
char const *AT_serial = SERIAL_NUMBER;

/*******************************************************************************
 **************************   GLOBAL FUNCTIONS   *******************************
 ******************************************************************************/

/**
 * @brief  User setup function
 **/
void TD_USER_Setup(void)
{
    // Perform indefinite CEM RX/TX or antenna matching tests
#if RF_CEM_RX_TEST
    RF_CEM_RX();
#elif RF_CEM_TX_TEST
    RF_CEM_TX();
#elif RF_ANTENNA_MATCHING
    while (1) {
    	RF_CERT_TX(&RF_Params, 60, 0, 0);
    }
#endif

    // Initialize the LEUART
    init_printf(TD_UART_Init(9600, true, false),
    		TD_UART_Putc,
    		TD_UART_Start,
    		TD_UART_Stop);

#if AT_CERT_TEST
	AT_AddExtension(&cert_test_extension);
#endif
#if AT_MFG_TEST
	AT_AddExtension(&mfg_test_extension);
#endif
#if AT_RADIO_INFO
	AT_AddExtension(&radio_extension);
#endif
#if AT_SIGFOX
	AT_AddExtension(&sigfox_extension);
#endif
#if AT_LAN_RF
	AT_AddExtension(&lan_extension);
#endif


    // Initialize the AT command parser
    AT_Init();
#if AT_MFG_TEST
if (AT_quietResult) RF_MFG_StartRXTest(false ,false);
#endif


}

/**
 * @brief  User loop function
 **/
void TD_USER_Loop(void)
{
	int c;

#if AT_LAN_RF

	// Process local RF
	TD_LAN_Process();
#endif

	while ((c = TD_UART_GetChar()) >= 0) {
		AT_Parse(c);
	}

#if AT_MFG_TEST

	// Answer to manufacturing PING messages
	RF_MFG_EchoServer();
#elif AT_CERT_TEST

	// Perform RF Certification receive test
	RF_CERT_RXServer();
#endif
}
