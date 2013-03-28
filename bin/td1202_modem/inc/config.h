/******************************************************************************
 * @file config.h
 * @brief Configuraiton file for the TD1202 module firmware
 * @author Telecom Design S.A.
 * @version 1.0.0
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2012 Telecom Design S.A., http://www.telecom-design.com</b>
 ******************************************************************************
 *
 * This source code is the property of Telecom Design S.A.
 *
 * This copyright notice may not be removed from the source code nor changed.
 *
 *****************************************************************************/
#ifndef CONFIG_H_
#define CONFIG_H_

#include <td_module.h>

/** Manufacturer */
#define MANUFACTURER		"Telecom Design"

/** Product */
#define PRODUCT				"TD1202"

/** Hardware revision */
#if MODULE_REVISION == REVISION_D
#define HARDWARE_VERSION	"0D"
#elif MODULE_REVISION == REVISION_E
qsd;;
#define HARDWARE_VERSION	"0E"
#elif MODULE_REVISION == REVISION_F
#define HARDWARE_VERSION	"0F"
#endif

/** Software version */
#define SOFTWARE_VERSION	"SOFTWARE"

/** Release data */
#define RELEASE_DATE		"M01+2013"

/** Telecom Design 12-digit serial number */
#define SERIAL_NUMBER		"123456789012"

/** RF certification test AT parser extension */
#define AT_CERT_TEST		0

/** TD LAN RF AT parser extension */
#define AT_LAN_RF			1

/** Manufacturing test AT parser extension */
#define AT_MFG_TEST			0

/** RF chip info AT parser extension */
#define AT_RADIO_INFO		1

/** SigFox AT parser extension */
#define AT_SIGFOX			1

/** Module test mode */
#define MODULE_TEST			0

/** COVEA test mode */
#define COVEA_TEST			0

/** COVEA test mode */
#define COVEA_TEST_ALL_IN_ONE           0

/** Continuous TX CEM test mode (disables the AT parser) */
#define RF_CEM_TX_TEST		0

/** Continuous RX CEM test mode (disables the AT parser) */
#define RF_CEM_RX_TEST		0

/** Continuous CW TX  antenna matching test mode (disables the AT parser) */
#define RF_ANTENNA_MATCHING	0
//#define DEBUG 1

/** Force Revision E board for MFG test reference board */
#define MFG_TEST_REF_PCB	0

#if AT_CERT_TEST
#include "at_cert_test.h"
#else
#define AT_CERT_TEST_EXTENSION
#endif

#if AT_LAN_RF
#include <at_lan.h>
#else
#define AT_LAN_EXTENSION
#endif

#if AT_MFG_TEST
#include "at_mfg_test.h"
#else
#define AT_MFG_TEST_EXTENSION
#endif

#if AT_RADIO_INFO
#include <at_radio.h>
#else
#define AT_RADIO_EXTENSION
#endif

#if AT_SIGFOX
#include <at_sigfox.h>
#else
#define AT_SIGFOX_EXTENSION
#endif

#if AT_SENSOR
#include <at_sensor.h>
#include <at_sensor_lan.h>
#include <at_sensor_monitor.h>
#endif


// ****************************************************************************
#endif // CONFIG_H_
// ****************************************************************************
