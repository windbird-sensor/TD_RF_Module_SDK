/***************************************************************************//**
 * @file
 * @brief GPS NMEA parser.
 * @author Telecom Design S.A.
 * @version 1.0.0
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

#ifndef __NMEA_PARSER_H
#define __NMEA_PARSER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

	/***********************************************************************//**
	 * @addtogroup TD_NMEA NMEA Parser
	 * @brief NMEA GPS parser.
	 * @{
	 **************************************************************************/

	/***************************************************************************
	 **************************  ENUM   ****************************************
	 **************************************************************************/

	/** @addtogroup TD_NMEA_ENUMERATIONS Enumerations
	 * @{ */

	/** NMEA message type */
	typedef enum {
		TD_NMEA_GPGGA,
		TD_NMEA_GPRMC
	} TD_NMEA_Message_t;

	/** @} */

	/***************************************************************************
	 *************************   TYPEDEFS   ************************************
	 **************************************************************************/

	/** @addtogroup TD_NMEA_TYPEDEFS Typedefs
	 * @{ */

	/** Structure containing NMEA Global Positioning System Fix Data */
	typedef struct {
		int32_t latitude;		///< Latitude
		int32_t longitude;		///< Longitude
		int16_t altitude;		///< Altitude
		uint16_t hdop;			///< Horizontal Dilution of Precision
		uint8_t hour;			///< Hour
		uint8_t minute;			///< Minute
		uint8_t second;			///< Seconds
		uint8_t sat;			///< Number of Satellites
	} TD_NMEA_GPGGA_t;

	/** Structure containing NMEA Recommended minimum specific GPS/Transit data */
	typedef struct {
		int32_t latitude;		///< Latitude
		int32_t longitude;		///< Longitude
		uint16_t speed;			///< Speed over ground in Knots
		uint8_t year;			///< Year
		uint8_t month;			///< Month
		uint8_t day;			///< Day
		uint8_t hour;			///< Hour
		uint8_t minute;			///< Minute
		uint8_t second;			///< Seconds
	} TD_NMEA_GPRMC_t;

	/** NMEA parser function. data contains the character to parse */
	typedef bool (*TD_NMEA_parser_t)(char data);

	/** @} */

	/***************************************************************************
	 *************************   PROTOTYPES   **********************************
	 **************************************************************************/

	/** @addtogroup TD_NMEA_GLOBAL_FUNCTIONS Global Functions
	 * @{ */

	void TD_NMEA_Init(void);
	void TD_NMEA_Reset(void);
	bool TD_NMEA_ParseBuffer(char *buffer, int length);
	bool TD_NMEA_PARSER_Parse(char data);
	bool TD_NMEA_PARSER_Process(void);
	bool TD_NMEA_PARSER_IsMessageUpdated(TD_NMEA_Message_t message);
	void *TD_NMEA_PARSER_SetDataPointer(TD_NMEA_Message_t message, void *ptr);
	bool TD_NMEA_PARSER_isDisplayUsed(void);
	TD_NMEA_parser_t TD_NMEA_GetParser(void);

	/** @} */

	/** @addtogroup TD_NMEA_USER_FUNCTIONS User Functions
	 * @{ */

	void TD_NMEA_EnableOutput(bool enabled, char *command);

	/** @} */

	/** @} (end addtogroup TD_NMEA) */

#ifdef __cplusplus
}
#endif

#endif // __NMEA_PARSER_H
