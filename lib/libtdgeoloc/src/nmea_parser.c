/***************************************************************************//**
 * @file
 * @brief GPS NMEA parser.
 * @author Telecom Design S.A.
 * @version 1.0.1
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2013-2014 Telecom Design S.A., http://www.telecomdesign.fr</b>
 ******************************************************************************
 ******************************************************************************
 *
 * This source code is the property of Telecom Design S.A.
 *
 * This copyright notice may not be removed from the source code nor changed.
 *
 ******************************************************************************/

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <td_rtc.h>
#include <td_core.h>
#include <td_printf.h>
#include <td_utils.h>

#include "ubx7.h"
#include "nmea_parser.h"

/***************************************************************************//**
 * @addtogroup TD_NMEA NMEA Parser
 * @brief NMEA parser.
 * @{
 ******************************************************************************/

/******************************************************************************
 **************************  DEFINES   ****************************************
 ******************************************************************************/

/** @addtogroup TD_NMEA_DEFINES Defines
 * @{ */

/** Number of entries in the NMEA command table */
#define LAST_COMMAND			(sizeof (NMEACommands) / sizeof (NMEA_command_t))

/** Maximum field length */
#define MAX_FIELD_COUNT			20

/** Maximum data length */
#define MAX_DATA				255

/** Maximum command length (NMEA address) */
#define MAX_COMMAND				8


/** NMEA parser states */
#define NMEA_STATE_SOM			0			///< Search for start of message
#define NMEA_STATE_CMD			1			///< Get command
#define NMEA_STATE_DATA			2			///< Get data
#define NMEA_STATE_CHECKSUM_1	3			///< Get first checksum character
#define NMEA_STATE_CHECKSUM_2	4			///< Get second checksum character

/** Turn on trace mode if tfp_printf not commented */
//#define DEBUG_PARSER
//#define DEBUG_PARSER_INFO

#ifdef DEBUG_PARSER_INFO
#define DEBUG_PRINTF(...) tfp_printf(__VA_ARGS__)
#else
#define DEBUG_PRINTF(...)
#endif

/** @} */

/*******************************************************************************
 ***********************   ENUMERATIONS   **************************************
 ******************************************************************************/

/** @addtogroup TD_NMEA_ENUMERATIONS Enumerations
 * @{ */

/** NMEA command tokens */
typedef enum nmea_command_token_t {
	NMEA_UNKNOWN = 0,
	NMEA_GPGGA = 1,
	NMEA_GPRMC = 2,
	NMEA_GPTXT = 3,
	NMEA_GPGSV = 4
} nmea_command_token;

/** @} */

/*******************************************************************************
 *************************   TYPEDEFS   ****************************************
 ******************************************************************************/

/** @addtogroup TD_NMEA_TYPEDEFS Typedefs
 * @{ */

/** NMEA command structure */
typedef struct {
	char *ascii;						///< ASCII command representation
	nmea_command_token token;			///< Corresponding command token
	uint8_t length;						///< ASCII command length in characters
	bool display;						///< Display flag
	bool (*process)(void);				///< Command process function
} NMEA_command_t;

/** @} */

/*******************************************************************************
 ************************   PROTOTYPES   ********************************
 ******************************************************************************/

static bool TD_NMEA_ProcessGPGGA(void);
static bool TD_NMEA_ProcessGPRMC(void);
static bool TD_NMEA_ProcessGPTXT(void);
static bool TD_NMEA_ProcessGPGSV(void);

/*******************************************************************************
 ************************   PRIVATE VARIABLES   ********************************
 ******************************************************************************/

/** @addtogroup TD_NMEA_LOCAL_VARIABLES Local Variables
 * @{ */

#pragma pack(1)

/** Computed NMEA sentence checksum */
static uint8_t ComputedChecksum = 0;

/** Received NMEA sentence checksum (if exists) */
static uint8_t ReceivedChecksum = 0;

/** Index used for both commands and data */
static int Index = 0;

/** NMEA command buffer */
static char Command[MAX_COMMAND] = "";

/** NMEA data buffer */
static char Data[MAX_DATA] = "";

/** Data Field Index list */
static uint8_t FieldIndexList[MAX_FIELD_COUNT];

/** Data Field Count */
static uint8_t FieldCount = 0;

#pragma pack()

/** NMEA parser state */
static uint8_t NMEAState = NMEA_STATE_SOM;

/** NMEA command table */
static NMEA_command_t NMEACommands[] = {
	{"GPGGA", NMEA_GPGGA, 5, false, TD_NMEA_ProcessGPGGA},
	{"GPRMC", NMEA_GPRMC, 5, false, TD_NMEA_ProcessGPRMC},
	{"GPTXT", NMEA_GPTXT, 5, false, TD_NMEA_ProcessGPTXT},
	{"GPGSV", NMEA_GPGSV, 5, false, TD_NMEA_ProcessGPGSV},
};

/** Global display flag */
static bool DisplayAll = false;

/** Last NMEA command */
static NMEA_command_t LastCommand = {0, NMEA_UNKNOWN, 0, false, 0};

/** Flag array used during NMEA command parsing */
static bool CommandsElector[LAST_COMMAND];

/**  */
static bool FixUpdate = false;

/**  */
static TD_NMEA_GPGGA_t *Gpgga = NULL;

/**  */
static bool GpggaUpdated = false;

/**  */
static TD_NMEA_GPRMC_t *Gprmc = NULL;

/**  */
static bool GprmcUpdated = false;

/** The NMEA parser function pointer */
static TD_NMEA_parser_t NMEAParser = 0;

/** @} */

/*******************************************************************************
 **************************  PRIVATE FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup TD_NMEA_FUNCTIONS Private Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Reset NMEA command parsing structure.
 ******************************************************************************/
static void TD_NMEA_ResetElector(void)
{
	memset(CommandsElector, true, LAST_COMMAND);
}

/***************************************************************************//**
 * @brief
 *   Process a time information within any NMEA command
 *
 * @param[in] index
 *   Index of time into the global string array Data.
 *
 * @return
 *   Returns true if valid time is found, false otherwise.
 ******************************************************************************/
static bool TD_NMEA_ProcessTime(uint8_t index, uint8_t *hour, uint8_t *minute,
	uint8_t *second)
{
	char temp[3];
	char *field = &Data[FieldIndexList[index]];

	if (field[0] != 0 && hour != NULL && minute != NULL && second != NULL) {

		// Hour
		temp[0] = field[0];
		temp[1] = field[1];
		temp[2] = '\0';
		*hour = atoi(temp);

		// minute
		temp[0] = field[2];
		temp[1] = field[3];
		*minute = atoi(temp);

		// Second
		temp[0] = field[4];
		temp[1] = field[5];
		*second = atoi(temp);

		//Milliseconds
		return true;
	} else {
		*hour = 0xFF;
		*minute = 0xFF;
		*second = 0xFF;
	}
	return false;
}

/***************************************************************************//**
 * @brief
 *   Process a date within any NMEA command
 *
 * @param[in] index
 *   Index of date into the global string array Data.
 *
 * @return
 *   Returns true if valid date is found, false otherwise.
 ******************************************************************************/
static bool TD_NMEA_ProcessDate(uint8_t index, uint8_t *year, uint8_t *month,
	uint8_t *day)
{
	char *field = &Data[FieldIndexList[index]];

	if (field[0] != 0) {
		*day = (field[0] - '0') * 10 + (field[1] - '0');
		*month = (field[2] - '0') * 10 + (field[3] - '0');
		*year = (field[4] - '0') * 10 + (field[5] - '0');
		return true;
	} else {
		*day = 0xFF;
		*month = 0xFF;
		*year = 0xFF;
	}
	return false;
}

/***************************************************************************//**
 * @brief
 *   Process a position within any NMEA command
 *
 * @param[in] index
 *   Index of position into the global string array Data.
 *
 * @return
 *   Returns true if valid position is found, false otherwise.
 ******************************************************************************/
static bool TD_NMEA_ProcessPosition(uint8_t index, int32_t *latitude,
	int32_t *longitude)
{
	char *field_pos, * field_sign;
	bool update = false;

	field_pos = &Data[FieldIndexList[index]];
	field_sign = &Data[FieldIndexList[index + 1]];

	// Latitude
	if (field_pos[0] != 0 && field_sign[0] != 0) {
		*latitude = atolli(field_pos, '.');

		// South latitudes are converted to negative values
		if (field_sign[0] == 'S') {
			*latitude = -(*latitude);
		}
		update = true;
	} else {
		*latitude = 0x7FFFFFFF;
	}
	field_pos = &Data[FieldIndexList[index + 2]];
	field_sign = &Data[FieldIndexList[index + 3]];

	// Longitude
	if (field_pos[0] != 0 && field_sign[0] != 0) {
		*longitude = atolli(field_pos, '.');

		// West longitude are converted to negative values
		if (field_sign[0] == 'W') {
			*longitude = -(*longitude);
		}
		update = true;
	} else {
		*longitude = 0x7FFFFFFF;
	}
	return update;
}

/***************************************************************************//**
 * @brief
 *   Process a GPTXT NMEA command and reset Ublox if dumping errors
 *
 * @return
 *   Returns false.
 ******************************************************************************/
static bool TD_NMEA_ProcessGPTXT(void)
{
	char *field;

	if (FieldCount >= 3) {
		field = &Data[FieldIndexList[3]];
		if (field[0] != 0) {
			if (field[0] == 'e' && field[1] == 'x' && field[2] == 'c' &&
				field[3] == 'e') {
				DEBUG_PRINTF("TD_NMEA_ProcessGPTXT UBX7 RESET - ERROR \r\n");
				TD_UBX7_PowerOff();
				TD_RTC_Delay(T1S);
				TD_UBX7_PowerUp(false, false);
				//TD_UBX7_PollMonExcept();
			}
		}
	} else {
		DEBUG_PRINTF("TD_NMEA_ProcessGPTXT Field count %d - ERROR", FieldCount);
	}
	return false;
}

/***************************************************************************//**
 * @brief
 *   Process a GPGGA NMEA command.
 *
 * @return
 *   Returns true if the GPGGA is available, false otherwise.
 ******************************************************************************/
static bool TD_NMEA_ProcessGPGGA(void)
{
	char *field;

	if (Gpgga != NULL) {
		if (FieldCount >= 7) {

			// Time field 0
			TD_NMEA_ProcessTime(0, &Gpgga->hour, &Gpgga->minute, &Gpgga->second);

			// Position: field 1,2,3,4
			TD_NMEA_ProcessPosition(1, &Gpgga->latitude, &Gpgga->longitude);

			// Satellites in use
			field = &Data[FieldIndexList[6]];
			if (field[0] != 0) {
				Gpgga->sat = atoi(field);
			} else {
				Gpgga->sat = 0xFF;
			}

			// HDOP
			field = &Data[FieldIndexList[7]];
			if (field[0] != 0) {
				Gpgga->hdop = atolli(field, '.');
			} else {
				Gpgga->hdop = 9999;
			}

			// Altitude
			field = &Data[FieldIndexList[8]];
			if (field[0] != 0) {
				Gpgga->altitude = atoi(field);
			} else {
				Gpgga->altitude = 0x7FFF;
			}
		} else {
			DEBUG_PRINTF("TD_NMEA_ProcessGPGGA Field count %d - ERROR",
				FieldCount);
		}
		GpggaUpdated  = true;
		return true;
	}
	return false;
}

/***************************************************************************//**
 * @brief
 *   Process a GPRMC NMEA command.
 *
 * @return
 *   Returns false.
 ******************************************************************************/
static bool TD_NMEA_ProcessGPRMC(void)
{
	char *field;

	if (Gprmc != NULL) {

		if (FieldCount >= 8) {

			// Position: field 1,2,3,4
			TD_NMEA_ProcessTime(0, &Gprmc->hour, &Gprmc->minute, &Gprmc->second);

			// Position: field 2,3,4,5
			TD_NMEA_ProcessPosition(2, &Gprmc->latitude, &Gprmc->longitude);

			// Speed over ground
			field = &Data[FieldIndexList[6]];
			if (field[0] != 0) {
				Gprmc->speed = atoi(field);
			} else {
				Gprmc->speed = 0xFFFF;
			}

			// Date
			TD_NMEA_ProcessDate(8, &Gprmc->year, &Gprmc->month, &Gprmc->day);
			GprmcUpdated = true;
			return true;
		} else {
			DEBUG_PRINTF("TD_NMEA_ProcessGPRMC Field count %d - ERROR",
				FieldCount);
		}
	}
	return false;
}

/***************************************************************************//**
 * @brief
 *   Process a GPGSV NMEA command.
 *
 * @return
 *   Returns false.
 ******************************************************************************/
static bool TD_NMEA_ProcessGPGSV(void)
{
#if 0
	char *field, *field2;
	bool ret = false;
	uint8_t page = 0;
	uint8_t total;
	int i;

	field = &Data[FieldIndexList[0]];
	field2 = &Data[FieldIndexList[1]];
	if (field != 0 && field2 != 0) {
		page = atoi(field2);
		if (atoi(field) == page) {
			Fix.sat.polled = false;
			ret = true;
		}
	}
	if (page == 1) {
		Fix.sat.count = 0;
	}
	field = &Data[FieldIndexList[2]];
	if (field != 0) {
		total = atoi(field);
	}

	for (i = 0; i < 4; i++) {

		// Up to 4 sat per sentence
		if (Fix.sat.count < TD_GEOLOC_MAX_SATELLITES && Fix.sat.count < total) {

			// Get ID
			field = &Data[FieldIndexList[4 * (i + 1) - 1]];
			if (field == 0) {
				Fix.sat.info[Fix.sat.count].id = 0;
			} else {
				Fix.sat.info[Fix.sat.count].id = atoi(field);
			}

			// Get SNR
			field = &Data[FieldIndexList[4 * (i + 1) + 2]];
			if (field == 0) {
				Fix.sat.info[Fix.sat.count].level = 0;
			} else {
				Fix.sat.info[Fix.sat.count].level = atoi(field);
			}
			Fix.sat.count++;
		}
	}
#endif
	return false;
}
/***************************************************************************//**
 * @brief
 *   Display a valid NMEA trame.
 *
 * @param[in] command
 *   Pointer to the command ascii representation.
 *
 * @param[in] checksum
 *   Checksum.
 ******************************************************************************/
static void TD_NMEA_DisplayCommand(char *command, uint8_t checksum)
{
	int i;

	tfp_printf("$%s", command);
	for (i = 0; i <= FieldCount; i++) {
		tfp_printf(",");
		tfp_printf(&Data[FieldIndexList[i]]);
	}
	tfp_printf("*%02X\r\n", checksum);
}

/***************************************************************************//**
 * @brief
 *   Process a valid NMEA command.
 *
 * @param[in] command
 *   Pointer to the NMEA command structure.
 *
 * @param[in] data
 *   Pointer to the 0-splitted NMEA data string.
 *
 * @param[in] checksum
 *   Trame checksum.
 ******************************************************************************/
static void TD_NMEA_ProcessCommand(NMEA_command_t *command, char *data,
	uint8_t checksum)
{
	bool update = false;

	if (command->token != NMEA_UNKNOWN) {
		update = (*command->process)();

		if (command->display && !DisplayAll) {
			TD_NMEA_DisplayCommand(command->ascii, checksum);
		}
	}
	if (update) {
		FixUpdate = true;
	}
}

/** @} */

/*******************************************************************************
 **************************   PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup TD_NMEA_GLOBAL_FUNCTIONS Global Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Process an NMEA character. Will call appropriate command process if
 *   the character is the last of the command. Will also call
 *   the UpdateCallback and display NMEA if enabled.
 *
 * @param[in] data
 *   The NMEA character to process.
 ******************************************************************************/
bool TD_NMEA_PARSER_Parse(char data)
{
	int i;

	// Discard all 0xFF
	if (data == 0xFF) {
		return false;
	}

#ifdef DEBUG_PARSER
	if ((data >= ' ' && data <= 'z') || (data == 0xD) || (data == 0xA)) {
		tfp_printf("%c", data);
	} else {
		if (data == 0xB5) {
			tfp_printf("\r\n");
		}
		tfp_printf("0x%02X(%c)", data, data ? data : ' ');
	}
#endif

	switch (NMEAState) {

	// Search for start of message '$'
	case NMEA_STATE_SOM:
		if (data == '$') {
			TD_NMEA_ResetElector();
			LastCommand.token = NMEA_UNKNOWN;

			// Reset checksum
			ComputedChecksum = 0;

			// Initialize command Counter
			Index = 0;
			NMEAState = NMEA_STATE_CMD;
		}
		break;

	// Retrieve command (NMEA Address)
	case NMEA_STATE_CMD:
		if (data != ',' && data != '*') {
			for (i = 0; i < LAST_COMMAND; i++) {
				if (CommandsElector[i]) {

					// If elector for current command is still valid
					if (Index < NMEACommands[i].length) {

						// If length is OK
						if (NMEACommands[i].ascii[Index] != data) {
							CommandsElector[i] = false;
						}
					} else {
						CommandsElector[i] = false;
					}
				}
			}
			Command[Index++] = data;
			ComputedChecksum ^= data;

			// Check for command overflow
			if (Index >= MAX_COMMAND) {
				NMEAState = NMEA_STATE_SOM;
			}
		} else {
			LastCommand.token = NMEA_UNKNOWN;

			// Should be only one left
			for (i = 0; i < LAST_COMMAND; i++) {
				if (CommandsElector[i]) {
					memcpy(&LastCommand, &NMEACommands[i], sizeof(NMEA_command_t));
				}
			}

			// Terminate command
			Command[Index] = '\0';
			ComputedChecksum ^= data;

			// Initialize data counter
			Index = 0;
			FieldCount = 0;
			FieldIndexList[0] = 0;

			// Go to get data state
			NMEAState = NMEA_STATE_DATA;
		}
		break;

	// Store data and check for end of sentence or checksum flag
	case NMEA_STATE_DATA:
		if (FieldCount + 1 >= MAX_FIELD_COUNT) {
			NMEAState = NMEA_STATE_SOM;
			break;
		}
		if (data == '*') {

			// Checksum flag
			Data[Index] = '\0';
			NMEAState = NMEA_STATE_CHECKSUM_1;
		} else {

			// No checksum flag -> bad
			if (data == '\r') {
				NMEAState = NMEA_STATE_SOM;
			}

			// Split parameters with 0s
			if (data == ',') {
				FieldIndexList[++FieldCount] = Index + 1;
				Data[Index] = '\0';
			} else {
				Data[Index] = data;
			}

			// Store data and calculate checksum
			ComputedChecksum ^= data;

			// Check for buffer overflow
			if (++Index >= MAX_DATA) {
				NMEAState = NMEA_STATE_SOM;
			}
		}
		break;

	case NMEA_STATE_CHECKSUM_1:
		if ((data - '0') <= 9) {
			ReceivedChecksum = (data - '0') << 4;
		} else {
			ReceivedChecksum = (data - 'A' + 10) << 4;
		}
		NMEAState = NMEA_STATE_CHECKSUM_2;
		break;

	case NMEA_STATE_CHECKSUM_2:
		if ((data - '0') <= 9) {
			ReceivedChecksum |= (data - '0');
		} else {
			ReceivedChecksum |= (data - 'A' + 10);
		}
		NMEAState = NMEA_STATE_SOM;
		if (ComputedChecksum == ReceivedChecksum) {
			if (DisplayAll) {

#ifdef DEBUG_PARSER
				tfp_printf("\r\n");
#endif

				TD_NMEA_DisplayCommand(Command, ReceivedChecksum);

#ifdef DEBUG_PARSER
				tfp_printf("|");
#endif

			}

			// All parameters are global, no real need to use them as parameters...
			TD_NMEA_ProcessCommand(&LastCommand, Data, ReceivedChecksum);

			//command done (save for /r/n which can be ignored)
			return true;
		}
		break;

	default:
		NMEAState = NMEA_STATE_SOM;
		break;
	}

	return false;
}

/***************************************************************************//**
 * @brief
 *   Initialize the NMEA parser.
 ******************************************************************************/
void TD_NMEA_Init(void)
{
	TD_NMEA_Reset();
	LastCommand.token = NMEA_UNKNOWN;
}

/***************************************************************************//**
 * @brief
 *   Reset the NMEA parser. Must be called each time the GPS is power on to
 *   make sure all buffers are cleared.
 ******************************************************************************/
void TD_NMEA_Reset(void)
{
	NMEAState = NMEA_STATE_SOM;
	TD_NMEA_ResetElector();
	FieldCount = 0;
	GprmcUpdated = false;
	GpggaUpdated = false;
}

/***************************************************************************//**
 * @brief
 *   Parse an NMEA buffer.
 *
 * @param[in] buffer
 *   Pointer to the NMEA buffer to parse.
 *
 * @param[in] length
 *   Length in characters of the NMEA buffer to parse.
 *
 * @return
 *   Returns true if the buffer was correctly parsed, false otherwise.
 ******************************************************************************/
bool TD_NMEA_ParseBuffer(char *buffer, int length)
{
	int i;

	for (i = 0; i < length; i++) {
		TD_NMEA_PARSER_Parse(buffer[i]);
	}
	return true;
}

/** @} */

/** @addtogroup TD_NMEA_USER_FUNCTIONS User Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Enable output of given NMEA commands. Call several times (one per NMEA
 *   command you want to enable/disable). Default is disabled. Use * to
 *   outputs raw GPS NMEA. Handled NMEA commands are GPGGA and GPRMC.
 *
 * @param[in] enabled
 *   Enable/disable flag.
 *
 * @param[in] command
 *   NMEA command as an ASCII string to process, "*" processes all commands.
 ******************************************************************************/
void TD_NMEA_EnableOutput(bool enabled, char *command)
{
	int i;

	NMEAParser = TD_NMEA_PARSER_Parse;
	if (strcmp(command, "*") == 0) {
		DisplayAll = enabled;
	} else {
		for (i = 0; i < LAST_COMMAND; i++) {
			if (strcmp(NMEACommands[i].ascii, command) == 0) {
				NMEACommands[i].display = enabled;
				break;
			}
		}
	}
}

/***************************************************************************//**
 * @brief
 *   Returns if a message has been updated or not.
 *
 * @param[in] message
 *   The message to check for update.
 *
 * @return
 *   Returns true if the message has been updated, false otherwise.
 ******************************************************************************/
bool TD_NMEA_PARSER_IsMessageUpdated(TD_NMEA_Message_t message)
{
	bool ret = false;

	switch ((TD_NMEA_Message_t) message) {
	case TD_NMEA_GPGGA:
		ret = GpggaUpdated;
		GpggaUpdated = false;
		break;

	case TD_NMEA_GPRMC:
		ret = GprmcUpdated;
		GprmcUpdated = false;
		break;

	default:
		break;
	}
	return ret;
}

/***************************************************************************//**
 * @brief
 *   Assign a pointer to a message where the decoded data should be copied.
 *
 * @param[in] message
 *   The message to check for update.
 *
 * @param[in] ptr
 *   Pointer to the buffer where the data should be copied.
 *
 * @return
 *   Returns a opaque pointer to the message type.
 ******************************************************************************/
void *TD_NMEA_PARSER_SetDataPointer(TD_NMEA_Message_t message, void *ptr)
{
	void *p = NULL;

	switch ((TD_NMEA_Message_t) message) {
	case TD_NMEA_GPGGA:
		p = Gpgga;
		Gpgga = (TD_NMEA_GPGGA_t *) ptr;
		break;

	case TD_NMEA_GPRMC:
		p = Gprmc;
		Gprmc = (TD_NMEA_GPRMC_t *) ptr;
		break;

	default:
		break;
	}
	return p;
}

/***************************************************************************//**
 * @brief
 *   Check if some NMEA display is activated.
 *
 * @return
 *   Returns true if some display is activated, false otherwise.
 ******************************************************************************/
bool TD_NMEA_PARSER_isDisplayUsed(void)
{
	int i;

	if (DisplayAll) {
		return true;
	}
	for (i = 0; i < LAST_COMMAND; i++) {
		if (NMEACommands[i].display) {
			return true;
		}
	}
	return false;
}

/***************************************************************************//**
 * @brief
 *   NMEA parser background process.
 ******************************************************************************/
bool TD_NMEA_PARSER_Process(void)
{
	bool temp;

	temp = FixUpdate;
	FixUpdate = false;
	return temp;
}

/***************************************************************************//**
 * @brief
 *   Returns the NMEA parser function.
 *
 * @return
 *   Returns the NMEA parser function.
 ******************************************************************************/
TD_NMEA_parser_t TD_NMEA_GetParser(void)
{
	return NMEAParser;
}

/** @} */

/** @} (end addtogroup TD_NMEA) */
