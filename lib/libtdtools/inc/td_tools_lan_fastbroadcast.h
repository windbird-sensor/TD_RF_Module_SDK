/** @cond TD_PRIVATE */
/***************************************************************************//**
 * @file
 * @brief Tools LAN API for the TDxxxx RF modules API for the TDxxxx RF modules.
 * @author Telecom Design S.A.
 * @version 1.0.0
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2014 Telecom Design S.A., http://www.telecomdesign.fr</b>
 ******************************************************************************
 *
 * This source code is the property of Telecom Design S.A.
 *
 * This copyright notice may not be removed from the source code nor changed.
 *
 ******************************************************************************/
#ifndef __TD_TOOLS_LAN_FASTBROADCAST_H
#define __TD_TOOLS_LAN_FASTBROADCAST_H

#ifdef __cplusplus
extern "C" {
#endif

	/***********************************************************************//**
	 * @addtogroup TOOLS_LAN LAN Tools
	 * @briefLAN Tools API for the TDxxxx RF modules
	 * @{
	 **************************************************************************/

	/***************************************************************************
	 *************************   DEFINES   *************************************
	 **************************************************************************/

	/** @addtogroup TOOLS_LAN_DEFINES Defines
	 * @{ */

#define TD_TOOLS_LAN_FASTBROADCAST_INFINITE		0	/* Infinite reception time */

	/** @} */

	/***************************************************************************
	 *************************   PROTOTYPES   **********************************
	 **************************************************************************/

	/** @addtogroup TOOLS_LAN_USER_FUNCTIONS User Functions
	 * @{ */

bool TD_TOOLS_LAN_FASTBROADCAST_Send(uint8_t * data, uint16_t packet_count,
	uint16_t packet_size, uint32_t baudrate, uint32_t freq, uint8_t power,
	void (*callback)(void));
bool TD_TOOLS_LAN_FASTBROADCAST_Receive(uint16_t packet_count,
	uint16_t packet_size, uint32_t max_wait, uint32_t baudrate, uint32_t freq,
	void (*callback)(uint8_t *packet, uint8_t count));

	/** @} */

	/** @} (end addtogroup TOOLS_LAN) */

#ifdef __cplusplus
}
#endif

#endif // __TD_TOOLS_LAN_FASTBROADCAST_H
/** @endcond */
