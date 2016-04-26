/** @cond TD_PRIVATE */
/***************************************************************************//**
 * @file
 * @brief RF debug for the TDxxxx RF module.
 * @author Telecom Design S.A.
 * @version 1.0.0
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2014 Telecom Design S.A., http://www.telecomdesign.fr</b>
 ******************************************************************************
 ******************************************************************************
 *
 * This source code is the property of Telecom Design S.A.
 *
 * This copyright notice may not be removed from the source code nor changed.
 *
 ******************************************************************************/

#ifndef __TD_RDEBUG_H
#define __TD_RDEBUG_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "td_stream.h"
#include "td_boot.h"

/***************************************************************************//**
 * @addtogroup TD_RDEBUG TD RF Debug
 * @brief RF Debug for the TDxxxx RF modules
 * @{
 ******************************************************************************/

/*******************************************************************************
 *************************   PROTOTYPES   **************************************
 ******************************************************************************/

	/** @addtogroup RDEBUG_GLOBAL_FUNCTIONS Global Functions
	 * @{ */
	/** @addtogroup RDEBUG_PROTOTYPES Prototypes
	 * @{ */

	TD_STREAM_t *TD_RDEBUG_Open(uint32_t mode);
	void TD_RDEBUG_Close(void);
	TD_STREAM_t *TD_RDEBUG_Reopen(void);
	bool TD_RDEBUG_Print(uint8_t *buf, uint8_t n);
	void TD_RDEBUG_ChannelSetup(uint8_t const *data);
	void TD_RDEBUG_ChannelSetup32(uint32_t dat);
	void TD_RDEBUG_SetFrequency(uint32_t frequency);
	void TD_RDEBUG_PowerSetup(uint8_t val);
	void TD_RDEBUG_SetPriority(uint8_t priority);
	bool TD_RDEBUG_IsOpen(void);
	int32_t TD_RDEBUG_GetResetLastFrequencyOffset(void);
	uint32_t TD_RDEBUG_GetCount(void);
	void TD_RDEBUG_ResetCount(void);

	/** @} */
	/** @} */

	/** @} (end addtogroup TD_RDEBUG) */

#ifdef __cplusplus
}
#endif

#endif // __TD_RDEBUG_H
/** @endcond */
