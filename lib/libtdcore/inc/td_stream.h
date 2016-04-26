/***************************************************************************//**
 * @file
 * @brief I/O stream API for the TDxxxx RF modules.
 * @author Telecom Design S.A.
 * @version 1.0.0
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 Telecom Design S.A., http://www.telecomdesign.fr</b>
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

#ifndef __TD_STREAM_H
#define __TD_STREAM_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

	/***********************************************************************//**
	 * @addtogroup STREAM
	 * @brief I/O stream API for the TDxxxx RF modules
	 * @{
	 **************************************************************************/

	/***************************************************************************
	 *************************   DEFINES   *************************************
	 **************************************************************************/

	/** @addtogroup STREAM_DEFINES Defines
	 * @{ */

	/** Flag for opening a stream in panic mode */
#define TD_STREAM_PANIC			(1 << 0)

	/** Flag for reusing an existing stream */
#define TD_STREAM_REUSE			(1 << 1)

	/** Flag for opening a stream in read-only mode */
#define TD_STREAM_RDONLY		(1 << 2)

	/** Flag for opening a stream in write-only mode */
#define TD_STREAM_WRONLY		(1 << 3)

	/** Flag for opening a stream in read/write mode */
#define TD_STREAM_RDWR			(1 << 4)

	/** @} */

	/***************************************************************************
	 ***********************   ENUMERATIONS   **********************************
	 **************************************************************************/

	/** @addtogroup STREAM_ENUMERATIONS Enumerations
	 * @{ */

	/** @} */

	/***************************************************************************
	 *************************   TYPEDEFS   ************************************
	 **************************************************************************/

	/** @addtogroup STREAM_TYPEDEFS Typedefs
	 * @{ */
#ifdef TD_STREAM_EXPLICIT

/** Explicit TD stream type */
#define TD_STREAM_TYPE TD_STREAM_t
#else
/** Opaque TD stream type */
#define TD_STREAM_TYPE TD_STREAM_cast_t

/** Opaque TD stream type to avoid recursive definition */
typedef void TD_STREAM_t;
#endif
	/** Stream structure */
	typedef struct __TD_STREAM_t {
		bool (*start)(struct __TD_STREAM_t *stream);						/**< Start stream function pointer */
		void (*stop)(struct __TD_STREAM_t *stream);							/**< Stop stream function pointer */
		int (*read)(struct __TD_STREAM_t *stream);							/**< Read stream function pointer */
		bool (*write)(struct __TD_STREAM_t *stream, char c);				/**< Write single character to stream function pointer */
		bool (*write_buffer)(struct __TD_STREAM_t *stream, char *buffer,
			int length);													/**< Write buffer to stream function pointer */
		int (*available_chars)(struct __TD_STREAM_t *stream);				/**< Return number of avialable character in stream function pointer */
		void (*flush)(struct __TD_STREAM_t *stream);						/**< Flush stream function pointer */
		bool (*start_mem)(struct __TD_STREAM_t *stream);					/**< Start stream function pointer */
		void (*stop_mem)(struct __TD_STREAM_t *stream);						/**< Stop stream function pointer */
		void *io_handle;													/**< Opaque pointer to underlying stream driver */
		uint32_t mode;														/**< Stream open mode */
		int read_index;														/**< Stream receive FIFO read index */
		int write_index;													/**< Stream receive FIFO write index */
		char *fifo;															/**< Stream receive FIFO buffer */
		int overflow;														/**< Stream receive FIFO overflow counter */
	} TD_STREAM_TYPE;

	/** @} */

	/***************************************************************************
	 *************************  CONSTANTS  *************************************
	 **************************************************************************/

	/** @addtogroup STREAM_CONSTANTS Constants
	 * @{ */

	/** @} */

	/***************************************************************************
	 *************************   PROTOTYPES   **********************************
	 **************************************************************************/

	/** @addtogroup STREAM_USER_FUNCTIONS User Functions
	 * @{ */
	/** @addtogroup STREAM_PROTOTYPES Prototypes
	 * @{ */

	TD_STREAM_t *TD_STREAM_Alloc(void *io_handle);
	void TD_STREAM_Panic(void);
	void TD_STREAM_Free(TD_STREAM_t *stream);
	void TD_STREAM_PutFIFO(TD_STREAM_t *stream, char c);
	int TD_STREAM_GetFIFO(TD_STREAM_t *stream);
	int TD_STREAM_AvailableCharsInFIFO(TD_STREAM_t *stream);
	void TD_STREAM_FlushFIFO(TD_STREAM_t *stream);
	int TD_STREAM_GetFIFOOverflow(TD_STREAM_t *stream);
	void TD_STREAM_ResetFIFOOverflow(TD_STREAM_t *stream);
	void TD_STREAM_Continuous(TD_STREAM_t *stream, bool start);

	/** @} */
	/** @} */

	/***************************************************************************
	 **************************   PUBLIC VARIABLES   ***************************
	 **************************************************************************/

	/** @addtogroup STREAM_GLOBAL_VARIABLES Global Variables
	 * @{ */
	/** @addtogroup STREAM_EXTERN External Declarations
	 * @{ */

	/** @} */
	/** @} */

	/** @} (end addtogroup STREAM) */

#ifdef __cplusplus
}
#endif

#endif // __TD_STREAM_H
