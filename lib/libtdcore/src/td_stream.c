/***************************************************************************//**
 * @file
 * @brief I/O stream API for the TDxxxx RF modules.
 * @author Telecom Design S.A.
 * @version 1.0.1
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2015-2016 Telecom Design S.A., http://www.telecomdesign.fr</b>
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

#include <stdint.h>
#include <stdbool.h>

#include <em_assert.h>

#include "td_config_ext.h"

/** Flag to get explicit TD stream structure pointers */
#define TD_STREAM_EXPLICIT
#include "td_stream.h"

/***************************************************************************//**
 * @addtogroup STREAM
 * @brief I/O stream API for the TDxxxx RF modules
 * @{
 ******************************************************************************/

/*******************************************************************************
 *************************   DEFINES   *****************************************
 ******************************************************************************/

/** @addtogroup STREAM_DEFINES Defines
 * @{ */

/** @} */

/*******************************************************************************
 *************************  CONSTANTS  *****************************************
 ******************************************************************************/

/** @addtogroup STREAM_CONSTANTS Constants
 * @{ */

/** @} */

/*******************************************************************************
 *************************   PUBLIC VARIABLES   ********************************
 ******************************************************************************/

/** @addtogroup STREAM_GLOBAL_VARIABLES Global Variables
 * @{ */

/** @} */

/*******************************************************************************
 *************************   PRIVATE VARIABLES   *******************************
 ******************************************************************************/

/** @addtogroup STREAM_LOCAL_VARIABLES Local Variables
 * @{ */

/** Array of all available streams */
extern TD_STREAM_t TD_STREAM[];

/** FIFO of all available streams */
extern char TD_STREAM_FIFO[];

/** Flag to test if a stream is opened in read mode */
#define TD_STREAM_READ			(TD_STREAM_RDONLY | TD_STREAM_RDWR)

/** Flag to test if a stream is opened in write mode */
#define TD_STREAM_WRITE			(TD_STREAM_WRONLY | TD_STREAM_RDWR)

/** @} */

/*******************************************************************************
 **************************   PRIVATE FUNCTIONS   ******************************
 ******************************************************************************/

/** @addtogroup STREAM_LOCAL_FUNCTIONS Local Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Dummy I/O stream start function.
 *
 * @param[in] stream
 *   Pointer to the stream to start.
 *
 * @return
 *   Always return true.
 ******************************************************************************/
static bool dummy_start(TD_STREAM_t *stream)
{
	return true;
}

/***************************************************************************//**
 * @brief
 *   Dummy I/O stream read function.
 *
 * @param[in] stream
 *   Pointer to the stream to read from.
 *
 * @return
 *   Always return -1.
 ******************************************************************************/
static int dummy_read(TD_STREAM_t *stream)
{
	return -1;
}

/***************************************************************************//**
 * @brief
 *   Dummy I/O stream write function.
 *
 * @param[in] stream
 *   Pointer to the stream to write to.
 *
 * @param[in] c
 *   The character to write.
 *
 * @return
 *   Always return true.
 ******************************************************************************/
static bool dummy_write(TD_STREAM_t *stream, char c)
{
	return true;
}

/***************************************************************************//**
 * @brief
 *   Dummy I/O stream buffer write function.
 *
 * @param[in] stream
 *   Pointer to the stream to write to.
 *
 * @param[in] buffer
 *   Pointer to the buffer containign the characters to write.
 *
 * @param[in] length
 *   Length in characters of the buffer to write.
 *
 * @return
 *   Returns true if operation was successful, false otherwise.
 ******************************************************************************/
static bool dummy_write_buffer(TD_STREAM_t *stream, char *buffer, int length)
{
	int i;

	for (i = 0; i < length; i++) {
		if (!stream->write(stream, *buffer++)) {
			return false;
		}
	}
	return true;
}

/***************************************************************************//**
 * @brief
 *   Dummy I/O stream available character count function.
 *
 * @param[in] stream
 *   Pointer to the stream to read from.
 *
 * @return
 *   Always return -1.
 ******************************************************************************/
static int dummy_available_chars(TD_STREAM_t *stream)
{
  return -1;
}

/***************************************************************************//**
 * @brief
 *   Dummy I/O stream flush function.
 *
 * @param[in] stream
 *   Pointer to the stream to flush.
 ******************************************************************************/
static void dummy_flush(TD_STREAM_t *stream)
{
}

/***************************************************************************//**
 * @brief
 *   Dummy I/O stream stop function.
 *
 * @param[in] stream
 *   Pointer to the stream to stop.
 ******************************************************************************/
static void dummy_stop(TD_STREAM_t *stream)
{
}

/** @} */

/*******************************************************************************
 **************************   PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup STREAM_USER_FUNCTIONS User Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Allocate an I/O stream.
 *
 * @param[in] io_handle
 *   Pointer on the low-level I/O structure.
 *
 * @return
 *   Returns a pointer to the allocated stream or 0 if no more stream is
 *   available.
 ******************************************************************************/
TD_STREAM_t *TD_STREAM_Alloc(void *io_handle)
{
	int i = 0;

	if (io_handle != 0) {
		for (i = 0; i < CONFIG_TD_STREAM_COUNT; i++) {
			if (TD_STREAM[i].io_handle == io_handle) {
				break;
			}
		}
	}
	if (i == CONFIG_TD_STREAM_COUNT) {
		for (i = 0; i < CONFIG_TD_STREAM_COUNT; i++) {
			if (TD_STREAM[i].start == 0) {
				break;
			}
		}
	}
	if (i == CONFIG_TD_STREAM_COUNT) {
		return 0;
	}
	memset(&TD_STREAM[i], 0, sizeof (TD_STREAM_t));
	TD_STREAM[i].io_handle = io_handle;
	TD_STREAM[i].start = dummy_start;
	TD_STREAM[i].read = dummy_read;
	TD_STREAM[i].write = dummy_write;
	TD_STREAM[i].write_buffer = dummy_write_buffer;
	TD_STREAM[i].available_chars = dummy_available_chars;
	TD_STREAM[i].flush = dummy_flush;
	TD_STREAM[i].stop = dummy_stop;
	TD_STREAM[i].fifo = &TD_STREAM_FIFO[CONFIG_TD_STREAM_FIFOSIZE * i];
	memset(TD_STREAM[i].fifo, 0, CONFIG_TD_STREAM_FIFOSIZE);
	return &TD_STREAM[i];
}

/***************************************************************************//**
 * @brief
 *   Ensure empty stream for panic mode.
 *
 ******************************************************************************/
void TD_STREAM_Panic(void)
{
	memset(&TD_STREAM[0], 0, sizeof (TD_STREAM_t));
}

/***************************************************************************//**
 * @brief
 *   Free an I/O stream.
 *
 * @param[in] stream
 *   Pointer to the stream to free.
 ******************************************************************************/
void TD_STREAM_Free(TD_STREAM_t *stream)
{
	memset(stream, 0, sizeof (TD_STREAM_t));
}

/***************************************************************************//**
 * @brief
 *   Write a single character to stream receive FIFO.
 *
 * @param[in] stream
 *   Pointer to the stream to write to.
 *
 * @param[in] c
 *   The character to add to the stream FIFO.
 ******************************************************************************/
void TD_STREAM_PutFIFO(TD_STREAM_t *stream, char c)
{
	int next;

	next = stream->write_index + 1;
	if (next >= CONFIG_TD_STREAM_FIFOSIZE) {
		next = 0;
	}

	// Is there an overflow?
	if (next != stream->read_index) {

		// Store and update index
		stream->fifo[stream->write_index] = c;
		stream->write_index = next;
	} else {
		if (stream->overflow != 0xFF) {
			stream->overflow++;
		}
	}
}

/***************************************************************************//**
 * @brief
 *   Get a single character from the stream receive FIFO..
 *
 * @param[in] stream
 *   Pointer to the stream to read from.
 *
 * @return
 *   Returns the character from the receive FIFO, or -1 if no character is
 *   available.
 ******************************************************************************/
int TD_STREAM_GetFIFO(TD_STREAM_t *stream)
{
	int c;
	int next;

	if (stream->read_index == stream->write_index) {
		return -1;
	}

	/* Here we get one character, and THEN update pointer.
	 * If buffer is full and an IRQ would add a character, no possible loss
	 */
	c = stream->fifo[stream->read_index];
	next = stream->read_index + 1;
	if (next >= CONFIG_TD_STREAM_FIFOSIZE) {

		// Wrapped read index
		next = 0;
	}
	stream->read_index = next;
	return c;
}

/***************************************************************************//**
 * @brief
 *   Returns the number of characters available in the stream receive FIFO.
 *
 * @param[in] stream
 *   Pointer to the stream to check.
 *
 * @return
 *   Returns the number of characters available in the stream receive FIFO.
 ******************************************************************************/
int TD_STREAM_AvailableCharsInFIFO(TD_STREAM_t *stream)
{
	int count = stream->write_index - stream->read_index;

	return count < 0 ? CONFIG_TD_STREAM_FIFOSIZE + count : count;
}

/***************************************************************************//**
 * @brief
 *   Flush a stream receive FIFO.
 *
 * @param[in] stream
 *   Pointer to the stream to flush.
 ******************************************************************************/
void TD_STREAM_FlushFIFO(TD_STREAM_t *stream)
{
	stream->read_index = stream->write_index;
}

/***************************************************************************//**
 * @brief
 *   Get the number of overflow from a stream receive FIFO.
 *
 * @param[in] stream
 *   Pointer to the stream to check.
 *
 * @return
 *   Returns the number of overflow from a stream receive FIFO.
 ******************************************************************************/
int TD_STREAM_GetFIFOOverflow(TD_STREAM_t *stream)
{
	return stream->overflow;
}

/***************************************************************************//**
 * @brief
 *   Reset the number of overflow from a stream receive FIFO.
 *
 * @param[in] stream
 *   Pointer to the stream to check.
 ******************************************************************************/
void TD_STREAM_ResetFIFOOverflow(TD_STREAM_t *stream)
{
	stream->overflow = 0;
}

/***************************************************************************//**
 * @brief
 *   Dummy stub function for start & stop function in continuous mode
 *
 * @param[in] stream
 *   Pointer to the stream.
 *
 ******************************************************************************/
void TD_STREAM_Dummy(TD_STREAM_t *stream)
{

}

/***************************************************************************//**
 * @brief
 *   Start/stop continuous mode on stream
 *
 * @param[in] stream
 *   Pointer to the stream.
 *
 * @param[in] start
 *   true : Start continuous mode, falsde: stop coninuous mode.
 ******************************************************************************/
void TD_STREAM_Continuous(TD_STREAM_t *stream, bool start)
{
	if (start) {
		stream->start(stream);
		stream->start_mem = stream->start;
		stream->stop_mem = stream->stop;
		stream->start = (bool (*)(struct __TD_STREAM_t *stream)) TD_STREAM_Dummy;
		stream->stop = TD_STREAM_Dummy;
	}
	else{
		stream->start = stream->start_mem;
		stream->stop = stream->stop_mem;
		stream->stop(stream);
	}
}

/** @} */

/** @} (end addtogroup STREAM) */
