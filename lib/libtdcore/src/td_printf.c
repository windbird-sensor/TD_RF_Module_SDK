/***************************************************************************//**
 * @file
 * @brief printf utility for the TDxxxx RF modules.
 * @author Kustaa Nyholm
 * @version 2.1.1
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2004 Kustaa Nyholm</b>
 * <b>(C) Copyright 2012-2016 Telecom Design S.A., http://www.telecomdesign.fr</b>
 *******************************************************************************
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 ******************************************************************************/

#include <em_usart.h>

#include "td_core.h"
#include "td_utils.h"
#include "td_printf.h"

/** Flag to get explicit TD stream structure pointers */
#define TD_STREAM_EXPLICIT
#include "td_stream.h"

/***************************************************************************//**
 * @addtogroup PRINTF
 * @brief printf utility for the TDxxxx RF modules
 * @{
 ******************************************************************************/

/*******************************************************************************
 *******************************   DEFINES   ***********************************
 ******************************************************************************/

/** @addtogroup PRINTF_DEFINES Defines
 * @{ */


/** @} */

/*******************************************************************************
 *************************   TYPEDEFS   ****************************************
 ******************************************************************************/

/** @addtogroup PRINTF_TYPEDEFS Typedefs
 * @{ */

/** Pointer to function for flushing characters out */
typedef void (*putcf)(void *, char);

/** @} */

/*******************************************************************************
 *************************   PRIVATE VARIABLES   *******************************
 ******************************************************************************/

/** @addtogroup PRINTF_LOCAL_VARIABLES Local Variables
 * @{ */

/** Pointer to current stream */
static TD_STREAM_t *Stream = 0;

/** @} */

/*******************************************************************************
 **************************  PRIVATE FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup PRINTF_LOCAL_FUNCTIONS Local Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Output an argument with left padding.
 *
 * @param[in] putp
 *   Pointer to character output function.
 *
 * @param[in] putf
 *   Pointer to flush output function.
 *
 * @param[in] n
 *   Number of left-padding characters is positive, right padding if negative.
 *
 * @param[in] z
 *   Padding with zero characters if not null, with blank caracters otherwise.
 *
 * @param[in] bf
 *   Pointer to the input buffer.
 ******************************************************************************/
static void putchw(void *putp, putcf putf, int n, char z, char *bf)
{
	char fc = z ? '0' : ' ';
	char ch;
	char *p = bf;
	bool end = false;

	// If pad size is negative, pad at end
	if (n < 0) {
		n = -n;
		end = true;
	}

	// Compute padding size
	while (*p++ && n > 0) {
		n--;
	}
	if (!end) {
		while (n-- > 0) {
			putf(putp, fc);
		}
	}
	while ((ch = *bf++)) {
		putf(putp, ch);
	}
	if (end) {
		while (n-- > 0) {
			putf(putp, fc);
		}
	}
}

/***************************************************************************//**
 * @brief
 *   Function to output a character to a string.
 *
 * @param[out] p
 *   Pointer to the output buffer.
 *
 * @param[in] c
 *   Input character.
******************************************************************************/
static void putcp(void *p, char c)
{
	*(*((char **)p))++ = c;
}

/***************************************************************************//**
 * @brief
 *   Function to output a string.
 *
 * @param[in] putp
 *   Pointer to character output function.
 *
 * @param[in] putf
 *   Pointer to flush output function.
 *
 * @param[in] bf
 *   Pointer to the input buffer.
******************************************************************************/
static void putcs(void *putp, putcf putf, char *bf)
{
	char ch;
	while ((ch = *bf++)) {
		putf(putp, ch);
	}
}

/** @} */

/*******************************************************************************
 **************************   PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup PRINTF_USER_FUNCTIONS User Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Output a string according to a format.
 *
 * @param[in] putp
 *   Pointer to character output function.
 *
 * @param[in] putf
 *   Pointer to flush output function.
 *
 * @param[in] fmt
 *   Pointer to format string in printf format.
 *
 * @param[in] va
 *   Variable list of arguments according to the format string.
 ******************************************************************************/
void tfp_format(void *putp, putcf putf, const char *fmt, va_list va)
{
	char bf[25];

	char ch;
	char neg = 0;

	if (!putf) {
		return;
	}
	while ((ch = *(fmt++))) {
		if (ch != '%') {
			neg = 0;
			putf(putp, ch);
		} else {
			char lz = 0;
#ifdef PRINTF_LONG_SUPPORT
			char lng = 0;
#endif
			int w = 0;
			ch = *(fmt++);
			if (ch == '0') {
				ch = *(fmt++);
				lz = 1;
			}
			if (ch == '-') {
				ch = *(fmt++);
				neg = 1;
			}
			if (ch >= '0' && ch <= '9') {
				ch = a2i(ch, (char **) & fmt, 10, &w);
				if (neg) {
					w = -w;
				}
			}
#ifdef PRINTF_LONG_SUPPORT
			if (ch == 'l') {
				ch = *(fmt++);
				lng = 1;
			}
#endif
			switch (ch) {
			case 0:
				goto abort;
			case 'u': {
#ifdef PRINTF_LONG_SUPPORT
				if (lng) {
					if (CONFIG_PRINTF_INT64_SUPPORT_ON) {
						((void (*)(uint64_t num, unsigned int base, int uc, char *bf))CONFIG_PRINTF_ULONG)(va_arg(va, uint64_t), 10, 0, bf);
					}
					else{
						((void (*)(unsigned long int num, unsigned int base, int uc, char *bf))CONFIG_PRINTF_ULONG)(va_arg(va, unsigned long), 10, 0, bf);
					}
				} else
#endif
					ui2a(va_arg(va, unsigned int), 10, 0, bf);
				putchw(putp, putf, w, lz, bf);
				break;
			}
			case 'd': {
#ifdef PRINTF_LONG_SUPPORT
				if (lng) {
					if (CONFIG_PRINTF_INT64_SUPPORT_ON) {
						((void (*)(int64_t num, char *bf))CONFIG_PRINTF_LONG)(va_arg(va, int64_t), bf);
					}
					else{
						((void (*)(long int num, char *bf))CONFIG_PRINTF_LONG)(va_arg(va, long int), bf);
					}
				} else
#endif
					i2a(va_arg(va, int), bf);
				putchw(putp, putf, w, lz, bf);
				break;
			}
			case 'x':
			case 'X':
#ifdef PRINTF_LONG_SUPPORT
				if (lng) {
					if (CONFIG_PRINTF_INT64_SUPPORT_ON) {
						((void (*)(uint64_t num, unsigned int base, int uc, char *bf))CONFIG_PRINTF_ULONG)(va_arg(va, uint64_t), 16, (ch == 'X'), bf);
					}
					else{
						((void (*)(unsigned long int num, unsigned int base, int uc, char *bf))CONFIG_PRINTF_ULONG)(va_arg(va, unsigned long), 16, (ch == 'X'), bf);
					}
				} else
#endif
					ui2a(va_arg(va, unsigned int), 16, (ch == 'X'), bf);
				putchw(putp, putf, w, lz, bf);
				break;
			case 'c':
				putf(putp, (char)(va_arg(va, int)));
				break;
			case 's':
				putchw(putp, putf, w, 0, va_arg(va, char *));
				break;
			case '%':
				putf(putp, ch);
				break;
			default:
				break;
			}
		}
	}
abort:
	;
}

/***************************************************************************//**
 * @brief
 *   Printf initialization function.
 *
 * @param[in] putp
 *   Pointer to character output function.
 *
 * @param[in] putf
 *   Pointer to flush output function.
 *
 * @param[in] start
 *   Pointer to function for starting to use the output stream.
 *
 * @param[in] stop
 *   Pointer to function for stopping to use the output stream.
 ******************************************************************************/
void init_printf(void *putp, tfp_putf_t putf, tfp_start_stop_t start,
	tfp_start_stop_t stop)
{
	if (putp != 0) {
		Stream = (TD_STREAM_t *) putp;
	}
	if (Stream == 0) {
		return;
	}
	Stream->write = (bool (*) (TD_STREAM_t *, char)) putf;
	Stream->start = (bool (*) (TD_STREAM_t *)) start;
	Stream->stop = (void (*) (TD_STREAM_t *)) stop;
}

/***************************************************************************//**
 * @brief
 *   Set this stream as system stream (for printf)
 *
 * @param[in] stream
 *   Pointer to the stream.
 *
 ******************************************************************************/
void init_printf_stream(void *stream)
{
	Stream = stream;
}

/***************************************************************************//**
 * @brief
 *   Get Printf stream
 *
 * @return System stream for printf
 ******************************************************************************/
void *get_printf(void)
{
	return Stream;
}

/***************************************************************************//**
 * @brief
 *   Output a string according to a format.
 *
 * @param[in] stream
 *   Pointer to stream to use
 *
 * @param[in] fmt
 *   Pointer to format string in printf format.
 *
 * @param[in] ...
 *   Number and types of arguments according to the format string.
 ******************************************************************************/
void tfp_printf_stream(void *stream, const char *fmt, ...)
{
	va_list va;

	if (stream == 0) {
		return;
	}
	va_start(va, fmt);
	if (((TD_STREAM_t *)stream)->start) {
		((TD_STREAM_t *)stream)->start(stream);
	}
	tfp_format(((TD_STREAM_t *)stream),
		(void (*)(void *, char)) ((TD_STREAM_t *) stream)->write, fmt, va);
	if (((TD_STREAM_t *)stream)->stop) {
		((TD_STREAM_t *)stream)->stop(stream);
	}
	va_end(va);
}

/***************************************************************************//**
 * @brief
 *   Output a string according to a format.
 *
 * @param[in] fmt
 *   Pointer to format string in printf format.
 *
 * @param[in] ...
 *   Number and types of arguments according to the format string.
 ******************************************************************************/
void tfp_printf(const char *fmt, ...)
{
	va_list va;

	if (Stream == 0) {
		return;
	}
	va_start(va, fmt);
	if (((TD_STREAM_t *)Stream)->start) {
		((TD_STREAM_t *)Stream)->start(Stream);
	}
	tfp_format(((TD_STREAM_t *)Stream),
		(void (*)(void *, char)) ((TD_STREAM_t *) Stream)->write, fmt, va);
	if (((TD_STREAM_t *)Stream)->stop) {
		((TD_STREAM_t *)Stream)->stop(Stream);
	}
	va_end(va);
}

/***************************************************************************//**
 * @brief
 *   Output a buffer.
 *
 * @param[in] buf
 *   Pointer to a buffer string
 *
 * @param[in] len
 *   Buffer length
 *
 ******************************************************************************/
void tfp_print_buf(char *buf, int len)
{
	Stream->start(Stream);
	Stream->write_buffer(Stream, buf, len);
	Stream->stop(Stream);
}

/***************************************************************************//**
 * @brief
 *   Output a string wit hvaraible argument list according to a format.
 *
 * @param[in] fmt
 *   Pointer to format string in printf format.
 *
 * @param[in] va
 *   Variable list of arguments according to the format string.
 ******************************************************************************/
void tfp_vprintf(const char *fmt, va_list va)
{
	Stream->start(Stream);
	tfp_format(Stream, (void (*)(void *, char)) Stream->write, fmt, va);
	Stream->stop(Stream);
}

/***************************************************************************//**
 * @brief
 *   Output a string into another string according to a format.
 *
 * @param[out] s
 *   Pointer to the output string buffer.
 *
 * @param[in] fmt
 *   Pointer to format string in printf format.
 *
 * @param[in] ...
 *   Variable list of arguments according to the format string.
 ******************************************************************************/
void tfp_sprintf(char *s, const char *fmt, ...)
{
	va_list va;

	va_start(va, fmt);
	tfp_format(&s, putcp, fmt, va);
	putcp(&s, 0);
	va_end(va);
}

/***************************************************************************//**
 * @brief
 *   Dump a buffer in hexadecimal.
 *
 * @param[in] prompt
 *   Pointer to a prompt string to ouptut in front of the hexadecimal dump.
 *
 * @param[in] buffer
 *   Pointer to the input buffer.
 *
 * @param[in] length
 *   Length of the input buffer in characters.
 ******************************************************************************/
void tfp_dump(char *prompt, unsigned char *buffer, unsigned char length)
{
	char dump[(3 * 16) + 3];
	unsigned char i, j, k;
	char hex[4];

	Stream->start(Stream);
	if (prompt) {
		putcs(Stream, (void (*)(void *, char)) Stream->write, prompt);
	}
	for (i = 0, k = 0; i < length; i++) {
		tfp_sprintf(hex, "%02x ", buffer[i]);
		for (j = 0; hex[j]; j++) {
			dump[k++] = hex[j];
		}
		if (i && ((i % 16) == 15)) {
			dump[k++] = '\r';
			dump[k++] = '\n';
			dump[k] = '\0';
			putcs(Stream, (void (*)(void *, char)) Stream->write, dump);
			k = 0;
		}
	}
	if (k > 0) {
		dump[k++] = '\r';
		dump[k++] = '\n';
		dump[k] = '\0';
		putcs(Stream, (void (*)(void *, char)) Stream->write, dump);
	}
	Stream->stop(Stream);
}

/** @} */

/** @} (end addtogroup PRINTF) */
