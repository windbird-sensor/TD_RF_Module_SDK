/***************************************************************************//**
 * @file
 * @brief Sensor Utils
 * @author Telecom Design S.A.
 * @version 1.1.0
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

#ifndef __TD_SENSOR_UTILS_H
#define __TD_SENSOR_UTILS_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

	/***********************************************************************//**
	 * @addtogroup TD_SENSOR_UTILS Sensor Utils
	 * @{ */

	/***************************************************************************
	 *************************   DEFINES   *************************************
	 **************************************************************************/

	/** @addtogroup TD_SENSOR_UTILS_DEFINES Defines
	 * @{ */

	/* Bits to byte conversion macro */
#define BITS_TO_BYTES(b) ((b + 7) / 8)

	/** Shift a value so that all their useful bits (n) are on left
	 * 00101011 -> 6 bits saved in 8 bits
	 * converted to:
	 * 10101100 -> all useful bits on left
	 */
#define BITS_TO_LEFT(v, n) (v << ((sizeof (v) * 8) - n))

	/** Write bits in char word */
	/* b7  b6  b5  b4  b3  b2  b1  b0
	 *       | 1   0   1   0  |
	 *       |                |
	 *       |                |
	 *       |                RANK = 2
	 *       LENGTH (END)     LENGTH = 4 (START)
	 *                        DATA = 0xA
	 */
#define WRITE_BITS_C(WORD, RANK, LENGTH, DATA) \
	(WORD) = (((uint8_t)(WORD)) & (~(~(((uint8_t)0xFF) << (LENGTH)) << (RANK)))) | \
	((((uint8_t)DATA) & (~(((uint8_t)0xFF) << (LENGTH)))) << (RANK))

	/** Read bits from char word */
#define READ_BITS_C(WORD, RANK, LENGTH) \
	((((uint8_t)WORD) >> (RANK)) & (~((uint8_t)0xFF << (LENGTH))))

	/** Write bits in short word */
#define WRITE_BITS_S(WORD, RANK, LENGTH, DATA) \
	(WORD) = (((uint16_t)(WORD)) & (~(~(((uint16_t)0xFFFF) << (LENGTH)) << (RANK)))) | \
	((((uint16_t)DATA) & (~(((uint16_t)0xFFFF) << (LENGTH)))) << (RANK))

	/** Read bits from short word */
#define READ_BITS_S(WORD, RANK, LENGTH) \
	((((uint16_t)WORD) >> (RANK)) & (~((uint16_t)0xFFFF << (LENGTH))))

	/** Write bits in long word */
#define WRITE_BITS_L(WORD, RANK, LENGTH, DATA) \
	(WORD) = (((uint32_t)(WORD)) & (~(~(((uint32_t)0xFFFFFFFF) << (LENGTH)) << (RANK)))) | \
	((((uint32_t)DATA) & (~(((uint32_t)0xFFFFFFFF) << (LENGTH)))) << (RANK))

	/** Read bits from long word */
#define READ_BITS_L(WORD, RANK, LENGTH) \
	((((uint32_t)WORD) >> (RANK)) & (~((uint32_t)0xFFFFFFFF << (LENGTH))))

	/** Write bits in long long word */
#define WRITE_BITS_LL(WORD, RANK, LENGTH, DATA) \
	(WORD) = (((uint64_t)(WORD)) & (~(~(((uint64_t)0xFFFFFFFFFFFFFFFF) << (LENGTH)) << (RANK)))) | \
	((((uint64_t)DATA) & (~(((uint64_t)0xFFFFFFFFFFFFFFFF) << (LENGTH)))) << (RANK))

	/* Read bits from long long word */
#define READ_BITS_LL(WORD, RANK, LENGTH) \
	((((uint64_t)WORD) >> (RANK)) & (~((uint64_t)0xFFFFFFFFFFFFFFFF << (LENGTH))))

	/** Host to Network short conversion macro */
#define HTONS(n) ((uint16_t)(((n) >> 8) | ((n) << 8)))

	/** Host to Network long conversion macro */
#define HTONL(n) ((((n) & 0x000000FF) << 24)  \
	+(((n) & 0x0000FF00) << 8)  \
	+(((n) & 0x00FF0000) >> 8)  \
	+(((n) & 0xFF000000) >> 24))

	/** Host to Network long long conversion macro */
#define HTONLL(ll) ((((uint64_t)HTONL(ll)) << 32) + HTONL(ll >> 32))

	/** Network to Host short conversion macro */
#define NTOHS(n) HTONS(n)

	/** Network to Host long conversion macro */
#define NTOHL(n) HTONL(n)

	/** Network to Host long long conversion macro */
#define NTOHLL(n) HTONLL(n)

	/** @} */

	/***************************************************************************
	 *************************   PROTOTYPES   **********************************
	 **************************************************************************/

	/** @addtogroup TD_SENSOR_UTILS_USER_FUNCTIONS User Functions
	 * @{ */

	void TD_SENSOR_UTILS_BitConcat(uint8_t *data,  uint8_t *len,
		uint8_t *data_append, uint8_t len_append);

	/** @} */

	/** @} (end addtogroup TD_SENSOR_UTILS) */

#ifdef __cplusplus
}
#endif

#endif /* TD_SENSOR_UTILS_H_ */
