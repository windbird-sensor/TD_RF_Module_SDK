/***************************************************************************//**
 * @file
 * @brief Serial Peripheral Interface (SPI) peripheral API for the TDxxxx RF
 * modules.
 * @author Telecom Design S.A.
 * @version 1.3.0
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2012-2016 Telecom Design S.A., http://www.telecomdesign.fr</b>
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

#ifndef __TD_SPI_H
#define __TD_SPI_H

#include <em_usart.h>
#include "td_gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

	/***********************************************************************//**
	 * @addtogroup SPI SPI
	 * @brief Serial Peripheral Interface (SPI) peripheral API for the TDxxxx RF
	 * modules
	 * @{
	 **************************************************************************/

	/***************************************************************************
	 *************************   DEFINES   *************************************
	 **************************************************************************/

	/** @addtogroup SPI_DEFINES Defines
	 * @{ */

/** SPI IDs for SPI bus locking/unlocking */
#define RF_SPI_ID 			1		///< SPI lock ID for RF chip
#define ACCELERO_SPI_ID 	2		///< SPI lock ID for accelerometer chip
#define GPS_SPI_ID 			3		///< SPI lock ID for GPS chip
#define RFIO_SPI_ID 		4		///< SPI lock ID for RF chip GPIOs
#define PRESSURE_SPI_ID 	5		///< SPI lock ID for pressure chip
#define MAGNETO_SPI_ID		6		///< SPI lock ID for magnetometer chip
#define FLASH_SPI_ID		7		///< SPI lock ID for serial flash chip

/** Maximum SPI lock ID, MUST be set to the maximum SPI ID defined */
#define	MAX_SYSTEM_SPI_ID	FLASH_SPI_ID

/** Retrieve a user-defined SPI lock ID.
 *
 * @note
 * It is possible to reserve additional SPI lock IDs by defining the
 * MAX_SPI_ID parameter in td_config.h, and retrieve the corresponding
 * lock IDs using this macro. */
#define USER_SPI_ID(x)		((x) + MAX_SYSTEM_SPI_ID)

#define SPI_BUS0	0			///< SPI Bus 0
#define SPI_BUS1	1			///< SPI Bus 1
#define SPI_BUS2	2			///< SPI Bus 2
#define SPI_LOC0	0			///< SPI Location 0
#define SPI_LOC1	1			///< SPI Location 1
#define SPI_LOC2	2			///< SPI Location 2
#define SPI_LOC3	3			///< SPI Location 3
#define SPI_LOC4	4			///< SPI Location 4

/** Macro to encode an SPI bus/location pair */
#define SPI_BUS(bus,loc) ((bus) | ((((loc) + 1) << 4)))

#ifdef EFM32TG210F32

/** Use a simple single-device SPI bus without locking */
//#define SIMPLE_SPI_BUS
#endif

#ifdef SIMPLE_SPI_BUS
#define TD_SPI_Lock(id, callback) true
#define TD_SPI_UnLock(id)
#define TD_SPI_Register(id, friend_id, bus, freq, mode)
#else

/***************************************************************************//**
 * @brief
 *   Try to lock the given SPI bus.
 *
 * @param[in] id
 *   The unique ID used for SPI bus locking.
 *
 * @param[in] callback
 *   Pointer to the call-back function that will be called when the bus is
 *   unlocked.
 *
 * @return
 * 	 true if bus was successfully locked, false otherwise
 ******************************************************************************/
#define TD_SPI_Lock(id, callback)	TD_SPI_Lock_(id, callback, __LINE__,__get_IPSR())

/***************************************************************************//**
 * @brief
 *   Try to lock the given SPI bus with custom context.
 *
 * @param[in] id
 *   The unique ID used for SPI bus locking.
 *
 * @param[in] callback
 *   Pointer to the call-back function that will be called when the bus is
 *   unlocked.
 *
 * @param[in] context
 *   Custom context for locking bus
 *
 * @return
 * 	 true if bus was successfully locked, false otherwise
 ******************************************************************************/
#define TD_SPI_LockContext(id, callback, context)	TD_SPI_Lock_(id, callback, __LINE__,context)

/***************************************************************************//**
 * @brief
 *   Unlock the given SPI bus.
 *
 * @param[in] id
 *   The unique ID used for SPI bus locking.
 ******************************************************************************/
#define TD_SPI_UnLock(id)			TD_SPI_UnLock_(id, __LINE__,__get_IPSR())

/***************************************************************************//**
 * @brief
 *   Unlock the given SPI bus with custom context
 *
 * @param[in] id
 *   The unique ID used for SPI bus locking.
 *
 * @param[in] context
 *   Context for unlocking
 ******************************************************************************/
#define TD_SPI_UnLockContext(id,context)			TD_SPI_UnLock_(id, __LINE__,context)

/***************************************************************************//**
 * @brief
 *   Register a SPI device with given parameters.
 *
 * @param[in] id
 *   ID usage to initialize.
 *
 * @param[in] friend_id
 *   Other ID on which we can take gracefully take ownership. Set to 0xFF if no
 *   friend id.
 *
 * @param[in] bus
 * 	 Bus to speak on.
 *
 * @param[in] freq
 * 	 Speed of bus for this use.
 *
 * @param[in] mode
 * 	 SPI mode for this use.
 ******************************************************************************/
#define TD_SPI_Register(id, friend_id, bus, freq, mode) \
	TD_SPI_Register_(id, friend_id, bus, freq, mode)
#endif

/***************************************************************************//**
 * @brief
 *   Macro to register a SPI device.
 *
 * @see TD_SPI_Register()
 ******************************************************************************/
#define TD_SPI_REGISTER(id, friend_id, bus, freq,mode) \
	TD_SPI_Register(id, friend_id, bus, freq, mode)

/***************************************************************************//**
 * @brief
 *   Macro to lock a SPI device.
 *
 * @see TD_SPI_Lock()
 ******************************************************************************/
#define TD_SPI_LOCK(id, callback)	TD_SPI_Lock(id, callback)

/***************************************************************************//**
 * @brief
 *   Macro to unlock a SPI device.
 *
 * @see TD_SPI_Unlock()
 ******************************************************************************/
#define TD_SPI_UNLOCK(id)			TD_SPI_UnLock(id)

/** Accelerometer lock context for polling */
#define TD_SPI_LOCK_CONTEXT_ACCELERO_POLLING		254

	/** @} */

	/***************************************************************************
	 *************************   TYPEDEFS   ************************************
	 **************************************************************************/

	/** @addtogroup SPI_TYPEDEFS Typedefs
	* @{ */

	/** User SPI callback function that will be called when The SPI bus
	 * is unlocked. */
	typedef void (*TD_SPI_LockedCallback)(void);

	/** SPI USER */
	typedef struct {
		uint8_t bus;						///< The used SPI bus
		uint32_t freq;						///< The SPI bus frequency
		USART_ClockMode_TypeDef	mode;		///< The SPI bus clock mode
		USART_TypeDef *usart;        		///< The USART base pointer
		TD_GPIO_Port_TypeDef csPort;		///< The associated Chip Select port
		unsigned int csBit;					///< The associated Chip Select bit
		uint8_t location;					///< The associated pin location
	} TD_SPI_Conf_t;

	/** @} */

	/***************************************************************************
	 *************************   PROTOTYPES   **********************************
	 **************************************************************************/

	/** @addtogroup SPI_GLOBAL_FUNCTIONS Global Functions
	 * @{ */

	void TD_SPI_WriteBuffer(uint8_t id, uint8_t count, uint8_t *buffer);
	void TD_SPI_WriteBuffer_PN9(uint8_t id, uint8_t count, uint8_t *buffer,
		bool reset);
	void TD_SPI_ReadBuffer(uint8_t id, uint8_t count, uint8_t *buffer);
	void TD_SPI_WriteByte(uint8_t id, uint8_t c);
	uint8_t TD_SPI_WriteReadByte(uint8_t id, uint8_t c, bool write);
	uint16_t TD_SPI_BackToBack(uint8_t id, uint8_t *write, uint8_t *read,
		uint16_t count);
	void TD_SPI_StartBackToBack(uint8_t id);
	void TD_SPI_EndBackToBack(uint8_t id);
	uint16_t TD_SPI_FullGenericReadWrite(uint8_t id, uint8_t *write, uint8_t *read,
		uint16_t count);
	uint32_t TD_SPI_WriteReadDouble(uint8_t id, uint32_t value);
	void TD_SPI_WriteDouble(uint8_t id, uint32_t value);
	bool TD_SPI_Lock_(uint8_t id, TD_SPI_LockedCallback callback,
		uint16_t line,uint8_t context);
	void TD_SPI_UnLock_(uint8_t id, uint16_t line,uint8_t context);
	void TD_SPI_Register_(uint8_t id, uint8_t friend_id, uint8_t bus,
		uint32_t freq, USART_ClockMode_TypeDef mode);
	void TD_SPI_InitBus(uint8_t bus);
	void TD_SPI_InitBusExt(uint8_t bus,uint8_t loc);
	void TD_SPI_LockDump(void);
#ifdef SIMPLE_SPI_BUS
	void TD_SPI_SimpleInit(uint32_t freq, USART_ClockMode_TypeDef mode);
#endif
	void TD_SPI_UnLockForce(uint8_t id);
	void TD_SPI_RegisterCS(uint8_t id, TD_GPIO_Port_TypeDef csPort,
		unsigned int csBit);
	void TD_SPI_FullWriteRegister(uint8_t id, uint8_t regacc, uint8_t value);
	uint8_t TD_SPI_FullReadRegister(uint8_t id, uint8_t regacc);
	void TD_SPI_FullWriteBuffer(uint8_t id, uint8_t register_address,
		uint8_t *buffer, uint8_t count);
	void TD_SPI_FullReadBuffer(uint8_t id, uint8_t regacc, uint8_t *buffer,
		uint8_t count);
	void TD_SPI_ConfDump(void);
	void TD_SPI_CS(uint8_t id, bool on);

	/** @} */

	/** @} (end addtogroup SPI) */

#ifdef __cplusplus
}
#endif

#endif // __TD_SPI_H
