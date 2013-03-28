/***************************************************************************//**
 * @file
 * @brief Flash controller (MSC) peripheral API for the TDxxxx RF modules.
 * @author Telecom Design S.A.
 * @version 2.0.0
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2012-2013 Telecom Design S.A., http://www.telecom-design.com</b>
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


#include <stdbool.h>
#include <stdint.h>
#include <efm32.h>
#include "td_core.h"
#include <td_printf.h>
#include "td_flash.h"

/***************************************************************************//**
 * @addtogroup FLASH
 * @brief Flash controller (MSC) Peripheral API for the TD1202 module
 * @{
 ******************************************************************************/

/*******************************************************************************
 *******************************   DEFINES   ***********************************
 ******************************************************************************/

/** @addtogroup FLASH_DEFINES Defines
 * @{ */

#ifdef __ICCARM__
#pragma language = extended
#pragma segment = "USER_ROM_CONFIG"
#else

#endif

#ifdef __ICCARM__
/** Special user config page (512 bytes) */
#define E2P_USER       (uint32_t) __segment_begin("USER_ROM_CONFIG")
#else
/** Beginning of Flash memory from linker script */
extern const char __userrom_start;

/** Special user config page (512 bytes) */
#define E2P_USER   (uint32_t) &__userrom_start
#endif

/** Special factory page (512 bytes) */
#define E2P_FACTORY     0x0FE00000

/** DWORD size */
#define DWORDSZ         sizeof (uint32_t)

/** MAX allowed data pointers */
#define MAX_FLASH_DATA_POINTER 10

/** @} */

/** Flash variable structure
 *
 * @note
 *  Each variable is assigned a unique virtual address automatically when first
 *  written to, or when using the declare function.
 */
typedef struct {
	uint8_t *data_pointer;
	uint16_t data_size;
} TD_FLASH_variable_t;

/*******************************************************************************
 *******************************   PRIVATE VARIABLES   *************************
 ******************************************************************************/

/** @addtogroup FLASH_PRIVATE_VARIABLES Private Variables
 * @{ */

/** List of pointer to data which should be stored in Flash memory */
static TD_FLASH_variable_t FlashDataList[MAX_FLASH_DATA_POINTER];

/** Data list count */
static uint8_t data_count = 0;

/** Total size of flash memory reserved */
static uint16_t total_size = 0;

/** @} */

/*******************************************************************************
 **************************  PRIVATE FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup FLASH_PRIVATE_FUNCTIONS Private Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Enables the flash controller for writing.
 ******************************************************************************/
void TD_FLASH_Init(void)
{
    // Enable writing to the MSC
	MSC->WRITECTRL |= MSC_WRITECTRL_WREN;

    // Unlock the MSC
	MSC->LOCK = MSC_UNLOCK_CODE;

    // Disable writing to the MSC
	MSC->WRITECTRL &= ~MSC_WRITECTRL_WREN;
}

/***************************************************************************//**
 * @brief
 *   Disables the flash controller for writing.
 ******************************************************************************/
void TD_Flash_Deinit(void)
{
    // Enable writing to the MSC
	MSC->WRITECTRL |= MSC_WRITECTRL_WREN;

    // Lock the MSC
	MSC->LOCK = 0;

    // Disable writing to the MSC
	MSC->WRITECTRL &= ~MSC_WRITECTRL_WREN;
}

/***************************************************************************//**
 * @brief
 *   Programs a single word into flash.
 *
 * @note
 *   The flash must be erased prior to writing a new word.
 *   This function must be run from RAM. Failure to execute this portion
 *   of the code in RAM will result in a hardfault. For IAR, Rowley and
 *   Codesourcery this will be achieved automatically. For Keil uVision 4 you
 *   must define a section called "ram_code" and place this manually in your
 *   project's scatter file.
 *
 * @note
 *   This function will not return until the data has been programmed.
 *
 * @details
 *   This function will program one word into the on-chip flash.
 *   Programming consists of ANDing the new data with the existing data; in
 *   other words bits that contain 1 can remain 1 or be changed to 0, but bits
 *   that are 0 can not be changed to 1.  Therefore, a word can be programmed
 *   multiple times so long as these rules are followed; if a program operation
 *   attempts to change a 0 bit to a 1 bit, that bit will not have its value
 *   changed.
 *
 * @param[in] address
 *   Pointer to the flash word to write to. Must be aligned to long words.
 * @param[in] data
 *   Data to write to flash.
 ******************************************************************************/
__RAMFUNCTION void TD_FLASH_WriteWord(uint32_t *address, uint32_t data)
{
    MSC->WRITECTRL |= MSC_WRITECTRL_WREN;

    // Load address
    MSC->ADDRB    = (uint32_t) address;
    MSC->WRITECMD = MSC_WRITECMD_LADDRIM;

    // Load data
    MSC->WDATA    = data;

    // Trigger write once
    MSC->WRITECMD = MSC_WRITECMD_WRITEONCE;

    // Wait for the write to complete
    while ((MSC->STATUS & MSC_STATUS_BUSY)) {
    	;
    }
}

/***************************************************************************//**
 * @brief
 *   Erases a block of flash.
 *
 * @note
 *   This function MUST be executed from RAM. Failure to execute this portion
 *   of the code in RAM will result in a hardfault. For IAR, Rowley and
 *   Codesourcery this will be achieved automatically. For Keil uVision 4 you
 *   must define a section called "ram_code" and place this manually in your
 *   project's scatter file.
 *
 * @note
 *   This function will not return until the data has been erased.
 *
 * @details
 *   This function will erase one blocks on the on-chip flash.  After erasing,
 *   the block will be filled with 0xff bytes.  Read-only and execute-only
 *   blocks can not be erased.
 *
 * @param[in] blockStart
 *   Pointer to the flash page to erase. Must be aligned to beginning of page
 *   boundary.
 ******************************************************************************/
__RAMFUNCTION void TD_FLASH_ErasePage(uint32_t *blockStart)
{
    MSC->WRITECTRL |= MSC_WRITECTRL_WREN;

    // Load address
    MSC->ADDRB    = (uint32_t) blockStart;
    MSC->WRITECMD = MSC_WRITECMD_LADDRIM;

    // Send Erase Page command
    MSC->WRITECMD = MSC_WRITECMD_ERASEPAGE;

    // Wait for erase to complete
    while ((MSC->STATUS & MSC_STATUS_BUSY)) {
    	;
    }
}

 /***************************************************************************//**
 * @brief
 *   Computes CCITT CRC32.
 *
 * @param[in] buffer
 *   Pointer to the buffer containing the data.
 *
 * @param[in] size
 *   Size of the buffer in bytes.
 *
 * @return
 *   Returns the computed CRC32 of the buffer.
 ******************************************************************************/
 static uint32_t TD_FLASH_CRC32(uint32_t *buffer, uint32_t size)
 {
     uint32_t i, crc = 0, data;

     while (size--) {
         data = *buffer++;
         for (i = 32; i; i--) {
             if ((crc ^ data) & 1) {
                 crc = (crc >> 1) ^ 0x8408;
             } else {
                 crc = (crc >> 1);
             }
             data >>= 1;
         }
     }

     return crc;
 }

 /***************************************************************************//**
 * @brief
 *   Computes CRC-8
 *
 * @param[in] buffer
 *   Pointer to the buffer containing the data.
 *
 * @param[in] size
 *   Size of the buffer in bytes.
 *
 * @return
 *   Returns the computed CRC8 of the buffer.
 ******************************************************************************/
 static uint8_t TD_FLASH_CRC8(uint8_t *buffer, uint32_t size)
 {
     uint8_t i, crc = 0, data;
     uint8_t * pdata=buffer;

     while (size--) {
         data = *pdata++;
         for (i = 8; i; i--) {
             if ((crc ^ data) & 1) {
                 crc = (crc >> 1) ^ 0xEA;
             } else {
                 crc = (crc >> 1);
             }
             data >>= 1;
         }
     }

     return crc;
 }

 /***************************************************************************//**
 * @brief
 *   Writes a buffer to a given Flash region.
 *
 * @param[in] start
 *   Pointer to the flash region to write to.
 *
 * @param[in] buffer
 *   Pointer to the source buffer.
 *
 * @param[in] count
 *   Buffer size in bytes.
 *
 * @param[in] extended_buffer
 *   Pointer to the extended source buffer.
 *
 * @param[in] extended_count
 *   Extended buffer size in bytes.
 ******************************************************************************/
static void TD_FLASH_WriteRegion(uint32_t start, void *buffer, uint32_t count, void *extended_buffer, uint32_t extended_count)
{
    uint32_t *flash_pointer = (uint32_t *) start;
    uint32_t *ram_pointer = (uint32_t *) buffer;
    uint32_t i, acc = 0xFFFFFFFF;
    uint32_t crc;

    if (count % DWORDSZ) {

    	// Round count to DWORD
    	count += DWORDSZ;
    }
    count /= DWORDSZ;
    if (extended_count % DWORDSZ) {

    	// Round count to DWORD
    	extended_count += DWORDSZ;
    }
    extended_count /= DWORDSZ;

    // Compute buffer CRC
    crc = TD_FLASH_CRC32(ram_pointer, count);

    // Optimization - check if block is already erased. This will typically happen when the chip is new
    for (i = 0; i < FLASH_PAGE_SIZE / DWORDSZ; i++) {
        acc &= *flash_pointer++;
    }
    flash_pointer = (uint32_t *) start;

    __disable_irq();
    TD_FLASH_Init();

    // If the accumulator is unchanged, there is no need to do an erase
    if (acc != 0xFFFFFFFF) {
    	TD_FLASH_ErasePage(flash_pointer);
    }

    // Write buffer CRC first
    TD_FLASH_WriteWord(flash_pointer++, crc);

    // Copy buffer into Flash
    for (i = 0; i < count; i++) {
        TD_FLASH_WriteWord(flash_pointer++, *ram_pointer++);
    }
    if (extended_buffer != 0 && extended_count != 0) {

    	// Copy extended buffer into Flash if present
    	ram_pointer = (uint32_t *) extended_buffer;
        for (i = 0; i < extended_count; i++) {
            TD_FLASH_WriteWord(flash_pointer++, *ram_pointer++);
        }

        // Compute full page-wide CRC
        crc = TD_FLASH_CRC32((uint32_t *) start, FLASH_PAGE_SIZE / DWORDSZ - 1);

        // Write full page-wide CRC at the end of page
        TD_FLASH_WriteWord((uint32_t *) (start + FLASH_PAGE_SIZE - sizeof (uint32_t)), crc);
    }

    // Disable writing to the MSC
    MSC->WRITECTRL &= ~MSC_WRITECTRL_WREN;
    TD_Flash_Deinit();

    // Interrupts can be enabled whenever not writing to or erasing flash
    __enable_irq();
}

/***************************************************************************//**
* @brief
*   Reads a buffer from a given Flash region.
*
* @param[in] start
*   Pointer to the flash region to read from.
*
* @param[in] buffer
*   Pointer to the destination buffer.
*
* @param[in] count
*   Buffer size in bytes.
*
* @param[in] extended_buffer
*   Pointer to the destination extended buffer.
*
* @param[in] extended_count
*   Extended buffer size in bytes.
*
* @return
*   Returns true upon success, false if a checksum error has been detected
******************************************************************************/
static bool TD_FLASH_ReadRegion(uint32_t *start, void *buffer, uint32_t count, void *extended_buffer, uint32_t extended_count)
{
    uint32_t i, *flash_pointer = start;
    uint32_t *ram_pointer = (uint32_t *) buffer;
    uint32_t crc;

    if (count % DWORDSZ) {

    	// Round count to DWORD
    	count += DWORDSZ;
    }
    count /= DWORDSZ;
    if (extended_count % DWORDSZ) {

    	// Round count to DWORD
    	extended_count += DWORDSZ;
    }
    extended_count /= DWORDSZ;

    // Read the buffer CRC first
    crc = *flash_pointer++;

    for (i = 0; i < count; i++) {
        *ram_pointer++ = *flash_pointer++;
    }
    if (crc != TD_FLASH_CRC32(buffer, count)) {
        return false;
    }
    if (extended_buffer != 0 && extended_count != 0) {

    	// Copy Flash into extended buffer if present
    	ram_pointer = (uint32_t *) extended_buffer;
        for (i = 0; i < extended_count; i++) {
        	*ram_pointer++ = *flash_pointer++;
        }

        // Read the full page-wide CRC last
        crc = *(start + FLASH_PAGE_SIZE / DWORDSZ - 1);
        if (crc != TD_FLASH_CRC32((uint32_t *) start, FLASH_PAGE_SIZE / DWORDSZ - 1)) {
            return false;
        }
    }
    return true;
}

/** @} */

/*******************************************************************************
 **************************   PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup FLASH_PUBLIC_FUNCTIONS Public Functions
 * @{ */

/***************************************************************************//**
* @brief
* Delete all variables from flash.
*
*
******************************************************************************/
void TD_FLASH_DeleteVariables(void)
{
    __disable_irq();
    TD_FLASH_Init();

	TD_FLASH_ErasePage((uint32_t *) E2P_USER);

	 // Disable writing to the MSC
	 MSC->WRITECTRL &= ~MSC_WRITECTRL_WREN;
	 TD_Flash_Deinit();

	 // Interrupts can be enabled whenever not writing to or erasing flash
	 __enable_irq();
}

/***************************************************************************//**
* @brief
*   Update all the declared variables.
*
******************************************************************************/
void TD_FLASH_WriteVariables(void)
{
	uint32_t i, acc = 0xFFFFFFFF;
	uint32_t crc;
	uint32_t temp = 0, count;
	uint32_t total_word = 0;
	uint8_t *pdata;
	int j, k;

	// Optimization - check if block is already erased. This will typically happen when the chip is new
	for (i = (uint32_t) E2P_USER; i < ((uint32_t) E2P_USER + total_size + 1); i++) {
		acc &= *((int32_t *) i);
	}

	__disable_irq();
	TD_FLASH_Init();

	// If the accumulator is unchanged, there is no need to do an erase
	if (acc != 0xFFFFFFFF) {
		TD_FLASH_ErasePage((uint32_t *) E2P_USER);
	}

	// For each variable
	for (i = 0; i < data_count; i++) {

		// How many blocks of 4 bytes do we need?
		count = FlashDataList[i].data_size;
		if (count % DWORDSZ) {
			count += DWORDSZ;
		}
		count /= DWORDSZ;

		// Copy data pointer
		pdata = FlashDataList[i].data_pointer;

		// Compute crc
		crc = TD_FLASH_CRC8(FlashDataList[i].data_pointer, FlashDataList[i].data_size);

		// Append index and size info
		temp = (crc << 24) | (FlashDataList[i].data_size&0xFFFF) << 8 | (i);

		// Write first byte
		TD_FLASH_WriteWord((uint32_t *) (E2P_USER + total_word), temp);
		total_word += DWORDSZ;

		// Write data
		for (j = 0; j < count; j++) {
			temp = 0;
			for (k = 0; k < 4; k++) {
				if ((j << 2) + k < FlashDataList[i].data_size) {
					temp |= (*pdata++) << (k << 3);
				} else {
					break;
				}
			}
			TD_FLASH_WriteWord((uint32_t *) (E2P_USER + total_word), temp);
			total_word += DWORDSZ;
		}
	}

	// Disable writing to the MSC
	MSC->WRITECTRL &= ~MSC_WRITECTRL_WREN;
	TD_Flash_Deinit();

	// Interrupts can be enabled whenever not writing to or erasing flash
	__enable_irq();
}

/***************************************************************************//**
* @brief
*   Declare a variable to Flash and try to read its previous value.
*
* @param[in] variable
*   Pointer to the variable. This one must be global as data is NOT copied.
*
* @param[in] size
*   Variable size in bytes.
*
* @param[out] index
*   Index of variable for further read. Will be 0xFF if the flash is full or
*    if too many variable have already been added.
*
* @return
* 	Returns true if the variable has been found in flash and its value has been
* 	updated, false otherwise.
*
******************************************************************************/
bool TD_FLASH_DeclareVariable(uint8_t *variable, uint16_t size, uint8_t *index)
{
	uint16_t temp_size;

	// Make sure that there are not too many variables declared
	if (data_count > MAX_FLASH_DATA_POINTER) {
		if (index != 0) {
			*index = 0xFF;
		}
		return false;
	} else {

		// How many blocks of 4 bytes do we need?
		temp_size = size;
		if (temp_size % DWORDSZ) {
			temp_size += DWORDSZ;
		}
		temp_size /= DWORDSZ;

		// Make sure there is enough room for this variable
		if (total_size + size + 4 > FLASH_PAGE_SIZE) {
			if (index != 0) {
				*index = 0xFF;
			}
			return false;
		} else {

			// Append variable to the list
			FlashDataList[data_count].data_pointer = variable;
			FlashDataList[data_count].data_size = size;
			if (index != 0) {
				*index = data_count;
			}

			// Save size and data count
			total_size += size + 4;
			data_count++;

			// Try to read previous value
			if (TD_FLASH_ReadVariable(data_count - 1, variable) == 0) {
				return false;
			}
			return true;
		}
	}
}

/***************************************************************************//**
* @brief
*   Read a buffer from Flash memory.
*
* @param[in] index
*   Index of data into the List. You will get this parameter when using
*   TD_FLASH_DeclareFlashVariable to declare a new variable.
*
* @param[out] buffer
*   Pointer to the buffer where data value should be copied.
*
* @return
* 	Returns read data count if successful, 0 otherwise.
*
******************************************************************************/
uint16_t TD_FLASH_ReadVariable(uint8_t index, uint8_t *buffer)
{
	 uint32_t i, j, crc, count, temp;
	 uint32_t *pr;
	 uint8_t *pdata = buffer;

	 pr = (uint32_t *) E2P_USER;

	 // Make sure index is within valid range
	 if (index > data_count) {
		 return 0;
	 }

	 // Find out our data address
	 for (i = 0; i < index; i++) {
		count = FlashDataList[i].data_size;
		if (count % DWORDSZ) {
			count += DWORDSZ;
		}
		count /= DWORDSZ;
		pr += count + 1;
	 }

	// Get data size is blocks of 4 bytes.
	count = FlashDataList[index].data_size;
	if (count % DWORDSZ) {

	   // Round count to DWORD
	   count += DWORDSZ;
	}
	count /= DWORDSZ;
	temp = *pr++;
	crc = (temp >> 24) & 0xFF;

	 // Check index and size
	if (((temp & 0xFF) != index) || (((temp >> 8) & 0xFFFF) != FlashDataList[index].data_size)) {
		return 0;
	}

	// Read data
	for (i = 0; i < count; i++) {
	    temp = *pr++;
	    for (j = 0; j < 4; j++) {
	    	if ((i << 2) + j < FlashDataList[index].data_size) {
	    		*pdata++ = (temp >> (j << 3)) & 0xFF;
	    	} else {
	    		break;
	    	}
	    }
	 }

	// Make sure the CRC is fine
	if (crc != TD_FLASH_CRC8(buffer, FlashDataList[index].data_size)) {
	   return 0;
	}

	// Return read data size
	return FlashDataList[index].data_size;
}

/***************************************************************************//**
* @brief
*   Writes a buffer to Flash memory.
*
* @param[in] buffer
*   Pointer to the source buffer.
*
* @param[in] count
*   Buffer size in bytes.
******************************************************************************/
void TD_FLASH_Write(void *buffer, uint32_t count)
{
    TD_FLASH_WriteRegion((uint32_t)E2P_USER, buffer, count, 0, 0);
}

/***************************************************************************//**
* @brief
*   Reads a buffer from Flash memory.
*
* @param[in] buffer
*   Pointer to the destination buffer.
*
* @param[in] count
*   Buffer size in bytes.
*
* @return
*   Returns true upon success, false if a checksum error has been detected
******************************************************************************/
bool TD_FLASH_Read(void *buffer, uint32_t count)
{
    return TD_FLASH_ReadRegion((uint32_t *) E2P_USER, buffer, count, 0, 0);
}

/** @cond RESTRICTED */
/***************************************************************************//**
* @brief
*   Writes a buffer to factory Flash memory.
*
* @param[in] device
*   Pointer to the source TD_DEVICE buffer.
*
* @warning
*   Using this function destroys the unique module ID!
*
******************************************************************************/
void TD_FLASH_DeviceWrite(TD_DEVICE *device)
{
	TD_FLASH_WriteRegion(E2P_FACTORY, device, sizeof (TD_DEVICE), 0, 0);
}

/***************************************************************************//**
* @brief
*   Writes buffers to factory Flash memory.
*
* @param[in] device
*   Pointer to the source TD_DEVICE buffer.
*
* @param[in] device_ext
*   Pointer to the source TD_DEVICE_EXT buffer.
*
* @warning
*   Using this function destroys the unique module ID!
*
******************************************************************************/
void TD_FLASH_DeviceWriteExtended(TD_DEVICE *device, TD_DEVICE_EXT *device_ext)
{
	TD_FLASH_WriteRegion(E2P_FACTORY, device, sizeof (TD_DEVICE), device_ext, sizeof (TD_DEVICE_EXT));
}
/** @endcond */

/***************************************************************************//**
* @brief
*   Reads a buffer from factory Flash memory.
*
* @param[in] device
*   Pointer to the destination TD_DEVICE buffer.
*
* @return
*   Returns true upon success, false if a checksum error has been detected
******************************************************************************/
bool TD_FLASH_DeviceRead(TD_DEVICE *device)
{
    return TD_FLASH_ReadRegion((uint32_t *) E2P_FACTORY, device, sizeof (TD_DEVICE), 0, 0);
}

/***************************************************************************//**
* @brief
*   Reads a buffer from factory Flash memory.
*
* @param[in] device
*   Pointer to the destination TD_DEVICE buffer.
*
* @param[in] device_ext
*   Pointer to the destination TD_DEVICE_EXT buffer.
*
* @return
*   Returns true upon success, false if a checksum error has been detected
******************************************************************************/
bool TD_FLASH_DeviceReadExtended(TD_DEVICE *device, TD_DEVICE_EXT *device_ext)
{
    return TD_FLASH_ReadRegion((uint32_t *) E2P_FACTORY, device, sizeof (TD_DEVICE), device_ext, sizeof (TD_DEVICE_EXT));
}

/** @} */

/** @} (end addtogroup FLASH) */
