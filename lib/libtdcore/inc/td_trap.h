/***************************************************************************//**
 * @file
 * @brief Utility functions for the TDxxxx RF modules.
 * @author Telecom Design S.A.
 * @version 1.2.1
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

#ifndef __TD_TRAP_H
#define __TD_TRAP_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "td_config_ext.h"

#ifdef __cplusplus
extern "C" {
#endif

	/***********************************************************************//**
	 * @addtogroup TRAP
	 * @brief Trap functions for the TDxxxx RF modules
	 * @{
	 **************************************************************************/

	/***************************************************************************
	 *************************   DEFINES   *************************************
	 **************************************************************************/

	/** @addtogroup TRAP_DEFINES Defines
	 * @{ */

/** Magic value to sign a RAM trap trace */
#define TRAP_MAGIC_TRACE		0xA90B0B00

/** Maximum number of RAM TRAP traces */
#define TRAP_MAX_TRACE			18

/** Maximum number of user variables in RAM TRAP traces */
#ifdef EZR32LG230F256
#define TRAP_MAX_USER 			201
#else
#define TRAP_MAX_USER 			9
#endif

/** Flag for global oversize detection in RAM TRAP traces */
#define TRACE_MAGIC_OVERSIZE	(1 << 0)

/** Flag for stack overrun detection in RAM TRAP traces */
#define TRACE_MAGIC_OVERSTACK	(1 << 1)

/** Flag for stack trace in RAM TRAP traces */
#define TRACE_MAGIC_STACK		(1 << 6)

/** Flag for trap detection */
#define TRACE_MAGIC_TRAP		(1 << 7)

/** Flag for first custom trap handler */
#define TRAP_CUST_HANDLER1		1

/** Flag for second custom trap handler */
#define TRAP_CUST_HANDLER2		2

/** Integrated detection of re-entrance TD_CHECK_REENTRENCY_IN /
 * TD_CHECK_REENTRENCY_OUT */
#define TD_CHECK_REENTRENCY_IN(trap_fnc)	\
	static uint8_t __reenter_in = false;\
	uint8_t __reenter_msk = __get_PRIMASK();\
	__set_PRIMASK(1);\
	if (__reenter_in){\
		trap_fnc;\
	} else{\
		__reenter_in=true;\
	}\
	__set_PRIMASK(__reenter_msk);

	/** Integrated detection of re-entrance TD_CHECK_REENTRENCY_IN /
	 * TD_CHECK_REENTRENCY_OUT */
#define TD_CHECK_REENTRENCY_OUT	__reenter_in = false;

	/** @} */

/*******************************************************************************
	 *************************   ENUM   ****************************************
	 **************************************************************************/

	/** @addtogroup TRAP_ENUMERATIONS Enumerations
	 * @{ */

	/** TRAP information */
	typedef enum {
		TRAP_NONE,					/**< 0-Value not used, invalid trap */
		TRAP_LINE,					/**< 1-Trap at specific line */
		TRAP_SPI_MAX_LOCK,			/**< 2-Trap add a callback next to an SPI locked access */
		TRAP_SPI_INVALID_UNLOCK,	/**< 3-Trap unlock a non-locked SPI bus */
		TRAP_SPI_NOT_AVAILABLE,		/**< 4-Trap SPI can't be available here */
		TRAP_SPI_INVALID_ID,		/**< 5-Trap SPI ID out of range */
		TRAP_SPI_INVALID_BUS,		/**< 6-Trap SPI BUS out of range */
		TRAP_SPI_NI,				/**< 7-Trap SPI not implemented */
		TRAP_REMOVED_CODE,			/**< 8-Trap Code removed */
		TRAP_ILLEGAL_FREQ,			/**< 9-Trap Frequency forbidden */
		TRAP_SCHEDULER_QUEUE_OVF,	/**< 10-Trap Scheduler Queue overflow */
		TRAP_FLASH_VAR_FULL,		/**< 11-Trap Flash Variables Full */
		TRAP_FLASH_POINTER_OVF,		/**< 12-Trap Flash Pointers Overflow */
		TRAP_RADIO_CHIP,			/**< 13-Trap Radio Chip */
		TRAP_RTC_DELAY,				/**< 14-Trap RTC Delay */
		TRAP_NOT_ALLOWED_IN_IRQ,	/**< 15-Trap Can't do this in IRQ */
		TRAP_CUSTOM_1,				/**< 16-Trap User 1 */
		TRAP_CUSTOM_2,				/**< 17-Trap User 2 */
		TRAP_CUSTOM_3,				/**< 18-Trap User 3 */
		TRAP_CUSTOM_4,				/**< 19-Trap User 4 */
		TRAP_CUSTOM_5,				/**< 20-Trap User 5 */
		TRAP_CUSTOM_6,				/**< 21-Trap User 6 */
		TRAP_CUSTOM_7,				/**< 22-Trap User 7 */
		TRAP_CUSTOM_8,				/**< 23-Trap User 8 */
		TRAP_LAN_CALLBACK_LATE,		/**< 24-Trap LAN callback timeout */
		TRAP_GPS_HARD_ERR,			/**< 25-Trap GPS Hard error */
		TRAP_NMI_FAULT,				/**< 26-Trap Non Maskable Interrupt Fault */
		TRAP_HARD_FAULT,			/**< 27-Hard fault */
		TRAP_MEM_MANAGE_FAULT,		/**< 28-Memory manage fault */
		TRAP_BUS_FAULT,				/**< 29-Bus fault */
		TRAP_USAGE_FAULT,			/**< 30-Usage fault */
		TRAP_DUMP_NOT_AVAILABLE,	/**< 31-This type of dump is not available  */
		TRAP_BAD_SIGFOX_ID,			/**< 32-Bad SIGFOX ID */
		TRAP_BAD_SIGFOX_KEY,		/**< 33-Bad SIGFOX Key */
		TRAP_TIMER_INVALID,			/**< 34-Invalid timer id  */
		TRAP_AT_PERSIST_OVERSIZE,	/**< 35-AT Persist data oversized  */
		TRAP_USER_FLASH_OVW_CODE,	/**< 36-User Flash try to overwrite code  */
		TRAP_POWER_COEF_UNDEFINED,	/**< 37-Coef for power calculation unknown for desired level  */
		TRAP_POWER_CONTEXT_UNHANDLED,/**< 38-Unknown radio power context  */
		TRAP_FREQ_OUT_RANGE,		/**< 39-Frequency out of range  */
		TRAP_CUSTOM_FUNC,			/**< 40-call func in param  */
		TRAP_LAN_ADDRESS_SIZE,		/**< 41-Trying to use lan adress or mask longer than av. bits */
		TRAP_UART_CONFIG,			/**< 42-Invalid uart configuration */
		TRAP_TIMER_CONFIG,			/**< 43-Invalid timer configuration */
		TRAP_MOD_OUT_RANGE,			/**< 44-Modulation out of range  */
		TRAP_UNHANDLED_IRQ,			/**< 45-IRQ vector unhandled */
		TRAP_TRACERAM_SIZE_INVALID,	/**< 46-Trace ram size invalid */
		TRAP_ACCELERO_MODE,			/**< 47-Accelero mode transition invalid */
		TRAP_SCHEDULER_OVERRUN,		/**< 48-Scheduler manager overrun */
		TRAP_BLE,					/**< 49-Bluetooth error */
		TRAP_ASSERT_EFM,			/**< 50-emlib assertion error (only activated if DEBUG_EFM_USER is defined when compiling emlib)*/
		TRAP_FLASH_LAYOUT,			/**< 51-invalid flash layout*/
		TRAP_SIGFOX_CHANNEL,		/**< 52-Bad SIGFOX channel */
	} TD_TRAP_t;

	/** SYSTEM dumps */
	typedef enum {
		DUMP_GPIO,					/**< TD_GPIO_Dump() */
		DUMP_RF,					/**< TD_RF_Dump() */
		DUMP_IRQ,					/**< TD_IRQ_Dump() */
		DUMP_SCHEDULER,				/**< TD_SCHEDULER_Dump() */
		DUMP_GEOLOC,				/**< TD_GEOLOC_DUMP -> TD_UBX7_Dump() */
		DUMP_SPILOCK,				/**< TD_SPILock_Dump() */
		DUMP_LIS3DH,				/**< TD_ACCELERO_Dump() -> TD_LIS3DH_Dump with SPI Lock */
		DUMP_SENSOR,				/**< TD_SENSOR_Dump()*/
	} TD_Dump_t;

	/** @} */

	/***************************************************************************
	 *************************   TYPEDEFS   ************************************
	 **************************************************************************/

	/** @addtogroup TRAP_TYPEDEFS Typedefs
	 * @{ */

	/** Action to perform upon trap handler exit */
	typedef enum {
		TRAP_CONTINUE,				/**< Continue normal execution */
		TRAP_SLEEP,					/**< Sleep product (standard, EM2) */
		TRAP_DEEP_SLEEP,			/**< Sleep product (EM4 mode, can only be reset)*/
		TRAP_RESTART,				/**< Reboot product */
		TRAP_HANG					/**< Hang execution and stay here */
	} TD_TRAP_action_t;

	/** Trap trace */
#pragma pack(1)
	typedef struct {
		uint32_t magic;				///< Magic trace
		TD_TRAP_t trap;          	///< Trap
		uint16_t user_trap;			///< User defined trap
		uint32_t param;				///< Param
		uint16_t trace_cnt;			///< Trace count number
		uint16_t trace[TRAP_MAX_TRACE];	///< Trace data
		uint8_t user[TRAP_MAX_USER];	///< Trace count number
	} TD_TRAP_Frame_t;
#pragma pack()

	/** Callback when TRAP occurs.
	 * trap is TD_Trap_t enumeration that define source of TRAP
	 * param is context-dependent
	 * This function must return the action to take */
	typedef TD_TRAP_action_t (*TD_TRAP_callback_t)(TD_TRAP_t trap,
		uint32_t param);

	/** Default "Removed code" handler */
	#define TD_TRAP_HERE TD_TrapHere

	/** Default "Nop code" handler */
	#define TD_NOP_HERE TD_NopHere

	/** Remove all stack history. Only use of this macro is a desperate attempt
	 * to TRAP after a stack fault */
	#if defined(__ICCARM__)
	#define TD_STACK_RESET\
		extern char CSTACK$$Limit[];\
		__set_MSP((uint32_t)CSTACK$$Limit);
	#endif
	#if defined(__GNUC__)
	#define TD_STACK_RESET\
		extern char __cs3_stack[];\
		__set_MSP((uint32_t) (__cs3_stack - CONFIG_STACK_SIZE));
	#endif

	/** Bootloader handling function pointer */
	typedef void (*TD_StackProtect_t)(uint8_t enable);

	/** @} */

	/***************************************************************************
	 *************************   PROTOTYPES   **********************************
	 **************************************************************************/

	/** @addtogroup TRAP_USER_FUNCTIONS User Functions
	 * @{ */

	void TD_TRAP_Set(TD_TRAP_callback_t trap);
	TD_TRAP_callback_t TD_TRAP_Get(void);
	void TD_Trap(TD_TRAP_t trap, uint32_t param);
	bool TD_TRAP_DirectToFlash(TD_TRAP_t *trap, uint32_t *param);
	TD_TRAP_action_t TD_TRAP_Reset_Callback(TD_TRAP_t trap, uint32_t param);
	TD_TRAP_action_t TD_TRAP_Mini_Callback(TD_TRAP_t trap, uint32_t param);
	TD_TRAP_action_t TD_TRAP_Flash_Callback(TD_TRAP_t trap, uint32_t param);
	TD_TRAP_action_t TD_TRAP_Printf_Callback(TD_TRAP_t trap, uint32_t param);
	void TD_TRAP_VectorTableDump(void);
	void TD_TrapHere(void);
	bool TD_NopHere(void);
	void TD_TRAP_TraceDump(bool force);
	void TD_TRAP_TraceStore(TD_TRAP_t trap, uint32_t param, uint16_t user_trap);
	void TD_TRAP_TraceDumpSigfox(uint8_t retry);
	bool TD_TRAP_TraceUserSet(uint8_t entry, uint32_t val);
	void TD_TRAP_StackProtect(uint8_t enable);
	void TD_TRAP_StackTrace(void);
	bool TD_TRAP_StackCheck(void);
	void TD_TRAP_DumpSystemId(void);
	DECLARE_DYNAMIC(void, TD_SystemDump, TD_Dump_t dump);
#if defined(__GNUC__)
	uint8_t *TD_TRAP_TraceUserGetRaw(uint16_t *sz);
#endif
	bool TD_TRAP_TraceGet(TD_TRAP_t *trap, uint32_t *param, uint16_t *user_trap);
	uint8_t TD_TRAP_EFM_ASSERT_Get(uint16_t *buf);
	/** @} */

	/** @} (end addtogroup TRAP) */

#ifdef __cplusplus
}
#endif

#endif // __TD_TRAP_H
