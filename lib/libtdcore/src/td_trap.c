/***************************************************************************//**
 * @file
 * @brief Utility functions for the TDxxxx RF modules.
 * @author Telecom Design S.A.
 * @version 1.3.0
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2014-2016 Telecom Design S.A., http://www.telecomdesign.fr</b>
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

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <string.h>

#include <em_emu.h>
#include <em_mpu.h>

#include "td_config_ext.h"
#include "td_core.h"
#include "td_printf.h"
#include "td_scheduler.h"
#include "td_flash.h"
#include "td_printf.h"
#include "td_uart.h"
#include "td_rtc.h"
#include "td_trap.h"

#include <td_sigfox.h>
#include <td_spi.h>

/***************************************************************************//**
 * @addtogroup TRAP
 * @brief Trap functions for the TDxxxx RF modules. Debug purposes.
 * @{
 ******************************************************************************/

/*******************************************************************************
 *******************************   DEFINES   ***********************************
 ******************************************************************************/

/** @addtogroup TRAP_DEFINES Defines
 * @{ */

#ifndef __ICCARM__

/** Beginning of trace RAM from linker script */
extern const char __traceram_start;

/** Beginning of trace RAM from linker script */
#define TRACE_RAM_START   (uint32_t) &__traceram_start

/** Size of trace RAM in bytes */
#define TRACE_RAM_SIZE    sizeof(TD_TRAP_Frame_t)
#endif

/** Maximum number of trap on assertion failed */
#define MAX_EFM_ASSERT_TRAP		3

/** @} */

/*******************************************************************************
 *************************   PUBLIC VARIABLES   ********************************
 ******************************************************************************/

/** @addtogroup TRAP_GLOBAL_VARIABLES Global Variables
 * @{ */

/** Print trap callback function */
extern TD_TRAP_callback_t TD_Trap_Callback;

/** Flash trap callback function */
extern TD_TRAP_callback_t const TD_Trap_Callback2;

TD_TRAP_action_t TD_TRAP_Printf_Callback(TD_TRAP_t trap, uint32_t param);
TD_TRAP_action_t TD_TRAP_Flash_Callback(TD_TRAP_t trap, uint32_t param);
static void TD_TRAP_TraceDumpInt(bool force, uint32_t *stack);

/** @} */

/*******************************************************************************
 *************************   PRIVATE VARIABLES   *******************************
 ******************************************************************************/

/** @addtogroup TRAP_LOCAL_VARIABLES Local Variables
 * @{ */

/** Current trap number */
static TD_TRAP_t CurrentTrap = TRAP_NONE;

/** Current trap parameter */
static uint32_t CurrentParam = 0;

static uint16_t asserTrap[MAX_EFM_ASSERT_TRAP];

static uint8_t asserTrapCount = 0;

/** @} */

/*******************************************************************************
 **************************   PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup TRAP_USER_FUNCTIONS User Functions
 * @{ */

/***************************************************************************//**
* @brief
*   Perform a system Dump for a given type.
*
* @param[in] dump
*   The type of dump to perform.
******************************************************************************/
void DYNAMIC(TD_SystemDump)(TD_Dump_t dump)
{
	if ((dump < TD_TRAP_MaxSystemDump) &&
		TD_TRAP_SystemDumpFunc[(uint8_t) dump]) {
		TD_TRAP_SystemDumpFunc[(uint8_t) dump]();
	} else {
		TD_Trap(TRAP_DUMP_NOT_AVAILABLE, (uint32_t) dump);
	}
}

/***************************************************************************//**
* @brief
*   Minimal trap handler: print the trap and restart the CPU.
*
* @param[in] trap
*   The trap type that occurred.
*
* @param[in] param
*   The optional parameter associated to the trap type.
*
* @return
*   Returns TRAP_RESTART.
*******************************************************************************/
TD_TRAP_action_t TD_TRAP_Mini_Callback(TD_TRAP_t trap, uint32_t param)
{
	tfp_printf("*TRP %d,%d\r\n", (uint32_t) trap, param);
	return TRAP_RESTART;
}

/***************************************************************************//**
* @brief
*   Trap reset handler: just restart the CPU.
*
* @param[in] trap
*   The trap type that occurred.
*
* @param[in] param
*   The optional parameter associated to the trap type.
*
* @return
*   Returns TRAP_RESTART.
*******************************************************************************/
TD_TRAP_action_t TD_TRAP_Reset_Callback(TD_TRAP_t trap, uint32_t param)
{
	return TRAP_RESTART;
}

/***************************************************************************//**
* @brief
*   Default trap handler: verbose print and hang the CPU.
*
* @param[in] trap
*   The trap type that occurred.
*
* @param[in] param
*   The optional parameter associated to the trap type.
*
* @return
*   Returns TRAP_HANG.
*******************************************************************************/
TD_TRAP_action_t TD_TRAP_Printf_Callback(TD_TRAP_t trap, uint32_t param)
{
	static const char trap_list[] = \
		"Invalid TRAP0|"					//0
		"Trap at line|"
		"SPI Lock|"
		"SPI Invalid Unlock (not locked)|"
		"SPI Unavailable|"
		"SPI Invalid ID|"					//5
		"SPI Invalid BUS|"
		"SPI Configuration Not Implemented|"
		"Code Removed|"
		"Frequency not allowed|"
		"Scheduler Queue Overflow|"			//10
		"Flash Variables Full|"
		"Flash Pointer Overflow|"
		"Radio Chip|"
		"RTC Delay|"
		"Forbidden in IRQ context|"			//15
		"Custom 1|"
		"Custom 2|"
		"Custom 3|"
		"Custom 4|"
		"Custom 5|"							//20
		"Custom 6|"
		"Custom 7|"
		"Custom 8|"
		"LAN Callback take too much time|"
		"GPS hard error|"					//25
		"NMI|"
		"Hard Fault|"
		"Mem manage|"
		"Bus Fault|"
		"Usage fault|"						//30
		"Dump not available|"
		"Bad SIGFOX ID|"
		"Bad SIGFOX Key|"
		"Invalid Timer ID|"
		"AT Persist oversized|"				//35
		"User Flash Overwrite Code|"
		"Pwr Coeff undefined|"
		"Pwr ctx unknown|"
		"Freq out range|"
		"Custom ext trap|"					//40
		"Lan address size overrange|"
		"Uart config error|"
		"timer config error|"
		"rfmod out range|"
		"unhandled IRQ|"					//45
		"traceram invalid sz|"
		"accelero mode trap|"
		"scheduler overrun|"
		"bluetooth init error|";
	const char *tstr = trap_list;

	uint64_t time;
	uint32_t msec, t;
	uint16_t hour, min, sec;
	char const *str = NULL;
	char *(*ptr)(uint32_t*, TD_TRAP_t *);
	uint8_t trap_lst_cnt = 0;

	time = TD_SCHEDULER_GetTime();

	// If trap have an embed trap function
	if (trap == TRAP_CUSTOM_FUNC) {
		ptr = (char*(*)(uint32_t *, TD_TRAP_t*))(param);
		str = ptr(&param, &trap);
	} else {
		while (*tstr && trap_lst_cnt < trap) {
			if (*tstr == '|') {
				trap_lst_cnt++;
			}
			tstr++;
		}
		// Standard trap
	}
	msec = time & 0x3FFF;
	time >>= 15;
	t = time;
	sec = t % 60;
	t = t / 60;
	min = t % 60;
	t = t / 60;
	hour = t % 24;
	t = t / 24;

	tfp_printf(LGC("[TRAP] Time:%d.%02d:%02d:%02d.%3d : "),
		(uint32_t) t, hour, min, sec, (msec * 1000) >> 15);
	if (str || tstr) {
		tfp_printf("%d:", (uint32_t) trap);
		if (str) {
			tfp_printf(str);
		} else {
			while (*tstr != '|') {
				tfp_printf(LGC("%c"), *tstr++);
			}
		}
	} else {
		tfp_printf("Code:%d", (uint32_t) trap);
	}
	tfp_printf(LGC(" | Param:0x%08X(%d)\r\n"), param, param);

	// Grab callstack
	SystemInit();

	// Display
	TD_TRAP_TraceDumpInt(true, (uint32_t *) param);
	if (trap >= TRAP_SPI_MAX_LOCK && trap <= TRAP_SPI_NI) {
		TD_SPI_ConfDump();
		TD_SPI_LockDump();
	}
	return TRAP_HANG;
}

/***************************************************************************//**
* @brief
*   Override the default trap handler.
*
* @param[in] trap
*   Pointer to the TD_TRAP_callback_t callback function to setup.
*******************************************************************************/
void TD_TRAP_Set(TD_TRAP_callback_t trap)
{
	TD_Trap_Callback = trap;
}

/***************************************************************************//**
* @brief
*   Return default trap handler.
*******************************************************************************/
TD_TRAP_callback_t TD_TRAP_Get(void)
{
	return TD_Trap_Callback;
}

/***************************************************************************//**
* @brief
*   Default handler for "Removed code".
******************************************************************************/
void TD_TrapHere(void)
{
	TD_Trap(TRAP_REMOVED_CODE, 0);
}

/***************************************************************************//**
* @brief
*   Default handler for "Nop code".
*******************************************************************************/
bool TD_NopHere(void)
{
	return false;
}

/***************************************************************************//**
* @brief
*   Store TRAP trace information and trace in RAM to recover them upon startup.
*
* @param[in] trap
*   The trap type that occurred.
*
* @param[in] param
*   The optional parameter associated to the trap type.
*
* @param[in] user_trap
*   The optional user parameter.
*******************************************************************************/
void TD_TRAP_TraceStore(TD_TRAP_t trap, uint32_t param, uint16_t user_trap)
{

#if defined(__GNUC__)
	TD_TRAP_Frame_t *f = (TD_TRAP_Frame_t *) TRACE_RAM_START;

	f->user_trap = user_trap;
	f->trap = trap;
	f->param = param;
	if (sizeof (TD_TRAP_Frame_t) > TRACE_RAM_SIZE) {
		f->magic = TRAP_MAGIC_TRACE | TRACE_MAGIC_OVERSIZE | TRACE_MAGIC_TRAP;
	} else {
		f->magic = TRAP_MAGIC_TRACE | TRACE_MAGIC_TRAP;
	}
#endif

}

/***************************************************************************//**
* @brief
*   Get TRAP trace information (and reset trap stored flag).
*
* @param[in] trap
*   Pointer to the trap type that occurred.
*
* @param[in] param
*   Pointer to the optional parameter associated to the trap type.
*
* @param[in] user_trap
*   Pointer to the optional user parameter.
*
* @return
*   True if trap information available
*******************************************************************************/
bool TD_TRAP_TraceGet(TD_TRAP_t *trap, uint32_t *param, uint16_t *user_trap)
{
#if defined(__GNUC__)
	uint32_t magic, magic_flag;
	TD_TRAP_Frame_t *f = (TD_TRAP_Frame_t *) TRACE_RAM_START;

	magic = (f->magic & 0xFFFFFF00);
	if (magic != TRAP_MAGIC_TRACE) {
		return false;
	}
	magic_flag = f->magic & 0xFF;
	if (magic_flag & TRACE_MAGIC_TRAP) {
		f->magic &= (~TRACE_MAGIC_TRAP);
		*trap = f->trap;
		*param = f->param;
		*user_trap = f->user_trap;
		return true;
	}
#endif
	return false;
}

/***************************************************************************//**
* @brief
*   Clear TRAP trace information in RAM.
*******************************************************************************/
void TD_TRAP_TraceClear(void)
{

#if defined(__GNUC__)
	TD_TRAP_Frame_t *f = (TD_TRAP_Frame_t *) TRACE_RAM_START;
	f->magic = 0;
#endif

}

/***************************************************************************//**
* @brief
*   Set a user-provided TRAP trace.
*
* @param[in] entry
*   Index in the trap frame to store the user-provided value.
*
* @param[in] value
*   The value to store in the TRAP trace.
*
* @return
*   Returns true if the operation was successful, false otherwise.
*******************************************************************************/
bool TD_TRAP_TraceUserSet(uint8_t entry, uint32_t value)
{

#if defined(__GNUC__)
	TD_TRAP_Frame_t *f = (TD_TRAP_Frame_t *) TRACE_RAM_START;
	if (entry >= sizeof (f->user) / sizeof (f->user[0])) {
		return false;
	}
	f->user[entry] = value;
	return true;
#else
	return false;
#endif

}

#if defined(__GNUC__)
/***************************************************************************//**
* @brief
*   Get raw user trace buffer.
*
* @param[in] *size
*   Size of buffer
*
* @return
*   Pointer to buffer.
*******************************************************************************/
uint8_t *TD_TRAP_TraceUserGetRaw(uint16_t *size)
{
	extern char __cs3_region_start_ram[];
	TD_TRAP_Frame_t *f = (TD_TRAP_Frame_t *) TRACE_RAM_START;

	*size = sizeof (f->user);
	if ((TRACE_RAM_START + sizeof (TD_TRAP_Frame_t)) >=
		(uint32_t)__cs3_region_start_ram) {
		TD_Trap(TRAP_TRACERAM_SIZE_INVALID, (uint32_t)__cs3_region_start_ram);
	}
	return &f->user[0];
}
#endif

/***************************************************************************//**
* @brief
*   Returns the status of he given NVIC interrupt.
*
* @param[in] IRQn
*   The IRQ type to check for.
*
* @return
*   Returns 1 if an IRQ is pending, O otherwise.
*******************************************************************************/
__STATIC_INLINE uint32_t NVIC_GetEnabledIRQ(IRQn_Type IRQn)
{
	 // Return 1 if pending else 0
	return (uint32_t) ((NVIC->ISER[(uint32_t) (IRQn) >> 5] &
		(1 << ((uint32_t) (IRQn) & 0x1F))) ? 1 : 0);
}

/***************************************************************************//**
* @brief
*   Dump the vector table.
*******************************************************************************/
void TD_TRAP_VectorTableDump(void)
{
int i;
uint32_t v;
	static const char *lst[] = {"SP", "Reset", "NMI", "HardF", "MMF", "BusF",
	"UsgF",	"Rsvd", "Rsvd", "Rsvd", "Rsvd", "SVCall", "Dbg", "Rsvd", "PendSV",
	"SysTick", "IRQxx"};

	tfp_printf(LGC("IRQTable:0x%08X\r\n"), SCB->VTOR);
	v = NVIC->IABR[0];
	tfp_printf(LGC("NVIC->IABR:0x%08X,0x%08X\r\n"),v, NVIC->IABR[1]);
	v = NVIC->ISPR[0];
	tfp_printf(LGC("NVIC->ISPR:0x%08X,0x%08X\r\n"),v, NVIC->ISPR[1]);
	v = NVIC->ISER[0];
	tfp_printf(LGC("NVIC->ISER:0x%08X,0x%08X\r\n"),v, NVIC->ISER[1]);
	for (i = 0; i < 64 * 4; i += 4) {
		if (i >= 16 * 4) {
			v = SCB->VTOR + i;
			tfp_printf(LGC("0x%08X:%03d:0x%08X:E:%d:P:%d:%s\r\n"),
					v,
					(i >> 2) - 16,
					*(uint32_t *)(SCB->VTOR + i),
					NVIC_GetEnabledIRQ((IRQn_Type)((i >> 2) - 16)),
					NVIC_GetPendingIRQ((IRQn_Type)((i >> 2) - 16)),
					(i >> 2) < sizeof (lst) / sizeof (char *) ? lst[i >> 2] : "");
		} else {
			v = SCB->VTOR + i;
			tfp_printf(LGC("0x%08X:%03d:0x%08X:%s\r\n"),
				v,
				(i >> 2) - 16,
					*(uint32_t *) (SCB->VTOR + i),
					(i >> 2) < sizeof (lst) / sizeof (char *) ? lst[i >> 2] : "");
		}
	}
}

/***************************************************************************//**
* @brief
*   Dump the the system IDs.
*******************************************************************************/
void TD_TRAP_DumpSystemId(void)
{
	tfp_printf(LGC("Unique ID  : 0x%08X%08X\r\n"), *(uint32_t *) 0x0FE081F4,
		*(uint32_t *) 0x0FE081F0);
	tfp_printf(LGC("Flash size : %4d kb\r\n"), *(uint16_t *) 0x0FE081F8);
	tfp_printf(LGC("Ram size   : %4d kb\r\n"), *(uint16_t *) 0x0FE081FA);
	tfp_printf(LGC("Part number: %d\r\n"), *(uint16_t *) 0x0FE081FC);
	tfp_printf(LGC("Part family: %d\r\n"), *(uint8_t *) 0x0FE081FE);
	tfp_printf(LGC("Part rev   : %d\r\n"), *(uint8_t *) 0x0FE081FF);
}

/***************************************************************************//**
* @brief
*   Internal function to dump TRAP trace information.
*
* @param[in] force
*   Set to true to force trace dump even if the trace seems corrupted, false
*   otherwise.
*
* @param[in] stack
*   Pointer to the stack.
*******************************************************************************/
static void TD_TRAP_TraceDumpInt(bool force, uint32_t *stack)
{

#if defined(__GNUC__)
	extern char __cs3_stack[];
	uint32_t magic;
	uint16_t i, j;
	uint8_t magic_flag;
	TD_TRAP_Frame_t *f;
#endif
	uint32_t fs, v;

	tfp_printf(LGC("-- TRAP DUMP --\r\n"));
#if defined(__GNUC__)
	tfp_printf(LGC("MSP:0x%08X LINK:0x%08X\r\n"), (uint32_t) stack, __builtin_return_address(0));
#else
	tfp_printf(LGC("MSP:0x%08X\r\n"), (uint32_t) stack);
#endif

	tfp_printf(LGC("IRQTable:0x%08X\r\n"), SCB->VTOR);
	v = NVIC->IABR[0];
	tfp_printf(LGC("NVIC->IABR:0x%08X,0x%08X\r\n"), v, NVIC->IABR[1]);
	tfp_printf(LGC("HardFaultStR:0x%08X\r\n"), SCB->HFSR);
	if (SCB->HFSR & 0x40000000) {

		// A fault is not handled and lead to a hard fault
		tfp_printf(LGC("FaultEscalation\r\n"));
	}
	fs = SCB->CFSR;
	tfp_printf(LGC("FaultStatus:0x%08X:"), fs);
	if (fs & 0x1) {
		tfp_printf(LGC("ExecuteInvalidAdd,"));
	}
	if (fs & 0x2) {
		tfp_printf(LGC("DataAccesError,"));
	}
	if (fs & 0x8) {
		tfp_printf(LGC("UnstackEx,"));
	}
	if (fs & 0x10) {
		tfp_printf(LGC("StackEx,"));
	}
	if (fs & 0x100) {
		tfp_printf(LGC("ExecBusErr,"));
	}
	if (fs & 0x200) {
		tfp_printf(LGC("DataBusErr,"));
	}
	if (fs & 0x400) {
		tfp_printf(LGC("DataBusErr2,"));
	}
	if (fs & 0x800) {
		tfp_printf(LGC("UnstackBEx,"));
	}
	if (fs & 0x1000) {
		tfp_printf(LGC("StackBEx,"));
	}
	if (fs & 0x10000) {
		tfp_printf(LGC("UndefInst,"));

		}
	if (fs & 0x20000) {
		tfp_printf(LGC("InvEPSR,"));
	}
	if (fs & 0x40000) {
		tfp_printf(LGC("InvPC,"));
	}
	if (fs & 0x80000) {
		tfp_printf(LGC("NoCopro,"));
	}
	if (fs & 0x1000000) {
		tfp_printf(LGC("UnalignMemAcc,"));
	}
	if (fs & 0x2000000) {
		tfp_printf(LGC("DivByZero,"));
	}
	tfp_printf("\r\n");
	if (fs & 0x80) {
		tfp_printf(LGC("MemFaultAdd:0x%08X\r\n"), SCB->MMFAR);
	}
	if (fs & 0x8000) {
		tfp_printf(LGC("BusFaultAdd:0x%08X\r\n"), SCB->BFAR);
	}
#if defined(__GNUC__)
	if (fs & (0x200000 | 0x10000 | 0x1)) {
		if ((void *) stack > (void *) (__cs3_stack - CONFIG_STACK_SIZE) &&
			(void *) stack < (void *) __cs3_stack) {
			tfp_printf(LGC("Add:0x%08X\r\n"), *(stack + 3));
		} else {
			tfp_printf(LGC("Add:-invalid stack-\r\n"));
		}
	}
#endif

#if defined(__GNUC__)
	f = (TD_TRAP_Frame_t *) TRACE_RAM_START;
	tfp_printf(LGC("Stack:0x%08X-0x%08X\r\n"),
		__cs3_stack - CONFIG_STACK_SIZE, __cs3_stack);
	magic = (f->magic & 0xFFFFFF00);
	if (!force && (magic != TRAP_MAGIC_TRACE)) {
		return;
	}
	stack = (uint32_t *) (__GET_MSP - 0x40);
	for (i = 0; i < 16; i++) {
		tfp_printf("0x%08X : ", stack);
		for (j = 0; j < 4; j++) {
			tfp_printf("%08X ", *stack);
			stack++;
		}
		tfp_printf("\r\n");
	}
	tfp_printf(LGC("-- TRAP Trace@0x%08X --\r\n"), (int32_t) f);
	magic_flag = f->magic & 0xFF;
	if (magic != TRAP_MAGIC_TRACE) {
		tfp_printf(LGC("TRAP Trace invalid : 0x%08X<=>0x%08X\r\n"),
			f->magic, TRAP_MAGIC_TRACE);
	}
	if (magic_flag & TRACE_MAGIC_OVERSIZE) {
		tfp_printf(LGC("Trace glob size overflow !!\r\n"));
	}
	if (magic_flag & TRACE_MAGIC_OVERSTACK) {
		tfp_printf(LGC("Trace stack truncated\r\n"));
	}
#if TRAP_MAX_USER > 8
	for (i = 0; i < 8; i++) {
		tfp_printf(LGC("Var %d:%d\r\n"), i, f->user[i]);
	}
#else
	for (i = 0; i < TRAP_MAX_USER; i++) {
		tfp_printf("Var %d:%d\r\n", i, f->user[i]);
	}
#endif
	if (magic_flag & TRACE_MAGIC_TRAP) {
		tfp_printf(LGC("TRP %d,%d\r\n"), (uint32_t) f->trap, f->param);
	}
	if (magic_flag & TRACE_MAGIC_STACK) {
		tfp_printf(LGC("-Stack trace-\r\n"));
		for (i = 0; i < f->trace_cnt; i++) {
			tfp_printf(LGC("%2d|0x%08X\r\n"),
				i, (((uint32_t) f->trace[i]) << 1) + 1);
		}
	}
#endif

}

/***************************************************************************//**
* @brief
*   Dump TRAP trace information.
*
* @param[in] force
*   Set to true to force trace dump even if the trace seems corrupted, false
*   otherwise.
*******************************************************************************/
void TD_TRAP_TraceDump(bool force)
{
	TD_TRAP_TraceDumpInt(force, (uint32_t *) __GET_MSP);
}

/***************************************************************************//**
* @brief
*   Dump TRAP trace information over the SIGFOX network.
*
* @param[in] retry
*   Number of retries to perform for SIGFOX RF frames.
*******************************************************************************/
void TD_TRAP_TraceDumpSigfox(uint8_t retry)
{

#if defined(__GNUC__)
	uint32_t magic;
	uint8_t magic_flag;
	uint8_t sig_frame[12];
	TD_TRAP_Frame_t *f = (TD_TRAP_Frame_t *) TRACE_RAM_START;
	uint8_t *s, *se;
	uint8_t sz, n;

	tfp_printf(LGC("TraceDumpSigfox\r\n"));
	magic = (f->magic & 0xFFFFFF00);
	sig_frame[0] = (magic == TRAP_MAGIC_TRACE) ? 0xC0 : 0xB0;
	magic_flag = f->magic & 0xFF;
	sig_frame[1] = magic_flag;
	sig_frame[2] = f->trace_cnt;
	memcpy(&sig_frame[3], &f->user[0], 9);
	if ((!(magic_flag & TRACE_MAGIC_STACK)) || (sig_frame[0] == 0xB0)) {
		f->trace_cnt = 0;
	}
	tfp_printf(LGC("Send HEADER:0x%02X %d frame will follow ...\r\n"),
		sig_frame[0], (f->trace_cnt * 2 + 10) / 11);
	TD_SIGFOX_Send(sig_frame, 12, retry);
	s = (uint8_t *) f->trace;
	sz = f->trace_cnt * 2;
	se = s + sz;
	while (s < se) {
		sig_frame[0]++;
		n = sz;
		if (n > 11) {
			n = 11;
		}
		memcpy(&sig_frame[1], s, n);
		tfp_printf(LGC("Send FRAME:0x%02X\r\n"), sig_frame[0]);
		TD_SIGFOX_Send(sig_frame, n + 1, retry);
		s += n;
	}
	if (magic_flag & TRACE_MAGIC_TRAP) {
		sig_frame[0]++;
		sig_frame[1] = f->trap;
		memcpy(&sig_frame[2], &f->param, 4);
		TD_SIGFOX_Send(sig_frame, 6, retry);
	}
#endif

}

/***************************************************************************//**
* @brief
*   This function gets called when something goes wrong and the system can't
*   automatically handle it.
*   To find out what happened exactly, you can then se debug stack TRAP traces.
*
* @param[in] trap
*   The trap type that occurred.
*
* @param[in] param
*   The optional parameter associated to the trap type.
*******************************************************************************/
void TD_Trap(TD_TRAP_t trap, uint32_t param)
{
	static bool trap_in_progress = false;

	if (trap_in_progress) {
		return;
	}
	trap_in_progress = true;
	TD_TRAP_TraceStore(trap, param, 0);

	// No trap handler, hang here
	if (!TD_Trap_Callback) {
		while (1) {
			;
		}
	}

	// Call trap handler
	switch (TD_Trap_Callback(trap, param)) {
	case TRAP_CONTINUE:
		trap_in_progress = false;
		return;
		break;
	case TRAP_SLEEP:
		TD_RTC_Sleep();
		break;
	case TRAP_DEEP_SLEEP:
		EMU_EnterEM4();
		break;
	case TRAP_RESTART:
		NVIC_SystemReset();
		break;
	default:
	case TRAP_HANG:
		while (1) {
			;
		}
		break;
	}
}

/***************************************************************************//**
* @brief
*   Flash trap handler: write arguments to Flash memory, then perform normal
*   trap handling.
*
* @param[in] trap
*   The trap type that occurred.
*
* @param[in] param
*   The optional parameter associated to the trap type.
*
* @return
*   Returns the TD_TRAP_action_t action to perform upon exit.
*******************************************************************************/
TD_TRAP_action_t TD_TRAP_Flash_Callback(TD_TRAP_t trap, uint32_t param)
{
	CurrentTrap = trap;
	CurrentParam = param;
	TD_FLASH_WriteVariables();
	return TD_Trap_Callback2(trap, param);
}

/***************************************************************************//**
* @brief
*   Direct Trap to flash storage. Must be called once near startup.
*   Return true if a trap was detected in flash.
*
* @param[out] trap
*  Previous trap stored in flash
*
* @param[out] param
*  Previous parameters stored in flash
*******************************************************************************/
bool TD_TRAP_DirectToFlash(TD_TRAP_t *trap, uint32_t *param)
{
	if (!TD_FLASH_DeclareVariable((uint8_t *) &CurrentTrap,
		sizeof(TD_TRAP_t), 0)) {
		CurrentTrap = TRAP_NONE;
	}
	if (!TD_FLASH_DeclareVariable((uint8_t *) &CurrentParam,
		sizeof(CurrentParam), 0)) {
		CurrentParam = 0;
	}
	*trap = CurrentTrap;
	*param = CurrentParam;
	CurrentTrap = TRAP_NONE;
	CurrentParam = 0;
	TD_Trap_Callback = TD_TRAP_Flash_Callback;
	return *trap != TRAP_NONE;
}

#ifndef __ICCARM__

/***************************************************************************//**
* @brief
*   Enable/Disable stack overflow protection by adding a 32 bytes stack
*   protection at end of stack.
*   32 last byte of stack will be lost and all access will throw a MemManage
*   Fault
*
* @param[in] enable
*  Enable stack protection
*******************************************************************************/
void TD_TRAP_StackProtect(uint8_t enable)
{
	extern char __cs3_stack[];
	MPU_RegionInit_TypeDef mpu = {
		true,
		0,
		0,
		mpuRegionSize4Gb,
		mpuRegionApFullAccess,
		0,
		1,
		1,
		0,
		0,
		0};

	if (!enable) {
		MPU_Disable();
		return;
	}
	MPU_ConfigureRegion(&mpu);
	mpu.regionNo = 1;
	mpu.baseAddress = ((uint32_t) ((__cs3_stack - CONFIG_STACK_SIZE + 1))) & (~0x1F);
	mpu.size = mpuRegionSize32b;
	mpu.accessPermission = mpuRegionNoAccess;
	MPU_ConfigureRegion(&mpu);
	MPU_Enable(0);
}

/***************************************************************************//**
* @brief
*   Low-level initialization before main(), overloaded to retrieve boot traces.
*
* @note
*   Remember that at this stage absolutely nothing is initialized...
*******************************************************************************/
#if !defined(EZR32LG230F256) && !defined(EZR32LG230F128)
void SystemInit(void)
{
	extern const char __traceram_start;
	extern char __cs3_stack[];
	extern char __cs3_start_asm[];
	extern char __cs3_rodata[];
	uint32_t *s = (uint32_t *) (__cs3_stack - CONFIG_STACK_SIZE);
	uint32_t i;
	uint32_t *sp = (uint32_t *) __GET_MSP;
	bool over = false;
	TD_TRAP_Frame_t *f = (TD_TRAP_Frame_t *) TRACE_RAM_START;

	f->trace_cnt = 0;
	if ((uint32_t)sp > (uint32_t) __cs3_stack - 32) {
		sp = 0;
	}
	if (sp < s) {

		// No stack trace in stack overflow
		return;
	}
	for (i = 0; i < CONFIG_STACK_SIZE >> 2; i++) {

		// Returned 'LINK' processor register value in stack trace seem to
		// always be odd
		if (s < sp) {
			s++;
			continue;
		}
		if (((*s) > (uint32_t) __cs3_start_asm) &&
			((*s) < (uint32_t) __cs3_rodata) && ((*s) & 1)) {
			f->trace[f->trace_cnt++] = ((*s) >> 1);
		}
		if (f->trace_cnt >= TRAP_MAX_TRACE) {
			over = true;
			break;
		}
		s++;
	}

	// Here &0xFFFFFF00 lead to immediate that is not initialized
	i = f->magic;
	i = i ^ (i ^ 0xFF);
	if (i != TRAP_MAGIC_TRACE) {
		f->magic = TRAP_MAGIC_TRACE;
	}
	f->magic |= (over ? TRACE_MAGIC_OVERSTACK : 0) | (TRACE_MAGIC_STACK);
}
#endif
#endif

/***************************************************************************//**
* @brief
*   Check stack corruption, use last stack byte for special value
*
* @return
*   true if sequence broken, false otherwise
*
* @note
*   call a first time and ignore return result. After that each call
*   should return true or someone else have modified last stack byte
*******************************************************************************/
bool TD_TRAP_StackCheck(void)
{
#ifndef __ICCARM__
	extern char __cs3_stack[];
	uint8_t *s = (uint8_t *) (__cs3_stack - CONFIG_STACK_SIZE);
#else
	extern char CSTACK$$Base[];
	uint8_t *s = (uint8_t *) (CSTACK$$Base);
#endif
	static uint8_t stack_seq = 0xCC;

	if (*s != stack_seq) {
		*s = stack_seq;
		return false;
	} else {
		stack_seq++;
		*s = stack_seq;
		return true;
	}
}

/***************************************************************************//**
* @brief
*   Output a Stack Trace
*******************************************************************************/
void TD_TRAP_StackTrace(void)
{
	extern const char __traceram_start;
#ifndef __ICCARM__
	extern char __cs3_stack[];
	extern char __cs3_start_asm[];
	extern char __cs3_rodata[];
	uint32_t *s = (uint32_t *) (__cs3_stack - CONFIG_STACK_SIZE);
	char *limit = __cs3_stack;
	char *code_start = __cs3_start_asm;
	char *code_end = __cs3_rodata;
#else
	extern char CSTACK$$Base[];
	extern char CSTACK$$Limit[];
	uint32_t *s = (uint32_t *) (CSTACK$$Base);
	char *limit = CSTACK$$Limit;
	char *code_start = (char *) 0x9C;
	char *code_end = (char *) 0x7C00;
#endif
	uint32_t i;
	uint32_t *sp = (uint32_t *) __GET_MSP;

	tfp_printf(LGC("---- STACK TRACE ----\r\n"));
	tfp_printf(LGC("SP:0x%08X\r\n"), sp);
	tfp_printf(LGC("Code:0x%08X-0x%08X\r\n"), code_start, code_end);
	tfp_printf(LGC("Stack:0x%08X-0x%08X\r\n"), s, limit);
	if ((uint32_t) sp > (uint32_t) limit - 32) {
		sp = 0;
	}
	if (sp < s) {

		// No stack trace in stack overflow
		return;
	}
	for (i = 0; i < CONFIG_STACK_SIZE >> 2; i++) {

		// Returned 'LINK' processor register value in stack trace seem to
		// always be odd
		if (s < sp) {
			s++;
			continue;
		}
		if (((*s) > (uint32_t) code_start) &&
			((*s) < (uint32_t) code_end) && ((*s) & 1)) {
			tfp_printf("0x%08X\r\n", ((*s) >> 1));
		}
		s++;
	}
}

/***************************************************************************//**
* @brief
*   NMI (Non Maskable IRQ) handler, overloaded to call the trap system.
*******************************************************************************/
void NMI_Handler(void)
{
	TD_Trap(TRAP_NMI_FAULT, __GET_MSP);
}

/***************************************************************************//**
* @brief
*   Hardware Fault handler, overloaded to call the trap system.
*   Hard Fault :
*   	* Error during exception processing
*    	* Bus fault during IRQ vector reading
*******************************************************************************/
void HardFault_Handler(void)
{
	TD_Trap(TRAP_HARD_FAULT, __GET_MSP);
}

/***************************************************************************//**
* @brief
*   Memory Management handler (MPU system), overloaded to call the trap system.
*******************************************************************************/
void MemManage_Handler(void)
{
	TD_Trap(TRAP_MEM_MANAGE_FAULT, __GET_MSP);
}

/***************************************************************************//**
* @brief
*   Bus Fault handler, overloaded to call the trap system.
*******************************************************************************/
void BusFault_Handler(void)
{
	TD_Trap(TRAP_BUS_FAULT, __GET_MSP);
}

/***************************************************************************//**
* @brief
*   Usage Fault handler, overloaded to call the trap system.
*******************************************************************************/
void UsageFault_Handler(void)
{
	TD_Trap(TRAP_USAGE_FAULT, __GET_MSP);
}

/***************************************************************************//**
* @brief
*   Default handler, overloaded to call the trap system.
*******************************************************************************/
void DefaultHandler(void)
{
	uint32_t m;
	uint8_t i;

	//tfp_printf("%08X %08X\r\n",NVIC->IABR[0],NVIC->IABR[1]);
	m = 1;
	for (i = 0; i < 32; i++) {
		if (NVIC->IABR[0] & m) {
			TD_Trap(TRAP_UNHANDLED_IRQ, i);
		}
		m <<= 1;
	}
	m = 1;
	for (i = 0; i < 32; i++) {
		if (NVIC->IABR[1] & m) {
			TD_Trap(TRAP_UNHANDLED_IRQ, i + 32);
		}
		m <<= 1;
	}
	TD_Trap(TRAP_UNHANDLED_IRQ, 0xFFFFFFFF);
}

/***************************************************************************//**
 * @brief
 *   EFM assert handling.
 *
 *   This function is invoked through EFM_ASSERT() macro usage only, it should
 *   not be used explicitly.
 *
 *   This implementation either returns or generate a trap, depending on the
 *   value of CONFIG_EFM_ASSERT_NOT_TRAPPED.
 *
 *   Please notice that this function is not used unless DEBUG_EFM is defined
 *   during preprocessing of EFM_ASSERT() usage.
 *
 * @param[in] file
 *   Name of source file where assertion failed.
 *
 * @param[in] line
 *   Line number in source file where assertion failed.
 ******************************************************************************/
void assertEFM(const char *file, int line)
{
	if (asserTrapCount < MAX_EFM_ASSERT_TRAP) {
		asserTrap[asserTrapCount++] = line;
	}
	if (CONFIG_EFM_ASSERT_NOT_TRAPPED) {
		return;
	}
	TD_Trap(TRAP_ASSERT_EFM, line);
}

/***************************************************************************//**
 * @brief
 *   return a copy of the pending assertion table.
 *
 * @param[in] buf
 *   Pointer to a buffer taht will receive the assertion table copy.
 *
 * @return
 *   Returns the number of assrtion table entries.
 ******************************************************************************/
uint8_t TD_TRAP_EFM_ASSERT_Get(uint16_t *buf)
{
	if (buf) {
		memcpy(buf, asserTrap, sizeof (uint16_t) * asserTrapCount);
	}
	return asserTrapCount;
}

/** @} */

/** @} (end addtogroup TRAP) */
