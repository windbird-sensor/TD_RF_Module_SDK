/***************************************************************************//**
 * @file
 * @brief Utility functions for the TDxxxx RF modules.
 * @author Telecom Design S.A.
 * @version 1.1.0
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2014 Telecom Design S.A., http://www.telecomdesign.fr</b>
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
#define TRACE_RAM_SIZE    64
#endif

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
	static char const *trap_list[] = {
		"Invalid TRAP0",
		"Trap at line",
		"SPI Lock",
		"SPI Invalid Unlock (not locked)",
		"SPI Unavailable",
		"SPI Invalid ID",
		"SPI Invalid BUS",
		"SPI Configuration Not Implemented",
		"Code Removed",
		"Frequency not allowed",
		"Scheduler Queue Overflow",
		"Flash Variables Full",
		"Flash Pointer Overflow",
		"Radio Chip",
		"RTC Delay",
		"Forbidden in IRQ context",
		"Custom 1",
		"Custom 2",
		"Custom 3",
		"Custom 4",
		"Custom 5",
		"Custom 6",
		"Custom 7",
		"Custom 8",
		"LAN Callback take too much time",
		"GPS hard error",
		"NMI",
		"Hard Fault",
		"Mem manage",
		"Bus Fault",
		"Usage fault",
		"Dump not available",
		"Bad SIGFOX ID",
		"Bad SIGFOX Key",
		"Invalid Timer ID",
		"AT Persist oversized",
		"User Flash Overwrite Code",
		"Pwr Coeff undefined",
		"Pwr ctx unknown"
	};
	uint64_t time;
	uint32_t msec, t;
	uint16_t hour, min, sec;
	char const *str = NULL;
	char *(*ptr)(uint32_t*, TD_TRAP_t *);

	time = TD_SCHEDULER_GetTime();

	// If trap have an embed trap function
	if (trap == TRAP_CUSTOM_FUNC) {
		ptr = (char*(*)(uint32_t *, TD_TRAP_t*))(param);
		str = ptr(&param, &trap);
	} else {

		// Standard trap
		if (trap < sizeof (trap_list) / sizeof (trap_list[0])) {
			str = trap_list[(uint8_t) trap];
		}
	}
	msec = time & 0x3FFF;
	time >>= 15;
	t = time;
	sec = t % 60;
	time = t / 60;
	min = t % 60;
	time = t / 60;
	hour = t % 24;
	time = t / 24;

	tfp_printf("[TRAP] Time:%d.%02d:%02d:%02d.%3d : ",
		(uint32_t) time, hour, min, sec, (msec * 1000) >> 15);
	if (str) {
		tfp_printf("%d:%s",(uint32_t) trap, (char *) str);
	} else {
		tfp_printf("Code:%d", (uint32_t) trap);
	}
	tfp_printf(" | Param:0x%08X(%d)\r\n", param, param);

	// Grab callstack
	SystemInit();

	// Display
	TD_TRAP_TraceDump(true);
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
*******************************************************************************/
void TD_TRAP_TraceStore(TD_TRAP_t trap, uint32_t param)
{

#if defined(__GNUC__)
	TD_TRAP_Frame_t *f = (TD_TRAP_Frame_t *) TRACE_RAM_START;

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

#if defined(__GNUC__)
	extern char __cs3_stack[];
	uint32_t magic;
	uint16_t i;
	uint8_t magic_flag;
	TD_TRAP_Frame_t *f;
#endif
	uint32_t fs;

	tfp_printf("-- TRAP DUMP --\r\n");
	tfp_printf("MSP:0x%08X\r\n", __get_MSP());
	tfp_printf("HardFaultSt:0x%08X\r\n", SCB->HFSR);
	if (SCB->HFSR & 0x40000000) {

		// A fault is not handled and lead to a hard fault
		tfp_printf("FaultEscalation\r\n");
	}
	fs = SCB->CFSR;
	tfp_printf("FaultStatus:0x%08X:", fs);
	if (fs & 0x1) {
		tfp_printf("ExecuteInvalidAdd,");
	}
	if (fs & 0x2) {
		tfp_printf("DataAccesError,");
	}
	if (fs & 0x8) {
		tfp_printf("UnstackEx,");
	}
	if (fs & 0x10) {
		tfp_printf("StackEx,");
	}
	if (fs & 0x100) {
		tfp_printf("ExecBusErr,");
	}
	if (fs & 0x200) {
		tfp_printf("DataBusErr,");
	}
	if (fs & 0x400) {
		tfp_printf("DataBusErr2,");
	}
	if (fs & 0x800) {
		tfp_printf("UnstackBEx,");
	}
	if (fs & 0x1000) {
		tfp_printf("StackBEx,");
	}
	if (fs & 0x10000) {
		tfp_printf("InvInst,");

		}
	if (fs & 0x20000) {
		tfp_printf("InvEPSR,");
	}
	if (fs & 0x40000) {
		tfp_printf("InvPC,");
	}
	if (fs& 0x80000) {
		tfp_printf("NoCopro,");
	}
	if (fs & 0x1000000) {
		tfp_printf("UnalignMemAcc,");
	}
	if (fs & 0x2000000) {
		tfp_printf("DivByZero,");
	}
	if (fs & 0x80){
		tfp_printf("MemFaultAdd:0x%08X\r\n", SCB->MMFAR);
	}
	if (fs & 0x8000){
		tfp_printf("BusFaultAdd:0x%08X\r\n", SCB->BFAR);
	}

#if defined(__GNUC__)
	f = (TD_TRAP_Frame_t *) TRACE_RAM_START;
	tfp_printf("Stack:0x%08X-0x%08X\r\n",
		__cs3_stack - CONFIG_STACK_SIZE, __cs3_stack);
	magic = (f->magic & 0xFFFFFF00);
	if (!force && (magic != TRAP_MAGIC_TRACE)) {
		return;
	}
	tfp_printf("-- TRAP Trace@0x%08X --\r\n", (int32_t) f);
	magic_flag = f->magic & 0xFF;
	if (magic != TRAP_MAGIC_TRACE) {
		tfp_printf("TRAP Trace invalid : 0x%08X<=>0x%08X\r\n",
			f->magic, TRAP_MAGIC_TRACE);
	}
	if (magic_flag & TRACE_MAGIC_OVERSIZE) {
		tfp_printf("Trace glob size overflow !!\r\n");
	}
	if (magic_flag & TRACE_MAGIC_OVERSTACK) {
		tfp_printf("Trace stack truncated\r\n");
	}
	for (i = 0; i < TRAP_MAX_USER; i++) {
		tfp_printf("Var %d:%d\r\n",i,f->user[i]);
	}
	if (magic_flag & TRACE_MAGIC_TRAP) {
		tfp_printf("TRP %d,%d\r\n", (uint32_t) f->trap, f->param);
	}
	if (magic_flag & TRACE_MAGIC_STACK) {
		tfp_printf("-Stack trace-\r\n");
		for (i = 0; i < f->trace_cnt; i++) {
			tfp_printf("%2d|0x%08X\r\n",
				i, (((uint32_t) f->trace[i]) << 1) + 1);
		}
	}
#endif

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

	tfp_printf("TraceDumpSigfox\r\n");
	magic = (f->magic & 0xFFFFFF00);
	sig_frame[0] = (magic == TRAP_MAGIC_TRACE) ? 0xC0 : 0xB0;
	magic_flag = f->magic & 0xFF;
	sig_frame[1] = magic_flag;
	sig_frame[2] = f->trace_cnt;
	memcpy(&sig_frame[3], &f->user[0], 9);
	if ((!(magic_flag & TRACE_MAGIC_STACK)) || (sig_frame[0] == 0xB0)) {
		f->trace_cnt = 0;
	}
	tfp_printf("Send HEADER:0x%02X %d frame will follow ...\r\n",
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
		tfp_printf("Send FRAME:0x%02X\r\n", sig_frame[0]);
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
	TD_TRAP_TraceStore(trap, param);

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

	if (!enable){
		MPU_Disable();
		return;
	}
	MPU_ConfigureRegion(&mpu);
	mpu.regionNo = 1;
	mpu.baseAddress=((uint32_t)((__cs3_stack - CONFIG_STACK_SIZE + 1))) &
		(~0x1F);
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
void SystemInit(void)
{
	extern const char __traceram_start;
	extern char __cs3_stack[];
	extern char __cs3_start_asm[];
	extern char __cs3_rodata[];
	uint32_t *s = (uint32_t *) (__cs3_stack - CONFIG_STACK_SIZE);
	uint32_t i;
	uint32_t *sp = (uint32_t *) __get_MSP();
	bool over = false;
	TD_TRAP_Frame_t *f = (TD_TRAP_Frame_t *) TRACE_RAM_START;

	f->trace_cnt = 0;
	if ((uint32_t)sp > (uint32_t) __cs3_stack - 32) {
		sp = 0;
	}
	if (sp < s){

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

/***************************************************************************//**
* @brief
*   Check stack corruption, use last stack byte for special value
* @return
*   true if sequence broken, false otherwise
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
	uint8_t *s = (uint8_t*)(CSTACK$$Base);
#endif
	static uint8_t stack_seq=0xCC;

	if (*s != stack_seq) {
		*s = stack_seq;
		return false;
	} else{
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
	char *limit=__cs3_stack;
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
	uint32_t *sp = (uint32_t *) __get_MSP();

	tfp_printf("---- STACK TRACE ----\r\n");
	tfp_printf("SP:0x%08X\r\n",sp);
	tfp_printf("Code:0x%08X-0x%08X\r\n", code_start, code_end);
	tfp_printf("Stack:0x%08X-0x%08X\r\n", s, limit);
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
			tfp_printf("0x%08X\r\n",((*s) >> 1));
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
	TD_Trap(TRAP_NMI_FAULT, 0);
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
	TD_Trap(TRAP_HARD_FAULT, 0);
}

/***************************************************************************//**
* @brief
*   Memory Management handler (MPU system), overloaded to call the trap system.
*******************************************************************************/
void MemManage_Handler(void)
{
	TD_Trap(TRAP_MEM_MANAGE_FAULT, 0);
}

/***************************************************************************//**
* @brief
*   Bus Fault handler, overloaded to call the trap system.
*******************************************************************************/
void BusFault_Handler(void)
{
	TD_Trap(TRAP_BUS_FAULT, 0);
}

/***************************************************************************//**
* @brief
*   Usage Fault handler, overloaded to call the trap system.
*******************************************************************************/
void UsageFault_Handler(void)
{
	TD_Trap(TRAP_USAGE_FAULT, 0);
}

/** @} */

/** @} (end addtogroup TRAP) */
