/***************************************************************************//**
 * @file
 * @brief Un Asynchronous Receiver/Transmitter (UART) peripheral API for the
 * TDxxxx RF modules.
 * @author Telecom Design S.A.
 * @version 3.2.0
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

#include <stdint.h>
#include <stdbool.h>

#include <em_cmu.h>
#include <em_gpio.h>
#include <em_leuart.h>
#include <em_usart.h>


#include "td_core.h"
#include "td_uart.h"
#include "td_rtc.h"
#include "td_printf.h"
#include "td_utils.h"
#include "td_stream.h"

/***************************************************************************//**
 * @addtogroup UART
 * @brief Asynchronous Receiver/Transmitter (UART) peripheral API for the TDxxxx
 * RF modules
 * @{
 ******************************************************************************/

/*******************************************************************************
 *************************   DEFINES   *****************************************
 ******************************************************************************/

/** @addtogroup UART_DEFINES Defines
 * @{ */

/** printf macro for UART debug */
//#define DEBUG_UART_ALLOC
//#define DEBUG_UART
//#define DEBUG_IRQ
#ifdef DEBUG_UART
#define DEBUG_PRINTF(...) tfp_printf(__VA_ARGS__)
#else
#define DEBUG_PRINTF(...)
#endif
#ifdef DEBUG_IRQ

/** Macro to debug UART IRQs */
#define DEBUG_IRQ_PRINTF(...) tfp_printf(__VA_ARGS__)
#else
/** Macro to debug UART IRQs */
#define DEBUG_IRQ_PRINTF(...)
#endif

/** Default UART index [aka system uart = CONFIG_LEUART]
    First opened uart in system see more in depth explanation later */
#define DEFAULT_UART			0

/** Non-routed UART IRQ */
#define NOT_ROUTED_IRQ			0

#ifdef DEBUG_UART_ALLOC

/** Maximum number of allocatable UARTs */
#define DEBUG_UART_ALLOC_MAX	10
#endif

/** Macro to define an USART */
#define USART_DEF(x)\
		if (interface == (void *) USART##x) {\
			TD_UART[index].clk = cmuClock_USART##x;\
			TD_UART[index].irq = USART##x##_RX_IRQn;\
			TD_UART[index].rx_flag = USART_STATUS_RXDATAV;\
			TD_UART[index].tx_flag = USART_STATUS_TXC;\
			usart = true;\
		}

/** Macro to define an UART */
#define UART_DEF(x)\
		if (interface == (void *) UART##x) {\
			TD_UART[index].clk = cmuClock_UART##x;\
			TD_UART[index].irq = UART##x##_RX_IRQn;\
			TD_UART[index].rx_flag = USART_STATUS_RXDATAV;\
			TD_UART[index].tx_flag = USART_STATUS_TXC;\
		}

/** Macro to define an LEUART */
#define LEUART_DEF(x)\
		if (interface == (void *) LEUART##x) {\
			TD_UART[index].clk = cmuClock_LEUART##x;\
			TD_UART[index].irq = LEUART##x##_IRQn;\
			TD_UART[index].rx_flag = LEUART_STATUS_RXDATAV;\
			TD_UART[index].tx_flag = LEUART_STATUS_TXC;\
		}

/** @} */

/******************************************************************************
 *************************   TYPEDEFS   ***************************************
 ******************************************************************************/

/** @addtogroup UART_TYPEDEFS Typedefs
 * @{ */

/** UART clock usage structure */
typedef struct {
	uint8_t leuart_corelediv2;
	uint8_t leuart_lfxo;
} TD_UART_ClockUsage;

/** @} */

/*******************************************************************************
 **************************   PUBLIC VARIABLES   *******************************
 ******************************************************************************/

/** @addtogroup UART_GLOBAL_VARIABLES Global Variables
 * @{ */

/** Array of available UARTs */
extern TD_UART_port_t TD_UART[];

/** Table of UART IRQs */
uint8_t uart_irq_table[MAX_IRQ_INDEX]={0};

/** @} */

/*******************************************************************************
 *************************   PRIVATE VARIABLES   *******************************
 ******************************************************************************/

/** @addtogroup UART_LOCAL_VARIABLES Local Variables
 * @{ */

#ifdef DEBUG_UART_ALLOC

/** List of allocatable UARTs */
static uint32_t alloc_list[DEBUG_UART_ALLOC_MAX] = {0};

/** Number of allocated UARTs */
static uint8_t alloc_list_cnt = 0;
#endif

/** RX IRQ received flag*/
static bool RxFinised = false;

/** @} */

/*******************************************************************************
 **************************  PRIVATE FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup UART_LOCAL_FUNCTIONS Local Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Set the UART port.
 *
 * @param[in] p
 *   Pointer to the TD_UART_port_t structure.
 *
 * @param[in] port
 *   The port to set, must be of type GPIO_Port_TypeDef.
 *
 * @param[in] bit
 *   The port bit to set.
 ******************************************************************************/
static void TD_UART_SetPort(TD_UART_port_t *p, GPIO_Port_TypeDef port,
	uint8_t tx_bit)
{
	p->tx_port = port;
	p->tx_bit = tx_bit;
	p->rx_port = port;
	p->rx_bit = tx_bit+1;
}

/***************************************************************************//**
 * @brief
 *   Initialize an LEUART location.
 *
 * @param[in] port
 *   The port to set, must be of type GPIO_Port_TypeDef.
 *
 * @param[in] interface
 *   The interface to use, must be either LEUART0 or LEURART1.
 *
 * @param[in] location
 *   The location to use for the LEUART.
 ******************************************************************************/
static bool TD_UART_InitLocationLeuart(TD_UART_port_t *port, void *interface,
	uint8_t location)
{
bool found = false;

	switch (location) {
	case 0:
		port->location = LEUART_ROUTE_LOCATION_LOC0;
#ifdef LEUART0
		if (interface == (void *) LEUART0) {
			TD_UART_SetPort(port,gpioPortD,4);
			port->irq_index = LEUART0_IRQ_INDEX;
			port->rx_data = &LEUART0->RXDATA;
			port->tx_data = &LEUART0->TXDATA;
			port->status = &LEUART0->STATUS;
			found = true;
		}
#endif
#ifdef LEUART1
		if (interface == (void *) LEUART1) {
			TD_UART_SetPort(port,gpioPortC,6);
			port->irq_index = LEUART1_IRQ_INDEX;
			port->rx_data = &LEUART1->RXDATA;
			port->tx_data = &LEUART1->TXDATA;
			port->status = &LEUART1->STATUS;
			found = true;
		}
#endif
		break;

	case 1:
		port->location = LEUART_ROUTE_LOCATION_LOC1;
#ifdef LEUART0
		if (interface == (void *) LEUART0) {
			TD_UART_SetPort(port,gpioPortB,13);
			port->irq_index = LEUART0_IRQ_INDEX;
			port->rx_data = &LEUART0->RXDATA;
			port->tx_data = &LEUART0->TXDATA;
			port->status = &LEUART0->STATUS;
			found = true;
		}
#endif
		break;
	}
	return found;
}

/***************************************************************************//**
 * @brief
 *   Initialize an USART location.
 *
 * @param[in] port
 *   The port to set, must be of type GPIO_Port_TypeDef.
 *
 * @param[in] interface
 *   The interface to use, must be either USART0 or USART2.
 *
 * @param[in] location
 *   The location to use for the USART.
 ******************************************************************************/
static bool TD_UART_InitLocationUsart(TD_UART_port_t *port, void *interface,
	uint8_t location)
{
bool found = false;

	switch (location) {
	case 0:
		port->location = USART_ROUTE_LOCATION_LOC0;
#ifdef USART0
		if (interface == USART0) {
			TD_UART_SetPort(port,gpioPortC,0);
			port->irq_index = USART0_IRQ_INDEX;
			port->rx_data = &USART0->RXDATA;
			port->tx_data = &USART0->TXDATA;
			port->status = &USART0->STATUS;
			found = true;
		}
#endif
		break;

	case 1:
		port->location = USART_ROUTE_LOCATION_LOC1;
#ifdef USART2
		if (interface == USART2) {
			TD_UART_SetPort(port,gpioPortB,3);
			port->irq_index = USART2_IRQ_INDEX;
			port->rx_data = &USART2->RXDATA;
			port->tx_data = &USART2->TXDATA;
			port->status = &USART2->STATUS;
			found = true;
		}
#endif
		break;
	}
	return found;
}

/***************************************************************************//**
 * @brief
 *   Initialize an UART location.
 *
 * @param[in] port
 *   The port to set, must be of type GPIO_Port_TypeDef.
 *
 * @param[in] interface
 *   The interface to use, must be UART1.
 *
 * @param[in] location
 *   The location to use for the UART.
 ******************************************************************************/
static bool TD_UART_InitLocationUart(TD_UART_port_t *port, void *interface,
	uint8_t location)
{
bool found = false;

	switch (location) {
	case 1:
#ifdef UART_ROUTE_LOCATION_LOC1
		port->location = UART_ROUTE_LOCATION_LOC1;
#endif
	#ifdef UART1
		if (interface == UART1) {
			TD_UART_SetPort(port,gpioPortE,0);
			port->irq_index = UART1_IRQ_INDEX;
			port->rx_data = &UART1->RXDATA;
			port->tx_data = &UART1->TXDATA;
			port->status = &UART1->STATUS;
			found = true;
		}
	#endif
		break;
	}
	return found;
}

/***************************************************************************//**
 * @brief
 *   Get an UART clock usage.
 *
 * @param[in] cu
 *   Pointer to a buffer that will receive the clock usage counts.
 *
 * @param[in] ignore
 *   Pointer on a TD_UART_prot_t to ignore.
 ******************************************************************************/
static void TD_UART_GetClockUsage(TD_UART_ClockUsage *cu, TD_UART_port_t *ignore)
{
uint8_t i;

	memset(cu,0,sizeof(TD_UART_ClockUsage));
	for (i=0;i<CONFIG_TD_UART_COUNT;i++){
		if (!UART_IS_OPENED(&TD_UART[i])){
			continue;
		}
		if (&TD_UART[i]==ignore){
			continue;
		}
		if (TD_UART[i].leuart) {
			if (TD_UART[i].use_lfxo) {
				cu->leuart_lfxo++;
			} else {
				cu->leuart_corelediv2++;
			}
		}
	}
}

/***************************************************************************//**
 * @brief
 *   Callback function called when an RX IRQ is received.
 *
 * @param[in] mask
 *   The interrupt mask that triggered the call.
 ******************************************************************************/
static void RXIRQReceived(uint32_t mask)
{
	RxFinised = true;
}

/** @} */

/*******************************************************************************
 **************************   PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup UART_USER_FUNCTIONS User Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Open an UART stream.
 *
 * @param[in] options
 *   Pointer to the TD_UART_Options_t structure.
 *
 * @param[in] mode
 *   Stream mode flag bit field.
 *
 * @return
 *   Returns a stream descriptor if OK, 0 otherwise.
 ******************************************************************************/
TD_STREAM_t *TD_UART_Open(TD_UART_Options_t *options, uint32_t mode)
{
	TD_STREAM_t *stream;
	uint32_t databits, parity, stopbits;
	bool rx_enabled;
	static const uint16_t databit_cfg[] = {
		usartDatabits4,
		usartDatabits5,
		usartDatabits6,
		usartDatabits7,
		usartDatabits8,
		usartDatabits9,
		usartDatabits10,
		usartDatabits11,
		usartDatabits12,
		usartDatabits13,
		usartDatabits14,
		usartDatabits15,
		usartDatabits16
	};

	if (mode & TD_STREAM_RDONLY) {

		// Only reading from UART is not supported yet!
		return 0;
	} else if (mode & TD_STREAM_WRONLY) {
		rx_enabled = false;
	} else {
		rx_enabled = true;
	}
	if (options->interface == (void *) LEUART0
#ifdef LEUART1
		|| options->interface == (void *) LEUART1
#endif
	) {
		switch (options->data_bits) {
		case 8:
			databits = leuartDatabits8;
			break;

		case 9:
			databits = leuartDatabits9;
			break;

		default:
			return 0;
		}
		switch (options->parity | 0x20) {
		case 'n':
			parity = leuartNoParity;
			break;

		case 'e':
			parity = leuartEvenParity;
			break;

		case 'o':
			parity = leuartOddParity;
			break;

		default:
			return 0;
		}
		switch (options->stop_bits) {
		case 1:
			stopbits = leuartStopbits1;
			break;

		case 2:
			stopbits = leuartStopbits2;
			break;

		default:
			return 0;
		}
	} else {
		databits = 0;
		if (options->data_bits >= 4 && options->data_bits <= 16) {
			databits = databit_cfg[options->data_bits - 4];
		}
		switch (options->parity | 0x20) {
		case 'n':
			parity = usartNoParity;
			break;

		case 'e':
			parity = usartEvenParity;
			break;

		case 'o':
			parity = usartOddParity;
			break;

		default:
			return 0;
		}
		switch (options->stop_bits) {
		case 1:
			stopbits = usartStopbits1;
			break;

		case 2:
			stopbits = usartStopbits2;
			break;

		default:

			// We don't handle half bit parities
			return 0;
		}
	}
	stream = TD_UART_InitGlobal(options->interface, options->location,
		options->baudrate, rx_enabled, options->shared_pins, databits, parity,
		stopbits, options->modecomm, options->rs485_port, options->rs485_bit);
	if (stream == 0) {
		return 0;
	}
	init_printf(stream, TD_UART_Putc, TD_UART_Start, TD_UART_Stop);
	return stream;
}

/***************************************************************************//**
 * @brief
 *   Wait synchronously for all pending uart transfer to complete in order to
 *   be able to change baudrate without breaking current byte
 *
 ******************************************************************************/
void TD_UART_WaitForPendingTx(void)
{
	int i;

	for (i = 0; i < CONFIG_TD_UART_COUNT; i++) {
		if (!UART_IS_OPENED(&TD_UART[i])){
			continue;
		}
		if(!TD_UART[i].tx) {
			continue;
		}

		// Wait for transmission complete on uart or leuart
		if (TD_UART[i].leuart) {
			if( (((LEUART_TypeDef *) TD_UART[i].Raw)->STATUS) &
				LEUART_STATUS_TXENS) {
				while (!(((LEUART_TypeDef *) TD_UART[i].Raw)->STATUS &
					LEUART_STATUS_TXC)) {
					;
				}
			}
		} else {
			while (!(((USART_TypeDef *) TD_UART[i].Raw)->STATUS & USART_STATUS_TXC)) {
				;
			}
		}
}
}

/***************************************************************************//**
 * @brief
 *   Close an UART stream. (Disable and close stream)
 *
 * @param[in] stream
 *   Pointer to the stream structure to close.
 ******************************************************************************/
void TD_UART_Close(TD_STREAM_t *stream)
{
	TD_UART_port_t *uart;

	if (!STREAM_UART_IS_OPENED(stream)) {
		return;
	}
	uart = ((TD_STREAM_cast_t *) stream)->io_handle;
	TD_UART_DisableExtended(stream);
	memset(uart, 0, sizeof (TD_UART_port_t));
	TD_STREAM_Free(stream);
}

/***************************************************************************//**
 * @brief
 *   Set an UART baudrate.
 *
 * @param[in] stream
 *   Pointer to the stream structure to close.
 *
 * @param[in] baudrate
 *   The baudrate to set.
 ******************************************************************************/
void TD_UART_SetBaudrate(TD_STREAM_t *stream,uint32_t baudrate)
{
	TD_UART_ClockUsage cu;

	// If we want LEUART at 9600 bps or less, use LFXO
	// This is the ONLY source that will be accurate enough
	bool use_lfxo = (baudrate <= 9600);
	bool new_power_mode_em1=false;
	TD_UART_port_t *p = ((TD_STREAM_cast_t *) stream)->io_handle;

	if (!p->clk){
		DEBUG_PRINTF("UART Invalid Type !\r\n");
		TD_Trap(TRAP_UART_CONFIG, 0);
		return;
	}
	TD_UART_WaitForPendingTx();
	if (p->leuart) {
		TD_UART_GetClockUsage(&cu, p);

		// We want CORELEDIV2 clock and someone use lfxo
		if (!use_lfxo && cu.leuart_lfxo){

			// Here if trap ignored, we will break previous opened serial port
			// that use LFXO
		//	TD_Trap(TRAP_UART_CONFIG, 4);
		}

		// We want LFXO clock and someone use CORELEDIV2
		if (use_lfxo && cu.leuart_corelediv2) {
			if (!CONFIG_ALLOW_LEUART_LOW_BAUDRATE_ON_HFRCO){
			//	TD_Trap(TRAP_UART_CONFIG, 5);
			}
			// If trap ignored we will not use LFXO. It will not break anything
			// but source will not be ACCURATE enough
			use_lfxo = false;
		}
		if (!use_lfxo) {
			CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_CORELEDIV2);
			CMU_ClockDivSet(p->clk, cmuClkDiv_4);
			p->use_lfxo = false;
			// init.refreq will be automatically calculated by emlib
		} else {
			CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFXO);
			CMU_ClockDivSet(p->clk, cmuClkDiv_1);
			p->use_lfxo = true;
		}
	} else {

		// Enable the UxARTx clock
		CMU_ClockEnable(cmuClock_HFPER, true);
		// Do nothing ! just throws an EFM assert as it's HFCLK that must be
		// routed to LFXCLK and will result with a 32kHz processor
		//CMU_ClockSelectSet(cmuClock_HFPER, cmuSelect_LFXO);
	}

	// Enable UART clock
	CMU_ClockEnable(p->clk, true);
	CMU_ClockEnable(cmuClock_GPIO, true);
	if (p->leuart){
		LEUART_BaudrateSet(p->Raw,0,baudrate);
		((LEUART_TypeDef *)p->Raw)->CMD=LEUART_CMD_CLEARRX;
	} else {
		USART_BaudrateSyncSet(p->Raw,0,baudrate);
		((USART_TypeDef *)p->Raw)->CMD=USART_CMD_CLEARRX;
	}

	// If !use_lfxo (speed > 9600 or bypass) (we use HF CLK en LEUART) or using
	// USART or UART must put additionnal wakeup on RX pin
	if ( ((!use_lfxo) || (!p->leuart)) && p->rxEnable) {
		new_power_mode_em1 = true;
	}
	if (p->em1 != new_power_mode_em1){
		p->em1 = new_power_mode_em1;
		TD_RTC_SetPowerMode(new_power_mode_em1 ? TD_RTC_EM1:TD_RTC_EM2);
	}
	p->baudrate=baudrate;
}

/***************************************************************************//**
 * @brief
 *   Dump the UART allocation list.
 *
 * @param[in] list
 *   Pointer to a uint32_t buffer that will receive the UART list.
 *
 * @return
 *   Returns the number of elements in the list.
 ******************************************************************************/
uint8_t TD_UART_AllocListDump(uint32_t *list)
{
#ifdef DEBUG_UART_ALLOC
	if (list){
		memcpy(list,alloc_list,sizeof(uint32_t)*alloc_list_cnt);
	}
	return alloc_list_cnt;
#else
	return 0;
#endif
}

/***************************************************************************//**
 * @brief
 *   Initialize the UART peripheral.
 *
 * @param[in] interface
 *   Interface pointer (LEUART0, USART1, ...)
 *
 * @param[in] location
 *   UART location (as stated in EFM32 datasheet)
 *
 * @param[in] speed
 *   The baudrate in bps.
 *
 * @param[in] rxEnable
 *   Enable receive flag.
 *
 * @param[in] shared
 *   Flag for sharing the UART port with alternative GPIO functions.
 *
 * @param[in] databits
 *    Number of databits in frame. Should be leuartxxx or usartxxx function of
 *    targeted UART
 *
 * @param[in] parity
 *    Parity mode to use. Should de leuartxxx or usartxxx function of targeted
 *    UART
 *
 * @param[in] stopbits
 *    Number of stopbits to use. Should de leuartxxx or usartxxx function of
 *    targeted UART
 *
 * @param[in] mode
 *    The communication mode to use, standard, hal or full dplex.
 *
 * @param[in] rs485_port
 *    The GPIO port to use for RS485 half-duplex control.
 *
 * @param[in] rs485_bit
 *    The GPIO bit to use for RS485 half-duplex control.
 *
 * @return
 *   Returns an opaque stream descriptor.
 ******************************************************************************/
TD_STREAM_t *TD_UART_InitGlobal(TD_STREAM_t *interface, uint8_t location,
	uint32_t speed,	bool rxEnable, bool shared, uint32_t databits,
	uint32_t parity, uint32_t stopbits, Comm_Mode_t mode,
	GPIO_Port_TypeDef rs485_port, uint32_t rs485_bit)
{
#ifdef DEBUG_UART_ALLOC
	if (alloc_list_cnt<DEBUG_UART_ALLOC_MAX){
		alloc_list[alloc_list_cnt++]=interface;
	}
#endif
	TD_STREAM_cast_t *stream;
	uint8_t index;
	bool usart = false;
	bool loc_ok = false;

	// Search free TD_UART index
	for (index = 0; index < CONFIG_TD_UART_COUNT && TD_UART[index].Raw; index++) {
		;
	}
	if (index >= CONFIG_TD_UART_COUNT){
		tfp_printf("%d > %d, %08X\r\n", index, CONFIG_TD_UART_COUNT, interface);
		TD_Trap(TRAP_UART_CONFIG, 1);
		return NULL;
	}

	// Reset descriptor
	memset(&TD_UART[index], 0, sizeof (TD_UART_port_t));

	// Set hardware (emlib) descriptor
	TD_UART[index].Raw = interface;

	// Allocate and fill in a stream
	stream = TD_STREAM_Alloc(&TD_UART[index]);
	if (stream == 0) {
		DEBUG_PRINTF("No available stream!\r\n");
		TD_Trap(TRAP_UART_CONFIG, 3);
		return NULL;
	}
	stream->start = (bool (*) (TD_STREAM_cast_t *)) TD_UART_Start;
	stream->stop = (void (*) (TD_STREAM_cast_t *)) TD_UART_Stop;
	stream->read = (int (*) (TD_STREAM_cast_t *)) TD_UART_GetCharExtended;
	stream->write = (bool (*) (TD_STREAM_cast_t *, char)) TD_UART_Putc;
	stream->available_chars = (int (*) (TD_STREAM_cast_t *)) TD_UART_AvailableCharsExt;
	stream->flush = (void (*) (TD_STREAM_cast_t *)) TD_UART_FlushExt;
	stream->mode = rxEnable ? TD_STREAM_RDWR : TD_STREAM_RDONLY;
	TD_UART[index].stream = stream;
	TD_UART[index].rxEnable = rxEnable;
	if (interface == (void *) LEUART0
#ifdef LEUART1
			|| interface == (void *) LEUART1
#endif
			) {
		// LEUART
		LEUART_Init_TypeDef init = LEUART_INIT_DEFAULT;
#ifdef LEUART0
		LEUART_DEF(0);
#endif
#ifdef LEUART1
		LEUART_DEF(1);
#endif
		TD_UART[index].leuart = true;
		init.enable = leuartDisable;
		init.baudrate = speed;
		if (databits == 0xFF){
			databits = leuartDatabits8;
			parity = leuartNoParity;
			stopbits = leuartStopbits1;
		}
		init.databits = (LEUART_Databits_TypeDef) databits;
		init.parity = (LEUART_Parity_TypeDef) parity;
		init.stopbits = (LEUART_Stopbits_TypeDef) stopbits;

		// Enable clock
		TD_UART_SetBaudrate(stream,speed);

		// Reset and initialize interface
		LEUART_Reset(interface);
		LEUART_Init(interface, &init);
		LEUART_IntClear(interface, LEUART_IF_RXDATAV);
		LEUART_IntEnable(interface, LEUART_IF_RXDATAV);
		loc_ok = TD_UART_InitLocationLeuart(&TD_UART[index], interface, location);
	} else {
#ifdef USART0
		USART_DEF(0)
#endif
#ifdef USART1
		USART_DEF(1)
#endif
#ifdef USART2
		USART_DEF(2)
#endif
#ifdef UART0
		UART_DEF(0)
#endif
#ifdef UART1
		UART_DEF(1)
#endif
		USART_InitAsync_TypeDef init = USART_INITASYNC_DEFAULT;
		init.baudrate = speed;
		init.enable = usartDisable;
		if (databits == 0xFF){
			databits = usartDatabits8;
			parity = usartNoParity;
			stopbits = usartStopbits1;
		}
		init.databits = (USART_Databits_TypeDef) databits;
		init.parity = (USART_Parity_TypeDef) parity;
		init.stopbits = (USART_Stopbits_TypeDef) stopbits;

		//Enable needed clock
		TD_UART_SetBaudrate(stream,speed);

		//Setup Usart
		USART_InitAsync(interface, &init);
		USART_IntClear(interface, USART_IF_RXDATAV);
		USART_IntEnable(interface, USART_IF_RXDATAV);

		//Setup Pins
		if (usart){
			loc_ok = TD_UART_InitLocationUsart(&TD_UART[index], interface,
				location);
		} else {
			loc_ok = TD_UART_InitLocationUart(&TD_UART[index], interface,
				location);
		}
	}
	if (!loc_ok){
		DEBUG_PRINTF("UART Invalid Location !\r\n");
		TD_Trap(TRAP_UART_CONFIG, 2);
		return NULL;
	}

	// TX PORT
	GPIO_PinModeSet(TD_UART[index].tx_port, TD_UART[index].tx_bit,
		gpioModePushPull, 1);

	// Always enable TX
	TD_UART[index].PortEnable = TD_UART[index].leuart ? leuartEnableTx :

		// Note : leuartEnablexxx and usartEnablexxx are the same
		usartEnableTx;
	if (rxEnable) {

		// RX PORT
	    GPIO_PinModeSet(TD_UART[index].rx_port, TD_UART[index].rx_bit,
	    	gpioModeInputPull, 1);

		// Route IRQ
		uart_irq_table[TD_UART[index].irq_index] = index+1;

		// Clear previous RX interrupts
		NVIC_ClearPendingIRQ(TD_UART[index].irq);

		// Enable RX interrupts
		NVIC_EnableIRQ(TD_UART[index].irq);

		// Enable RX
		TD_UART[index].PortEnable |= (TD_UART[index].leuart ?
			leuartEnableRx : usartEnableRx);
		DEBUG_PRINTF("Enable RX : %d 0x%08X\r\n", index,
			TD_UART[index].PortEnable);
	}
	if (!shared) {

		// Port will not be shared
		// Temporarily set this flag to set initial route
		TD_UART[index].PortShared = true;
		TD_UART_Start(TD_UART[index].stream);
	}

	// Save port shared flag
	TD_UART[index].PortShared = shared;
	TD_UART[index].modecomm = mode;
	TD_UART[index].rs485_bit = rs485_bit;
	TD_UART[index].rs485_port = rs485_port;

	// Eventually enable UART
	if (TD_UART[index].leuart) {
		LEUART_Enable(TD_UART[index].Raw,
			(LEUART_Enable_TypeDef) TD_UART[index].PortEnable);
	}else{
		USART_Enable(TD_UART[index].Raw,
			(USART_Enable_TypeDef) TD_UART[index].PortEnable);
	}
	return stream;
}

/***************************************************************************//**
 * @brief
 *   Initialize the UART peripheral.
 *
 * @param[in] speed
 *   The baudrate in bps.
 *
 * @param[in] rxEnable
 *   Enable receive flag.
 *
 * @param[in] shared
 *   Flag for sharing the UART port with alternative GPIO functions.
 *
 * @param[in] databits
 *    Number of databits in frame.
 *
 * @param[in] parity
 *    Parity mode to use.
 *
 * @param[in] stopbits
 *    Number of stopbits to use.
 *
 * @return
 *   Returns an opaque stream descriptor (fixed to LEUART0).
 ******************************************************************************/
TD_STREAM_t *TD_UART_InitExtended(uint32_t speed, bool rxEnable, bool shared,
	LEUART_Databits_TypeDef databits, LEUART_Parity_TypeDef parity,
	LEUART_Stopbits_TypeDef stopbits)
{
	return TD_UART_InitGlobal(CONFIG_LEUART_DEVICE,
		CONFIG_LEUART_LOCATION, speed, rxEnable, shared,
		databits, parity, stopbits, COM_STD, (GPIO_Port_TypeDef)0, 0);
}

/***************************************************************************//**
 * @brief
 *   Initialize the UART peripheral.
 *
 * @param[in] speed
 *   The baudrate in bps.
 *
 * @param[in] rxEnable
 *   Enable receive flag.
 *
 * @param[in] shared
 *   Flag for sharing the UART port with alternative GPIO functions.
 *
 * @return
 *   Returns an  stream descriptor (fixed to LEUART0).
 ******************************************************************************/
TD_STREAM_t *TD_UART_Init(uint32_t speed, bool rxEnable, bool shared)
{
	return TD_UART_InitExtended(
		speed,
		rxEnable,
		shared,
		(LEUART_Databits_TypeDef) 0xFF,
		(LEUART_Parity_TypeDef) 0xFF,
		(LEUART_Stopbits_TypeDef) 0xFF);
}

/***************************************************************************//**
 * @brief
 *   Start using the UART peripheral.
 *
 * @param[in] stream
 *   Pointer to the stream opaque descriptor.
 ******************************************************************************/
void TD_UART_Start(TD_STREAM_t *stream)
{
	STREAM_GET_UART(stream,uart,);
	LEUART_TypeDef *leuart = (LEUART_TypeDef *)uart->Raw;
	USART_TypeDef *usart = (USART_TypeDef *)uart->Raw;

	if (uart->PortShared) {
		DEBUG_PRINTF("UART_Start shared leuart:%d shared:%d\r\n", PTR->leuart,
			PTR->PortShared);

		// Enable LEUART0 TX and RX route
		if (uart->leuart) {
			if (uart->PortEnable & leuartEnableRx) {
				leuart->ROUTE = LEUART_ROUTE_TXPEN | LEUART_ROUTE_RXPEN |
					uart->location;
				leuart->CMD = LEUART_CMD_TXDIS | LEUART_CMD_RXDIS |
					LEUART_CMD_CLEARTX | LEUART_CMD_CLEARRX;
				leuart->CMD = LEUART_CMD_TXEN | LEUART_CMD_RXEN;
			} else {
				leuart->ROUTE = LEUART_ROUTE_TXPEN | uart->location;
				leuart->CMD = LEUART_CMD_TXDIS | LEUART_CMD_CLEARTX;
				leuart->CMD = LEUART_CMD_TXEN;
			}
		} else {
			DEBUG_PRINTF("UART_Start not leuart 0x%08X 0x%08X\r\n",
				uart->PortEnable, usartEnableRx);
			if (uart->PortEnable & usartEnableRx){
				DEBUG_PRINTF("UxARTx->ROUTE RX\r\n");
		      	usart->ROUTE = USART_ROUTE_RXPEN | USART_ROUTE_TXPEN |
		      			uart->location;
				usart->ROUTE = USART_ROUTE_RXPEN | USART_ROUTE_TXPEN |
					uart->location;
			} else {
				DEBUG_PRINTF("USART1->ROUTE TX only\r\n");
		      	usart->ROUTE = USART_ROUTE_TXPEN | uart->location;
			}
		}
	}
	if(uart->modecomm == COM_RS485_FULL){
		if (uart->leuart) {
				LEUART_Enable(uart->Raw, leuartEnableTx);
		} else {
				USART_Enable(uart->Raw, usartEnableTx);
		}
		GPIO_PinOutSet(uart->rs485_port, uart->rs485_bit);
	}
}

/***************************************************************************//**
 * @brief
 *   Get the Communication Baudrate
 *
 * @param[in] stream
 *   Pointer to the stream opaque descriptor.
 ******************************************************************************/
uint32_t TD_UART_GetBaudrateExtended(TD_STREAM_t * stream)
{
	STREAM_GET_UART(stream,uart,0);
	return uart->baudrate;
}

/***************************************************************************//**
 * @brief
 *   Get the Communication Mode (if RS485, give half or full duplex)
 *
 * @param[in] stream
  *   Pointer to the stream opaque descriptor.
 ******************************************************************************/
uint32_t TD_UART_GetModeCommExtended(TD_STREAM_t * stream)
{
	STREAM_GET_UART(stream,uart,0);
	return uart->modecomm;
}

/***************************************************************************//**
 * @brief
 *   Get the RS485 Port
 *
 * @param[in] stream
  *   Pointer to the stream opaque descriptor.
 ******************************************************************************/
GPIO_Port_TypeDef TD_UART_GetPortRS485(TD_STREAM_t * stream)
{
	STREAM_GET_UART(stream,uart,(GPIO_Port_TypeDef)0);
	return uart->rs485_port;
}

/***************************************************************************//**
 * @brief
 *   Get the RS485 Bit
 *
 * @param[in] stream
 *   Pointer to the stream opaque descriptor.
 ******************************************************************************/

uint32_t TD_UART_GetBitRS485(TD_STREAM_t *stream)
{
	STREAM_GET_UART(stream,uart,0);
	return uart->rs485_bit;
}

/***************************************************************************//**
 * @brief
 *   Stop using the UART peripheral.
 *
 * @param[in] stream
 *   Pointer to the stream opaque descriptor.
 ******************************************************************************/
void TD_UART_Stop(TD_STREAM_t *stream)
{
	STREAM_GET_UART(stream,uart,);
	LEUART_TypeDef *leuart = (LEUART_TypeDef *)uart->Raw;
	USART_TypeDef *usart = (USART_TypeDef *)uart->Raw;

	if (uart->leuart) {

		// Wait end of transmission if there was a previous one
		while (!(leuart->STATUS & LEUART_STATUS_TXC) && uart->tx) {
			;
		}
		if (uart->PortShared) {

			// Disable LEUART0 TX and RX route
			leuart->CMD = LEUART_CMD_TXDIS | LEUART_CMD_RXDIS |
				LEUART_CMD_CLEARTX | LEUART_CMD_CLEARRX;
			leuart->ROUTE = 0;
		}
	} else {
		// Wait end of transmission
		while (!(usart->STATUS & USART_STATUS_TXC) && uart->tx) {
			;
		}
		if (uart->PortShared) {
			usart->CMD = USART_CMD_TXDIS | USART_CMD_RXDIS |
				USART_CMD_CLEARTX | USART_CMD_CLEARRX;
			usart->ROUTE = 0;
		}
	}

	// Unroute IRQ
	if(uart->modecomm == COM_RS485_FULL){
		GPIO_PinOutClear(uart->rs485_port, uart->rs485_bit);
		TD_RTC_Delay(T500MICROS);
		if (uart->leuart) {
			LEUART_Enable(uart->Raw, leuartEnable);
		} else {
			USART_Enable(uart->Raw, usartEnable);
		}
	}
}

/***************************************************************************//**
 * @brief
 *   Control the UART hardware pin modes.
 *
 * @param[in] stream
 *   Pointer to the stream opaque descriptor.
 *
 * @param[in] mode_tx
 *   The pin mode to use for TX.
 *
 * @param[in] out_tx
 *   The pin value to set for TX when set in output mode.
 *
 * @param[in] mode_rx
 *   The pin mode to use for RX.
 *
 * @param[in] out_rx
 *   The pin value to set for RX when set in output mode.
 ******************************************************************************/
void TD_UART_PinMode(TD_STREAM_t *stream, GPIO_Mode_TypeDef mode_tx,
	unsigned int out_tx, GPIO_Mode_TypeDef mode_rx, unsigned int out_rx)
{
	STREAM_GET_UART(stream,uart,);
	GPIO_PinModeSet(uart->tx_port, uart->tx_bit, mode_tx, out_tx);
	GPIO_PinModeSet(uart->rx_port, uart->rx_bit, mode_rx, out_rx);
}

/***************************************************************************//**
 * @brief
 *   Wait raw state on RX pin
 *   TODO : should be done in low power mode ...
 *
 * @param[in] p
 *   Pointer to the stream opaque descriptor.
 *
 * @param[in] state
 *   The state to wait for.
 ******************************************************************************/
void TD_UART_RxRawWait(TD_STREAM_t *p, bool state)
{
	TD_UART_RxRawWaitExtended(p,state,0);
}

/***************************************************************************//**
 * @brief
 *   Wait raw state on RX pin
 *   TODO : should be done in low power mode ... (be careful to not break app
 *   with quick response time needed).
 *
 * @param[in] stream
 *   Pointer to the stream opaque descriptor.
 *
 * @param[in] state
 *   The state to wait for.
 *
 * @param[in] timeout
 *   The timeout to wait in seconds.
 *
 * @return
 * Returns false if the timeout has expired, false otherwise.
 ******************************************************************************/
bool TD_UART_RxRawWaitExtended(TD_STREAM_t *stream, bool state,uint32_t timeout)
{
	STREAM_GET_UART(stream,uart,false);
	uint32_t rtc=RTC->CNT;

	while (((bool)GPIO_PinInGet(uart->rx_port, uart->rx_bit)) != state) {
		if (timeout && (((RTC->CNT-rtc) & 0xFFFFFF) > timeout)) {
			return false;
	}
	}
	return true;
}

/***************************************************************************//**
 * @brief
 *   Set a GPIO IRQ callback function to wait for UART RX.
 *
 * @param[in] stream
 *   Pointer to the stream opaque descriptor.
 *
 * @param[in] callback
 *   Callback function to call when an UART RX IRQ is received.
 ******************************************************************************/
void TD_UART_RxIRQSetup(TD_STREAM_t *stream,TD_GPIO_callback_t callback)
{
	STREAM_GET_UART(stream,uart,);
	GPIO_IntConfig(uart->rx_port,uart->rx_bit,false,true,callback!=NULL);
	TD_GPIO_SetCallbackExtended(uart->rx_bit,callback);
}

/***************************************************************************//**
 * @brief
 *   Wait for UART RX IRQ.
 *
 * @param[in] stream
 *   Pointer to the stream opaque descriptor.
 ******************************************************************************/
void TD_UART_RxIRQWaitExtended(TD_STREAM_t *stream)
{
	STREAM_GET_UART(stream,uart,);
	RxFinised = false;
	GPIO_IntConfig(uart->rx_port,uart->rx_bit,true,true,true);
	TD_GPIO_SetCallbackExtended(uart->rx_bit, RXIRQReceived);
	while (true) {
		__set_PRIMASK(1);
		if (RxFinised) {
			break;
		}
		TD_RTC_Sleep();
		__set_PRIMASK(0);
	}
	__set_PRIMASK(0);
	GPIO_IntConfig(uart->rx_port,uart->rx_bit,true,true,false);
}

/***************************************************************************//**
 * @brief
 *   Stop UART peripheral (unroute pin, remove EM1 mode, etc)
 *   In UART and USART mode, stop clock. In LEUART, don't stop clock
 ******************************************************************************/
void TD_UART_DisableExt(TD_STREAM_t *stream)
{
	STREAM_GET_UART(stream,uart,);
	// Disable LEUART0 TX and RX route
	TD_UART_Stop(stream);
	GPIO_PinModeSet(uart->tx_port, uart->tx_bit, gpioModeDisabled, 0);
	if (((TD_STREAM_cast_t*)stream)->mode & (TD_STREAM_RDONLY | TD_STREAM_RDWR)) {
		GPIO_PinModeSet(uart->rx_port, uart->rx_bit, gpioModeDisabled, 0);
	}

	// To avoid breaking something, stopping LEUART clock is done only in
	// Extended function, not in Ext function
	if (!uart->leuart) {
		CMU_ClockEnable(uart->clk, false);
	}
	if(uart->em1) {
		TD_RTC_SetPowerMode(TD_RTC_EM2);
	}
	uart_irq_table[uart->irq_index]=NOT_ROUTED_IRQ;
}

/***************************************************************************//**
 * @brief
 *   Stop UART peripheral (unroute pin, remove EM1 mode, etc) and stop associated
 *   clock.
 *   Identical to TD_UART_DisableExt for USART and UART
 *   Add clock stop for LEUART
 *
 * @param[in] stream
 *   Pointer to the stream opaque descriptor.
 ******************************************************************************/
void TD_UART_DisableExtended(TD_STREAM_t *stream)
{
	STREAM_GET_UART(stream,uart,);

	// Previous Code keep for information
/*	if (uart->leuart) {
		// To avoid breaking something, stopping LEUART clock is done only for
		// extended function
		TD_UART_Disable();
		CMU_ClockEnable(uart->clk, false);
	} else {
		CMU_ClockEnable(uart->clk, false);
	}*/
	// Prevent receiving char before setup ended
	NVIC_DisableIRQ(uart->irq);
	TD_UART_DisableExt(stream);
	CMU_ClockEnable(uart->clk, false);
}

/***************************************************************************//**
 * @brief
 *   Send a character to the UART.
 *
 * @param[in] stream
 *   Pointer to the stream opaque descriptor.
 *
 * @param[in] c
 *   The character to send.
 ******************************************************************************/
void TD_UART_Putc(TD_STREAM_t *stream, char c)
{
	STREAM_GET_UART(stream,uart,);
	uart->tx = true;
	if (uart->leuart) {
		LEUART_Tx((LEUART_TypeDef *)uart->Raw, c);
	} else {
		USART_Tx((USART_TypeDef *)uart->Raw, c);
	}
}

/***************************************************************************//**
 * @brief
 *   Receive a character from the UART stream.
 *
 * @param[in] stream
 *   Pointer to the stream opaque descriptor.
 *
 * @return
 *   The received character if one is available, -1 otherwise.
 ******************************************************************************/
int TD_UART_GetCharExtended(TD_STREAM_t *stream)
{
	if (!STREAM_UART_IS_OPENED(stream)){
		return -1;
	}
	return TD_STREAM_GetFIFO(stream);
}

/***************************************************************************//**
 * @brief
 *   Returns the number of available characters from the UART stream.
 *
 * @return
 *   The number of available characters.
 *
 * @param[in] stream
 *   Pointer to the stream opaque descriptor.
 *
 ******************************************************************************/
int TD_UART_AvailableCharsExt(TD_STREAM_t *stream)
{
	if (!STREAM_UART_IS_OPENED(stream)){
		return 0;
	}
	return TD_STREAM_AvailableCharsInFIFO(stream);
}

/***************************************************************************//**
 * @brief
 *   Flush the UART RX buffer.
 *
 * @param[in] stream
 *   Pointer to the stream opaque descriptor.
 ******************************************************************************/
void TD_UART_FlushExt(TD_STREAM_t *stream)
{
	if (!STREAM_UART_IS_OPENED(stream)){
		return;
	}
	TD_STREAM_FlushFIFO(stream);
}

/***************************************************************************//**
 * @brief
 *   Send a string to the UART stream.
 *
 * @param[in] stream
 *   Pointer to the stream opaque descriptor.
 *
 * @param[in] string
 *   Pointer to null-terminated string to send to the UART.
 ******************************************************************************/
void TD_UART_SendStringExt(TD_STREAM_t *stream, char *string)
{
	char c;

	if (!STREAM_UART_IS_OPENED(stream)){
		return;
	}
	while ((c = *string++) != '\0') {
		TD_UART_Putc(stream, c);
	}
}

/***************************************************************************//**
 * @brief
 *   Send a buffer to the UART stream.
 *
 * @param[in] stream
 *   Pointer to the stream opaque descriptor.

 * @param[in] buffer
 *   Pointer to buffer to send to the UART.
 *
 * @param[in] length
 *   Length in bytes of the buffer to send to the UART.
 ******************************************************************************/
void TD_UART_SendBuffer(TD_STREAM_t *stream, char *buffer, int length)
{
	while (length--) {
		TD_UART_Putc(stream, *buffer++);
	}
}

/***************************************************************************//**
 * @brief
 *   Send a character repeatedly to the UART stream.
 *
 * @param[in] stream
 *   Pointer to the stream opaque descriptor.

 * @param[in] ch
 *   Character to repeat
 *
 * @param[in] count
 *   Repeat count.
 ******************************************************************************/
void TD_UART_SendRepeat(TD_STREAM_t *stream, char ch, int count)
{
	if (!STREAM_UART_IS_OPENED(stream)){
		return;
	}
	while (count--) {
		TD_UART_Putc(stream, ch);
	}
}

/***************************************************************************//**
 * @brief
 *   Get UART stream statistics.
 *
 * @param[in] stream
 *   Pointer to the stream opaque descriptor.
 *
 * @param[in] statistics
 *   Pointer to statistics structure.
 ******************************************************************************/
void TD_UART_GetStats(TD_STREAM_t *stream, TD_UART_Stats_t *statistics)
{
	STREAM_GET_UART(stream,uart,);
	statistics->HardOverflow = uart->Stats.HardOverflow;
	statistics->SoftOverflow = TD_STREAM_GetFIFOOverflow(stream);
}

/***************************************************************************//**
 * @brief
 *   Get the UART port from a stream.
 *
 * @param[in] stream
 *   Pointer to the stream opaque descriptor.
 ******************************************************************************/
TD_UART_port_t *TD_UART_GetPort(TD_STREAM_t *stream)
{
	STREAM_GET_UART(stream,uart,NULL);
	return uart;
}

/***************************************************************************//**
 * @brief
 *   Set custom leuart interrupt handler
 *
 * @param[in] stream
 *   Pointer to callback. Callback should return true for normal interrupt
 *   handler to be executed.
 *
 * @param[in] hook
 *   The UART hook to set.
 ******************************************************************************/
void TD_UART_SetRxHook(TD_STREAM_t *stream, TD_UART_Hook_t hook)
{
	STREAM_GET_UART(stream,uart,);
	uart->rxHook = hook;
}

/***************************************************************************//**
 * @brief
 *   Dump the UART using a given port.
 *
 * @param[in] p
 *   Pointer to the UART port.
 ******************************************************************************/
void TD_UART_Dump(TD_UART_port_t *p)
{
	if (p->Raw){
		tfp_printf(LGC("TxPort:%c%d RxPort:%c%d EM1:%d\r\n"),
			p->tx_port + 'A',
			p->tx_bit,
			p->rx_port + 'A',
			p->rx_bit,
			p->em1);
		tfp_printf(LGC("Clk:0x%08X Irq:0x%08X\r\n"),p->clk,p->irq);
		tfp_printf(LGC("PowerMode:%d Enable:0x%08X\r\n"),
			TD_RTC_GetPowerMode(), p->PortEnable);
		tfp_printf(LGC("leuart:%d irq_index:%d tx:%d loc:%d\r\n"),
			p->leuart, p->irq_index, p->tx, p->location);
		tfp_printf(LGC("baudrate:%d Hook:0x%08X\r\n"),p->baudrate,p->rxHook);
	}
}

/***************************************************************************//**
 * @brief
 *   Dump all UARTs.
 ******************************************************************************/
void TD_UART_DumpAll(void)
{
uint8_t i;
TD_UART_ClockUsage cu;

	for (i=0;i<CONFIG_TD_UART_COUNT;i++){
		tfp_printf(LGC("----- TD_UART[%d] -----\r\n"),i);
		TD_UART_Dump(&TD_UART[i]);
	}
	tfp_printf(LGC("===Clock usage===\r\n"));
	TD_UART_GetClockUsage(&cu,NULL);
	tfp_printf(LGC("cu.leuart_corelediv2:%d\r\n"),cu.leuart_corelediv2);
	tfp_printf(LGC("cu.leuart_lfxo:%d\r\n"),cu.leuart_lfxo);
	tfp_printf(LGC("===IRQ routing===\r\n"));
	for (i=0;i<sizeof(uart_irq_table);i++){
		tfp_printf(LGC("uart_irq_table[%d]=%d\r\n"),i,uart_irq_table[i]);
	}
	TD_GPIO_Dump();
}

/** @} */

/***************************************************************************//**
 * @addtogroup LEUART_LEGACY LEUART_LEGACY
 * @brief Legacy function for UART handling
 * @{
 * @details
 *
 *   # History
 *	On first library, only LEUART (on fixed location was used)
 *	Main usage was 9600 bps LEUART in EM2 mode for AT modem
 *	Function are in type of TD_UART_GetChar, TD_UART_PutChar
 *
 *	During time, this "LEUART" aka "System UART" ha been made configurable with
 *	LEUART_DEVICE and LEUART_LOCATION. Main purpose is always "System UART"
 *  (system console) even in product without direct serial usage.
 *
 *	In the mean time, this LEUART device can also be not a "LEUART" device but
 *  a USART or UART device.
 *
 *	At some point, stream was added to unchain uart and data streaming for other
 *  devices (RF, ...)
 *
 *   # Now
 *
 *  System UART is defined by SYSTEM_UART and SYSTEM_UART_LOCATION directive
 *  (replace LEUART_DEVICE, LEUART_LOCATION).
 *  Legacy function without stream directive now always use first opened UART.
 *  In most case it will be system uart (as defined by SYSTEM_UART or old
 *  LEUART_DEVICE directive) acting as previous library.
 ******************************************************************************/

/***************************************************************************//**
 * @brief
 *   Disable the System UART (first opened) peripheral.
 ******************************************************************************/
void TD_UART_Disable(void)
{
	if (!UART_IS_OPENED(&TD_UART[DEFAULT_UART])){
		return;
	}
	TD_UART_DisableExt(TD_UART[DEFAULT_UART].stream);
}

/***************************************************************************//**
 * @brief
 *   Send a character to the LEUART0.
 *
 * @param[in] c
 *   The character to send.
 ******************************************************************************/
void TD_UART_PutChar(char c)
{
	if (!UART_IS_OPENED(&TD_UART[DEFAULT_UART])){
		return;
	}
	TD_UART_Putc(TD_UART[DEFAULT_UART].stream, c);
}

/***************************************************************************//**
 * @brief
 *   Receive a character from the LEUART0.
 *
 * @return
 *   Returns the received character if one is available, -1 otherwise.
 ******************************************************************************/
int TD_UART_GetChar(void)
{
	if (!UART_IS_OPENED(&TD_UART[DEFAULT_UART])){
		return -1;
	}
	return TD_UART_GetCharExtended(TD_UART[DEFAULT_UART].stream);
}

/***************************************************************************//**
 * @brief
 *   Get the Communication Baudrate
 *
 * @return
 *   Returns the UART baudrate.
 ******************************************************************************/
uint32_t TD_UART_GetBaudrate(void)
{
	if (!UART_IS_OPENED(&TD_UART[DEFAULT_UART])){
		return 0;
	}
	return TD_UART[DEFAULT_UART].baudrate;
}

/***************************************************************************//**
 * @brief
 *   Get the Communication Mode (if RS485, give half or full duplex)
 *
 * @return
 *   Returns the UART communication mode.
 ******************************************************************************/
uint32_t TD_UART_GetModeComm(void)
{
	if (!UART_IS_OPENED(&TD_UART[DEFAULT_UART])){
		return 0;
	}
	return TD_UART[DEFAULT_UART].modecomm;
}

/***************************************************************************//**
 * @brief
 *   Wait for UART RX IRQ.
 ******************************************************************************/
void TD_UART_RxIRQWait(void)
{
	if (!UART_IS_OPENED(&TD_UART[DEFAULT_UART])){
		return;
	}
	TD_UART_RxIRQWaitExtended(TD_UART[DEFAULT_UART].stream);
}


/***************************************************************************//**
 * @brief
 *   Returns the number of available characters from the LEUART0.
 *
 * @return
 *   The number of available characters.
 ******************************************************************************/
int TD_UART_AvailableChars(void)
{
	if (!UART_IS_OPENED(&TD_UART[DEFAULT_UART])){
		return 0;
	}
	return TD_UART_AvailableCharsExt(TD_UART[DEFAULT_UART].stream);
}

/***************************************************************************//**
 * @brief
 *   Flush the LEUART0 RX buffer.
 ******************************************************************************/
void TD_UART_Flush(void)
{
	if (!UART_IS_OPENED(&TD_UART[DEFAULT_UART])){
		return;
	}
	TD_UART_FlushExt(TD_UART[DEFAULT_UART].stream);
}

/***************************************************************************//**
 * @brief
 *   Send a string to the LEUART0.
 *
 * @param[in] string
 *   Pointer to null-terminated string to send to the UART.
 ******************************************************************************/
void TD_UART_SendString(char *string)
{
	if (!UART_IS_OPENED(&TD_UART[DEFAULT_UART])){
		return;
	}
	TD_UART_SendStringExt(TD_UART[DEFAULT_UART].stream, string);
}

/***************************************************************************//**
 * @brief
 *   Send a byte buffer to the LEUART0.
 *
 * @param[in] buffer
 *   Pointer to buffer to send to the UART.
 *
 * @param[in] length
 *   The length of the buffer to send to the UART.
 ******************************************************************************/
void TD_UART_Send(char *buffer, char length)
{
	if (!UART_IS_OPENED(&TD_UART[DEFAULT_UART])){
		return;
	}
	TD_UART_SendBuffer(TD_UART[DEFAULT_UART].stream, buffer, length);
}

/***************************************************************************//**
 * @brief
 *   LEUART interrupt handler.
 ******************************************************************************/
void LEUART_IRQHandler(uint8_t uart_index)
{
	bool ret = true;
	TD_UART_port_t *uart;

	DEBUG_IRQ_PRINTF("uart_index:%d\r\n",uart_irq_table[uart_index]);
	if (uart_irq_table[uart_index] == NOT_ROUTED_IRQ){
		TD_Trap(TRAP_UNHANDLED_IRQ,0x100+uart_index);
		return;
	}
	uart_index = uart_irq_table[uart_index];
	DEBUG_IRQ_PRINTF("S\r\n");
	uart=&TD_UART[uart_index-1];
	if (uart->rxHook) {
		ret = uart->rxHook(uart);
	}
	if (ret) {

		// Get the received byte
		DEBUG_PRINTF("IF:%02X ST:0x%02X EX:0x%02X\r\n",
			((LEUART_TypeDef *) uart->Raw)->IF,
			((LEUART_TypeDef *) uart->Raw)->STATUS,
			(((LEUART_TypeDef *) uart->Raw)->RXDATAXP) >> 8);
		uint16_t data = ((LEUART_TypeDef *)uart->Raw)->RXDATAX;
		if ((data&0xFF00)==0){
			TD_STREAM_PutFIFO(uart->stream, data&0xFF);
		}
	}
	TD_WakeMainLoop();
}

/***************************************************************************//**
 * @brief
 *   USART interrupt handler.
 ******************************************************************************/
void USART_IRQHandler(uint8_t uart_index)
{
	bool ret = true;
	TD_UART_port_t *uart;
	USART_TypeDef *usart;
	char data;

	DEBUG_IRQ_PRINTF("uart_index:%d\r\n",uart_irq_table[uart_index]);
	if (uart_irq_table[uart_index] == NOT_ROUTED_IRQ){
		TD_Trap(TRAP_UNHANDLED_IRQ,0x100+uart_index);
		return;
	}
	uart_index = uart_irq_table[uart_index];
	uart = &TD_UART[uart_index-1];
	usart = (USART_TypeDef *)uart->Raw;

	// Get the received byte
	DEBUG_IRQ_PRINTF("R\r\n");
	if (uart->rxHook) {
		ret = uart->rxHook(uart);
	}
	if (ret){
		if (usart->IF & USART_IF_RXOF) {
			if (uart->Stats.HardOverflow != 0xFF) {
				uart->Stats.HardOverflow++;
			}
		}
		while (usart->STATUS & USART_STATUS_RXDATAV) {
			data = usart->RXDATA;
			TD_STREAM_PutFIFO(uart->stream, data);
		}
	}
	TD_WakeMainLoop();
}

/***************************************************************************//**
 * @brief
 *   UART interrupt handler.
 ******************************************************************************/
#ifdef LEUART0
void LEUART0_IRQHandler(void)
{
	DEBUG_IRQ_PRINTF("LEUART0_IRQ\r\n");
	LEUART_IRQHandler(LEUART0_IRQ_INDEX);
}
#endif

/***************************************************************************//**
 * @brief
 *   LEUART1 interrupt handler.
 ******************************************************************************/
#ifdef LEUART1
void LEUART1_IRQHandler(void)
{
	DEBUG_IRQ_PRINTF("LEUART1_IRQ\r\n");
	LEUART_IRQHandler(LEUART1_IRQ_INDEX);
}
#endif

/***************************************************************************//**
 * @brief
 *   USART1 interrupt handler.
 ******************************************************************************/
#ifdef USART0
void USART0_RX_IRQHandler(void)
{
	DEBUG_IRQ_PRINTF("USART0_IRQ\r\n");
	USART_IRQHandler(USART0_IRQ_INDEX);
}
#endif

/***************************************************************************//**
 * @brief
 *   USART1 interrupt handler.
 ******************************************************************************/
#ifdef USART1
void USART1_RX_IRQHandler(void)
{
	DEBUG_IRQ_PRINTF("USART1_IRQ\r\n");
	USART_IRQHandler(USART1_IRQ_INDEX);
}
#endif

/***************************************************************************//**
 * @brief
 *   USART2 interrupt handler.
 ******************************************************************************/
#ifdef USART2
void USART2_RX_IRQHandler(void)
{
	DEBUG_IRQ_PRINTF("USART2_IRQ\r\n");
	USART_IRQHandler(USART2_IRQ_INDEX);
}
#endif

/***************************************************************************//**
 * @brief
 *   UART0 interrupt handler.
 ******************************************************************************/
#ifdef UART0
void UART0_RX_IRQHandler(void)
{
	DEBUG_IRQ_PRINTF("UART0_IRQ\r\n");
	USART_IRQHandler(UART0_IRQ_INDEX);
}
#endif

/***************************************************************************//**
 * @brief
 *   UART1 interrupt handler.
 ******************************************************************************/
#ifdef UART1
void UART1_RX_IRQHandler(void)
{
	DEBUG_IRQ_PRINTF("UART1_IRQ\r\n");
	USART_IRQHandler(UART1_IRQ_INDEX);
}
#endif

/** @} */

/** @} (end addtogroup UART) */
