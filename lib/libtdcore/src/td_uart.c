/***************************************************************************//**
 * @file
 * @brief Un Asynchronous Receiver/Transmitter (UART) peripheral API for the
 * TDxxxx RF modules.
 * @author Telecom Design S.A.
 * @version 2.0.3
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2012-2014 Telecom Design S.A., http://www.telecomdesign.fr</b>
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
#include "td_utils.h"

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

//#define DEBUG_UART
#ifdef DEBUG_UART

/** printf macro for UART debug */
#define DEBUG_PRINTF(...) tfp_printf(__VA_ARGS__)
#else

/** printf macro for UART debug */
#define DEBUG_PRINTF(...)
#endif

/** Macro to convert an opaque pointer to an UART port structure pointer */
#define PTR 			((TD_UART_port_t *) p)

/** Macro to convert an opaque pointer to an LEUART structure pointer */
#define PTR_RAW_LEUART 	((LEUART_TypeDef *) (PTR->Raw))

/** Macro to convert an opaque pointer to an USART structure pointer */
#define PTR_RAW_USART 	((USART_TypeDef *) (PTR->Raw))

/** @} */

/*******************************************************************************
 *************************   PRIVATE VARIABLES   *******************************
 ******************************************************************************/

/** @addtogroup UART_LOCAL_VARIABLES Local Variables
 * @{ */

/** Array of available UARTs */
extern TD_UART_port_t TD_UART[];

/** @} */

/*******************************************************************************
 **************************   PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup UART_USER_FUNCTIONS User Functions
 * @{ */

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
 *    Number of databits in frame. Should de leuartxxx or usartxxx function of
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
 * @return
 *   Returns an opaque UART descriptor (fixed to LEUART0).
 ******************************************************************************/
void *TD_UART_InitGlobal(void * interface, uint8_t location, uint32_t speed,
	bool rxEnable, bool shared, uint32_t databits, uint32_t parity,
	uint32_t stopbits)
{
	GPIO_Port_TypeDef tx_port, rx_port;
	uint8_t tx_bit, rx_bit;
	IRQn_Type irq = DMA_IRQn;
	uint8_t index = 0xFF;

	if (interface == (void *)LEUART0) {
		LEUART_Init_TypeDef init = LEUART_INIT_DEFAULT;
		index = 0;
		memset(&TD_UART[index], 0, sizeof (TD_UART_port_t));
		if (speed > 9600) {
			CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_CORELEDIV2);
			CMU_ClockDivSet(cmuClock_LEUART0, cmuClkDiv_4);

			// 14MHz / 2 pre-scaled by 4
			init.refFreq = 1750000;
		} else {
			CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFXO);
			CMU_ClockDivSet(cmuClock_LEUART0, cmuClkDiv_1);
			init.refFreq = 0;
		}
		// Enable the LEUART0 clock
		CMU_ClockEnable(cmuClock_LEUART0, true);
		init.enable = leuartDisable;
		init.baudrate = speed;
		init.databits = (LEUART_Databits_TypeDef) databits;
		init.parity = (LEUART_Parity_TypeDef) parity;
		init.stopbits = (LEUART_Stopbits_TypeDef) stopbits;

		// Reseting and initializing LEUART0
		LEUART_Reset(LEUART0);
		LEUART_Init(LEUART0, &init);
		LEUART_IntClear(LEUART0, LEUART_IF_RXDATAV);
		LEUART_IntEnable(LEUART0, LEUART_IF_RXDATAV);
		tx_port = CONFIG_LEUART_TX_PORT;
		tx_bit = CONFIG_LEUART_TX_BIT;
		rx_port = CONFIG_LEUART_RX_PORT;
		rx_bit = CONFIG_LEUART_RX_BIT;
		irq = LEUART0_IRQn;
		TD_UART[index].leuart = true;
		TD_UART[index].Raw = LEUART0;
	}
	if (interface == (void *)USART1) {
		index = 1;
		memset(&TD_UART[index],0,sizeof (TD_UART_port_t));
	    USART_InitAsync_TypeDef init = USART_INITASYNC_DEFAULT;
		CMU_ClockEnable(cmuClock_HFPER, true);
	  	CMU_ClockEnable(cmuClock_USART1, true);
      	CMU_ClockEnable(cmuClock_GPIO, true);

      	init.enable = usartDisable;
      	init.baudrate = speed;
      	//init.refFreq = 0;
      	//init.oversampling = usartOVS16;
      	init.databits = (USART_Databits_TypeDef) databits;
      	init.parity = (USART_Parity_TypeDef) parity;
      	init.stopbits = (USART_Stopbits_TypeDef) stopbits;

      	USART_InitAsync(USART1, &init);
      	USART_IntClear(USART1, USART_IF_RXDATAV);
      	USART_IntEnable(USART1, USART_IF_RXDATAV);
      	tx_port = gpioPortC;
      	tx_bit = 0;
      	rx_port = gpioPortC;
      	rx_bit = 1;
		irq = USART1_RX_IRQn;
		TD_UART[index].leuart = false;
		TD_UART[index].Raw = USART1;
		DEBUG_PRINTF("USART1 speed:%d index:%d rxEnable:%d rx:%c%d shared:%d leuart:%d\r\n",
			speed, index, rxEnable, 'A' + rx_port, rx_bit, shared,
			TD_UART[index].leuart);
	}
	if (index == 0xFF) {
		DEBUG_PRINTF("UART Invalid Index !\r\n");
		return NULL;
	}
	if (index >= CONFIG_TD_UART_COUNT) {
		DEBUG_PRINTF("UART Index out of range !\r\n");
		return NULL;
	}

	// TX PORT
	GPIO_PinModeSet(tx_port, tx_bit, gpioModePushPull, 1);

	// Always enable TX
	TD_UART[index].PortEnable = TD_UART[index].leuart ? leuartEnableTx :
		usartEnableTx;	// Note : leuartEnablexxx and usartEnablexxx are the same
	if (rxEnable) {

		// RX PORT
	    GPIO_PinModeSet(rx_port, rx_bit, gpioModeInput, 0);

		// Clear previous RX interrupts
		NVIC_ClearPendingIRQ(irq);

		// Enable RX interrupts
		NVIC_EnableIRQ(irq);

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
		DEBUG_PRINTF("TD_UART_Start %d 0x%08X UART:0x%08X PTR:0x%08X\r\n",
			index, TD_UART[index].PortEnable, TD_UART,&TD_UART[index]);
		TD_UART_Start(&TD_UART[index]);
	}

	// Save port shared flag
	TD_UART[index].PortShared = shared;

	// Eventually enable UART
	if (index == 0) {
		LEUART_Enable(LEUART0, (LEUART_Enable_TypeDef) TD_UART[index].PortEnable);
	}
	if (index == 1) {
		USART_Enable(USART1, (USART_Enable_TypeDef) TD_UART[index].PortEnable);
	}
	return &TD_UART[index];
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
 *   Returns an opaque UART descriptor (fixed to LEUART0).
 ******************************************************************************/
void *TD_UART_InitExtended(uint32_t speed, bool rxEnable, bool shared,
	LEUART_Databits_TypeDef databits, LEUART_Parity_TypeDef parity,
	LEUART_Stopbits_TypeDef stopbits)
{
	return TD_UART_InitGlobal((void *) LEUART0, 0, speed, rxEnable, shared,
		databits, parity, stopbits);
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
 *   Returns an opaque UART descriptor (fixed to LEUART0).
 ******************************************************************************/
void *TD_UART_Init(uint32_t speed, bool rxEnable, bool shared)
{
	return TD_UART_InitExtended(
		speed,
		rxEnable,
		shared,
		leuartDatabits8,
		leuartNoParity,
		leuartStopbits1);
}

/***************************************************************************//**
 * @brief
 *   Start using the UART peripheral.
 *
 * @param[in] p
 *   Pointer to the UART opaque descriptor.
 ******************************************************************************/
void TD_UART_Start(void *p)
{
	if (PTR->PortShared) {

		DEBUG_PRINTF("UART_Start shared leuart:%d shared:%d\r\n", PTR->leuart,
			PTR->PortShared);

		// Enable LEUART0 TX and RX route
		if (PTR->leuart){
			if (PTR->PortEnable & leuartEnableRx) {
				LEUART0->ROUTE = LEUART_ROUTE_TXPEN | LEUART_ROUTE_RXPEN |
					CONFIG_LEUART_LOCATION;
				LEUART0->CMD = LEUART_CMD_TXDIS | LEUART_CMD_RXDIS |
					LEUART_CMD_CLEARTX | LEUART_CMD_CLEARRX;
				LEUART0->CMD = LEUART_CMD_TXEN | LEUART_CMD_RXEN;
			}
			else{
				LEUART0->ROUTE = LEUART_ROUTE_TXPEN | CONFIG_LEUART_LOCATION;
				LEUART0->CMD = LEUART_CMD_TXDIS | LEUART_CMD_CLEARTX;
				LEUART0->CMD = LEUART_CMD_TXEN;
			}
		} else {
			DEBUG_PRINTF("UART_Start not leuart 0x%08X 0x%08X\r\n",
				PTR->PortEnable, usartEnableRx);
			if (PTR->PortEnable & usartEnableRx){
				DEBUG_PRINTF("USART1->ROUTE RX\r\n");
		      	USART1->ROUTE = USART_ROUTE_RXPEN | USART_ROUTE_TXPEN |
		      		USART_ROUTE_LOCATION_LOC0;
			}
			else{
				DEBUG_PRINTF("USART1->ROUTE TX only\r\n");
		      	USART1->ROUTE = USART_ROUTE_TXPEN | USART_ROUTE_LOCATION_LOC0;
			}
		}
	}
}

/***************************************************************************//**
 * @brief
 *   Stop using the UART peripheral.
 *
 * @param[in] p
 *   Pointer to the UART opaque descriptor.
 ******************************************************************************/
void TD_UART_Stop(void *p)
{
	// Wait end of transmission
	while (!(PTR_RAW_LEUART->STATUS & LEUART_STATUS_TXC));
	if (PTR->PortShared) {

		// Disable LEUART0 TX and RX route
		LEUART0->CMD = LEUART_CMD_TXDIS | LEUART_CMD_RXDIS |
			LEUART_CMD_CLEARTX | LEUART_CMD_CLEARRX;
		LEUART0->ROUTE = 0;
	}
}

/***************************************************************************//**
 * @brief
 *   Disable the LEUART peripheral.
 ******************************************************************************/
void TD_UART_Disable(void)
{
	// Disable LEUART0 TX and RX route
	LEUART0->CMD = LEUART_CMD_TXDIS | LEUART_CMD_RXDIS |
		LEUART_CMD_CLEARTX | LEUART_CMD_CLEARRX;
	LEUART0->ROUTE = 0;
	GPIO_PinModeSet(TX_PORT, TX_BIT, gpioModeDisabled, 1);
	GPIO_PinModeSet(RX_PORT, RX_BIT, gpioModeDisabled, 1);
}

/***************************************************************************//**
 * @brief
 *   Disable the UART.
 *
 * @param[in] p
 *   Pointer to the UART opaque descriptor.
 ******************************************************************************/
void TD_UART_DisableExtended(void *p)
{
	if (PTR->leuart){
		// To avoid breaking something, stopping LEUART clock is done only for
		// extended function
		CMU_ClockEnable(cmuClock_LEUART0, false);
		TD_UART_Disable();
	} else {
		CMU_ClockEnable(cmuClock_USART1, false);
	}
}

/***************************************************************************//**
 * @brief
 *   Send a character to the UART.
 *
 * @param[in] p
 *   Pointer to the UART opaque descriptor.
 *
 * @param[in] c
 *   The character to send.
 ******************************************************************************/
void TD_UART_Putc(void *p, char c)
{
	LEUART_Tx(LEUART0, c);
}

/***************************************************************************//**
 * @brief
 *   Receive a character from the UART.
 *
 * @return
 *   The received character if one is available, -1 otherwise.
 ******************************************************************************/
int TD_UART_GetCharExtended(void *p)
{
	int c;
	int next;

	if (PTR->RxReadIndex == PTR->RxWriteIndex) {
		return -1;
	}

	/* Here we get one char, and THEN update pointer.
	 * If buffer is full and an IRQ would add a character, no possible loss
	 */
	c = PTR->RxBuffer[PTR->RxReadIndex];
	next = PTR->RxReadIndex+1;
	if (next >= TD_UART_RXBUFSIZE) {

		// Wrapped RX read Index
		next = 0;
	}
	PTR->RxReadIndex = next;
	return c;
}

/***************************************************************************//**
 * @brief
 *   Receive a character from the UART.
 *
 * @return
 *   The received character if one is available, -1 otherwise.
 ******************************************************************************/
int TD_UART_GetChar(void)
{
	return TD_UART_GetCharExtended(&TD_UART[0]);
}

/***************************************************************************//**
 * @brief
 *   Returns the number of available characters from the UART.
 *
 * @return
 *   The number of available characters.
 *
 * @param[in] p
 *   Pointer to the UART opaque descriptor.
 *
 ******************************************************************************/
int TD_UART_AvailableCharsExt(void *p)
{
	int count = PTR->RxWriteIndex - PTR->RxReadIndex;

	return count < 0 ? TD_UART_RXBUFSIZE + count : count;
}

/***************************************************************************//**
 * @brief
 *   Returns the number of available characters from the UART.
 *
 * @return
 *   The number of available characters.
 ******************************************************************************/
int TD_UART_AvailableChars(void)
{
	return TD_UART_AvailableCharsExt(&TD_UART[0]);
}

/***************************************************************************//**
 * @brief
 *   Flush the UART RX buffer.
 ******************************************************************************/
void TD_UART_Flush(void)
{
	TD_UART[0].RxReadIndex = TD_UART[0].RxWriteIndex;
}

/***************************************************************************//**
 * @brief
 *   Flush the UART RX buffer.
 *
 * @param[in] p
 *   Pointer to the UART opaque descriptor.
 ******************************************************************************/
void TD_UART_FlushExt(void *p)
{
	PTR->RxReadIndex = PTR->RxWriteIndex;
}

/***************************************************************************//**
 * @brief
 *   Send a string to the UART.
 *
 * @param[in] string
 *   Pointer to null-terminated string to send to the UART.
 ******************************************************************************/
void TD_UART_SendString(char *string)
{
	char c;

	while ((c = *string++) != '\0') {
		LEUART_Tx(LEUART0, c);
	}
}

/***************************************************************************//**
 * @brief
 *   Send a buffer to the UART.
 *
 * @param[in] p
 *   Pointer to the UART opaque descriptor.

 * @param[in] buffer
 *   Pointer to buffer to send to the UART.
 *
 * @param[in] length
 *   Length in bytes of the buffer to send to the UART.
 ******************************************************************************/
void TD_UART_SendBuffer(void *p, char *buffer, int length)
{
	if (PTR->leuart) {
		while (length--) {
			LEUART_Tx(PTR_RAW_LEUART, *buffer++);
		}
	} else {
		while (length--) {
			USART_Tx(PTR_RAW_USART, *buffer++);
		}
	}
}

/***************************************************************************//**
 * @brief
 *   Get UART statistics.
 *
 * @param[in] p
 *   Pointer to the UART opaque descriptor.

 * @param[in] statistics
 *   Pointer to statistics structure.
 ******************************************************************************/
void TD_UART_GetStats(void *p, TD_UART_Stats_t *statistics)
{
	memcpy(statistics, &(PTR->Stats), sizeof (TD_UART_Stats_t));
}

/***************************************************************************//**
 * @brief
 *   Send a character repeatedly to the UART.
 *
 * @param[in] p
 *   Pointer to the UART opaque descriptor.

 * @param[in] ch
 *   Character to repeat
 *
 * @param[in] count
 *   Repeat count.
 ******************************************************************************/
void TD_UART_SendRepeat(void *p, char ch, int count)
{
	if (PTR->leuart) {
		while (count--) {
			LEUART_Tx(PTR_RAW_LEUART, ch);
		}
	} else {
		while (count--) {
			USART_Tx(PTR_RAW_USART, ch);
		}
	}
}

/***************************************************************************//**
 * @brief
 *   Send a byte buffer to the UART.
 *
 * @param[in] buffer
 *   Pointer to buffer to send to the UART.
 *
 * @param[in] length
 *   The length of the buffer to send to the UART.
 ******************************************************************************/
void TD_UART_Send(char *buffer, char length)
{
	int i;

	// Check that the transmit buffer is empty
	for (i = 0; i < length; i++) {
		LEUART_Tx(LEUART0, buffer[i]);
	}
}

/***************************************************************************//**
 * @brief
 *   UART interrupt handler.
 ******************************************************************************/
void LEUART0_IRQHandler(void)
{
	// Get the received byte
	DEBUG_PRINTF("IF:%02X ST:0x%02X EX:0x%02X\r\n",
		LEUART0->IF, LEUART0->STATUS, (LEUART0->RXDATAXP) >> 8);
	char data = LEUART0->RXDATA;
	int next;

	next = TD_UART[0].RxWriteIndex + 1;
	if (next >= TD_UART_RXBUFSIZE) {
		next = 0;
	}

	// Is not overflow?
	if (next != TD_UART[0].RxReadIndex) {

		// Store and update index
		TD_UART[0].RxBuffer[TD_UART[0].RxWriteIndex] = data;
		TD_UART[0].RxWriteIndex = next;
	} else {
		if (TD_UART[0].Stats.SoftOverflow != 0xFF) {
			TD_UART[0].Stats.SoftOverflow++;
		}
	}
	TD_WakeMainLoop();
}

/***************************************************************************//**
 * @brief
 *   UART interrupt handler.
 ******************************************************************************/
void USART1_RX_IRQHandler(void)
{
	// Get the received byte
	//DEBUG_PRINTF("IRQ\r\n");
	DEBUG_PRINTF("R\r\n");
	char data;
	int next;

	if (USART1->IF & USART_IF_RXOF) {
		if (TD_UART[1].Stats.HardOverflow != 0xFF) {
			TD_UART[1].Stats.HardOverflow++;
		}
	}
	while (USART1->STATUS & USART_STATUS_RXDATAV) {
		data = USART1->RXDATA;
		next = TD_UART[1].RxWriteIndex + 1;
		if (next >= TD_UART_RXBUFSIZE) {
			next = 0;
		}

		// Is not overflow?
		if (next != TD_UART[1].RxReadIndex) {

			// Store and update index
			TD_UART[1].RxBuffer[TD_UART[1].RxWriteIndex] = data;
			TD_UART[1].RxWriteIndex = next;
		} else {
			if (TD_UART[1].Stats.SoftOverflow != 0xFF) {
				TD_UART[1].Stats.SoftOverflow++;
			}
		}
	}
	TD_WakeMainLoop();
	//NVIC_ClearPendingIRQ(USART1_RX_IRQn);
}

/** @} */

/** @} (end addtogroup UART) */
