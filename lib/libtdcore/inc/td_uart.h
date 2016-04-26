/***************************************************************************//**
 * @file
 * @brief Un Asynchronous Receiver/Transmitter (UART) peripheral API for the
 * TDxxxx RF modules.
 * @author Telecom Design S.A.
 * @version 3.1.0
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

#ifndef __TD_UART_H
#define __TD_UART_H

#include <stdint.h>
#include <stdbool.h>

#include <em_leuart.h>
#include <em_gpio.h>
#include <em_cmu.h>
#include <td_gpio.h>

#include "td_stream.h"
#include "td_gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

	/***********************************************************************//**
	 * @addtogroup UART
	 * @brief Asynchronous Receiver/Transmitter (UART) peripheral API for the
	 * TDxxxx RF modules
	 * @{
	 **************************************************************************/

	/***************************************************************************
	 *************************   DEFINES   *************************************
	 **************************************************************************/

	/** @addtogroup UART_DEFINES Defines
	 * @{ */

	/** Receive buffer size */
#define TD_UART_RXBUFSIZE               61

	// This table route physical IRQ to opened ports
	// If set to NULL, IRQ is ignored
	// To add other IRQ, add a unique index and update MAX_IRQ_INDEX
#define LEUART0_IRQ_INDEX		0				/**< Index for LEUART0 IRQ */
#define LEUART1_IRQ_INDEX		1				/**< Index for LEUART1 IRQ */
#define USART0_IRQ_INDEX		2				/**< Index for USART0 IRQ */
#define USART1_IRQ_INDEX		3				/**< Index for USART1 IRQ */
#define USART2_IRQ_INDEX		4				/**< Index for USART2 IRQ */
#define UART0_IRQ_INDEX			5				/**< Index for UART0 IRQ */
#define UART1_IRQ_INDEX			6				/**< Index for UART1 IRQ */
#define MAX_IRQ_INDEX			7				/**< Maximum index for UART IRQ */

/** Macro to check whether an UART is opened or not */
#define UART_IS_OPENED(x)			((x) && (((TD_UART_port_t *)(x))->Raw))

/** Macro to check whether an UART stream is opened or not */
#define STREAM_UART_IS_OPENED(x)	((x) && (((TD_STREAM_cast_t*)(x))->io_handle) && \
	UART_IS_OPENED((((TD_STREAM_cast_t *)(x))->io_handle)))

/** Macro to get an UART pointer from a stream */
#define STREAM_GET_UART(stream, uart, retval)\
	if (!STREAM_UART_IS_OPENED(stream)) {\
		TD_Trap(TRAP_UART_CONFIG, 10);\
		return retval;\
	}\
	TD_UART_port_t *uart = ((TD_STREAM_cast_t *) stream)->io_handle;

	/** @} */

	/***************************************************************************
	 *************************   TYPEDEFS   ************************************
	 **************************************************************************/

	/** @addtogroup UART_TYPEDEFS Typedefs
	 * @{ */

	typedef struct TD_UART_port TD_UART_port_t;

	/** UART receive interrupt callback function pointer */
	typedef bool (*TD_UART_Hook_t)(TD_UART_port_t *p);

	/** UART communication mode */
	typedef enum Comm_Mode {
		COM_STD = 0,
		COM_RS485_FULL,
		COM_RS485_HALF,
	} Comm_Mode_t;

	/** UART open options */
	typedef struct {
		void *interface;						/**< Opaque pointer to hardware UART structure */
		uint8_t location;						/**< UART pin multiplexing location number */
		uint32_t baudrate;						/**< UART baudrate in bps */
		uint8_t data_bits;						/**< UART number of data bits */
		uint8_t parity;							/**< UART parity */
		uint8_t stop_bits;						/**< UART number of stop bits */
		bool shared_pins;						/**< UART flag for sharing GPIO pins */
		Comm_Mode_t modecomm;					/**< UART communication mode (standard, half or full duplex) */
		GPIO_Port_TypeDef rs485_port;			/**< UART GPIO port to use for RS485 control */
		uint32_t rs485_bit;						/**< UART GPIO bit to use for RS485 control */
	} TD_UART_Options_t;

	/** UART statistics */
	typedef struct {
		uint32_t HardOverflow;					/**< Overflow counter */
		uint32_t SoftOverflow;					/**< Overflow counter */
	} TD_UART_Stats_t;

	/** UART port structure */
	typedef struct TD_UART_port {
		void *Raw;								/**< Hard UART ref, if NULL not opened */
		bool leuart;							/**< Uart mode */
		bool PortShared;						/**< Flag for sharing the UART port with other GPIO functions */
		uint32_t PortEnable;					/**< UART enable flag */
		TD_UART_Stats_t Stats;					/**< UART statistics */
		uint16_t location;						/**< UART location */
		GPIO_Port_TypeDef tx_port;				/**< UART TX port */
		GPIO_Port_TypeDef rx_port;				/**< UART RX port */
		uint8_t rx_bit;							/**< UART TX bit */
		uint8_t tx_bit;							/**< UART TX bit */
		TD_STREAM_t *stream;					/**< UART stream */
		CMU_Clock_TypeDef clk;					/**< UART clk */
		IRQn_Type irq;							/**< UART irq */
		uint8_t irq_index;						/**< UART irq index for routing*/
		Comm_Mode_t modecomm;					/**< UART communication mode (standard, half or full duplex) */
		GPIO_Port_TypeDef rs485_port;			/**< UART GPIO port to use for RS485 control */
		uint8_t rs485_bit;						/**< UART GPIO bit to use for RS485 control */
		uint32_t baudrate;						/**< UART baudrate */
		bool tx;								/**< Indicate if at least one transmit was performed */
		bool em1;								/**< Indicate that this uart is forcing em1 */
		bool use_lfxo;							/**< Indicate that this uart is forcing em1 */
		TD_UART_Hook_t rxHook;					/**< UART hook */
		bool rxEnable;							/**< Rx enable for this port*/
		const volatile uint32_t *rx_data;		/**< UART RX data register */
		const volatile uint32_t *tx_data;		/**< UART TX data register */
		const volatile uint32_t *status;		/**< UART status register */
		uint32_t rx_flag;						/**< UART RX flag */
		uint32_t tx_flag;						/**< UART TX flag */
	} TD_UART_port_t;

	/** @} */

	/***************************************************************************
	 *************************   PROTOTYPES   **********************************
	 **************************************************************************/

	/** @addtogroup UART_USER_FUNCTIONS User Functions
	 * @{ */
	uint8_t TD_UART_AllocListDump(uint32_t *list);
	TD_STREAM_t *TD_UART_Open(TD_UART_Options_t *options, uint32_t mode);
	void TD_UART_Close(TD_STREAM_t *stream);
	TD_STREAM_t *TD_UART_InitGlobal(void *interface, uint8_t location, uint32_t speed,
		bool rxEnable, bool shared,	uint32_t databits, uint32_t parity,
		uint32_t stopbits, Comm_Mode_t mode, GPIO_Port_TypeDef rs485_port, uint32_t rs485_bit);
	TD_STREAM_t *TD_UART_InitExtended(uint32_t speed, bool rxEnable, bool shared,
		LEUART_Databits_TypeDef databits, LEUART_Parity_TypeDef parity,
		LEUART_Stopbits_TypeDef stopbits);
	TD_STREAM_t *TD_UART_Init(uint32_t speed, bool rxEnable, bool shared);
	void TD_UART_Start(TD_STREAM_t *stream);
	void TD_UART_Stop(TD_STREAM_t *stream);
	int TD_UART_GetCharExtended(TD_STREAM_t *stream);
	void TD_UART_SendBuffer(TD_STREAM_t *stream, char *buffer, int length);
	void TD_UART_SendRepeat(TD_STREAM_t *stream, char ch, int length);
	void TD_UART_Putc(TD_STREAM_t *stream, char c);
	void TD_UART_FlushExt(TD_STREAM_t *stream);
	void TD_UART_GetStats(TD_STREAM_t *stream, TD_UART_Stats_t *statistics);
	void TD_UART_DisableExt(TD_STREAM_t *stream);
	void TD_UART_PinMode(TD_STREAM_t *stream, GPIO_Mode_TypeDef mode_tx,
		unsigned int out_tx, GPIO_Mode_TypeDef mode_rx, unsigned int out_rx);
	void TD_UART_RxRawWait(TD_STREAM_t *stream, bool state);
	bool TD_UART_RxRawWaitExtended(TD_STREAM_t *stream, bool state,uint32_t timeout);
	void TD_UART_RxIRQWaitExtended(TD_STREAM_t *stream);
	void TD_UART_RxIRQSetup(TD_STREAM_t *uart,TD_GPIO_callback_t callback);
	TD_UART_port_t *TD_UART_GetPort(TD_STREAM_t *stream);
	void TD_UART_SetRxHook(TD_STREAM_t *stream, TD_UART_Hook_t hook);
	void TD_UART_SetBaudrate(TD_STREAM_t *stream,uint32_t baudrate);
	uint32_t TD_UART_GetModeCommExtended(TD_STREAM_t *stream);
	uint32_t TD_UART_GetBaudrateExtended(TD_STREAM_t *p);
	int TD_UART_AvailableCharsExt(TD_STREAM_t *stream);
	GPIO_Port_TypeDef TD_UART_GetPortRS485(TD_STREAM_t *p);
	uint32_t TD_UART_GetBitRS485(TD_STREAM_t *p);
	void TD_UART_DisableExtended(TD_STREAM_t *stream);
	void TD_UART_PutChar(char c);
	uint32_t TD_UART_GetModeComm(void);
	int TD_UART_GetChar(void);
	int TD_UART_AvailableChars(void);
	void TD_UART_Flush(void);
	uint32_t TD_UART_GetBaudrate(void);
	void TD_UART_SendString(char *string);
	void TD_UART_Send(char *buffer, char length);
	void TD_UART_Disable(void);
	void TD_UART_RxIRQWait(void);
	void TD_UART_DumpAll(void);
	void TD_UART_Dump(TD_UART_port_t *p);

	/** @} */

	/** @} (end addtogroup UART) */

#ifdef __cplusplus
}
#endif

#endif // __TD_UART_H
