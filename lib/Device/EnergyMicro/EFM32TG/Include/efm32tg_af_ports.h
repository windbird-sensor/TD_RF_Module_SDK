/**************************************************************************//**
 * @file
 * @brief efm32tg_af_ports Register and Bit Field definitions
 * @author Energy Micro AS
 * @version 3.20.2
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2012 Energy Micro AS, http://www.energymicro.com</b>
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
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Energy Micro AS has no
 * obligation to support this Software. Energy Micro AS is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Energy Micro AS will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 *****************************************************************************/
/**************************************************************************//**
 * @defgroup EFM32TG_AF_Ports
 * @{
 *****************************************************************************/

/** AF port number for location number i */
#define AF_CMU_CLK0_PORT(i)          ((i) == 0 ? 0 : (i) == 1 ? 2 : (i) == 2 ? 3 :  -1)
#define AF_CMU_CLK1_PORT(i)          ((i) == 0 ? 0 : (i) == 1 ? 3 : (i) == 2 ? 4 :  -1)
#define AF_LESENSE_CH0_PORT(i)       ((i) == 0 ? 2 :  -1)
#define AF_LESENSE_CH1_PORT(i)       ((i) == 0 ? 2 :  -1)
#define AF_LESENSE_CH2_PORT(i)       ((i) == 0 ? 2 :  -1)
#define AF_LESENSE_CH3_PORT(i)       ((i) == 0 ? 2 :  -1)
#define AF_LESENSE_CH4_PORT(i)       ((i) == 0 ? 2 :  -1)
#define AF_LESENSE_CH5_PORT(i)       ((i) == 0 ? 2 :  -1)
#define AF_LESENSE_CH6_PORT(i)       ((i) == 0 ? 2 :  -1)
#define AF_LESENSE_CH7_PORT(i)       ((i) == 0 ? 2 :  -1)
#define AF_LESENSE_CH8_PORT(i)       ((i) == 0 ? 2 :  -1)
#define AF_LESENSE_CH9_PORT(i)       ((i) == 0 ? 2 :  -1)
#define AF_LESENSE_CH10_PORT(i)      ((i) == 0 ? 2 :  -1)
#define AF_LESENSE_CH11_PORT(i)      ((i) == 0 ? 2 :  -1)
#define AF_LESENSE_CH12_PORT(i)      ((i) == 0 ? 2 :  -1)
#define AF_LESENSE_CH13_PORT(i)      ((i) == 0 ? 2 :  -1)
#define AF_LESENSE_CH14_PORT(i)      ((i) == 0 ? 2 :  -1)
#define AF_LESENSE_CH15_PORT(i)      ((i) == 0 ? 2 :  -1)
#define AF_LESENSE_ALTEX0_PORT(i)    ((i) == 0 ? 3 :  -1)
#define AF_LESENSE_ALTEX1_PORT(i)    ((i) == 0 ? 3 :  -1)
#define AF_LESENSE_ALTEX2_PORT(i)    ((i) == 0 ? 0 :  -1)
#define AF_LESENSE_ALTEX3_PORT(i)    ((i) == 0 ? 0 :  -1)
#define AF_LESENSE_ALTEX4_PORT(i)    ((i) == 0 ? 0 :  -1)
#define AF_LESENSE_ALTEX5_PORT(i)    ((i) == 0 ? 4 :  -1)
#define AF_LESENSE_ALTEX6_PORT(i)    ((i) == 0 ? 4 :  -1)
#define AF_LESENSE_ALTEX7_PORT(i)    ((i) == 0 ? 4 :  -1)
#define AF_ACMP0_OUT_PORT(i)         ((i) == 0 ? 4 : (i) == 1 ? -1 : (i) == 2 ? 3 :  -1)
#define AF_ACMP1_OUT_PORT(i)         ((i) == 0 ? 5 : (i) == 1 ? -1 : (i) == 2 ? 3 :  -1)
#define AF_USART0_TX_PORT(i)         ((i) == 0 ? 4 : (i) == 1 ? 4 : (i) == 2 ? 2 : (i) == 3 ? 4 : (i) == 4 ? 1 : (i) == 5 ? 2 :  -1)
#define AF_USART0_RX_PORT(i)         ((i) == 0 ? 4 : (i) == 1 ? 4 : (i) == 2 ? 2 : (i) == 3 ? 4 : (i) == 4 ? 1 : (i) == 5 ? 2 :  -1)
#define AF_USART0_CLK_PORT(i)        ((i) == 0 ? 4 : (i) == 1 ? 4 : (i) == 2 ? 2 : (i) == 3 ? 2 : (i) == 4 ? 1 : (i) == 5 ? 1 :  -1)
#define AF_USART0_CS_PORT(i)         ((i) == 0 ? 4 : (i) == 1 ? 4 : (i) == 2 ? 2 : (i) == 3 ? 2 : (i) == 4 ? 1 : (i) == 5 ? 1 :  -1)
#define AF_USART1_TX_PORT(i)         ((i) == 0 ? 2 : (i) == 1 ? 3 : (i) == 2 ? 3 :  -1)
#define AF_USART1_RX_PORT(i)         ((i) == 0 ? 2 : (i) == 1 ? 3 : (i) == 2 ? 3 :  -1)
#define AF_USART1_CLK_PORT(i)        ((i) == 0 ? 1 : (i) == 1 ? 3 : (i) == 2 ? 5 :  -1)
#define AF_USART1_CS_PORT(i)         ((i) == 0 ? 1 : (i) == 1 ? 3 : (i) == 2 ? 5 :  -1)
#define AF_TIMER0_CC0_PORT(i)        ((i) == 0 ? 0 : (i) == 1 ? 0 : (i) == 2 ? -1 : (i) == 3 ? 3 : (i) == 4 ? 0 : (i) == 5 ? 5 :  -1)
#define AF_TIMER0_CC1_PORT(i)        ((i) == 0 ? 0 : (i) == 1 ? 0 : (i) == 2 ? -1 : (i) == 3 ? 3 : (i) == 4 ? 2 : (i) == 5 ? 5 :  -1)
#define AF_TIMER0_CC2_PORT(i)        ((i) == 0 ? 0 : (i) == 1 ? 0 : (i) == 2 ? -1 : (i) == 3 ? 3 : (i) == 4 ? 2 : (i) == 5 ? 5 :  -1)
#define AF_TIMER0_CDTI0_PORT(i)      (-1)
#define AF_TIMER0_CDTI1_PORT(i)      (-1)
#define AF_TIMER0_CDTI2_PORT(i)      (-1)
#define AF_TIMER1_CC0_PORT(i)        ((i) == 0 ? 2 : (i) == 1 ? 4 : (i) == 2 ? -1 : (i) == 3 ? 1 : (i) == 4 ? 3 :  -1)
#define AF_TIMER1_CC1_PORT(i)        ((i) == 0 ? 2 : (i) == 1 ? 4 : (i) == 2 ? -1 : (i) == 3 ? 1 : (i) == 4 ? 3 :  -1)
#define AF_TIMER1_CC2_PORT(i)        ((i) == 0 ? 2 : (i) == 1 ? 4 : (i) == 2 ? -1 : (i) == 3 ? 1 : (i) == 4 ? 2 :  -1)
#define AF_TIMER1_CDTI0_PORT(i)      (-1)
#define AF_TIMER1_CDTI1_PORT(i)      (-1)
#define AF_TIMER1_CDTI2_PORT(i)      (-1)
#define AF_PRS_CH0_PORT(i)           ((i) == 0 ? 0 : (i) == 1 ? 5 :  -1)
#define AF_PRS_CH1_PORT(i)           ((i) == 0 ? 0 : (i) == 1 ? 5 :  -1)
#define AF_PRS_CH2_PORT(i)           ((i) == 0 ? 2 : (i) == 1 ? 5 :  -1)
#define AF_PRS_CH3_PORT(i)           ((i) == 0 ? 2 : (i) == 1 ? 4 :  -1)
#define AF_LEUART0_TX_PORT(i)        ((i) == 0 ? 3 : (i) == 1 ? 1 : (i) == 2 ? 4 : (i) == 3 ? 5 : (i) == 4 ? 5 :  -1)
#define AF_LEUART0_RX_PORT(i)        ((i) == 0 ? 3 : (i) == 1 ? 1 : (i) == 2 ? 4 : (i) == 3 ? 5 : (i) == 4 ? 0 :  -1)
#define AF_LETIMER0_OUT0_PORT(i)     ((i) == 0 ? 3 : (i) == 1 ? 1 : (i) == 2 ? 5 : (i) == 3 ? 2 :  -1)
#define AF_LETIMER0_OUT1_PORT(i)     ((i) == 0 ? 3 : (i) == 1 ? 1 : (i) == 2 ? 5 : (i) == 3 ? 2 :  -1)
#define AF_PCNT0_S0IN_PORT(i)        ((i) == 0 ? 2 : (i) == 1 ? -1 : (i) == 2 ? 2 : (i) == 3 ? 3 :  -1)
#define AF_PCNT0_S1IN_PORT(i)        ((i) == 0 ? 2 : (i) == 1 ? -1 : (i) == 2 ? 2 : (i) == 3 ? 3 :  -1)
#define AF_I2C0_SDA_PORT(i)          ((i) == 0 ? 0 : (i) == 1 ? 3 : (i) == 2 ? 2 : (i) == 3 ? -1 : (i) == 4 ? 2 : (i) == 5 ? 5 : (i) == 6 ? 4 :  -1)
#define AF_I2C0_SCL_PORT(i)          ((i) == 0 ? 0 : (i) == 1 ? 3 : (i) == 2 ? 2 : (i) == 3 ? -1 : (i) == 4 ? 2 : (i) == 5 ? 5 : (i) == 6 ? 4 :  -1)
#define AF_DBG_SWO_PORT(i)           ((i) == 0 ? 5 : (i) == 1 ? 2 :  -1)
#define AF_DBG_SWDIO_PORT(i)         ((i) == 0 ? 5 : (i) == 1 ? 5 :  -1)
#define AF_DBG_SWCLK_PORT(i)         ((i) == 0 ? 5 : (i) == 1 ? 5 :  -1)

/** @} End of group EFM32TG_AF_Ports */


