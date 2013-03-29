/***************************************************************************//**
 * @file sensor_config.h
 * @brief Sensor Configuration
 * @author Telecom Design S.A.
 * @version 1.0.0
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2013 Telecom Design S.A., http://www.telecom-design.com</b>
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

#ifndef SENSOR_CONFIG_H_
#define SENSOR_CONFIG_H_

#define DEFAULT_DEVICE_CLASS 0x0000
#define RELEASE 2
#define SUB_RELEASE 2

#ifdef EFM32TG210F32
#define MAX_DEVICE 3
#define MAX_RETRANSMISSION 10
#define MAX_TRANSMISSION_QUEUE 3
#define SENSOR_MAX_SWITCH 3
#define MAX_SENSOR_EVENT 5
#define MAX_NACKED_SENSOR_EVENT 20
#elif EFM32G210F128
#define MAX_DEVICE 16
#define MAX_RETRANSMISSION 80
#define MAX_TRANSMISSION_QUEUE 20
#define SENSOR_MAX_SWITCH 8
#define MAX_SENSOR_EVENT 15
#define MAX_NACKED_SENSOR_EVENT 30
#endif

#endif /* SENSOR_CONFIG_H_ */
