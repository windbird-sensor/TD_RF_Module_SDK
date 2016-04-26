/***************************************************************************//**
 * @file
 * @brief Temperature/Supply Voltage measure API for the TDxxxx RF modules.
 * @author Telecom Design S.A.
 * @version 2.4.1
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

#include <em_chip.h>
#include <em_emu.h>
#include <em_vcmp.h>
#include <em_cmu.h>
#include <em_emu.h>
#include <em_adc.h>

#include "td_core.h"
#include "td_rtc.h"
#include "td_measure.h"

/***************************************************************************//**
 * @addtogroup MEASURE
 * @brief Temperature/Supply Voltage measure API for the TDxxxx RF modules
 * @{
 ******************************************************************************/

/*******************************************************************************
 *************************   PRIVATE VARIABLES   *******************************
 ******************************************************************************/

/** @addtogroup MEASURE_LOCAL_VARIABLES Local Variables
 * @{ */

/** Flag to lock ADC */
static volatile bool AdcLock = false;
static int32_t LastFastVoltage = 0;
/** @} */
/*******************************************************************************
 **************************  PRIVATE FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup MEASURE_LOCAL_FUNCTIONS Local Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Convert ADC sample values to 1/10 of degrees Celsius.
 *
 * @note
 *   See section 2.3.4 in the reference manual for details on this calculation.
 *
 * @param adc_result
 *   Raw value from ADC to be converted to Celsius.
 *
 * @return
 *   The temperature in 1/10 of degrees Celsius.
 ******************************************************************************/
static int32_t convertToCelsiusThenth(int32_t adc_result)
{
	// Factory calibration temperature from device information page
	// See table 5.4 in EFM32TG Reference Manual
	int32_t cal_temp_0 = ((DEVINFO->CAL & _DEVINFO_CAL_TEMP_MASK)
		>> _DEVINFO_CAL_TEMP_SHIFT);

	// Factory calibration value from device information page
	// See table 5.4 in EFM32TG Reference Manual
	int32_t adc0_temp_0_read_1v25 = ((DEVINFO->ADC0CAL2 &
		_DEVINFO_ADC0CAL2_TEMP1V25_MASK) >> _DEVINFO_ADC0CAL2_TEMP1V25_SHIFT);

	/* Temperature measurement is given by the following formula from section
	 * 24.3.4.2 in the EFM32TG Reference Manual:
	 *
	 * t_celsius  = cal_temp_0 - (adc0_temp_0_read_1v25 - adc_result) * vref
	 *              / (4096 * tgrad_adcth)
	 *
	 * Where:
	 *   - vref = 1.25 V
	 *   - tgrad_adcth is -1.92e-3 from table 3.15 in EFM32TG210F32 Datasheet
	 *
	 * Thus:
	 *   coeff = vref / (4096 * tgrad_adcth) = -1.25 / (4096 * -1.92e-3)
	 *         = -0.159
	 *         = 1 / -6.3
	 *
	 * And:
	 * t_celsius = cal_temp_0 - (adc0_temp_0_read_1v25 - adc_result) * coeff
	 *           = cal_temp_0 + (adc0_temp_0_read_1v25 - adc_result) / 6.3
	 *           = cal_temp_0 + ((adc0_temp_0_read_1v25 - adc_result) * 10) / 63
	 *
	 * Eventually:
	 * t_celsius_thenth = round(cal_temp_0 * 10 + ((adc0_temp_0_read_1v25 -
	 * adc_result) * 100) / 63)
	 *
	 * We just pre-multiply this computation by 2 and post-divide it by 2,
	 * adding 1 for rounding if intermediate LSB is 1.
	 */
	int32_t t_celsius_tenth = cal_temp_0 * 20 +
		((adc0_temp_0_read_1v25 - adc_result) * 200) / 63;
	return (t_celsius_tenth & 1) ? (t_celsius_tenth >> 1) + 1 :
		t_celsius_tenth >> 1;
}

/***************************************************************************//**
 * @brief
 *   Initialize the ADC for measuring either the power supply voltage or the
 *   temperature.
 *
 * @param[in] mode
 *   If true, measure the temperature, if false, measure the power supply
 *   voltage.
 *
 * @param[in] continuous
 *   If true, set the ADC sample period to 1 MSPS, if false, set it to 40 kSPS.
 ******************************************************************************/
static void TD_MEASURE_ADC_Init(bool mode, bool continuous)
{

	// Base the ADC configuration on the default setup
	ADC_InitSingle_TypeDef single_init = ADC_INITSINGLE_DEFAULT;
	//ADC_Init_TypeDef init = ADC_INIT_DEFAULT;
	ADC_Init_TypeDef init = {
		adcOvsRateSel2,                // 2x oversampling (if enabled).
	    adcLPFilterBypass,             // No input filter selected.
	    adcWarmupKeepADCWarm,
	    _ADC_CTRL_TIMEBASE_DEFAULT,    // Use HW default value.
	    _ADC_CTRL_PRESC_DEFAULT,       // Use HW default value.
	    false                          // Do not use tailgate.
	};

	// Initialize timebase
	init.timebase = ADC_TimebaseCalc(0);
	if (!continuous) {
		init.prescale = ADC_PrescaleCalc(40000, 0);
	} else {
		init.prescale = ADC_PrescaleCalc(1000000, 0);
	}
	CMU_ClockEnable(cmuClock_ADC0, true);
	ADC_Init(ADC0, &init);

	// Set input to temperature sensor. Reference must be 1.25V
	single_init.reference = adcRef1V25;
	single_init.input = mode ? adcSingleInpTemp : adcSingleInpVDDDiv3;
	ADC_InitSingle(ADC0, &single_init);
}

/** @} */

/*******************************************************************************
 **************************   PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup MEASURE_USER_FUNCTIONS User Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Accurately measure the Power Supply voltage or the temperature.
 *
 * @param[in] mode
 *   If true, measure the temperature, if false, measure the power supply
 *   voltage.
 *
 * @return
 *   The measured temperature is given in 1/10 degrees Celsius, the power supply
 *    voltage is given in mV.  If ADC is not available, returns 0.
 ******************************************************************************/
int32_t TD_MEASURE_VoltageTemperatureExtended(bool mode)
{
	int32_t setpoint;
	bool AlreadyLocked;
	volatile int timeout;

	// Timeout of 50ms
	timeout = 100000;

	// Enter Critical Section
	__set_PRIMASK(1);
	if (AdcLock) {
		AlreadyLocked = true;
	} else {
		AlreadyLocked = false;
		AdcLock = true;
	}
	__set_PRIMASK(0);

	// Exit Critical Section
	if (!AlreadyLocked) {
		TD_MEASURE_ADC_Init(mode, false);

		// Start one ADC sample
		ADC_Start(ADC0, adcStartSingle);

		// Active wait for ADC to complete
		while ((timeout--) && ((ADC0->STATUS & ADC_STATUS_SINGLEDV) == 0)) {
				;
		}
		setpoint = ADC_DataSingleGet(ADC0);
		CMU_ClockEnable(cmuClock_ADC0, false);

		// Enter Critical Section
		__set_PRIMASK(1);
		AdcLock = false;
		__set_PRIMASK(0);

		// Exit Critical Section
		if (mode) {
			setpoint = convertToCelsiusThenth(setpoint);
		} else {
			setpoint = (setpoint * 3 * 1250) / 4096;
		}
	} else {

		// ADC no is not available, return 0
		setpoint = 0;
	}
	return(setpoint);
}

/***************************************************************************//**
 * @brief
 *   Repeat a Power Supply voltage measure.
 *
 * @return
 *   The measured temperature is given in 1/10 degrees Celsius, the power supply
 *   voltage is given in mV. If ADC is not available, return the last voltage
 *   measure.
 ******************************************************************************/
int32_t TD_MEASURE_VoltageRepeat(void)
{
	int32_t setpoint;
	bool AlreadyLocked;
	volatile int timeout;
	// Timeout of 50ms
	timeout = 100000;

	// Enter Critical Section 
	__set_PRIMASK(1);
	if (AdcLock) {
		AlreadyLocked = true;
	} else {
		AlreadyLocked = false;
		AdcLock = true;
	}
	__set_PRIMASK(0);

	// Exit Critical Section
	if (!AlreadyLocked) {
		if (!(CMU->HFPERCLKEN0>>_CMU_HFPERCLKEN0_ADC0_SHIFT)) {
			TD_MEASURE_ADC_Init(false, true);
		}

		// Start one ADC sample
		ADC_Start(ADC0, adcStartSingle);

		// Active wait for ADC to complete
		while ((timeout--) && ((ADC0->STATUS & ADC_STATUS_SINGLEDV) == 0)) {
			;
		}
		setpoint = ADC_DataSingleGet(ADC0);

		// Enter Critical Section
		__set_PRIMASK(1);
		AdcLock = false;
		__set_PRIMASK(0);

		// Exit Critical Section
		setpoint = (setpoint * 3 * 1250)>>12;
		LastFastVoltage = setpoint;
	} else {

		// ADC no is not available, return the last measure
	}
	return LastFastVoltage;
}

/***************************************************************************//**
 * @brief
 *   Accurately measure voltage on a given pin.
 *
 * @param[in] input
 *   Channel on which measurement should be done. PD4 = channel 4,
 *   PD5 = Channel 5 ...
 *
 * @param[in] ref
 *   Voltage reference to use. Your voltage reference should be > to your
 *   voltage value
 *
 * @return
 *   The ADC output. Corresponding voltage can be found by computing:
 *
 *   voltage = reference_range * input / 4096
 *
 *   ex: 1.25V input, return = 1000
 *
 *   voltage = 1.25 * 1000 / 4096 = 305 mV
 *
 *   ex: 5V diff input, return = 2048
 *
 *   voltage = -5000 + (10000 * 2048) / 4096 = 0V
 *
 *  If ADC is not available, returns 0.
 ******************************************************************************/
uint32_t TD_MEASURE_SingleVoltage(ADC_SingleInput_TypeDef input,
	ADC_Ref_TypeDef ref)
{
	uint32_t setpoint;
	bool AlreadyLocked;
	volatile int timeout;

	// Base the ADC configuration on the default setup
	ADC_InitSingle_TypeDef single_init = ADC_INITSINGLE_DEFAULT;
	ADC_Init_TypeDef init = ADC_INIT_DEFAULT;

	// Timeout of 50 ms
	timeout = 100000;

	// Enter Critical Section
	__set_PRIMASK(1);
	if (AdcLock) {
		AlreadyLocked = true;
	} else {
		AlreadyLocked = false;
		AdcLock = true;
	}
	__set_PRIMASK(0);

	// Exit Critical Section
	if (!AlreadyLocked) {

		// Initialize timebase
		init.timebase = ADC_TimebaseCalc(0);
		init.prescale = ADC_PrescaleCalc(40000, 0);
		CMU_ClockEnable(cmuClock_ADC0, true);
		ADC_Init(ADC0, &init);

		// Set reference and input
		single_init.reference = ref;
		single_init.input = input;
		ADC_InitSingle(ADC0, &single_init);

		// Start one ADC sample
		ADC_Start(ADC0, adcStartSingle);

		// Active wait for ADC to complete
		while ((timeout--) && ((ADC0->STATUS & ADC_STATUS_SINGLEDV) == 0)) {
			;
		}
		setpoint = ADC_DataSingleGet(ADC0);
		CMU_ClockEnable(cmuClock_ADC0, false);

		/* Enter Critical Section */
		__set_PRIMASK(1);
		AdcLock = false;
		__set_PRIMASK(0);
		/* Exit Critical Section */

	} else {

		// ADC no is not free, return 0
		setpoint = 0;
	}
	return(setpoint);
}

/***************************************************************************//**
 * @brief
 *   Measure the Power Supply voltage or the temperature.
 *
 * @param[in] mode
 *   If true, measure the temperature, if false, measure the power supply
 *   voltage.
 *
 * @note
 *   This function is limited to positive temperature in degrees Celsius only.
 *   Please consider using TD_MEASURE_VoltageTemperatureExtended instead for
 *   proper handling of negative temperatures and values in 1/10 degrees
 *   Celsius.
 *   This function returns voltages in a non-standard format and in tens of mV,
 *   please consider using TD_MEASURE_VoltageTemperatureExtended instead for
 *   values directly in mV.
 *
 * @return
 *   The measured temperature is given in degrees Celsius, and the power supply
 *   voltage in 1/10s of mV plus 2 V if MSB is 0, or plus 3 V if MSB is 1.
 ******************************************************************************/
uint8_t TD_MEASURE_VoltageTemperature(bool mode)
{
	int32_t measure = TD_MEASURE_VoltageTemperatureExtended(mode);

	if (mode) {
		if (measure < 0) {
			return 0;
		}

		// Divide by 10 with proper rounding
		measure /= 5;
		return (measure & 1) ? (measure >> 1) + 1 : measure >> 1;
	} else {
		return TD_MEASURE_VoltageConvert(measure);
	}
}

/***************************************************************************//**
 * @brief
 *   Convert a Power Supply voltage in mV to a packed 8 bit format.
 *
 * @param[in] measure
 *   The input measured power supply voltage in mV.
 *
 *
 * @return
 *   The measured power supply voltage in a compact 8 bit representation with
 *   0.1 V resolution and offset by 3V if MSB is 1, only by 2V if MSB is 0.
 ******************************************************************************/
uint8_t TD_MEASURE_VoltageConvert(uint32_t measure)
{
	uint8_t msb;

	if (measure >= 3000) {
		msb = 0x80;
		measure -= 3000;
	} else {
		msb = 0x00;
		measure -= 2000;
	}

	// Divide by 10 with proper rounding
	measure /= 5;
	return (measure & 1) ? (measure >> 1) + 1 + msb : (measure >> 1) + msb;
}

/** @} */

/** @} (end addtogroup MEASURE) */
