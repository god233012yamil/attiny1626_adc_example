/**
 * \file
 *
 * \brief ADC Basic driver implementation.
 *
 (c) 2020 Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms,you may use this software and
    any derivatives exclusively with Microchip products.It is your responsibility
    to comply with third party license terms applicable to your use of third party
    software (including open source software) that may accompany Microchip software.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 */

/**
 * \defgroup doc_driver_adc_basic ADC Basic Driver
 * \ingroup doc_driver_adc
 *
 * \section doc_driver_adc_rev Revision History
 * - v0.0.0.1 Initial Commit
 *
 *@{
 */
#include <adc_basic.h>

/**
 * \brief Initialize ADC interface
 * If module is configured to disabled state, the clock to the ADC is disabled
 * if this is supported by the device's clock system.
 *
 * \return Initialization status.
 * \retval 0 the ADC init was successful
 * \retval 1 the ADC init was not successful
 */
int8_t ADC_0_init() {

	ADC0.CTRLB = ADC_PRESC_DIV2_gc; /* System clock divided by 2 */
	
	ADC0.CTRLF = ADC_SAMPNUM_NONE_gc /* No accumulation */
			 | 0 << ADC_FREERUN_bp; /* ADC Free-run mode: disabled */
	
	ADC0.CTRLC = ADC_REFSEL_2500MV_gc     /* 2.5V */
	         | ADC_TIMEBASE_VALUE; /* timebase value */
	
	ADC0.CTRLD = ADC_WINCM_NONE_gc /* No Window Comparison */
			 | ADC_WINSRC_RESULT_gc; /* Result register */
	
	ADC0.CTRLE = 0x1b; /* Sample Duration: 0x1b */
	
	ADC0.DBGCTRL = 0 << ADC_DBGRUN_bp; /* Debug run: disabled */
	
	ADC0.COMMAND = 0 << ADC_DIFF_bp /* Differential ADC Conversion: disabled */
			 | ADC_MODE_SINGLE_12BIT_gc   /* ADC_MODE_SERIES_gc Single Conversion 12-bit */
			 | ADC_START_STOP_gc; /* Stop an ongoing conversion */
	
	ADC0.INTCTRL = 0 << ADC_RESRDY_bp /* Result Ready Interrupt Enable: disabled */
			 | 0 << ADC_WCMP_bp; /* Window Comparator Interrupt Enable: disabled */
	
	ADC0.MUXPOS = ADC_VIA_ADC_gc /* Via ADC */
			 | ADC_MUXPOS_AIN1_gc; /* ADC input pin 1 */
	
	ADC0.MUXNEG = ADC_VIA_ADC_gc /* Via ADC */
			 | ADC_MUXNEG_AIN1_gc; /* ADC input pin 1 */
	
	ADC0.WINHT = 0x0; /* Window Comparator High Threshold: 0x0 */
	
	ADC0.WINLT = 0x0; /* Window Comparator Low Threshold: 0x0 */
	
	ADC0.CTRLA = 1 << ADC_ENABLE_bp      /* ADC Enable: enabled */
	         | 0 << ADC_RUNSTDBY_bp; /* Run standby mode: disabled */
	return 0;
}

/**
 * \brief Enable ADC_0
 * 1. If supported by the clock system, enables the clock to the ADC
 * 2. Enables the ADC module by setting the enable-bit in the ADC control register
 *
 * \return Nothing
 */
void ADC_0_enable()
{
	ADC0.CTRLA |= ADC_ENABLE_bm;
}
/**
 * \brief Disable ADC_0
 * 1. Disables the ADC module by clearing the enable-bit in the ADC control register
 * 2. If supported by the clock system, disables the clock to the ADC
 *
 * \return Nothing
 */
void ADC_0_disable()
{
	ADC0.CTRLA &= ~ADC_ENABLE_bm;
}

/**
 * \brief Start a conversion on ADC_0
 *
 * \param[in] channel The ADC channel to start conversion on
 *
 * \return Nothing
 */
void ADC_0_start_conversion(adc_0_channel_t channel)
{
	ADC0.MUXPOS = channel;
	ADC0.COMMAND |= ADC_START_IMMEDIATE_gc;
}

/**
 * \brief Check if the ADC conversion is done
 *
 * \return The status of ADC converison done check
 * \retval true The ADC conversion is done
 * \retval false The ADC converison is not done
 */
bool ADC_0_is_conversion_done()
{
	return (ADC0.INTFLAGS & ADC_RESRDY_bm);
}

/**
 * \brief Read a conversion result from ADC_0
 *
 * \return Conversion result read from the ADC_0 ADC module
 */
adc_result_t ADC_0_get_conversion_result(void)
{
	return (ADC0.RESULT);
}

/**
 * \brief Start a conversion, wait until ready, and return the conversion result
 *
 * \return Conversion result read from the ADC_0 ADC module
 */
adc_result_t ADC_0_get_conversion(adc_0_channel_t channel)
{
	adc_result_t res;

	ADC_0_start_conversion(channel);
	while (!ADC_0_is_conversion_done())
		;
	res = ADC_0_get_conversion_result();
	ADC0.INTFLAGS |= ADC_RESRDY_bm;
	return res;
}

/**
 * \brief Return the number of bits in the ADC conversion result
 *
 * \return The number of bits in the ADC conversion result
 */
uint8_t ADC_0_get_resolution()
{
	return (ADC0.COMMAND & ADC_MODE_SINGLE_8BIT_gc) ? 8 : 12;
}
