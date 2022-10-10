/*
 *  \file:		main.c
 *
 *  \brief:		This software is designed for testing the ADC peripheral on
 *				the MCU ATtiny1626
 *
 *  \Created:	10/09/2022 
 *
 *  \Author:	Yamil Garcia
 */ 

#include <atmel_start.h>

/***************************** Start of Note Section. ***************************************/


/***************************** End of Note Section. *****************************************/
/***************************** Start of Define section. *************************************/
#define ADC_MEAS_CHANNEL_1		ADC_MUXPOS_AIN1_gc	// PA1
#define ADC_MEAS_CHANNEL_2		ADC_MUXPOS_AIN2_gc	// PA2
#define ADC_MEAS_CHANNEL_3		ADC_MUXPOS_AIN3_gc	// PA3

/***************************** End of Define section. ***************************************/
/***************************** Start of Macro Definitions Section. **************************/


/***************************** End of Macro Definitions Section. ****************************/
/***************************** Start of Global Variable Declaration Section. ****************/
// ADC0
uint8_t adc0_state = 0;
uint8_t adc0_channel = 0;
uint16_t adc0_voltage_1 = 0;
uint16_t adc0_voltage_2 = 0;
uint16_t adc0_voltage_3 = 0;
volatile uint16_t adc0_ms_counter = 0;

/***************************** End of Global Variable Declaration Section. ******************/
/***************************** Start of Prototype Function Declaration Section. *************/
void GPIOs_Init(void);
void TCA0_Init(uint16_t time_in_ms);
void WDT_Init(WDT_PERIOD_t wdt_period);
void WDT_Reset(void);
void ADC0_Measure_Voltage(void);

/***************************** End of Prototype Function Declaration Section. ***************/
/***************************** Start of Prototype Function Definitions Section. *************/
/*
* \brief	This function initializes GPIOs not used by peripherals.
*			GPIOs used by peripheral are initialized in peripheral functions
*			located in file "driver_init.c".
*
* \return	Nothing
*/
void GPIOs_Init(void) {
	
}

/**
 * \brief		This function setup the 16-bit Timer/Counter Type A (TCA) in normal operation
 *				to perform an overflow interrupt every "time_in_ms" mS.	
 *				This timer will be used as our House Keeper Timer.
 * 
 * \return		Nothing
 */
void TCA0_Init(uint16_t time_in_ms) {
	
	// PER = (Desire time in mS * Main Clock in Hz) / (Main Clock Prescaler * Timer/Counter Prescaler * 1000)
	// Timer/Counter Pre-scaler = 1
	// * 1000 is to count in mS.	
	TCA0.SINGLE.PER = (register16_t)((float)((time_in_ms * F_CPU) / (1 * 1000)));
	
	// Bit 0 – OVF Timer Overflow/Underflow Interrupt Enable
	// Enable overflow interrupt
	TCA0.SINGLE.INTCTRL = 0 << TCA_SINGLE_CMP0_bp /* Compare 0 Interrupt: disabled */
		| 0 << TCA_SINGLE_CMP1_bp /* Compare 1 Interrupt: disabled */
		| 0 << TCA_SINGLE_CMP2_bp /* Compare 2 Interrupt: disabled */
		| 1 << TCA_SINGLE_OVF_bp; /* Overflow Interrupt: enabled */
		
	// Bits 2:0 – WGMODE[2:0] Waveform Generation Mode.
	// Bits 4, 5, 6 – CMPEN Compare n Enable.
	TCA0.SINGLE.CTRLB = 0 << TCA_SINGLE_ALUPD_bp /* Auto Lock Update: disabled */
		| 0 << TCA_SINGLE_CMP0EN_bp /* Compare 0 Enable: disabled */
		| 0 << TCA_SINGLE_CMP1EN_bp /* Compare 1 Enable: disabled */
		| 0 << TCA_SINGLE_CMP2EN_bp /* Compare 2 Enable: disabled */
		| TCA_SINGLE_WGMODE_NORMAL_gc;/* Normal Mode */	
	
	// Bit 0 – ENABLE Enable
	// Enable the peripheral by writing a '1' to the ENABLE bit in the Control A register (TCAn.CTRLA).
	// Bits 3:1 – CLKSEL[2:0] Clock Select
	// Select the clock frequency to fCLK_PER/1 by clearing bits 3:1
	TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV1_gc /* System Clock */
		| 1 << TCA_SINGLE_ENABLE_bp; /* Module Enable: enabled */
}

/*
* \brief	This function enables the watchdog In Normal mode operation.
*			In this mode a single time-out period is set for the WDT. If the WDT is not reset from
*			software using the WDR any time before the time-out occurs, the WDT will issue a system Reset.
*			A new WDT time-out period will be started each time the WDT is reset by WDR.
*
* \return	Nothing.
*/
void WDT_Init(WDT_PERIOD_t wdt_period) {
	// Disable global interrupts.
	cli();
	// Reset Watchdog Timer
	asm("wdr");
	// Bits 3:0 – PERIOD[3:0] Period
	// Writing a non-zero value to this bit enables the WDT,
	// and selects the time-out period in Normal mode accordingly.
	//ccp_write_io((void*)&(WDT.CTRLA), WDT_PERIOD_OFF_gc | WDT_WINDOW_OFF_gc);
	ccp_write_io((void*)&(WDT.CTRLA), wdt_period | WDT_WINDOW_OFF_gc);
	// Enable global interrupts.
	sei();
}

/*
* \brief	This function reset the watchdog, and a new WDT time-out period
*			will be started each time the WDT is reset by WDR.
*
* \return	Nothing.
*/
void WDT_Reset(void) {
	// Disable global interrupts.
	cli();
	// Reset Watchdog Timer
	asm("wdr");
	// Enable global interrupts.
	sei();
}

/*
* \brief	This function is designed to measure voltage using the ADC
*           peripheral. 
*
* \return	Nothing.
*/
void ADC0_Measure_Voltage(void) {
	// Variable declaration and initialization in this scope (local variables).
	adc_result_t adc_result = 0;
	
	// Switch to the appropriate state.
	switch(adc0_state) {
		// State to select the ADC channel to measure.
		case 0:
			if(adc0_channel == 0) {
				ADC0.MUXPOS = ADC_MEAS_CHANNEL_1;
			} else if(adc0_channel == 1) {
				ADC0.MUXPOS = ADC_MEAS_CHANNEL_2;
			} else if(adc0_channel == 2) {
				ADC0.MUXPOS = ADC_MEAS_CHANNEL_3;
			} else {
				// nothing to do here.
				// The code execution should never enter here.
				asm("nop");
			}
			adc0_state = 1;
			adc0_ms_counter = 0;
			break;
		
		// Wait some time for the ADC Muxer to change channel
		// before to start a new conversion.
		case 1:
			if(adc0_ms_counter >= 1) {
				// Start a new conversion.
				ADC0.COMMAND |= ADC_START_IMMEDIATE_gc;
				// Change to next state.
				adc0_state = 2;
			}
			break;
		
		// Check if conversion is done.
		case 2:
			if(ADC_0_is_conversion_done()) {
				// Change state.
				adc0_state = 3;
			}
			break;
		
		// State to get the conversion result.
		case 3:
			// Get conversion from specified ADC channel.
			adc_result = ADC_0_get_conversion_result();
		
			// Formula to calculate the voltage at the ADC pin is:
			// Vadc_pin = (ADC Result * Vref) / ADC_Resolution
			// Vref = 2.5V, ADC_Resolution = 12 bits (4095)
			// Vadc_pin = (ADC Result * 2.5) / 4095
			
			// The voltage applied to the ADC pins is reduced by
			// a voltage divider. The voltage is reduced by a factor of 11.
			// R1 + R2 / R2 = (100K + 10K) / 10K = 11 
			// R3 + R4 / R4 = (100K + 10K) / 10K = 11 
			// R5 + R6 / R6 = (100K + 10K) / 10K = 11 			
		
			if(adc0_channel == 0) {
				// Calculate voltage Vin_1 (Refer to schematic).
				adc0_voltage_1 = ((adc_result * 2.5) / 4095) * 11;
			} else if(adc0_channel == 1) {
				// Calculate voltage Vin_2 (Refer to schematic).
				adc0_voltage_2 = ((adc_result * 2.5) / 4095) * 11;
			} else if(adc0_channel == 2) {
				// Calculate voltage Vin_2 (Refer to schematic).
				adc0_voltage_2 = ((adc_result * 2.5) / 4095) * 11;
			} else {
				// nothing to do here.
				// The code execution should never enter here.
				asm("nop");
			}
		
			// Increment to use next ADC channel next time.
			if(adc0_channel < 2) {
				adc0_channel++;
			} else {
				adc0_channel = 0;
			}
			// Change state.
			adc0_state = 4;
			// Clear counter.
			adc0_ms_counter = 0;
			// Clear the Result Ready Interrupt Flag.
			ADC0.INTFLAGS |= ADC_RESRDY_bm;
			break;
		
		// Wait a predefined time to start next measurement.
		case 4:
			if(adc0_ms_counter > 1) {// 1 milli seconds				
				// Change state.
				adc0_state = 0;
			}
			break;
		
		// In case of an error.
		default:
			// Change state.
			adc0_state = 0;
			// Clear the Result Ready Interrupt Flag.
			ADC0.INTFLAGS |= ADC_RESRDY_bm;
			break;
	}
}

/***************************** End of Prototype Function Definitions Section. ***************/
/***************************** Start of Interrupts Section **********************************/
/*
* \brief	Overflow interrupt service routine for TCA.
*			This routine is called every time an overflow
*			takes place.
*			This routine is used as a house keeping timer
*			to count milliseconds.
*			The counters is this routine are consumed by
*			other functions.
*
* \return	Nothing
*/
ISR(TCA0_OVF_vect) {
	//
	adc0_ms_counter++;
	
	// Clear overflow interrupt flag. Bit 0 – OVF Overflow/Underflow Interrupt Flag.
	TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;
}

/***************************** Start of Interrupts Section **********************************/


// Entry point to this program.
int main(void) {
	// Initializes MCU, drivers and middle-ware.
	atmel_start_init();
	// Initialize GPIOs not used by peripherals.
	GPIOs_Init();
	// Initializes the TCA0 to overflow every 1 milliseconds.
	// This timer is used as a house keeper timer.
	TCA0_Init(1);
	// Clear the Overflow Interrupt for TCA0.
	TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;
	// Enable global interrupts.
	sei();
	
	// Initializes the Watch Dog to 1K cycles (1.0s).
	WDT_Init(WDT_PERIOD_1KCLK_gc);

	// Main loop.
	while (1) {
		// Measure voltage.
		ADC0_Measure_Voltage();	
		
		// Reset Watch Dog.
		WDT_Reset();
	}
