
#include "IO_DashInit.h"

#include "../Task_ButtonInput.h" // To access the rotary interrupt variables


// static void rotaryEncoderInterruptFunction() {
// 	PIOA->PIO_ISR;
// 	if (PIOA->PIO_PDSR & (1<<20)) { // Check if PA20 is high
// 		rotary_cw = true;
// 	}
// 	else {
// 		rotary_ccw = true;
// 	}
// }
static void rotaryEncoderInterruptFunction() {
	PIOA->PIO_ISR;
	if (!pio_readPin(ROT_A_PIO, ROT_A_PIN)) { // Check if ROT A is low
		rotary_cw = true;
	}
	else {
		rotary_ccw = true;
	}
}
tgg

void dash_io_init() {
	// Husk WPEN bit for � kunne skrive til OER etc .. side 737 ( Disabled by default)
	
	pio_init(); // Enables peripheral clock and enables IRQ for all pio registers
	
	pio_enableOutput(FT800_POWERDOWN_PIO,FT800_POWERDOWN_PIN);
	pio_setOutput(FT800_POWERDOWN_PIO,FT800_POWERDOWN_PIN, PIN_HIGH);
	
	pio_inputDebounce(NAVIGATION_L_PIO,NAVIGATION_L_PIN,653,DEBOUNCE); // Needs periph clk for filter
	pio_inputDebounce(NAVIGATION_R_PIO,NAVIGATION_R_PIN,653,DEBOUNCE);
	pio_inputDebounce(NAVIGATION_D_PIO,NAVIGATION_D_PIN,653,DEBOUNCE);
	pio_inputDebounce(NAVIGATION_U_PIO,NAVIGATION_U_PIN,653,DEBOUNCE);
	pio_inputDebounce(START_PIO,START_PIN,653,DEBOUNCE);
	
	pio_enableOutput(BUZZER_PIO,BUZZER_PIN);
	pio_setOutput(BUZZER_PIO,BUZZER_PIN,PIN_LOW);
	pio_setPull(BUZZER_PIO,BUZZER_PIN,PULLDOWN);
	
	
	pio_inputPulldown(DETECT_USB_PIO,DETECT_USB_PIN,PULLDOWN);
	pio_disableOutput(DETECT_USB_PIO,DETECT_USB_PIN);
	pio_inputDebounce(DETECT_USB_PIO,DETECT_USB_PIN,3000,DEBOUNCE);
	// Double check if pulldown on usb is needed
	//pio_input_pulldown(DETECT_USB_PIO,DETECT_USB_PIN);
	
	pio_disableOutput(ROT_PUSH3_PIO,ROT_PUSH3_PIN);
	pio_disableOutput(ROT_A_PIO,ROT_A_PIN);
	pio_disableOutput(ROT_B_PIO,ROT_B_PIN);

	//Interrupts on rotary 
	//pio_enableInterrupt(ROT_B_PIO,ROT_B_PIN, RISING_EDGE,rotaryEncoderInterruptFunction);
	pio_enableInterrupt(ROT_B_PIO,ROT_B_PIN, FALLING_EDGE,rotaryEncoderInterruptFunction);
	PIOA->PIO_ISR;
}


void pio_inputPulldown(Pio *pio, uint8_t pin,enum PullType pull_type) {
	pio_disableOutput(pio,pin);
	pio_setPull(pio,pin,pull_type);
}

void pio_inputDebounce(Pio *pio, uint8_t pin, uint16_t div, enum FilterType filter_type) {
	pio_disableOutput(pio,pin);
	pio_setFilter(pio,pin,filter_type);
	pio->PIO_WPMR = 0x50494F00; // Disable
	pio->PIO_SCDR |= div;
	pio->PIO_WPMR = 0x50494F01; // Enable WriteProtection
	/*
	Debounce_time = t_slow *(DIV +1 )
	Slow clock = 32 768 kHz Debounce clock = 2xslow by default
	DIV = d-t / t ...  div = 653 -> 20 ms debounce
	*/
}