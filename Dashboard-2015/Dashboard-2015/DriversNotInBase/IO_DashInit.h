

#ifndef DASH_IO_H_
#define DASH_IO_H_

#include "../sam4e-base/RevolveDrivers/pio.h"
#include "../sam4e-base/RevolveDrivers/pmc.h"

#define ROT_A PIOA
#define ROT_A_PIN 20
#define ROT_B PIOA
#define ROT_B_PIN 19

#define ROT_PUSH3 PIOA
#define ROT_PUSH3_PIN 18

#define ROT_PUSH2 PIOA
#define ROT_PUSH2_PIN 17

#define JOYSTICK_L PIOE
#define JOYSTICK_R PIOC
#define JOYSTICK_D PIOC
#define JOYSTICK_U PIOC
#define JOYSTICK_L_PIN 3
#define JOYSTICK_R_PIN 0
#define JOYSTICK_D_PIN 27
#define JOYSTICK_U_PIN 26

#define KERS_PIO PIOC
#define KERS_PIN 13

#define DRIVE_PIO PIOC
#define DRIVE_PIN 31

#define LC_PIO PIOC
#define LC_PIN 29

#define FT800_POWERDOWN_PIO PIOD // PD9
#define FT800_POWERDOWN_PIN 9

#define AMS_LED_PIO PIOD	
#define AMS_LED_PIN 25

#define IMD_LED_PIO PIOD
#define IMD_LED_PIN 26

#define BUZZER_PIO PIOD
#define BUZZER_PIN 0

#define DETECT_USB_PIO PIOB
#define DETECT_USB_PIN 13


void dash_io_init();

void pio_inputDebounce(Pio *pio, uint8_t pin, uint16_t div, enum FilterType filter_type);
void pio_inputPulldown(Pio *pio, uint8_t pin,enum PullType pull_type);




//		PD25 = AMS 
//		PD26 = IMD
// pin22 = PA20 = OUTPUT A
// pin23 = PA19 = OUTPUT B
// pin24 = PA18 = PUSH PIN 3

// pin10 = PE3  = JOYSTICK LEFT
// pin11 = PC0  = JOYSTICK RIGHT
// pin12 = PC27 = JOYSTICK DOWN
// pin13 = PC26 = JOYSTICK UP
// pin14 = PC31 = START INVERTER = Drive enable
// pin15 = PC30 = TRACTIVE SYSTEM ((( Skal ikke inn til micro, men slutte shutdownkretsen )))
// pin16 = PC29 = LAUNCH CONTROL
// pin19 = PC13 = KERS

//Rotary header
//pin 1 = top left = 5V
//pin 2 = top right = output a
//pin 3 = 0V
//pin 4 = Output B
//pin 5 = push pin 2
//pin 6 = push pin 3

//Rotary encoder
//pin 1 = Ground
//pin 2 = push 2
//pin 3 = push 3
//pin 4 = output B
//pin 5 = Output A
//pin 6 = 5V


#endif /* DASH_IO_H_ */