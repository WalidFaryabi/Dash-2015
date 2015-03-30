

#ifndef DASH_IO_H_
#define DASH_IO_H_

#include "../sam4e-base/RevolveDrivers/pio.h"
#include "../sam4e-base/RevolveDrivers/pmc.h"

#define ROT_A_PIO PIOE
#define ROT_A_PIN 3
#define ROT_B_PIO PIOC
#define ROT_B_PIN 0

#define ROT_PUSH3_PIO PIOC
#define ROT_PUSH3_PIN 27 

#define NAVIGATION_L_PIO PIOD
#define NAVIGATION_L_PIN 23

#define NAVIGATION_R_PIO PIOD
#define NAVIGATION_R_PIN 22

#define NAVIGATION_D_PIO PIOA
#define NAVIGATION_D_PIN 25

#define NAVIGATION_U_PIO PIOC
#define NAVIGATION_U_PIN 5

#define START_PIO PIOD
#define START_PIN 25

#define LC_PIO PIOC
#define LC_PIN 6

#define SYS_ACK_PIO PIOD // Uses the slot meant for KERS switch
#define SYS_ACK_PIN 24

#define AMS_LED_PIO PIOA	
#define AMS_LED_PIN 22

#define IMD_LED_PIO PIOC
#define IMD_LED_PIN 1

#define TEMP_LED_PIO PIOC
#define TEMP_LED_PIN 2

#define ECU_LED_PIO PIOC
#define ECU_LED_PIN 3

#define VOLT_LED_PIO PIOA
#define VOLT_LED_PIN 16

#define TS_LED_PIO PIOA
#define TS_LED_PIN 23

#define LC_LED_PIO PIOD
#define LC_LED_PIN 27

#define DEVICE_LED_PIO PIOC
#define DEVICE_LED_PIN 7

#define BUZZER_PIO PIOD
#define BUZZER_PIN 0

#define FT800_POWERDOWN_PIO PIOD
#define FT800_POWERDOWN_PIN 9

#define DETECT_USB_PIO PIOB
#define DETECT_USB_PIN 13


void dash_io_init();

void pio_inputDebounce(Pio *pio, uint8_t pin, uint16_t div, enum FilterType filter_type);
void pio_inputPulldown(Pio *pio, uint8_t pin,enum PullType pull_type);
void pio_outputPulldownLow(Pio *pio, uint8_t pin);

#endif /* DASH_IO_H_ */