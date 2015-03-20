#include "Task_ButtonInput.h"
#include "Task_Menu.h"

#include "sam4e-base/RevolveDrivers/pio.h"

#include "DriversNotInBase/IO_DashInit.h"

volatile bool rotary_cw = false;
volatile bool rotary_ccw = false;

void Task_ButtonInput() {
	TickType_t xLastwakeTime;
	//xButtonStruct = xSemaphoreCreateMutex();
	//A switch will be true if there has been a transition from one state to another
	//It will be set to false after its handled in the menu task
	uint8_t prev_joystick_left = 1; //pio_readPin(&JOYSTICK_L,JOYSTICK_L_PIN);
	uint8_t prev_joystick_right = 1;
	uint8_t prev_joystick_down = 1;
	uint8_t prev_joystick_up = 1;
	EJoystickDirection prev_navigation = NAV_DEFAULT;
	uint8_t prev_acknowledge = 1; // Normally open push button
	uint8_t prev_launch_control = 1; // Normally open push button
	
	uint8_t prev_kers_value = 1; // pio_readPin(&KERS_PIO,KERS_PIN);
	uint8_t prev_drive_value = 1; //pio_readPin(&DRIVE_PIO,DRIVE_PIN); // Initialize them to current position
	
	// Implement a function that checks if all buttons are in the correct position at start up ?
	
	while(1) {
		xSemaphoreTake(xButtonStruct,portMAX_DELAY); // Wait indefinetely for access to Button struct
		
		// The order which the buttons are checked determines the priority order of the buttonpress that will be set to true
		// if multiple button presses are registered at the same time.
		//Check for joystick action
		if (btn.unhandledButtonAction == false) {
			if ( (pio_readPin(JOYSTICK_L,JOYSTICK_L_PIN) == 0) && (prev_navigation != LEFT) ) {
				btn.unhandledButtonAction = true;
				btn.btn_type = JOYSTICK;
				btn.navigation = LEFT;
				
			}
			else if ((pio_readPin(JOYSTICK_R,JOYSTICK_R_PIN) == 0) && (prev_navigation != RIGHT) ) {
				btn.unhandledButtonAction = true;
				btn.btn_type = JOYSTICK;
				btn.navigation = RIGHT;
			}
			else if ((pio_readPin(JOYSTICK_D,JOYSTICK_D_PIN) == 0) && (prev_navigation != DOWN) ) {
				btn.unhandledButtonAction = true;
				btn.btn_type = JOYSTICK;
				btn.navigation = DOWN;
			}
			else if ((pio_readPin(JOYSTICK_U,JOYSTICK_U_PIN) == 0) && (prev_navigation != UP) ) {
				btn.unhandledButtonAction = true;
				btn.btn_type = JOYSTICK;
				btn.navigation = UP;
			}
		}
		//Check the push button on the rotary encoder (acknowledge button)
		if (btn.unhandledButtonAction == false) {
			if ( (pio_readPin(ROT_PUSH3,ROT_PUSH3_PIN) == 0) && (prev_acknowledge == 1) ) {
				btn.unhandledButtonAction = true;
				btn.acknowledge = true;
				btn.btn_type = ACKNOWLEDGE;
			}
		}
		
		//Check rotary encoder for new input
		if (btn.unhandledButtonAction == false) {
			if (rotary_cw == true) {
				btn.unhandledButtonAction = true;
				btn.rotary_cw = true;	
				btn.btn_type = ROTARY;
				rotary_cw = false; // reset
			}
			else if (rotary_ccw == true) {
				btn.unhandledButtonAction = true;
				btn.rotary_ccw = true;
				btn.btn_type = ROTARY;
				rotary_ccw = false; // reset
			}
		}
		//Check if launch control has been pressed
		if (btn.unhandledButtonAction == false) {
			if ((pio_readPin(LC_PIO,LC_PIN) == 0) && (prev_launch_control == 1)) {
				btn.unhandledButtonAction = true;
				btn.launch_control = true;
				btn.btn_type = LAUNCH_CONTROL;
			}
		}
		
		//Check if the the KERS switch has changed state
		if (btn.unhandledButtonAction == false) {
			if ( (pio_readPin(KERS_PIO,KERS_PIN) == 0) && (prev_kers_value == 1) ) {
				btn.unhandledButtonAction = true;
				btn.kers_switch_on = true;
				btn.btn_type = KERS;
			}
			else if ((pio_readPin(KERS_PIO,KERS_PIN) == 1) && (prev_kers_value == 0) ) {
				btn.unhandledButtonAction = true;
				btn.kers_switch_off = true;
				btn.btn_type = KERS;
			}
		}
		
		//Check if the drive switch has changed state
		if (btn.unhandledButtonAction == false) {
			if ( (pio_readPin(DRIVE_PIO, DRIVE_PIN) == 0) && (prev_drive_value == 1) ) {
				btn.unhandledButtonAction = true;
				btn.drive_switch_enable = true;
				btn.btn_type = DRIVE;
			}
			else if ((pio_readPin(DRIVE_PIO, DRIVE_PIN) == 1) && (prev_drive_value == 0) ) {
				btn.unhandledButtonAction = true;
				btn.drive_switch_disable = true;
				btn.btn_type = DRIVE;
			}
		}
		prev_joystick_left = pio_readPin(JOYSTICK_L,JOYSTICK_L_PIN); 
		prev_joystick_right = pio_readPin(JOYSTICK_R,JOYSTICK_R_PIN);
		prev_joystick_down = pio_readPin(JOYSTICK_D,JOYSTICK_D_PIN);
		prev_joystick_up = pio_readPin(JOYSTICK_U,JOYSTICK_U_PIN);
		prev_acknowledge = pio_readPin(ROT_PUSH3,ROT_PUSH3_PIN); 
		prev_launch_control = pio_readPin(LC_PIO,LC_PIN);
		prev_kers_value = pio_readPin(KERS_PIO,KERS_PIN);
		prev_drive_value = pio_readPin(DRIVE_PIO, DRIVE_PIN);
		
		xSemaphoreGive(xButtonStruct);
		//Using vtaskdelay until it is confirmed that delayuntil is not funky
		//vTaskDelayUntil(&xLastwakeTime,30/portTICK_RATE_MS);
		vTaskDelay(50/portTICK_RATE_MS);
	}
}


