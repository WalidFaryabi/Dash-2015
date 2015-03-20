#include "Task_ButtonInput.h"
#include "Task_Menu.h"

#include "sam4e-base/RevolveDrivers/pio.h"

#include "DriversNotInBase/IO_DashInit.h"

volatile bool rotary_cw = false;
volatile bool rotary_ccw = false;

void Task_ButtonInput() {
	TickType_t xLastwakeTime;
	
	//A switch will be true if there has been a transition from one state to another
	//It will be set to false after its handled in the menu task
	uint8_t prev_navigation_left = 1; 
	uint8_t prev_navigation_right = 1;
	uint8_t prev_navigation_down = 1;
	uint8_t prev_navigation_up = 1;
	
	uint8_t prev_launch_control = 1; 
	uint8_t prev_dash_ack = 1; 
	uint8_t prev_sys_ack = 1;
	uint8_t prev_start = 1;
	
	// Implement a function that checks if all buttons are in the correct position at start up ?
	
	while(1) {
		xSemaphoreTake(xButtonStruct,portMAX_DELAY); // Wait indefinetely for access to Button struct
		
		// The order which the buttons are checked determines the priority order of the buttonpress that will be set to true
		// if multiple button presses are registered at the same time.
		//Check for navigation action
		if (btn.unhandledButtonAction == false) {
			if ( (pio_readPin(NAVIGATION_L_PIO,NAVIGATION_L_PIN) == 0) && (prev_navigation_left == 1) ) {
				btn.unhandledButtonAction = true;
				btn.btn_type = NAVIGATION;
				btn.navigation = LEFT;
			}
			else if ((pio_readPin(NAVIGATION_R_PIO,NAVIGATION_R_PIN) == 0) && (prev_navigation_right == 1) ) {
				btn.unhandledButtonAction = true;
				btn.btn_type = NAVIGATION;
				btn.navigation = RIGHT;
			}
			else if ((pio_readPin(NAVIGATION_D_PIO,NAVIGATION_D_PIN) == 0) && (prev_navigation_down == 1) ) {
				btn.unhandledButtonAction = true;
				btn.btn_type = NAVIGATION;
				btn.navigation = DOWN;
			}
			else if ((pio_readPin(NAVIGATION_U_PIO,NAVIGATION_U_PIN) == 0) && (prev_navigation_up == 1) ) {
				btn.unhandledButtonAction = true;
				btn.btn_type = NAVIGATION;
				btn.navigation = UP;
			}
		}
		//Check the push button on the rotary encoder (acknowledge button)
		if (btn.unhandledButtonAction == false) {
			if ( (pio_readPin(ROT_PUSH3_PIO,ROT_PUSH3_PIN) == 0) && (prev_dash_ack == 1) ) {
				btn.unhandledButtonAction = true;
				btn.dash_acknowledge = true;
				btn.btn_type = DASH_ACK;
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
		
		//Check if the the system acknowledge button has been pressed
		if (btn.unhandledButtonAction == false) {
			if ( (pio_readPin(SYS_ACK_PIO,SYS_ACK_PIN) == 0) && (prev_sys_ack == 1) ) {
				btn.unhandledButtonAction = true;
				btn.system_acknowledge = true;
				btn.btn_type = SYS_ACK;
			}
		}
		
		//Check if the start button has been pressed
		if (btn.unhandledButtonAction == false) {
			if ( (pio_readPin(START_PIO, START_PIN) == 0) && (prev_start== 1) ) {
				btn.unhandledButtonAction = true;
				btn.start_button = true;
				btn.btn_type = START;
			}

		}
		prev_navigation_left = pio_readPin(NAVIGATION_L_PIO,NAVIGATION_L_PIN); 
		prev_navigation_right = pio_readPin(NAVIGATION_R_PIO,NAVIGATION_R_PIN);
		prev_navigation_down = pio_readPin(NAVIGATION_D_PIO,NAVIGATION_D_PIN);
		prev_navigation_up = pio_readPin(NAVIGATION_U_PIO,NAVIGATION_U_PIN);
		
		prev_dash_ack = pio_readPin(ROT_PUSH3_PIO,ROT_PUSH3_PIN); 
		prev_sys_ack = pio_readPin(SYS_ACK_PIO,SYS_ACK_PIN);
		prev_launch_control = pio_readPin(LC_PIO,LC_PIN);
		prev_start = pio_readPin(START_PIO, START_PIN);
		
		xSemaphoreGive(xButtonStruct);
		//Using vtaskdelay until it is confirmed that delayuntil is not funky
		//vTaskDelayUntil(&xLastwakeTime,30/portTICK_RATE_MS);
		vTaskDelay(50/portTICK_RATE_MS);
	}
}


