#include "Task_ButtonInput.h"
#include "Task_Menu.h"

#include "sam4e-base/RevolveDrivers/pio.h"
#include "DriversNotInBase/IO_DashInit.h"

volatile bool rotary_cw = false;
volatile bool rotary_ccw = false;

static bool start_button_pushed_in = false;
static bool lc_button_pushed_in   = false;	
static uint32_t start_button_time_counter = 0;
static uint32_t lc_button_time_counter	  = 0;	
static TimerHandle_t startButtonTimer;
static TimerHandle_t lcButtontimer;
#define START_BUTTON_TIME_INCREMENT 5
#define START_BUTTON_DELAY			500


typedef struct previousButtonPressStates {
	uint8_t prev_navigation_left;
	uint8_t prev_navigation_right;
	uint8_t prev_navigation_down;
	uint8_t prev_navigation_up;
	
	uint8_t prev_launch_control;
	uint8_t prev_dash_ack;
	uint8_t prev_sys_ack;
	uint8_t prev_start;
	} prevBtnState;

prevBtnState prevBtn = {
	.prev_navigation_left	= 1,
	.prev_navigation_right	= 1,
	.prev_navigation_down	= 1,
	.prev_navigation_up		= 1,
	.prev_launch_control	= 1,
	.prev_dash_ack			= 1,
	.prev_sys_ack			= 1,
	.prev_start				= 1
};


static void vStartButtonCallback(TimerHandle_t xTimer) {
	if ( (pio_readPin(START_PIO,START_PIN) == 0) && (prevBtn.prev_start == 0) ) {
		start_button_pushed_in = true;
		start_button_time_counter += START_BUTTON_TIME_INCREMENT;
	}
	else {
		start_button_pushed_in = false;
		start_button_time_counter = 0;
		xTimerStop(startButtonTimer,0);
	}
}

static void vLCButtonCallback(TimerHandle_t xTimer) {
	if ( (pio_readPin(LC_PIO,LC_PIN) == 0) && (prevBtn.prev_launch_control == 0) ) {
		lc_button_pushed_in = true;
		lc_button_time_counter += START_BUTTON_TIME_INCREMENT;
	}
	else {
		lc_button_pushed_in = false;
		lc_button_time_counter = 0;
		xTimerStop(lcButtontimer,0);
	}
}

void Task_ButtonInput() {
	TickType_t xLastwakeTime;
	
	//A switch will be true if there has been a transition from one state to another
	//It will be set to false after its handled in the menu task

	startButtonTimer = xTimerCreate("startButton",5/portTICK_RATE_MS,pdTRUE,0,vStartButtonCallback);
	lcButtontimer	 = xTimerCreate("lcButton", 5/portTICK_RATE_MS, pdTRUE,0,vLCButtonCallback);
	// Implement a function that checks if all buttons are in the correct position at start up ?
	
	while(1) {
		xSemaphoreTake(xButtonStruct,portMAX_DELAY); // Wait indefinetely for access to Button struct
		
		// The order which the buttons are checked determines the priority order of the buttonpress that will be set to true
		// if multiple button presses are registered at the same time.
		//Check for navigation action
		if (btn.unhandledButtonAction == false) {
			
			
			//*************************************************************************************************//
			//*********************************************NAVIGATION******************************************//
			//*************************************************************************************************//
			if ( (pio_readPin(NAVIGATION_L_PIO,NAVIGATION_L_PIN) == 0) && (prevBtn.prev_navigation_left == 1) ) {
				btn.unhandledButtonAction = true;
				btn.btn_type = NAVIGATION;
				btn.navigation = LEFT;
			}
			else if ((pio_readPin(NAVIGATION_R_PIO,NAVIGATION_R_PIN) == 0) && (prevBtn.prev_navigation_right == 1) ) {
				btn.unhandledButtonAction = true;
				btn.btn_type = NAVIGATION;
				btn.navigation = RIGHT;
			}
			else if ((pio_readPin(NAVIGATION_D_PIO,NAVIGATION_D_PIN) == 0) && (prevBtn.prev_navigation_down == 1) ) {
				btn.unhandledButtonAction = true;
				btn.btn_type = NAVIGATION;
				btn.navigation = DOWN;
			}
			else if ((pio_readPin(NAVIGATION_U_PIO,NAVIGATION_U_PIN) == 0) && (prevBtn.prev_navigation_up == 1) ) {
				btn.unhandledButtonAction = true;
				btn.btn_type = NAVIGATION;
				btn.navigation = UP;
			}
			else if ( (pio_readPin(SYS_ACK_PIO,SYS_ACK_PIN) == 0) && (prevBtn.prev_sys_ack == 1) ) {
				btn.unhandledButtonAction = true;
				btn.btn_type = PUSH_ACK;
			}
			//*************************************************************************************************//
			//*********************************************ROTARY ENCODER**************************************//
			//*************************************************************************************************//	
			else if (rotary_cw == true) {
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
			//*************************************************************************************************//
			//*********************************************LAUNCH CONTROL**************************************//
			//*************************************************************************************************//
			else if ((pio_readPin(LC_PIO,LC_PIN) == 0) && (prevBtn.prev_launch_control == 1)) {
				lc_button_pushed_in = false;
				lc_button_time_counter = 0;
				xTimerReset(lcButtontimer,0);
				
			}
			else if ( (lc_button_pushed_in == true ) && (lc_button_time_counter == START_BUTTON_DELAY)) {
				btn.unhandledButtonAction = true;
				btn.btn_type = LAUNCH_CONTROL;
				
				xTimerStop(lcButtontimer,0);
				lc_button_pushed_in = false;
				lc_button_time_counter = 0;
			}
			//*************************************************************************************************//
			//*********************************************START/ DRIVE ENABLE*********************************//
			//*************************************************************************************************//
			else if ( (pio_readPin(START_PIO, START_PIN) == 0) && (prevBtn.prev_start == 1) ) {
				start_button_pushed_in = false;
				start_button_time_counter = 0;
				xTimerReset(startButtonTimer,0);
				// This button should only be true if it has been pushed down for 0.5 seconds
				// After first activation, continually check that button == 0 and prev == 0 for 0.5 seconds
			}
			else if ( (start_button_pushed_in == true) && (start_button_time_counter == START_BUTTON_DELAY) ) {
				btn.unhandledButtonAction = true;
				btn.btn_type = START;
				
				xTimerStop(startButtonTimer,0);
				start_button_pushed_in = false;
				start_button_time_counter = 0;
			}
		}
		prevBtn.prev_navigation_left = pio_readPin(NAVIGATION_L_PIO,NAVIGATION_L_PIN); 
		prevBtn.prev_navigation_right = pio_readPin(NAVIGATION_R_PIO,NAVIGATION_R_PIN);
		prevBtn.prev_navigation_down = pio_readPin(NAVIGATION_D_PIO,NAVIGATION_D_PIN);
		prevBtn.prev_navigation_up = pio_readPin(NAVIGATION_U_PIO,NAVIGATION_U_PIN);
		
		prevBtn.prev_dash_ack = pio_readPin(ROT_PUSH3_PIO,ROT_PUSH3_PIN); 
		prevBtn.prev_sys_ack = pio_readPin(SYS_ACK_PIO,SYS_ACK_PIN);
		prevBtn.prev_launch_control = pio_readPin(LC_PIO,LC_PIN);
		prevBtn.prev_start = pio_readPin(START_PIO, START_PIN);
		
		xSemaphoreGive(xButtonStruct);
		
		vTaskDelay(20/portTICK_RATE_MS);
	}
}
		//Check the push button on the rotary encoder (acknowledge button)
		// 		if (btn.unhandledButtonAction == false) {
		// 			if ( (pio_readPin(ROT_PUSH3_PIO,ROT_PUSH3_PIN) == 0) && (prev_dash_ack == 1) ) {
		// 				btn.unhandledButtonAction = true;
		// 				btn.dash_acknowledge = true;
		// 				btn.btn_type = PUSH_ACK;
		// 			}
		// 		}

		//Check if the the system acknowledge button has been pressed
		// 		if (btn.unhandledButtonAction == false) {
		// 			if ( (pio_readPin(SYS_ACK_PIO,SYS_ACK_PIN) == 0) && (prev_sys_ack == 1) ) {
		// 				btn.unhandledButtonAction = true;
		// 				btn.system_acknowledge = true;
		// 				btn.btn_type = SYS_ACK;
		// 			}
		// 		}

void Task_RotaryEncoder() {
	uint8_t rot_phase_a = 0;
	uint8_t rot_phase_b = 0;
	uint8_t rot_state = 0;
	uint8_t prev_rot_state = 0;
	while(1) {
		//Check rotary encoder for new input
		// ^ XOR . true when inputs differ
		rot_phase_a = pio_readPin(ROT_A_PIO, ROT_A_PIN);
		rot_phase_b = pio_readPin(ROT_B_PIO, ROT_B_PIN);
		rot_state = (rot_phase_a << 1) | rot_phase_b;
		switch (rot_state) {
			case 0b00:
			if (prev_rot_state == 0b10) {
				// CCW Rotation
				rotary_ccw = true;
			}
			else if (prev_rot_state == 0b01) {
				// CW Rotation
				rotary_cw = true;
			}
			break;
			case 0b01:
			if (prev_rot_state == 0b00) {
				// CCW Rotation
				rotary_ccw = true;
			}
			else if (prev_rot_state == 0b11) {
				// CW Rotation
				rotary_cw = true;
			}
			break;
			case 0b10:
			if (prev_rot_state == 0b11) {
				// CCW Rotation
				rotary_ccw = true;
			}
			else if (prev_rot_state == 0b00) {
				// CW Rotation
				rotary_cw = true;
			}
			break;
			case 0b11:
			if (prev_rot_state == 0b01) {
				// CCW Rotation
				rotary_ccw = true;
			}
			else if (prev_rot_state == 0b10) {
				// CW Rotation
				rotary_cw = true;
			}
			break;
			
			default:
			rotary_ccw = false;
			rotary_cw  = false;
			break;
		}
		prev_rot_state = rot_state;
		vTaskDelay(5/portTICK_RATE_MS);
	}
}
