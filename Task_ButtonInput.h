/*
 * buttonTask.h
 *
 * Created: 02.02.2015 16:27:08
 *  Author: Will
 */ 


#ifndef BUTTONTASK_H_
#define BUTTONTASK_H_

#include <stdbool.h>

void Task_RotaryEncoder();
void Task_ButtonInput(); 

extern volatile bool rotary_cw;
extern volatile bool rotary_ccw;
#endif /* BUTTONTASK_H_ */