#include <stdbool.h>

#ifndef SNAKEGAME_H_
#define SNAKEGAME_H_

#include "Task_Menu.h"


typedef enum {SNAKE_OFF, SNAKE_PREPPING, SNAKE_WAITFORSTART, SNAKE_RUNNING} SnakeState;
	
extern SnakeState snakeGameState;

void snakeControlFunction(bool buttonPressed, EJoystickDirection dir);

#endif /* SNAKEGAME_H_ */