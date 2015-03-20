#include "snakeGame.h"
#include "DriversNotInBase/FT800.h"
#include <stdlib.h>


//STATIC FUNCTION DECALARATIONS//
static void updateSnake();
static void snakeTimerCallback();
static void renderSnake();
static void initMap();
static bool checkForSelfCollision();
static void moveSnake(int8_t dx, int8_t dy);
static void changeDirection(ENavigationDirection dir, bool newButtonAction);
static void generateFood();
//END STATIC FUNCTION DECLARATIONS//

#define WINDOW_HEIGHT	272
#define WINDOW_WIDTH	480
#define CELL_SIZE		20
//#define NUM_SEGMENTS_X	WINDOW_WIDTH  / CELL_SIZE
//#define NUM_SEGMENTS_Y	WINDOW_HEIGHT / CELL_SIZE
#define NUM_SEGMENTS_X	24
#define NUM_SEGMENTS_Y	13

#define ADJUSTMENT_MENU_SNAKE 6
#define MAX_SNAKE_SIZE ((NUM_SEGMENTS_X-1)*(NUM_SEGMENTS_Y-1))

typedef enum {FOOD, SNAKE_PART, EMPTY_POS, WALL} mapItems;
typedef enum {SNAKE_UP, SNAKE_DOWN,SNAKE_LEFT,SNAKE_RIGHT} SnakeDir;


struct SnakeSegments {
	uint8_t x_pos;
	uint8_t y_pos;
};
struct SnakeSegments SnakeSegment;

TimerHandle_t snakeUpdateTimer;
static bool snakeUpdate = false;
static void snakeTimerCallback() {
	snakeUpdate = true;
}

SnakeState snakeGameState = SNAKE_OFF;

QueueHandle_t snakeQueue;
static SnakeDir SnakeDirection = SNAKE_RIGHT;

uint8_t gameMap[NUM_SEGMENTS_Y][NUM_SEGMENTS_X];
static uint16_t gameSpeed = 500; // The delay time between updates of the snake position

//static uint16_t snake_score1 = 0;
static uint16_t snake_score = 0;
void snakeControlFunction(bool buttonPressed, ENavigationDirection dir) {
	switch (snakeGameState) {
		case SNAKE_OFF:
			break;
		case SNAKE_PREPPING:
			SnakeDirection = SNAKE_RIGHT;
			snake_score = 0;
			gameSpeed = 500;
			snakeUpdateTimer = xTimerCreate("Snake",gameSpeed/portTICK_RATE_MS, pdPASS,0, snakeTimerCallback);
			xTimerChangePeriod(snakeUpdateTimer,gameSpeed/portTICK_RATE_MS,200/portTICK_RATE_MS);
			xTimerReset(snakeUpdateTimer, portMAX_DELAY);
			snakeQueue = xQueueCreate(MAX_SNAKE_SIZE, sizeof(SnakeSegment));
			initMap(); // Initializes map, creates the snake and the first food item
			snakeGameState = SNAKE_WAITFORSTART;
			renderSnake();
			break;
		case SNAKE_WAITFORSTART:
			snake_score = 0;
			if (buttonPressed) {
				snakeGameState = SNAKE_RUNNING;
			}
			break;
		case SNAKE_RUNNING:
			changeDirection(dir, buttonPressed);
			if (snakeUpdate == true) {
				snakeUpdate = false;
				// State will be changed to OFF if the collides
				updateSnake();
				if (snakeGameState != SNAKE_OFF) {
					renderSnake();
				}
			}
			break;
	}
}

static void updateSnake() {
	if (checkForSelfCollision()) {
		// Stop timers etc, delete queues etc
		selected = ADJUSTMENT_MENU_SNAKE;
		vQueueDelete(snakeQueue);
		snakeGameState = SNAKE_OFF;
		xTimerDelete(snakeUpdateTimer,200/portTICK_RATE_MS);
		
	}
	if (snakeGameState != SNAKE_OFF) {
		switch (SnakeDirection) {
			case SNAKE_UP:
			moveSnake(0,-1);
			break;
			case SNAKE_DOWN:
			moveSnake(0,1);
			break;
			case SNAKE_LEFT:
			moveSnake(-1,0);
			break;
			case SNAKE_RIGHT:
			moveSnake(1,0);
			break;
		}
	}

}

static void initMap() {
	// Set all fields to Empty
	for (uint8_t i = 0 ; i < NUM_SEGMENTS_Y; i++) {
		for (uint8_t j = 0; j < NUM_SEGMENTS_X; j++) {
			gameMap[i][j] = EMPTY_POS;
		}
	}
	// Init walls
	for (uint8_t width = 0 ; width < NUM_SEGMENTS_X; width ++) {
		gameMap[0][width] = WALL; // Draw Top Wall
		gameMap[NUM_SEGMENTS_Y-1][width] = WALL; // Draw Bottom Wall
	}
	for (uint8_t height = 0 ; height < NUM_SEGMENTS_Y; height ++) {
		gameMap[height][0] = WALL; // Draw Left Side Wall
		gameMap[height][NUM_SEGMENTS_X-1] = WALL; // Draw Right Side Wall
	}	
	
	struct SnakeSegments part1 = {
		.x_pos = 10,
		.y_pos = 5
		};
	struct SnakeSegments part2 = {
		.x_pos = 11,
		.y_pos = 5
	};
	struct SnakeSegments part3 = {
		.x_pos = 12,
		.y_pos = 5
	};
	gameMap[5][10] = SNAKE_PART;
	gameMap[5][11] = SNAKE_PART;
	gameMap[5][12] = SNAKE_PART;
	xQueueSendToBack(snakeQueue,&part1,0);
	xQueueSendToBack(snakeQueue,&part2,0);
	xQueueSendToBack(snakeQueue,&part3,0);
	generateFood();
}


static void changeDirection(ENavigationDirection dir, bool newButtonAction) {
	if (newButtonAction) {
		switch (dir) {
			case UP:
			if (SnakeDirection != SNAKE_DOWN) {
				SnakeDirection = SNAKE_UP;
			}
			break;
			case DOWN:
			if (SnakeDirection != SNAKE_UP) {
				SnakeDirection = SNAKE_DOWN;
			}
			break;
			case LEFT:
			if (SnakeDirection != SNAKE_RIGHT) {
				SnakeDirection = SNAKE_LEFT;
			}
			break;
			case RIGHT:
			if (SnakeDirection != SNAKE_LEFT) {
				SnakeDirection = SNAKE_RIGHT;
			}
			break;
		}
	}
}

static void moveSnake(int8_t dx, int8_t dy) {
	struct SnakeSegments popped;
	struct SnakeSegments tempSnake;
	QueueHandle_t tempSnakeQueue;
	tempSnakeQueue = xQueueCreate(MAX_SNAKE_SIZE,sizeof(tempSnake));
	
	// Need to peak at the last element which is the head of the snake.. 
	while ( xQueueReceive(snakeQueue, &tempSnake, 0) == pdPASS) {
		xQueueSendToBack(tempSnakeQueue, &tempSnake,0);
	}
	// When it exits this loop the tempSnake element will be the head
	struct SnakeSegments newHeadSegment;
	newHeadSegment.x_pos = tempSnake.x_pos + dx;
	newHeadSegment.y_pos = tempSnake.y_pos + dy;
	
	// Reconstruct the snakeQueue
	while ( xQueueReceive(tempSnakeQueue, &tempSnake, 0) == pdPASS) {
		xQueueSendToBack(snakeQueue, &tempSnake,0);
	}

	if ((newHeadSegment.x_pos == 0) || (newHeadSegment.y_pos == 0) || (newHeadSegment.x_pos == (NUM_SEGMENTS_X-1)) || (newHeadSegment.y_pos == (NUM_SEGMENTS_Y -1))) {
		vQueueDelete(snakeQueue);
		vQueueDelete(tempSnakeQueue);
		snakeGameState = SNAKE_OFF;
		selected = ADJUSTMENT_MENU_SNAKE;
		xTimerDelete(snakeUpdateTimer,200/portTICK_RATE_MS);
	}
	else if (gameMap[newHeadSegment.y_pos][newHeadSegment.x_pos] == FOOD ) {
		generateFood();
		if (gameSpeed >= 50) {
			gameSpeed -= 50;
		}
		snake_score += 10;
		xTimerChangePeriod(snakeUpdateTimer,gameSpeed/portTICK_RATE_MS,200/portTICK_RATE_MS);
	}
	else {
		xQueueReceive(snakeQueue, &popped,portMAX_DELAY); // Delete tail which is the front of the queue
		//Updata gameMap
		gameMap[popped.y_pos][popped.x_pos] = EMPTY_POS;
	}
	if ( snakeGameState != SNAKE_OFF) {
		// Add the new head to the back of the queue, in front of the old head
		xQueueSendToBack(snakeQueue,&newHeadSegment,0);
		vQueueDelete(tempSnakeQueue);
		//Update gameMap
		gameMap[newHeadSegment.y_pos][newHeadSegment.x_pos] = SNAKE_PART;
	}
}

static bool checkForSelfCollision() {
	struct SnakeSegments tempHeadPos;
	struct SnakeSegments tempSnake;
	QueueHandle_t tempSnakeQueue;
	tempSnakeQueue = xQueueCreate(MAX_SNAKE_SIZE,sizeof(tempSnake));
		
	// Need to peak at the last element which is the head of the snake..
	while ( xQueueReceive(snakeQueue, &tempSnake, 0) == pdPASS) {
		xQueueSendToBack(tempSnakeQueue, &tempSnake,0);
	}
	// When it exits this loop the tempSnake element will be the head
	tempHeadPos.x_pos = tempSnake.x_pos;
	tempHeadPos.y_pos = tempSnake.y_pos;

	// Reconstruct the snakeQueue
	uint8_t collisionCounter = 0;
	while ( xQueueReceive(tempSnakeQueue, &tempSnake, 0) == pdPASS) {
		if ( (tempHeadPos.x_pos == tempSnake.x_pos) && (tempHeadPos.y_pos == tempSnake.y_pos)) {
			collisionCounter += 1;
		}
		xQueueSendToBack(snakeQueue, &tempSnake,0);
	}
	vQueueDelete(tempSnakeQueue);
	
	if (collisionCounter >= 2){
		return true;
	}
	else {
		return false;
	}
}


static void renderSnake() {
	cmd(CMD_DLSTART);
	cmd(CLEAR(1,1,1));
	cmd(BEGIN(RECTS));
	for (uint8_t i = 0 ; i < NUM_SEGMENTS_Y; i++) {
		for (uint8_t j = 0; j < NUM_SEGMENTS_X; j++) {
			if (gameMap[i][j] == SNAKE_PART) {
				cmd(COLOR_RGB(0,255,0));
				cmd(VERTEX2F(j*CELL_SIZE*16, i*CELL_SIZE*16)); // Top left coordinates
				cmd(VERTEX2F((j*CELL_SIZE + CELL_SIZE)*16, (i*CELL_SIZE + CELL_SIZE) * 16)); // Bottom rightcoordinates
			}
			else if (gameMap[i][j] == FOOD) {
				cmd(COLOR_RGB(230,230,30));
				cmd(VERTEX2F(j*CELL_SIZE*16, i*CELL_SIZE*16)); // Top left coordinates
				cmd(VERTEX2F((j*CELL_SIZE + CELL_SIZE)*16, (i*CELL_SIZE + CELL_SIZE) * 16)); // Bottom rightcoordinates
			}
			else if (gameMap[i][j] == WALL) {
				cmd(COLOR_RGB(255,0,0));
				cmd(VERTEX2F(j*CELL_SIZE*16, i*CELL_SIZE*16)); // Top left coordinates
				cmd(VERTEX2F((j*CELL_SIZE + CELL_SIZE)*16, (i*CELL_SIZE + CELL_SIZE) * 16)); // Bottom rightcoordinates
			}
		}
	}
	// Color the head red
	struct SnakeSegments tempSnake;
	QueueHandle_t tempSnakeQueue;
	tempSnakeQueue = xQueueCreate(MAX_SNAKE_SIZE,sizeof(tempSnake));
	
	// Need to peak at the last element which is the head of the snake..
	while ( xQueueReceive(snakeQueue, &tempSnake, 0) == pdPASS) {
		xQueueSendToBack(tempSnakeQueue, &tempSnake,0);
	}
	// When it exits this loop the tempSnake element will be the head
	cmd(COLOR_RGB(255,0,0));
	cmd(VERTEX2F(tempSnake.x_pos*CELL_SIZE*16, tempSnake.y_pos*CELL_SIZE*16)); // Top left coordinates
	cmd(VERTEX2F((tempSnake.x_pos*CELL_SIZE + CELL_SIZE)*16, (tempSnake.y_pos*CELL_SIZE + CELL_SIZE) * 16)); // Bottom rightcoordinates
	// Reconstruct the snakeQueue
	while ( xQueueReceive(tempSnakeQueue, &tempSnake, 0) == pdPASS) {
		xQueueSendToBack(snakeQueue, &tempSnake,0);
	}
	vQueueDelete(tempSnakeQueue);
	cmd(COLOR_RGB(255,255,255));
	cmd_text(200,10,28,OPT_CENTER,"Score: ");
	cmd_number(250,10,28,OPT_CENTER,snake_score);
	cmd(DISPLAY()); // display the image
	cmd(CMD_SWAP);
	cmd_exec();
}
static void generateFood() {
	uint16_t x = 0;
	uint16_t y = 0;
	do {
		x = (rand() % ( (NUM_SEGMENTS_X - 1))) + 1;
		y = (rand() % ( (NUM_SEGMENTS_Y -1 ))) + 1;
	} while (gameMap[y][x] != EMPTY_POS);
	
	gameMap[y][x] = FOOD;
}