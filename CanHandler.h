/*
 * canHandler.h
 *
 * Created: 04.02.2015 22:49:23
 *  Author: Will
 */ 


#ifndef CANHANDLER_H_
#define CANHANDLER_H_

#include "sam4e-base/FreeRTOS/Source/include/FreeRTOS.h"
#include "sam4e-base/FreeRTOS/Source/include/task.h"
#include "sam4e-base/FreeRTOS/Source/include/semphr.h"
#include "sam4e-base/FreeRTOS/Source/include/queue.h"

extern QueueHandle_t xDeviceStatusQueue;
extern QueueHandle_t xRemoteControlQueue;
extern QueueHandle_t xDataLoggerQueue;
extern QueueHandle_t xDashQueue;

void init_CanHandling();




#endif /* CANHANDLER_H_ */