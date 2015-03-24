#ifndef DATALOGGERTASK_H_
#define DATALOGGERTASK_H_

#include "sam4e-base/FreeRTOS/Source/include/FreeRTOS.h"
#include "sam4e-base/FreeRTOS/Source/include/queue.h"
#include "sam4e-base/FreeRTOS/Source/include/task.h"
#include "sam4e-base/FreeRTOS/Source/include/semphr.h"


extern QueueHandle_t xDataloggerCommandQueue;
extern TaskHandle_t dataLoggerHandle;

enum EDataloggerCommands {CREATE_NEW_FILE,START_LOGGING, CLOSE_FILE, DELETE_ALL_FILES,NO_COMMAND};

void dataLoggerTask();




#endif /* DATALOGGERTASK_H_ */