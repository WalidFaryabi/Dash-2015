#ifndef DATALOGGERTASK_H_
#define DATALOGGERTASK_H_

#include "sam4e-base/FreeRTOS/Source/include/FreeRTOS.h"
#include "sam4e-base/FreeRTOS/Source/include/task.h"
#include "sam4e-base/FreeRTOS/Source/include/semphr.h"

extern TaskHandle_t dataLoggerHandle;

//void buffer
void dataLoggerTask();




#endif /* DATALOGGERTASK_H_ */