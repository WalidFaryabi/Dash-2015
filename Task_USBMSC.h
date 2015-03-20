/*
 * MSCTask.h
 *
 * Created: 04.02.2015 12:03:42
 *  Author: Will
 */ 


#ifndef MSCTASK_H_
#define MSCTASK_H_


#include "sam4e-base/FreeRTOS/Source/include/FreeRTOS.h"
#include "sam4e-base/FreeRTOS/Source/include/task.h"
#include "sam4e-base/FreeRTOS/Source/include/semphr.h"

extern SemaphoreHandle_t file_access_mutex;
extern TaskHandle_t mscTaskHandle;
void usbMscTask();

#endif /* MSCTASK_H_ */