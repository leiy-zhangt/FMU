#ifndef __TASKINIT_H
#define __TASKINIT_H

#include "main.h"
#include "cmsis_os.h"
#include "fatfs.h"
//TaskCreate
void TaskCreate(void);
//LEDTwink
extern BaseType_t LEDTwink_Ret;
extern UBaseType_t LEDTwink_Prio;
extern TaskHandle_t LEDTwink_TCB;

void LEDTwink(void *pvParameters);
//TaskMonitor
extern BaseType_t TaskMonitor_Ret;
extern UBaseType_t TaskMonitor_Prio;
extern TaskHandle_t TaskMonitor_TCB;

void TaskMonitor(void *pvParameters);
//SDWrite
extern BaseType_t SDWrite_Ret;
extern UBaseType_t SDWrite_Prio;
extern TaskHandle_t SDWrite_TCB;

void SDWrite(void);

#endif
