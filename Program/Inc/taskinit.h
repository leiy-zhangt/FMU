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

void SDWrite(void *pvParameters);

//IMUReceive
extern BaseType_t IMUReceive_Ret;
extern UBaseType_t IMUReceive_Prio;
extern TaskHandle_t IMUReceive_TCB;

void IMUReceive(void *pvParameters);

//GNSSReceive
extern BaseType_t GNSSReceive_Ret;
extern UBaseType_t GNSSReceive_Prio;
extern TaskHandle_t GNSSReceive_TCB;

void GNSSReceive(void *pvParameters);


#endif
