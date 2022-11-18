#ifndef __COMMAND_H
#define __COMMAND_H

#include "sys.h"
#include "computation.h"
#include "string.h"
#include "led.h" 
#include "serve.h"
#include "buzzer.h"
#include "bmm150.h"
#include "bmi088.h"
#include "adxl357.h"
#include "bmp388.h"
#include "fuse.h"
#include "w25q.h"
#include "lora.h"
#include "atgm336h.h"

#define BMI_START 1
#define BMI_STOP 2

#define AttitudeSolution_TEST 70
#define Sample_STOP 71
#define AccelerationSolution_TEST 72
#define VelocitySolution_TEST 73
#define PositionSolution_TEST 74
#define DATASTORAGE 100
#define DATAREAD 101

extern uint8_t Command_State;
extern uint8_t DataNumber;
extern uint32_t Storage_Number;
extern uint32_t Storage_Addr;//变量存储地址

void Command_Receive(uint8_t *buffer);
void AttitudeSolution_Ttst(void);
void W25Q_DataConsult(void);//查询已经保存的数据量及地址
void W25Q_DataClear(void);//清除已经保存的数据及地址表
ErrorStatus NumberChoose(uint8_t *buffer);//读取要读取的数据位数
void DataRead(uint32_t addr);
void DataStorage(void);//数据存入操作
void DataStorage_Init(void);
void Sample_Start(void);
void Sample_Stop(void);
void AccelerationSolution_Test(void);
void VelocitySolution_Test(void);
void PositionSolution_Test(void);
void MotionOffset_Init(void);//向第0扇区写入偏差量缓存
void MotionOffset_DeInit(void);//将偏差量重置为0
void MotionOffset_Get(void);//从第0扇区得到偏差量

#endif


