#ifndef __MS5525_H
#define __MS5525_H

#include "main.h"

typedef enum
{
	MS5525_ERR=0,
	MS5525_OK
}MS5525_Status;

typedef struct
{
	uint16_t MS5525_C[8];
	int64_t MS5525_Tref;
	int32_t MS5525_Temp;
	int32_t MS5525_Pre;

	uint32_t MS5525_D1;
	uint32_t MS5525_D2;
	
	double pre,temp;
	
	double pre_init;
}MS5525_DataStruct;

// Qx Coefficients Matrix by Pressure Range
//  5525DSO-DB015AS (Pmin = 0, Pmax = 15)
extern const uint8_t MS5525_Q1;
extern const uint8_t MS5525_Q2;
extern const uint8_t MS5525_Q3;
extern const uint8_t MS5525_Q4;
extern const uint8_t MS5525_Q5; 
extern const uint8_t MS5525_Q6;

//I2C接收缓冲Buff
extern uint8_t MS5525_Buff[8];
//I2C命令
extern uint8_t MS5525_CMD;
//风速计数据结构体
extern MS5525_DataStruct MS5525_StaticData;
extern MS5525_DataStruct MS5525_TotalData;
//MS5525返回状态值
extern MS5525_Status MS5525_Ret;

void MS5525_Reset(uint8_t addr);
void MS5525_Calibration(uint8_t addr,MS5525_DataStruct *MS5525_Data);
MS5525_Status MS5525_Measure(void);
uint32_t MS5525_GetValue(uint8_t addr);
void MS5525_Converse(MS5525_DataStruct *MS5525_Data);

#endif

