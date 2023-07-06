/*
指令分区：
0~9:BMI088模块
10~19:ADXL357模块
20~29:BMP388模块
30~39:BMM150模块
40~49:W25Q模块
50~59:Lora模块
60~69:GPS模块
70~99测试部分
100~：工作部分

W25Q存储分区：
第0块：用于存放系统参数
第0扇区：0~71Byte存放传感器稳态偏移量，72以后待定。
第1扇区及以后，存放已经保存的数据量及页数。

*/

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
#include "control.h"

#define BMI_TEST 1

#define ADXL_TEST 10

#define Height_TEST 20

#define MagnetismOffset_INIT 30
#define MagnetismOffset_STOP 31
#define BMM_TEST 32

#define AttitudeSolution_TEST 70
#define AccelerationSolution_TEST 72
#define AttitudeCompensation_TEST 73
#define VelocitySolution_TEST 74
#define PositionSolution_TEST 75

#define Sample_STOP 99

#define Data_STORAGE 100
#define Data_READ 101
#define ParafoilControl_START 102
#define MotorCal_START 103
#define MotorCal_STOP 104
#define IMUUpOffset 105
#define IMUBackOffset 106
#define FixdWingControl_TEST 107

extern uint8_t Remote_state;//遥控器状态
extern uint8_t Command_State;
extern uint8_t DataNumber;
extern uint32_t Storage_Number;
extern uint32_t Storage_Addr;//变量存储地址
extern uint32_t Fuse_State;
extern uint16_t RemoteChannel[5];//遥控器通道，最低为352，最高位1696，中立位置为1024
extern uint16_t ServeOutput[4];//舵机通道输出

void Command_Receive(uint8_t *buffer);
void Q_Init(void);//四元数初始化，确定当前姿态角
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
void IMUOffset_Init(void);//向第0扇区写入偏差量缓存
void MagnetismOffset_Init(void);//向0扇区写入磁场偏差量
void FMUOffset_DeInit(void);//将偏差量重置为0
void FMUOffset_Get(void);//从第0扇区得到偏差量
void Height_Test(void);//气压计测试程序
void Position_DeInit(void);//位置参数清楚初始化
void Position_Init(void);//位置参数初始化
void MotorCal_Start(void);//电机校准启动
void MotorCal_Stop(void);//电机校准结束
void ParafoilControl_Start(void);//翼伞控制开始
void ParafoilControl_Stop(void);//翼伞控制结束
void FixdWingControl_Start(void);//固定翼飞行测试
#endif


