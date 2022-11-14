#ifndef __COMPUTATION_H
#define __COMPUTATION_H

#include "sys.h"
#include "command.h"

#define PI 3.141592654

typedef enum{Frequency_1Hz = 999999,Frequency_10Hz = 99999,Frequency_50Hz = 19999,Frequency_100Hz = 9999,Frequency_200Hz = 4999}SampleFrequency;

typedef struct
{
  double acc_x,acc_y,acc_z;
  double gyr_x,gyr_y,gyr_z;
  double pitch,yaw,roll;
  double velocity_x,velocity_y,velocity_z;
  double postion_x,position_y,position_z;
  double pressure,height;
}MotionDataStruct;

typedef struct
{
  double acc_x_offset;
  double acc_y_offset;
  double acc_z_offset;
  double gyr_x_offset;
  double gyr_y_offset;
  double gyr_z_offset;
  double adxl_x_offset;
  double adxl_y_offset;
  double adxl_z_offset;
}MotionOffsetStruct;

extern uint8_t sample_state;
extern float dt;
extern MotionDataStruct MotionData;
extern MotionOffsetStruct MotionOffset;
extern double sample_time;
extern double q[4];

void SampleFrequency_Configuration(SampleFrequency frequency);
void AttitudeSolution(void);//得到弧度制姿态角
void MotionOffset_Init(void);//向第0扇区写入偏差量缓存
void MotionOffset_Get(void);//从第0扇区得到偏差量

#endif

