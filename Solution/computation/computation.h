#ifndef __COMPUTATION_H
#define __COMPUTATION_H

#include "sys.h"
#include "command.h"

typedef enum{Frequency_1Hz = 999999,Frequency_10Hz = 99999,Frequency_50Hz = 19999,Frequency_100Hz = 9999,Frequency_200Hz = 4999}SampleFrequency;

typedef struct
{
  double acc_x,acc_y,acc_z;
  double gyr_x,gyr_y,gyr_z;
  double pitch,yaw,roll;
  double velocity_x,velocity_y,velocity_z;
  double position_x,position_y,position_z;
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

extern const double PI;
extern uint8_t sample_state;
extern float dt;
extern MotionDataStruct MotionData;
extern MotionOffsetStruct MotionOffset;
extern double sample_time;
extern double q[4];
extern double T_11,T_12,T_13,T_21,T_22,T_23,T_31,T_32,T_33;//坐标准换矩阵

void SampleFrequency_Configuration(SampleFrequency frequency);
void AttitudeSolution(double gyr_x,double gyr_y,double gyr_z);//得到弧度制姿态角,输入为弹体坐标系下的角速度
void AccelerationSolution(double acc_x,double acc_y,double acc_z);//得到惯性坐标系下的加速度
void VelocitySolution(void);//得到惯性坐标系下的速度
void PositionSolution(void);//得到惯性坐标系下的位置

#endif

