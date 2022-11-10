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

extern uint8_t sample_state;
extern float dt;
extern MotionDataStruct MotionData;
extern double sample_time;
extern double dq[4],q[4];

void SampleFrequency_Configuration(SampleFrequency frequency);
void AttitudeSolution(void);

#endif

