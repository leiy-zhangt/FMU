#ifndef __IMU_H
#define __IMU_H

#include "fatfs.h"
#include "cmsis_os.h"

typedef struct
{
	double acc_x,acc_y,acc_z,tran_acc_x,tran_acc_y,tran_acc_z;
	double gyr_x,gyr_y,gyr_z,tran_gyr_x,tran_gyr_y,tran_gyr_z;
	double pitch,roll,yaw,tran_pitch,tran_roll,tran_yaw;
	double pressure,height;
	double temperature;
	double quaternion[4];
	double height_Init;
}IMUDateStruct;

typedef enum 
{
	IMU_OK = 0,
	IMU_Receive_ERR,
	IMU_Data_ERR
}IMUStatus;

typedef enum
{
	IMU_NO_Rotation = 0,
	IMU_Roll_180
}IMU_RotationDirection;

extern uint8_t IMUReceiveBuff[];
extern uint8_t IMUFifoBuff[];

extern IMUDateStruct IMUData;

extern SemaphoreHandle_t IMUSemaphore;
extern BaseType_t IMUHigherTaskSwitch;

IMUStatus IMUDataConvert(uint8_t *DataBuff);
void IMURotationTransform(IMU_RotationDirection direction);

#endif
