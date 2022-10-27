/*
BMI088操作逻辑：
在初始化时向加速度计写一个空信号，随后发送指令：
  a. Power up the sensor 
  b. Wait 1 ms 
  c. Enter normal mode by writing ‘4’ to ACC_PWR_CTRL 
  d. Wait for 50 ms
*/

#ifndef __BMI088_H
#define __BMI088_H

#include "sys.h"
#include "delay.h"
#include "spi.h"

#define ACC_CS PCout(2)
#define GYR_CS PCout(4) 
#define ACC_Port GPIOC
#define ACC_CS_Pin GPIO_Pin_2
#define ACC_INT_Pin GPIO_Pin_3
#define GYR_Port GPIOC
#define GYR_CS_Pin GPIO_Pin_4
#define GYR_INT_Pin GPIO_Pin_5

#define ODR_100Hz_32BD 0X07
#define ODR_200Hz_65BD 0X06
#define ODR_100Hz_12BD 0X05
#define ODR_200Hz_23BD 0X04
#define ODR_400Hz_47BD 0X03
#define ODR_1000Hz_116BD 0X02
#define ODR_2000Hz_230BD 0X01
#define ODR_2000Hz_523BD 0X10

typedef struct 
{
  uint8_t buffer[6];
  int16_t acc_x_int;
  int16_t acc_y_int;
  int16_t acc_z_int;
  double acc_x;
  double acc_y;
  double acc_z;
  int16_t gyr_x_int;
  int16_t gyr_y_int;
  int16_t gyr_z_int;
  double gyr_x;
  double gyr_y;
  double gyr_z;
  int16_t tem_int;
  double tem;
}BMI088_DataStruct;

typedef enum {ACC_Choose=0,GYR_Choose=1}IMU_Choose;//IMU选择
typedef enum {ACC_Range_3g = 0x00,ACC_Range_6g = 0x01,ACC_Range_12g = 0x02,ACC_Range_24g = 0x03}ACC_Range_Choose;//加速度计量程选择
typedef enum {GYR_Range_125 = 0X04,GYR_Range_250 = 0X03,GYR_Range_500 = 0X02,GYR_Range_1000 = 0X01,GYR_Range_2000 = 0X00}GYR_Range_Choose;//陀螺仪量程原则

extern const float g;//重力加速度
extern int16_t ACC_Range,GYR_Range;//惯导量程选择
extern float dt;//积分时间步长
extern BMI088_DataStruct BMI088_Data;

void BMI088_Configuration(ACC_Range_Choose acc_range,GYR_Range_Choose gyr_range); 
void BMI088_WriteData(IMU_Choose IMU,uint8_t addr,uint8_t data);
uint8_t BMI088_ReadData(IMU_Choose IMU,uint8_t addr); 
void BMI088_ReadBuffer(IMU_Choose IMU,uint8_t addr,uint8_t* buffer,uint8_t length); 
void BMI088_Measure(BMI088_DataStruct *BMI088_Data);

#endif
