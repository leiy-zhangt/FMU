#ifndef __ADXL357_H
#define __ADXL357_H

#include "sys.h"
#include "delay.h"
#include "spi.h"

#define ADXL_CS PCout(13)
#define ADXL_Port GPIOC
#define ADXL_CS_Pin GPIO_Pin_13
#define ADXL_INT_Pin GPIO_Pin_14


typedef struct 
{
  uint8_t buffer[11];
  int32_t acc_x_int;
  int32_t acc_y_int;
  int32_t acc_z_int;
  int16_t tem_int;
  double acc_x;
  double acc_y;
  double acc_z;
  float tem;
}ADXL357_DataStruct;

typedef enum {ADXL_Range_10g = 0x01,ADXL_Range_20g = 0x02,ADXL_Range_40g = 0x03}ADXL_Range_Choose;//加速度计量程选择

extern int8_t ADXL_Range;//加速度计量程选择
extern ADXL357_DataStruct ADXL357_Data;

void ADXL357_Configuration(ADXL_Range_Choose adxl_range); 
void ADXL357_WriteData(uint8_t addr,uint8_t data);
uint8_t ADXL357_ReadData(uint8_t addr); 
void ADXL357_ReadBuffer(uint8_t addr,uint8_t* buffer,uint8_t length); 
void ADXL357_Measure(ADXL357_DataStruct *ADXL357_Data);

#endif
