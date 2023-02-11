#ifndef __BMP388_H
#define __BMP388_H

#include "sys.h"
#include "delay.h"
#include "spi.h"
#include "math.h"

#define BMP_CS PCout(1)
#define BMP_CS_Pin GPIO_Pin_1
#define BMP_CmdRd while((BMP388_ReadData(0x03)&0x10)==0)
    
typedef struct 
{
    uint16_t par_t1;
    uint16_t par_t2;
    int8_t par_t3;
    int16_t par_p1;
    int16_t par_p2;
    int8_t par_p3;
    int8_t par_p4;
    uint16_t par_p5;
    uint16_t par_p6;
    int8_t par_p7;
    int8_t par_p8;
    int16_t par_p9;
    int8_t par_p10;
    int8_t par_p11;
}BMP388_Calibration_DataStruct;

typedef struct 
{
  double par_t1;
  double par_t2;
  double par_t3;
  double par_p1;
  double par_p2;
  double par_p3;
  double par_p4;
  double par_p5;
  double par_p6;
  double par_p7;
  double par_p8;
  double par_p9;
  double par_p10;
  double par_p11;
  double t_lin;
}BMP388_Calibration_QuantizedDataStruct;

typedef struct
{
  uint8_t buffer[3];
  int32_t uncomp_data;
  double tem;
  double pre;
}BMP388_DataStruct;

extern BMP388_Calibration_DataStruct BMP388_CalibrationData;
extern BMP388_Calibration_QuantizedDataStruct BMP388_Calibration_QuantizedData;
extern BMP388_DataStruct BMP388_Data;
extern double height_init;

void BMP388_Configuration(void);//初始化函数
void BMP388_WriteData(uint8_t addr,uint8_t data);//BMP388发送数据函数
uint8_t BMP388_ReadData(uint8_t addr);//BMP388读数据函数
void BMP388_ReadBuffer(uint8_t addr,uint8_t *buffer,uint8_t length);
uint8_t BMP388_StatusGet(void);//BMP388状态获取函数
void BMP388_Calibration(BMP388_Calibration_DataStruct *data_int,BMP388_Calibration_QuantizedDataStruct *data_float);//BMP388校正系数获取
void BMP388_TemperatureGet(BMP388_DataStruct *BMP388_Data);//BMP388温度值获取,单位为°/C
void BMP388_PressureGet(BMP388_DataStruct *BMP388_Data);//BMP388气压值获取,单位为Pa
double BMP388_HeightGet(void);//获取当前海拔高度，单位为m
double BMP388_HeightCalibration(void);//BMP388校准公式
#endif

