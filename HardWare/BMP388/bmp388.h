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


void BMP388_Configuration(void);//��ʼ������
void BMP388_WriteData(uint8_t addr,uint8_t data);//BMP388�������ݺ���
uint8_t BMP388_ReadData(uint8_t addr);//BMP388�����ݺ���
void BMP388_ReadBuffer(uint8_t addr,uint8_t *buffer,uint8_t length);
uint8_t BMP388_StatusGet(void);//BMP388״̬��ȡ����
void BMP388_Calibration(BMP388_Calibration_DataStruct *data_int,BMP388_Calibration_QuantizedDataStruct *data_float);//BMP388У��ϵ����ȡ
void BMP388_TemperatureGet(BMP388_DataStruct *BMP388_Data);//BMP388�¶�ֵ��ȡ,��λΪ��/C
void BMP388_PressureGet(BMP388_DataStruct *BMP388_Data);//BMP388��ѹֵ��ȡ,��λΪPa
double BMP388_HeightGet(void);//��ȡ��ǰ���θ߶ȣ���λΪm
#endif

