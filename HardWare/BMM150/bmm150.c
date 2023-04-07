/*
@breaf:BMM150驱动代码，可以完成对磁传感器得初始化配置与数据读取。
BMM150的工作模式为强制模式，读取时测量磁场大小。
BMM150的校准程序存在问题，不建议使用。
*/
#include "bmm150.h"

BMM150_DataStruct BMM150_Data;
BMM150_TrimStruct BMM150_Trim;
BMM150_CalStruct BMM150_CalData;

void BMM150_Configuration(void)
{
  delay_us(1);
  BMM150_WriteData(0x4B,0x82);//软件复位
  delay_ms(10);
  BMM150_WriteData(0x4B,0x01);//退出待机模式，进入睡眠模式
  delay_ms(10);
//  BMM150_WriteData(0x4E,0x38);//使能输出通道
  BMM150_WriteData(0x4E,0x00);//使能输出通道
  BMM150_Trim_Get(&BMM150_Trim);
  BMM150_WriteData(0x51,23);//设置nXY = 1+2*23;
  BMM150_WriteData(0x52,82);//设置nXY = 1+82
  BMM150_WriteData(0x4C,0x28);//配置正常模式，输出速率为20Hz
//  delay_ms(100);
}

void BMM150_WriteData(uint8_t addr,uint8_t data)
{
  BMM_CS = 0;
  delay_us(1);
  SPI_ReadWriteByte(SPI1,addr&0x7f);
  SPI_ReadWriteByte(SPI1,data);
  BMM_CS = 1;
  delay_us(10);
}

uint8_t BMM150_ReadData(uint8_t addr)
{
  static uint8_t res;
  BMM_CS = 0;
  delay_us(1);
  SPI_ReadWriteByte(SPI1,addr|0x80);
  res = SPI_ReadWriteByte(SPI1,addr);
  BMM_CS = 1;
  delay_us(10);
  return res;
}

void BMM150_ReadBuffer(uint8_t addr,uint8_t *buffer,uint8_t length)
{
  BMM_CS = 0;
  delay_us(1);
  SPI_ReadWriteByte(SPI1,addr|0x80);
  for(;length>0;length--)
  {
    *buffer = SPI_ReadWriteByte(SPI1,0x00);
    buffer++;
  }
  BMM_CS = 1;
  delay_us(10);
}

void BMM150_Measure(BMM150_DataStruct *BMM150_Data)
{
//  uint8_t status;
//  BMM150_WriteData(0x4C,0x2B);
//  while(1) 
//  {
//    status = BMM150_ReadData(0x4C);
//    if((BMM150_ReadData(0x48)&0x01)) break;
//    if((status&0x02)&&(status&0x04)) break;;
//  }
  BMM150_ReadBuffer(0x42,BMM150_Data->buffer,8);
  BMM150_Data->data_x_int = (int16_t)BMM150_Data->buffer[1]<<8 | BMM150_Data->buffer[0];
  BMM150_Data->data_x_int = BMM150_Data->data_x_int>>3;
  BMM150_Data->data_y_int = (int16_t)BMM150_Data->buffer[3]<<8 | BMM150_Data->buffer[2];
  BMM150_Data->data_y_int = BMM150_Data->data_y_int>>3;
  BMM150_Data->data_z_int = (int16_t)BMM150_Data->buffer[5]<<8 | BMM150_Data->buffer[4];
  BMM150_Data->data_z_int = BMM150_Data->data_z_int>>1;
  BMM150_Data->data_rhall_uint = (((((uint16_t)BMM150_Data->buffer[7])<<8)) | (uint16_t)BMM150_Data->buffer[6])>>2;
  
#if BMM150_Cal
  //开始计算x轴磁场
  BMM150_Data->data_x = (BMM150_CompensateX(BMM150_Data,&BMM150_Trim)-BMM150_CalData.offset_x)*BMM150_CalData.scale_x;
  //开始计算y轴磁场
  BMM150_Data->data_y = (BMM150_CompensateY(BMM150_Data,&BMM150_Trim)-BMM150_CalData.offset_y)*BMM150_CalData.scale_y;
  //开始计算z轴磁场
  BMM150_Data->data_z = (BMM150_CompensateZ(BMM150_Data,&BMM150_Trim)-BMM150_CalData.offset_z)*BMM150_CalData.scale_z;
#else
  BMM150_Data->data_x = BMM150_Data->data_x_int / 8191.0f * 2600.0f-MotionOffset.bmm_x_offset;
  BMM150_Data->data_y = BMM150_Data->data_y_int / 8191.0f * 2600.0f-MotionOffset.bmm_y_offset;
  BMM150_Data->data_z = (BMM150_Data->data_z_int / 32767.0f * 5000.0f-MotionOffset.bmm_z_offset)*2.26;
#endif
}

void BMM150_Trim_Get(BMM150_TrimStruct *BMM150_Trim)
{
  uint8_t trimX1Y1[2] = { 0 };
  uint8_t trimXYXData[4] = { 0 };
  uint8_t trimXY1XY2[10] = { 0 };
  uint16_t tempMsb = 0;
  
  delay_ms(1000);
  BMM150_ReadBuffer(BMM150_DIG_X1, trimX1Y1, 2);
  BMM150_ReadBuffer(BMM150_DIG_Z4_LSB, trimXYXData, 4);
  BMM150_ReadBuffer(BMM150_DIG_Z2_LSB, trimXY1XY2, 10);
  
  BMM150_Trim->digX1 = (int8_t)trimX1Y1[0];
  BMM150_Trim->digY1 = (int8_t)trimX1Y1[1];
  BMM150_Trim->digX2 = (int8_t)trimXYXData[2];
  BMM150_Trim->digY2 = (int8_t)trimXYXData[3];
  tempMsb = ((uint16_t)trimXY1XY2[3]) << 8;
  BMM150_Trim->digZ1 = (uint16_t)(tempMsb | trimXY1XY2[2]);
  tempMsb = ((uint16_t)trimXY1XY2[1]) << 8;
  BMM150_Trim->digZ2 = (int16_t)(tempMsb | trimXY1XY2[0]);
  tempMsb = ((uint16_t)trimXY1XY2[7]) << 8;
  BMM150_Trim->digZ3 = (int16_t)(tempMsb | trimXY1XY2[6]);
  tempMsb = ((uint16_t)trimXYXData[1]) << 8;
  BMM150_Trim->digZ4 = (int16_t)(tempMsb | trimXYXData[0]);
  BMM150_Trim->digXY1 = trimXY1XY2[9];
  BMM150_Trim->digXY2 = (int8_t)trimXY1XY2[8];
  tempMsb = ((uint16_t)(trimXY1XY2[5] & 0x7F)) << 8;
  BMM150_Trim->digXYZ1 = (uint16_t)(tempMsb | trimXY1XY2[4]);
}

double BMM150_CompensateX(BMM150_DataStruct *BMM150_Data,BMM150_TrimStruct *BMM150_Trim)
{
  int16_t retval, processCompX1;
  uint16_t processCompX0, processCompX2;
  int32_t processCompX3, processCompX4, processCompX5, processCompX6, processCompX7, processCompX8, processCompX9, processCompX10;
  processCompX0 = BMM150_Data->data_rhall_uint;
  processCompX1 = ((int32_t)BMM150_Trim->digXYZ1) * 16384;
  processCompX2 = ((uint16_t)(processCompX1 / processCompX0)) - ((uint16_t)0x4000);
  retval = ((int16_t)processCompX2);
  processCompX3 = (((int32_t)retval) * ((int32_t)retval));
  processCompX4 = (((int32_t)BMM150_Trim->digXY2) * (processCompX3 / 128));
  processCompX5 = (int32_t)(((int16_t)BMM150_Trim->digXY1) * 128);
  processCompX6 = ((int32_t)retval) * processCompX5;
  processCompX7 = (((processCompX4 + processCompX6) / 512) + ((int32_t)0x100000));
  processCompX8 = ((int32_t)(((int16_t)BMM150_Trim->digX2) + ((int16_t)0xA0)));
  processCompX9 = ((processCompX7 * processCompX8) / 4096);
  processCompX10 = ((int32_t)BMM150_Data->data_x_int) * processCompX9;
  retval = ((int16_t)(processCompX10 / 8192));
  retval = (retval + (((int16_t)BMM150_Trim->digX1) * 8)) / 16;
  return retval/4096.0 * 1300;
}

double BMM150_CompensateY(BMM150_DataStruct *BMM150_Data,BMM150_TrimStruct *BMM150_Trim)
{
  int16_t retval;
  uint16_t processCompY0, processCompY2;
  int32_t processCompY1, processCompY3, processCompY4, processCompY5, processCompY6, processCompY7, processCompY8, processCompY9;
  processCompY0 = BMM150_Data->data_rhall_uint;
  processCompY1 = (((int32_t)BMM150_Trim->digXYZ1) * 16384) / processCompY0;
  processCompY2 = ((uint16_t)processCompY1) - ((uint16_t)0x4000);
  retval = ((int16_t)processCompY2);
  processCompY3 = ((int32_t) retval) * ((int32_t)retval);
  processCompY4 = ((int32_t)BMM150_Trim->digXY2) * (processCompY3 / 128);
  processCompY5 = ((int32_t)(((int16_t)BMM150_Trim->digXY1) * 128));
  processCompY6 = ((processCompY4 + (((int32_t)retval) * processCompY5)) / 512);
  processCompY7 = ((int32_t)(((int16_t)BMM150_Trim->digY2) + ((int16_t)0xA0)));
  processCompY8 = (((processCompY6 + ((int32_t)0x100000)) * processCompY7) / 4096);
  processCompY9 = (((int32_t)BMM150_Data->data_y_int) * processCompY8);
  retval = (int16_t)(processCompY9 / 8192);
  retval = (retval + (((int16_t)BMM150_Trim->digY1) * 8)) / 16;
  return retval/4096.0 * 1300;
}

double BMM150_CompensateZ(BMM150_DataStruct *BMM150_Data,BMM150_TrimStruct *BMM150_Trim)
{
  int32_t retval, processCompZ1, processCompZ2, processCompZ3;
  int16_t processCompZ0, processCompZ4;
  processCompZ0 = ((int16_t)BMM150_Data->data_rhall_uint) - ((int16_t) BMM150_Trim->digXYZ1);
  processCompZ1 = (((int32_t)BMM150_Trim->digZ3) * ((int32_t)(processCompZ0))) / 4;
  processCompZ2 = (((int32_t)(BMM150_Data->data_z_int - BMM150_Trim->digZ4)) * 32768);
  processCompZ3 = ((int32_t)BMM150_Trim->digZ1) * (((int16_t)BMM150_Data->data_rhall_uint) * 2);
  processCompZ4 = (int16_t)((processCompZ3 + (32768)) / 65536);
  retval = ((processCompZ2 - processCompZ1) / (BMM150_Trim->digZ2 + processCompZ4));
  return retval/32767.0*5000;
}
