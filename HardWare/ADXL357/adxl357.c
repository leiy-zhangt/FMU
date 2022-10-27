#include "adxl357.h"

int8_t ADXL_Range;
ADXL357_DataStruct ADXL357_Data;

void ADXL357_Configuration(ADXL_Range_Choose adxl_range)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  
  //配置中断引脚
  GPIO_InitStructure.GPIO_Pin = ADXL_INT_Pin; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
    
  //配置ADXL357工作状况
  ADXL357_WriteData(0x28,0x03);//配置输出速率为500Hz,低通滤波器为250Hz
  ADXL357_WriteData(0x2C,adxl_range);//配置加速度计量程
  ADXL357_WriteData(0x2D,0x00);//加速度计进入测量模式
  //初始化量程
  if(adxl_range == ADXL_Range_10g) ADXL_Range = 20;
  else if(adxl_range == ADXL_Range_20g) ADXL_Range = 40;
  else if(adxl_range == ADXL_Range_40g) ADXL_Range = 80;
}



void ADXL357_WriteData(u8 addr,u8 data)
{
  ADXL_CS = 0;
  delay_us(1);
  SPI_ReadWriteByte(SPI1,addr<<1);
  SPI_ReadWriteByte(SPI1,data);
  ADXL_CS = 1;
  delay_us(1);
}

void ADXL357_ReadBuffer(u8 addr,u8* buffer,u8 length)
{
  ADXL_CS = 0;
  delay_us(1);
  SPI_ReadWriteByte(SPI1,(addr<<1)|0x01);
  for(;length>0;length--)
  {
      *buffer = SPI_ReadWriteByte(SPI1,0x00);
      buffer++;
  }
  ADXL_CS = 1;
  delay_us(1);
}

uint8_t ADXL357_ReadData(u8 addr)
{
  static uint8_t res;
  ADXL_CS = 0;
  delay_us(1);
  SPI_ReadWriteByte(SPI1,(addr<<1)|0x01);
  res = SPI_ReadWriteByte(SPI1,0x00);
  delay_us(1);
  ADXL_CS = 1;
  delay_us(1);
  return res;
}

void ADXL357_Measure(ADXL357_DataStruct *ADXL357_Data)
{
  ADXL357_ReadBuffer(0x06,ADXL357_Data->buffer,11);
  ADXL357_Data->tem_int = ((((int16_t)ADXL357_Data->buffer[0])<<12)|(((int16_t)ADXL357_Data->buffer[1])<<4))>>4 ;
  ADXL357_Data->tem = (1885 - ADXL357_Data->tem_int)/9.05f + 25;
  ADXL357_Data->acc_x_int = ((((int32_t)ADXL357_Data->buffer[2])<<24)|(((int32_t)ADXL357_Data->buffer[3])<<16)|(((int32_t)ADXL357_Data->buffer[4])<<8))>>12;
  ADXL357_Data->acc_y_int = ((((int32_t)ADXL357_Data->buffer[5])<<24)|(((int32_t)ADXL357_Data->buffer[6])<<16)|(((int32_t)ADXL357_Data->buffer[7])<<8))>>12;
  ADXL357_Data->acc_z_int = ((((int32_t)ADXL357_Data->buffer[8])<<24)|(((int32_t)ADXL357_Data->buffer[9])<<16)|(((int32_t)ADXL357_Data->buffer[10])<<8))>>12;
  ADXL357_Data->acc_x = (ADXL357_Data->acc_x_int) / 1048575.0 * ADXL_Range * g;
  ADXL357_Data->acc_y = (ADXL357_Data->acc_y_int) / 1048575.0 * ADXL_Range * g;
  ADXL357_Data->acc_z = (ADXL357_Data->acc_z_int) / 1048575.0 * ADXL_Range * g;
}

