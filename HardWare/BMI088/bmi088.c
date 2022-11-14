#include "bmi088.h"

const float g = 9.80665;//重力加速度
int16_t ACC_Range,GYR_Range;//惯导量程选择
BMI088_DataStruct BMI088_Data;//BMI088数据结构体

void BMI088_Configuration(ACC_Range_Choose acc_range,GYR_Range_Choose gyr_range)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  
  BMI088_ReadData(ACC_Choose,0x00);
  BMI088_WriteData(ACC_Choose,0x7E,0xB6);//复位加速度计
  BMI088_WriteData(ACC_Choose,0x14,0xB6);//复位陀螺仪
  delay_ms(1);
  BMI088_ReadData(ACC_Choose,0x00);
  BMI088_WriteData(ACC_Choose,0x7C,0x00);//加速度计退出待机模式
  delay_ms(1);
  BMI088_WriteData(ACC_Choose,0x40,0xA9); //配置加速度计输出频率为200Hz
  BMI088_WriteData(ACC_Choose,0x41,acc_range); //配置加速度计输出频率为200Hz
  BMI088_WriteData(ACC_Choose,0x7D,0x04);//打开加速度计 
  BMI088_WriteData(GYR_Choose,0x0F,gyr_range);//陀螺仪计量程选择
  BMI088_WriteData(GYR_Choose,0x10,ODR_200Hz_65BD);//陀螺仪输出速率设置 
  //配置中断引脚
  GPIO_InitStructure.GPIO_Pin = ACC_INT_Pin|GYR_INT_Pin; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
    
  if(acc_range == ACC_Range_3g) ACC_Range = 6;
  else if(acc_range == ACC_Range_6g) ACC_Range = 12;
  else if(acc_range == ACC_Range_12g) ACC_Range = 24;
  else if(acc_range == ACC_Range_24g) ACC_Range = 48;
  //
  if(gyr_range == GYR_Range_125) GYR_Range = 250;
  else if(gyr_range == GYR_Range_250) GYR_Range = 500;
  else if(gyr_range == GYR_Range_500) GYR_Range = 1000;
  else if(gyr_range == GYR_Range_1000) GYR_Range = 2000;
  else if(gyr_range == GYR_Range_2000) GYR_Range = 4000;  
}



void BMI088_WriteData(IMU_Choose IMU,u8 addr,u8 data)
{
  if(IMU == ACC_Choose) ACC_CS = 0;
  else  GYR_CS = 0;
  delay_us(1);
  SPI_ReadWriteByte(SPI1,addr&0x7F);
  SPI_ReadWriteByte(SPI1,data);
  if(IMU == ACC_Choose) ACC_CS = 1;
  else  GYR_CS = 1;
  delay_us(10);
}

void BMI088_ReadBuffer(IMU_Choose IMU,u8 addr,u8* buffer,u8 length)
{
  if(IMU == ACC_Choose) ACC_CS = 0;
  else  GYR_CS = 0;
  delay_us(1);
  SPI_ReadWriteByte(SPI1,addr|0x80);
  if(IMU == ACC_Choose) SPI_ReadWriteByte(SPI1,0x00);
  for(;length>0;length--)
  {
      *buffer = SPI_ReadWriteByte(SPI1,0x00);
      buffer++;
  }
  if(IMU == ACC_Choose) ACC_CS = 1;
  else  GYR_CS = 1;
  delay_us(1);
}

uint8_t BMI088_ReadData(IMU_Choose IMU,u8 addr)
{
  static uint8_t res;
  if(IMU == ACC_Choose) ACC_CS = 0;
  else  GYR_CS = 0;
  SPI_ReadWriteByte(SPI1,addr|0x80);
  if(IMU == ACC_Choose) SPI_ReadWriteByte(SPI1,0x00);
  res = SPI_ReadWriteByte(SPI1,0x00);
  if(IMU == ACC_Choose) ACC_CS = 1;
  else  GYR_CS = 1;
  delay_us(1);
  return res;
}

void BMI088_Measure(BMI088_DataStruct *BMI088_Data)
{
  BMI088_ReadBuffer(ACC_Choose,0x12,BMI088_Data->buffer,6);
  BMI088_Data->acc_x_int = (((int16_t)BMI088_Data->buffer[1])<<8)|BMI088_Data->buffer[0];
  BMI088_Data->acc_y_int = (((int16_t)BMI088_Data->buffer[3])<<8)|BMI088_Data->buffer[2];
  BMI088_Data->acc_z_int = (((int16_t)BMI088_Data->buffer[5])<<8)|BMI088_Data->buffer[4];
  BMI088_ReadBuffer(ACC_Choose,0x22,BMI088_Data->buffer,2);
  BMI088_Data->tem_int = (((int16_t)(BMI088_Data->buffer[0])<<8)|(BMI088_Data->buffer[1]))>>5;
  BMI088_Data->tem = BMI088_Data->tem_int * 0.125f + 23;
  BMI088_ReadBuffer(GYR_Choose,0x02,BMI088_Data->buffer,6);
  BMI088_Data->gyr_x_int = (((int16_t)BMI088_Data->buffer[1])<<8)|BMI088_Data->buffer[0];
  BMI088_Data->gyr_y_int = (((int16_t)BMI088_Data->buffer[3])<<8)|BMI088_Data->buffer[2];
  BMI088_Data->gyr_z_int = (((int16_t)BMI088_Data->buffer[5])<<8)|BMI088_Data->buffer[4];
  BMI088_Data->acc_x = BMI088_Data->acc_x_int / 65535.0 * ACC_Range * g - MotionOffset.acc_x_offset;
  BMI088_Data->acc_y = BMI088_Data->acc_y_int / 65535.0 * ACC_Range * g - MotionOffset.acc_y_offset;
  BMI088_Data->acc_z = BMI088_Data->acc_z_int / 65535.0 * ACC_Range * g - MotionOffset.acc_z_offset;
  BMI088_Data->gyr_x = BMI088_Data->gyr_x_int / 65535.0 * GYR_Range - MotionOffset.gyr_x_offset;
  BMI088_Data->gyr_y = BMI088_Data->gyr_y_int / 65535.0 * GYR_Range - MotionOffset.gyr_y_offset;
  BMI088_Data->gyr_z = BMI088_Data->gyr_z_int / 65535.0 * GYR_Range - MotionOffset.gyr_z_offset;
}

