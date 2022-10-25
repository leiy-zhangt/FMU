#include "bmi055.h"

const float g = 9.80665f;//重力加速度
int16_t ACC_Range,GYR_Range;//惯导量程选择
float dt;//积分时间步长
BMI055_DataStruct BMI055_Data;//BMI055数据结构体

void BMI055_Configuration(ACC_Range_Choose acc_range,GYR_Range_Choose gyr_range)
{
  volatile u8 res;
  GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  //配置芯片片选引脚
  GPIO_InitStructure.GPIO_Pin = ACC_CS_Pin|GYR_CS_Pin; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  ACC_CS = 1;
  GYR_CS = 1;
  
  res = BMI055_ReadData(ACC_Choose,0x00);
  res = BMI055_ReadData(ACC_Choose,0x00);
  BMI055_WriteData(ACC_Choose,0x0F,acc_range);//加速度计量程选择
  BMI055_WriteData(ACC_Choose,0x10,0x0D); //加速度计带宽为250Hz
  BMI055_WriteData(GYR_Choose,0x0F,gyr_range);//陀螺仪计量程选择
  BMI055_WriteData(GYR_Choose,0x10,ODR_200Hz_65BD);//陀螺仪输出速率设置 
  //配置中断引脚
  GPIO_InitStructure.GPIO_Pin = ACC_INT_Pin|GYR_INT_Pin; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
    
  if(acc_range == ACC_Range_2g) ACC_Range = 4;
  else if(acc_range == ACC_Range_4g) ACC_Range = 8;
  else if(acc_range == ACC_Range_8g) ACC_Range = 16;
  else if(acc_range == ACC_Range_16g) ACC_Range = 32;
  //
  if(gyr_range == GYR_Range_125) GYR_Range = 250;
  else if(gyr_range == GYR_Range_250) GYR_Range = 500;
  else if(gyr_range == GYR_Range_500) GYR_Range = 1000;
  else if(gyr_range == GYR_Range_1000) GYR_Range = 2000;
  else if(gyr_range == GYR_Range_2000) GYR_Range = 4000;  
}



void BMI055_WriteData(IMU_Choose IMU,u8 addr,u8 data)
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

void BMI055_ReadBuffer(IMU_Choose IMU,u8 addr,u8* buffer,u8 length)
{
  if(IMU == ACC_Choose) ACC_CS = 0;
  else  GYR_CS = 0;
  delay_us(1);
  SPI_ReadWriteByte(SPI1,addr|0x80);
  for(;length>0;length--)
  {
      *buffer = SPI_ReadWriteByte(SPI1,0x00);
      buffer++;
  }
  if(IMU == ACC_Choose) ACC_CS = 1;
  else  GYR_CS = 1;
  delay_us(1);
}

uint8_t BMI055_ReadData(IMU_Choose IMU,u8 addr)
{
  static uint8_t res;
  if(IMU == ACC_Choose) ACC_CS = 0;
  else  GYR_CS = 0;
  SPI_ReadWriteByte(SPI1,addr|0x80);
  res = SPI_ReadWriteByte(SPI1,0x00);
  if(IMU == ACC_Choose) ACC_CS = 1;
  else  GYR_CS = 1;
  delay_us(1);
  return res;
}

void BMI055_Measure(BMI055_DataStruct *BMI055_Data)
{
  BMI055_ReadBuffer(ACC_Choose,0x02,BMI055_Data->buffer,7);
  BMI055_Data->acc_x_int = (((int16_t)BMI055_Data->buffer[1])<<8)|BMI055_Data->buffer[0];
  BMI055_Data->acc_y_int = (((int16_t)BMI055_Data->buffer[3])<<8)|BMI055_Data->buffer[2];
  BMI055_Data->acc_z_int = (((int16_t)BMI055_Data->buffer[5])<<8)|BMI055_Data->buffer[4];
  BMI055_Data->tem = (int8_t)(BMI055_Data->buffer[6]) + 23;
  BMI055_ReadBuffer(GYR_Choose,0x02,BMI055_Data->buffer,6);
  BMI055_Data->gyr_x_int = (((int16_t)BMI055_Data->buffer[1])<<8)|BMI055_Data->buffer[0];
  BMI055_Data->gyr_y_int = (((int16_t)BMI055_Data->buffer[3])<<8)|BMI055_Data->buffer[2];
  BMI055_Data->gyr_z_int = (((int16_t)BMI055_Data->buffer[5])<<8)|BMI055_Data->buffer[4];
  BMI055_Data->acc_x = (BMI055_Data->acc_x_int>>4) / 4095.0 * ACC_Range * g;
  BMI055_Data->acc_y = (BMI055_Data->acc_y_int>>4) / 4095.0 * ACC_Range * g;
  BMI055_Data->acc_z = (BMI055_Data->acc_z_int>>4) / 4095.0 * ACC_Range * g;
  BMI055_Data->gyr_x = BMI055_Data->gyr_x_int / 65535.0 * GYR_Range;
  BMI055_Data->gyr_y = BMI055_Data->gyr_y_int / 65535.0 * GYR_Range;
  BMI055_Data->gyr_z = BMI055_Data->gyr_z_int / 65535.0 * GYR_Range;
}

