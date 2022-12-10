/*
数据记录仪V2.1对应代码
*/
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "spi.h"
#include "led.h" 
#include "serve.h"
#include "buzzer.h"
#include "bmm150.h"
#include "bmi088.h"
#include "adxl357.h"
#include "bmp388.h"
#include "fuse.h"
#include "w25q.h"
#include "lora.h"
#include "atgm336h.h"
#include "command.h"
#include "control.h"

int main(void)
{ 
  RCC_Configuration();
  NVIC_Configuration();
  FUSE_Configuration();
  SPI1_Configuration();
  LED_Configuration();
  BUZZER_Configuration(DISABLE);
  SERVE_Configuration(ENABLE);
  USART1_Configuration(512000,ENABLE);
  delay_ms(10);//等待芯片完成上电复位
  ADXL357_Configuration(ADXL_Range_10g);
  BMM150_Configuration();
  BMI088_Configuration(ACC_Range_3g,GYR_Range_250);
  BMP388_Configuration();
  W25Q_Configuration();
  ATGM336H_Configuration(ENABLE);
  LORA_Configuration(0x5252,38400);
  SampleFrequency_Configuration(Frequency_100Hz);
  MotionOffset_Get();
  delay_ms(100);
  printf("\r\nData Logger is ready!\r\n");
  LED_DIS;
  sample_state=1;
  while(1)
  {
    //测试代码开始
    
    //测试代码结束
    if(sample_state == 0)//执行采样后操作
    {
      LED_EN;
      switch(Command_State)
      {
        case 0:
          break;
        case BMI_START:
          printf("%+0.4f  %+0.4f  %+0.4f  %+0.4f  %+0.4f  %+0.4f\r\n",BMI088_Data.gyr_x,BMI088_Data.gyr_y,BMI088_Data.gyr_z,BMI088_Data.acc_x,BMI088_Data.acc_y,BMI088_Data.acc_z);
//         printf("%+0.4f\r\n",sample_time);
          sample_state = 1;
          break;
        case AttitudeSolution_TEST:
          AttitudeSolution(MotionData.gyr_x,MotionData.gyr_y,MotionData.gyr_z);
          AttitudeCompensation();
          printf("%+0.4f  %+0.4f  %+0.4f  %+0.4f\r\n",sample_time,MotionData.pitch*180/PI,MotionData.yaw*180/PI,MotionData.roll*180/PI);
//          printf("%+0.4f  %+0.4f  %+0.4f  %+0.4f\r\n",acc,MotionData.pitch*180/PI,MotionData.yaw*180/PI,MotionData.roll*180/PI);
          sample_state = 1;
          break;
        case AccelerationSolution_TEST:
          AttitudeSolution(MotionData.gyr_x,MotionData.gyr_y,MotionData.gyr_z);
//          AccelerationSolution(BMI088_Data.acc_x,BMI088_Data.acc_y,BMI088_Data.acc_z);
          AccelerationSolution(ADXL357_Data.acc_x,ADXL357_Data.acc_y,ADXL357_Data.acc_z);
          printf("%+0.4f  %+0.4f  %+0.4f  %+0.4f\r\n",sample_time,MotionData.acc_x,MotionData.acc_y,MotionData.acc_z);
          sample_state = 1;
          break;
        case VelocitySolution_TEST:
          AttitudeSolution(MotionData.gyr_x,MotionData.gyr_y,MotionData.gyr_z);
          AccelerationSolution(ADXL357_Data.acc_x,ADXL357_Data.acc_y,ADXL357_Data.acc_z);
          VelocitySolution();
          printf("%+0.4f  %+0.4f  %+0.4f  %+0.4f\r\n",sample_time,MotionData.velocity_x,MotionData.velocity_y,MotionData.velocity_z);
          sample_state = 1;
          break;
        case PositionSolution_TEST:
          AttitudeSolution(MotionData.gyr_x,MotionData.gyr_y,MotionData.gyr_z);
          AccelerationSolution(ADXL357_Data.acc_x,ADXL357_Data.acc_y,ADXL357_Data.acc_z);
          VelocitySolution();
          PositionSolution();
          printf("%+0.4f  %+0.4f  %+0.4f  %+0.4f\r\n",sample_time,MotionData.position_x,MotionData.position_y,MotionData.position_z);
          sample_state = 1;
          break;
        case DATASTORAGE:
          AttitudeSolution(MotionData.gyr_x,MotionData.gyr_y,MotionData.gyr_z);
          AttitudeCompensation();
          guidace(MotionData);
          if(sample_number%10 == 0)DataStorage();
          sample_state = 1;
          break;
      }
      LED_DIS;
    }
  }
}


