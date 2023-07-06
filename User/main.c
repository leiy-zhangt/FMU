/*
数据记录仪V2.1对应代码
*/
#include "sys.h"
#include "dma.h"
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
  DMA_Configuration();
  FUSE_Configuration();
  SPI1_Configuration();
  LED_Configuration();
  BUZZER_Configuration(DISABLE);
  SERVE_Configuration(ENABLE);
  USART1_Configuration(1500000,ENABLE);
  delay_ms(10);//等待芯片完成上电复位
  ADXL357_Configuration(ADXL_Range_10g);
  BMM150_Configuration();
  BMI088_Configuration(ACC_Range_3g,GYR_Range_250);
  BMP388_Configuration();
  W25Q_Configuration();
  LORA_Configuration(0x1234,38400);
  ATGM336H_Configuration(ENABLE);
  SampleFrequency_Configuration(Frequency_200Hz);
  FMUOffset_Get();
  delay_ms(100);
  printf("\r\nData Logger is ready!\r\n");
  USART3_printf("\r\nData Logger is ready!\r\n");
  USART4_Configuration(1000000,ENABLE);//配置遥控器接收
  LED_DIS;
  USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);//开启空闲中断
  while(1)
  {
    //测试代码开始
//    printf("%u %u %u %u %u\r\n",RemoteChannel[0],RemoteChannel[1],RemoteChannel[2],RemoteChannel[3],RemoteChannel[4]);
//    delay_ms(1000);
//    for(uint16_t i;i<500;i++)USART_SendData(USART2,USART2_RX_BUF[i]);
    //测试代码结束
    
    if(sample_state == 0)//执行采样后操作
    {
//      LED_EN;
      sample_state = Command_State;
      switch(Command_State)
      {
        case 0:
          break;
        case BMI_TEST:
          printf("%+0.4f  %+0.4f  %+0.4f  %+0.4f  %+0.4f  %+0.4f\r\n",BMI088_Data.gyr_x,BMI088_Data.gyr_y,BMI088_Data.gyr_z,BMI088_Data.acc_x,BMI088_Data.acc_y,BMI088_Data.acc_z);
          break;
        case ADXL_TEST:
           if(sample_number%50==0) printf("%+0.4f  %+0.4f  %+0.4f\r\n",ADXL357_Data.acc_x,ADXL357_Data.acc_y,ADXL357_Data.acc_z);
          break;
        case BMM_TEST:
          if(sample_number%50==0) printf("%+0.4f  %+0.4f  %+0.4f\r\n",BMM150_Data.data_x,BMM150_Data.data_y,BMM150_Data.data_z);
          break;
        case AttitudeSolution_TEST:
          AttitudeSolution(MotionData.gyr_x,MotionData.gyr_y,MotionData.gyr_z);
          if(sample_number%40 == 0) USART_printf("%0.2f pitch:%+0.4f yaw:%+0.4f roll:%+0.4f\r\n",sample_time,MotionData.pitch*180/PI,MotionData.yaw*180/PI,MotionData.roll*180/PI);
          break;
        case AttitudeCompensation_TEST:
          AttitudeSolution(MotionData.gyr_x,MotionData.gyr_y,MotionData.gyr_z);
          AttitudeCompensation();
          if(sample_number%50 == 0) 
            {
              USART_printf("pitch:%+0.4f yaw:%+0.4f roll:%+0.4f\r\n",MotionData.pitch*180/PI,MotionData.yaw*180/PI,MotionData.roll*180/PI);
            }
          break;
        case AccelerationSolution_TEST:
          AttitudeSolution(MotionData.gyr_x,MotionData.gyr_y,MotionData.gyr_z);
//          AccelerationSolution(BMI088_Data.acc_x,BMI088_Data.acc_y,BMI088_Data.acc_z);
          AccelerationSolution(ADXL357_Data.acc_x,ADXL357_Data.acc_y,ADXL357_Data.acc_z);
          USART_printf("%+0.4f  %+0.4f  %+0.4f\r\n",MotionData.acc_x,MotionData.acc_y,MotionData.acc_z);
          break;
        case VelocitySolution_TEST:
          AttitudeSolution(MotionData.gyr_x,MotionData.gyr_y,MotionData.gyr_z);
          AccelerationSolution(ADXL357_Data.acc_x,ADXL357_Data.acc_y,ADXL357_Data.acc_z);
          VelocitySolution();
          USART_printf("%+0.4f  %+0.4f  %+0.4f  %+0.4f\r\n",sample_time,MotionData.velocity_x,MotionData.velocity_y,MotionData.velocity_z);
          break;
        case PositionSolution_TEST:
          AttitudeSolution(MotionData.gyr_x,MotionData.gyr_y,MotionData.gyr_z);
          AccelerationSolution(ADXL357_Data.acc_x,ADXL357_Data.acc_y,ADXL357_Data.acc_z);
          VelocitySolution();
          PositionSolution();
          USART_printf("%+0.4f  %+0.4f  %+0.4f  %+0.4f\r\n",sample_time,MotionData.position_x,MotionData.position_y,MotionData.position_z);
          break;
        case Data_STORAGE:
          AttitudeSolution(MotionData.gyr_x,MotionData.gyr_y,MotionData.gyr_z);
          AttitudeCompensation();
          if(sample_number%50==0) 
          {
            DataStorage();
            USART_printf("height:%0.2f\r\n",MotionData.height);
          }
          break;
        case Height_TEST:
          if(sample_number%10 == 0) USART_printf("pre=%0.4f,height=%0.4f\r\n",MotionData.pressure,MotionData.height);
          break;
        case ParafoilControl_START:
          Parafoil_Control();
          break;
        case FixdWingControl_TEST:
          AttitudeSolution(MotionData.gyr_x,MotionData.gyr_y,MotionData.gyr_z);
          AttitudeCompensation();
          FixdWing_Control();
          break;
      }
//      LED_DIS;
    }
    switch(Command_State)
    {
      case IMUUpOffset:
        IMUOffset_Init();
        break;
      case MagnetismOffset_INIT:
        MagnetismOffset_Init();
        break;
    }
  }
}


