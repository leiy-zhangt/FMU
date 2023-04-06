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
  SERVE_Configuration(DISABLE);
  USART1_Configuration(512000,ENABLE);
  delay_ms(10);//等待芯片完成上电复位
  ADXL357_Configuration(ADXL_Range_10g);
  BMM150_Configuration();
  BMI088_Configuration(ACC_Range_3g,GYR_Range_250);
  BMP388_Configuration();
  W25Q_Configuration();
  LORA_Configuration(0x1234,38400);
//  ATGM336H_Configuration(ENABLE);
  SampleFrequency_Configuration(Frequency_100Hz);
  FMUOffset_Get();
  delay_ms(100);
  printf("\r\nData Logger is ready!\r\n");
  USART3_printf("\r\nData Logger is ready!\r\n");
  USART4_Configuration(100000,ENABLE);//配置遥控器接收
  LED_DIS;
  sample_state=0;
  while(1)
  {
    //测试代码开始
//    double pitch,roll,yaw,Xh,Yh;
//    LED_EN;
//    ADXL357_Measure(&ADXL357_Data);
//    BMM150_Measure(&BMM150_Data);
//    pitch = asin(ADXL357_Data.acc_y/MotionOffset.g_position);
//    roll = atan2(-ADXL357_Data.acc_x,ADXL357_Data.acc_z);
//    Xh= BMM150_Data.data_y*cos(roll)-BMM150_Data.data_z*sin(roll);
//    Yh= BMM150_Data.data_y*sin(roll)*sin(pitch)+BMM150_Data.data_x*cos(pitch)+BMM150_Data.data_z*sin(pitch)*cos(roll);
//    yaw = atan2(Xh,Yh);
//    printf("pitch=%+0.4f,roll=%+0.4f,yaw=%+0.4f\r\n",pitch*57.3,roll*57.3,yaw*57.3);
//    printf("%+0.4f  %+0.4f  %+0.4f\r\n",BMM150_Data.data_x,BMM150_Data.data_y,BMM150_Data.data_z);
//    LED_DIS;
//    delay_ms(100);
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
        case AttitudeSolution_TEST:
          AttitudeSolution(MotionData.gyr_x,MotionData.gyr_y,MotionData.gyr_z);
          AttitudeCompensation();
        if(sample_number%10 == 0) printf("pitch:%+0.4f yaw:%+0.4f roll:%+0.4f  %+0.4f\r\n",MotionData.pitch*180/PI,MotionData.yaw*180/PI,MotionData.roll*180/PI);
//          printf("pitch=%+0.4f,yaw=%+0.4f,roll=%+0.4f\r\n",MotionData.pitch*180/PI,MotionData.yaw*180/PI,MotionData.roll*180/PI);
          break;
        case AccelerationSolution_TEST:
          AttitudeSolution(MotionData.gyr_x,MotionData.gyr_y,MotionData.gyr_z);
//          AccelerationSolution(BMI088_Data.acc_x,BMI088_Data.acc_y,BMI088_Data.acc_z);
          AccelerationSolution(ADXL357_Data.acc_x,ADXL357_Data.acc_y,ADXL357_Data.acc_z);
          printf("%+0.4f  %+0.4f  %+0.4f  %+0.4f\r\n",sample_time,MotionData.acc_x,MotionData.acc_y,MotionData.acc_z);
          break;
        case VelocitySolution_TEST:
          AttitudeSolution(MotionData.gyr_x,MotionData.gyr_y,MotionData.gyr_z);
          AccelerationSolution(ADXL357_Data.acc_x,ADXL357_Data.acc_y,ADXL357_Data.acc_z);
          VelocitySolution();
          printf("%+0.4f  %+0.4f  %+0.4f  %+0.4f\r\n",sample_time,MotionData.velocity_x,MotionData.velocity_y,MotionData.velocity_z);
          break;
        case PositionSolution_TEST:
          AttitudeSolution(MotionData.gyr_x,MotionData.gyr_y,MotionData.gyr_z);
          AccelerationSolution(ADXL357_Data.acc_x,ADXL357_Data.acc_y,ADXL357_Data.acc_z);
          VelocitySolution();
          PositionSolution();
          printf("%+0.4f  %+0.4f  %+0.4f  %+0.4f\r\n",sample_time,MotionData.position_x,MotionData.position_y,MotionData.position_z);
          break;
        case Data_STORAGE:
          AttitudeSolution(MotionData.gyr_x,MotionData.gyr_y,MotionData.gyr_z);
          AttitudeCompensation();
          if(sample_number%50==0) 
          {
            DataStorage();
            USART3_printf("height:%0.2f\r\n",MotionData.height);
          }
          break;
        case Height_TEST:
          if(sample_number%10 == 0) USART3_printf("pre=%0.4f,height=%0.4f\r\n",MotionData.pressure,MotionData.height);
          break;
        case Control_START:
          AttitudeSolution(MotionData.gyr_x,MotionData.gyr_y,MotionData.gyr_z);
          Control();
          if(sample_number%20 == 0) 
          {
//            USART3_printf("height=%0.2f,f1=%0.2f,f2=%0.2f,f3=%0.2f,f4=%0.2f\r\n",MotionData.height,f1,f2,f3,f4);
//            USART3_printf("pitch=%0.2f,roll=%0.2f,yaw=%0.2f\r\n",MotionData.pitch*57.3,MotionData.roll*57.3,MotionData.yaw*57.3);
            USART3_printf("height=%0.2f,U1=%0.2f,U2=%0.2f,U3=%0.2f\r\n",MotionData.height,U1,U2,U3);
          }
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


