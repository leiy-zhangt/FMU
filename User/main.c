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

int main(void)
{ 
  RCC_Configuration();
  NVIC_Configuration();
  FUSE_Configuration();
  SPI1_Configuration();
  LED_Configuration();
  BUZZER_Configuration(DISABLE);
  SERVE_Configuration(DISABLE);
  USART1_Configuration(115200,ENABLE);
  delay_ms(10);//等待芯片完成上电复位
  ADXL357_Configuration(ADXL_Range_10g);
  BMM150_Configuration();
  BMI088_Configuration(ACC_Range_3g,GYR_Range_500);
  BMP388_Configuration();
  W25Q_Configuration();
  ATGM336H_Configuration(DISABLE);
  LORA_Configuration(0x5252,38400);
  SampleFrequency_Configuration(Frequency_100Hz);
  MotionOffset_Get();
  delay_ms(100);
  printf("\r\nData Logger is ready!\r\n");
  while(1)
  {
    if(sample_state == 0)
    {
      if(Command_State == AttitudeSolution_TEST)
      {
        AttitudeSolution();
        printf("%0.4f  %0.4f  %0.4f  %0.4f\r\n",sample_time,MotionData.pitch*180/PI,MotionData.yaw*180/PI,MotionData.roll*180/PI);
      }
    }
  }
}

