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

int main(void)
{ 
  RCC_Configuration();
  NVIC_Configuration();
  SPI1_Configuration();
  LED_Configuration();
  BUZZER_Configuration();
  SERVE_Configuration();
  USART1_Configuration(115200,DISABLE);
  delay_ms(100);//等待芯片完成上电复位
  ADXL357_Configuration(ADXL_Range_10g);
  BMM150_Configuration();
  BMI088_Configuration(ACC_Range_3g,GYR_Range_500);
  BMP388_Configuration();
  delay_ms(100);
    while(1)
    {
      double height;
      height = BMP388_HeightGet();
//      BMP388_PressureGet(&BMP388_Data);
//      printf("%0.4f\r\n",BMP388_Data.pre);
      printf("%0.4f\r\n",height);
      delay_ms(100);
    }
}

