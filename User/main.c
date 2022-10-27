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
  delay_ms(100);
    while(1)
    {
      ADXL357_Measure(&ADXL357_Data);
      printf("%0.4f  %0.4f  %0.4f  %0.4f\r\n",ADXL357_Data.tem,ADXL357_Data.acc_x,ADXL357_Data.acc_y,ADXL357_Data.acc_z);
      delay_ms(100);
    }
}

