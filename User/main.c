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
#include "bmi055.h"

int main(void)
{ 
  RCC_Configuration();
  NVIC_Configuration();
  SPI1_Configuration();
  LED_Configuration();
  BUZZER_Configuration();
  SERVE_Configuration();
  USART1_Configuration(115200,DISABLE);
  delay_ms(1);//等待芯片完成上电复位
  BMM150_Configuration();
  BMI055_Configuration(ACC_Range_2g,GYR_Range_500);
    while(1)
    {
      BMI055_Measure(&BMI055_Data);
      printf("%0.4f\r\n",BMI055_Data.gyr_x);
      delay_ms(10);
    }
}

