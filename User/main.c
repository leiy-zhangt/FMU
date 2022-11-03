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

double acc_max = 0,velocity = 0;

int main(void)
{ 
  RCC_Configuration();
  NVIC_Configuration();
  FUSE_Configuration();
  SPI1_Configuration();
  LED_Configuration();
  BUZZER_Configuration();
  SERVE_Configuration();
  USART1_Configuration(115200,DISABLE);
  delay_ms(10);//等待芯片完成上电复位
  ADXL357_Configuration(ADXL_Range_10g);
  BMM150_Configuration();
  BMI088_Configuration(ACC_Range_3g,GYR_Range_500);
  BMP388_Configuration();
  W25Q_Configuration();
  ATGM336H_Configuration(DISABLE);
  LORA_Configuration(0x5252,38400);
  delay_ms(100);
  acc_max = acc_max/50;
  while(1)
  {
//    BMP388_TemperatureGet(&BMP388_Data);
//    printf("height:%0.6f  pressure:%0.6f  temperature:%0.6f\r\n",BMP388_HeightGet(),BMP388_Data.pre,BMP388_Data.tem);
    printf("height:%0.6f  pressure:%0.6f\r\n",BMP388_HeightGet(),BMP388_Data.pre);
    LED =! LED;
    delay_ms(100);
  }
}

