/*
数据记录仪V2.1对应代码
*/
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "spi.h"
#include "led.h"  

int main(void)
{ 
  RCC_Configuration();
  NVIC_Configuration();
  SPI1_Configuration();
  //    delay_ms(1000);
  //    W25N_Configuration();
  LED_Init();
  //    SERVE_Configution(DISABLE);
  //    BUZZER_Configuration();
  //    BMP388_Configuration();
  ////    QMC5883L_Configuration();
  //    ATGM336H_Configuration(); 
  //    printf("DATA LOGGER has read\r\n");
  //    LED = 1;
    while(1)
    {
      LED = 1;
      delay_ms(1000);
      LED = 0;
      delay_ms(1000);
    }
}

