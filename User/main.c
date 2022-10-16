/*
数据记录仪V2.1对应代码
*/
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "spi.h"
#include "led.h"  
#include "bmm150.h"

int main(void)
{ 
  RCC_Configuration();
  NVIC_Configuration();
  SPI1_Configuration();
  //    delay_ms(1000);
  //    W25N_Configuration();
  LED_Configuration();
  BMM150_Configuration();
  USART1_Configuration(115200,DISABLE);
  //    SERVE_Configution(DISABLE);
  //    BUZZER_Configuration();
  //    BMP388_Configuration();
  ////    QMC5883L_Configuration();
  //    ATGM336H_Configuration(); 
  //    printf("DATA LOGGER has read\r\n");
  //    LED = 1;
//  BMM150_SendData(0x4c,);
    while(1)
    {
//      BMM150_SendData(0x4C,0x2B);
      BMM150_MeasureGet(&BMM150_Data);
      printf("x:%0.4f   y:%0.4f   z:%0.4f\r\n",BMM150_Data.data_x,BMM150_Data.data_y,BMM150_Data.data_z);
//      BMM150_Trim_Get(&BMM150_Trim);
//      BMM150_ReadBuffer(0x4E,buffer,1);
//      printf("%c\r\n",res);
      LED = !LED;
      delay_ms(500);
    }
}

