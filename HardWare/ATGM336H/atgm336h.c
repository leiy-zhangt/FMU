#include "atgm336h.h"

uint8_t GPS_status = 0;//指示GPS状态，0为未接收信号，1为正常，其余为数据不可信
uint8_t GPS_init = 0;
GPS_DataStruct GPS_Data;

void ATGM336H_Configuration(FunctionalState ATGM_State)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  
  USART2_Configuration(9600,DISABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOC, &GPIO_InitStructure); 
  
  ATGM336H_NRST = 0;
  delay_ms(500);
  if(ATGM_State == ENABLE)  ATGM336H_PWR = 1;
  else ATGM336H_PWR = 0;
  delay_ms(10);
  ATGM336H_NRST = 1;
  delay_ms(500);
  USART2_printf("\r\n\r\n");
  delay_ms(1000);
  USART2_printf("$PCAS02,500*1A\r\n");
  delay_ms(1000);
  USART2_printf("$PCAS01,2*1E\r\n");
  delay_ms(1000);
  
  LED_EN;
  GPS_status = 0;
  USART2_Configuration(19200,ENABLE);
  while(GPS_WaitReady()==ERROR);
  GPS_status = 1;
  while(GPS_init == 0) ;
  printf("GPS is ready!\r\n");
  LED_DIS;
}

ErrorStatus GPS_WaitReady(void)
{
  uint8_t i,j=0;
  while((USART2_RX_STA&0x8000) == 0);
  USART2_RX_STA = 0;
  if(USART2_RX_BUF[0] == 0x24)
  {
    if(((USART2_RX_BUF[1]=='G')&&(USART2_RX_BUF[2]=='L')&&(USART2_RX_BUF[3]=='L'))||((USART2_RX_BUF[3]=='G')&&(USART2_RX_BUF[4]=='L')&&(USART2_RX_BUF[5]=='L')))
    {
      for(i=1;i<200;i++)
      {
        if(USART2_RX_BUF[i]==',')
        {
          j++;
          if(j==6)
          {
            printf("GPS is prepare!\r\n");
            if(USART2_RX_BUF[i+1]=='A') return SUCCESS;
            else return ERROR;
          }
        }
      }
    }
  }
  return ERROR;
}

void Coordinate2Position(void)
{
  MotionData.position_y = (GPS_Data.lat-GPS_Data.lat_init)*PI/180*R;
  MotionData.position_x = (GPS_Data.lon-GPS_Data.lon_init)*PI/180*R;
}
