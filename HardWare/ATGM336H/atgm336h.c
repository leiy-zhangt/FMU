#include "atgm336h.h"

uint8_t GPS_state = 0;//指示GPS状态，0为未接收信号，1为正常，其余为数据不可信
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
//  GPS_status = 0;
  USART2_Configuration(19200,ENABLE);
//  while(GPS_WaitReady()==ERROR);
//  GPS_status = 1;
//  while(GPS_init == 0) ;
//  printf("GPS is ready!\r\n");
//  USART3_printf("GPS is ready!\r\n");
//  LED_DIS;
}


void Coordinate2Position(void)
{
  MotionData.position_y = (GPS_Data.lat-GPS_Data.lat_init)*PI/180*R;
  MotionData.position_x = (GPS_Data.lon-GPS_Data.lon_init)*PI/180*R;
}

void NMEASolution(void)
{
  uint8_t GPS_state;//GPS状态标志位
  uint16_t message,flag,para;//参数
  int16_t dot;//小数点位置
  volatile double lat,lon,height;//经纬度
  volatile double velocity,degree;//速度
  float point;//小数点指示位
  for(message=0;message<800;message++)
  {
    if(USART2_RX_BUF[message]=='$')
    {
      if(USART2_RX_BUF[message+3]=='G'&&USART2_RX_BUF[message+4]=='G'&&USART2_RX_BUF[message+5]=='A')
      {
        flag=0;
        for(para=6;1;para++)
        {
          if(USART2_RX_BUF[message+para]==',')flag++;
          if(flag==2)//得到纬度
          {
            if(USART2_RX_BUF[message+para+1]==',') 
            {
              lat = 0;
              continue;
            }
            para++;
            lat = (USART2_RX_BUF[message+para++]-0x30)*10;
            lat += (USART2_RX_BUF[message+para++]-0x30);
            lat += (USART2_RX_BUF[message+para++]-0x30)/6.0;
            lat += (USART2_RX_BUF[message+para++]-0x30)/60.0;
            para++;
            point = 0.1f;
            while(1)
            {
              lat += (USART2_RX_BUF[message+para]-0x30)*point/60;
              if(USART2_RX_BUF[message+para+1]==',')break;
              para++;
              point = 0.1f*point;
            }
          }
          else if(flag==3)
          {
            if(USART2_RX_BUF[message+para+1]==',') 
            {
              continue;
            }
            para++;
            if(USART2_RX_BUF[message+para]=='N') GPS_Data.lat = lat;
            else if(USART2_RX_BUF[message+para]=='S') GPS_Data.lat = -lat;
          }
          else if(flag==4)//得到经度
          {
            if(USART2_RX_BUF[message+para+1]==',') 
            {
              lon = 0;
              continue;
            }
            para++;
            lon = (USART2_RX_BUF[message+para++]-0x30)*100;
            lon += (USART2_RX_BUF[message+para++]-0x30)*10;
            lon += (USART2_RX_BUF[message+para++]-0x30);
            lon += (USART2_RX_BUF[message+para++]-0x30)/6.0;
            lon += (USART2_RX_BUF[message+para++]-0x30)/60.0;
            para++;
            point = 0.1f;
            while(1)
            {
              lon += (USART2_RX_BUF[message+para]-0x30)*point/60;
              if(USART2_RX_BUF[message+para+1]==',')break;
              para++;
              point = 0.1f*point;
            }
          }
          else if(flag==5)
          {
            if(USART2_RX_BUF[message+para+1]==',') 
            {
              continue;
            }
            para++;
            if(USART2_RX_BUF[message+para]=='E') GPS_Data.lon = lon;
            else if(USART2_RX_BUF[message+para]=='W') GPS_Data.lon = -lon;
          }
          else if(flag==9)
          {
            if(USART2_RX_BUF[message+para+1]==',') 
            {
              height = 0;
              continue;
            }
            for(dot=0;;dot++) if(USART2_RX_BUF[message+para+dot+1]=='.') break;
            height=0;
            para++;
            while(1)
            {
              height=height+(USART2_RX_BUF[message+para]-0x30)*pow(10,dot-1);
              if(USART2_RX_BUF[message+para+1]=='.') para++;
              if(USART2_RX_BUF[message+para+1]==',')break;
              dot--;
              para++;
            }
          }
          else if(flag>9)
          {
            message+=para;
            break;
          }
        }
      }
      if(USART2_RX_BUF[message+3]=='R'&&USART2_RX_BUF[message+4]=='M'&&USART2_RX_BUF[message+5]=='C')
      {
        flag=0;
        for(para=6;1;para++)
        {
          if(USART2_RX_BUF[message+para]==',')flag++;
          if(flag==2)//状态检测
          {
            if(USART2_RX_BUF[message+para+1]==',') 
            {
              GPS_state = 0;
              continue;
            }
            para++;
            if(USART2_RX_BUF[message+para]=='V') GPS_state=0;
            else if(USART2_RX_BUF[message+para]=='A') GPS_state=1;
          }
          else if(flag==7)//得到速度
          {
            if(USART2_RX_BUF[message+para+1]==',') 
            {
              velocity = 0;
              continue;
            }
            for(dot=0;;dot++) if(USART2_RX_BUF[message+para+dot+1]=='.') break;
            velocity=0;
            para++;
            while(1)
            {
              velocity=velocity+(USART2_RX_BUF[message+para]-0x30)*pow(10,dot-1);
              if(USART2_RX_BUF[message+para+1]=='.') para++;
              if(USART2_RX_BUF[message+para+1]==',')
              {
                velocity*=0.514444;
                break;
              }
              dot--;
              para++;
            }
          }
          else if(flag==8)//得到航向角
          {
            if(USART2_RX_BUF[message+para+1]==',') 
            {
              degree = 0;
              continue;
            }
            for(dot=0;;dot++) if(USART2_RX_BUF[message+para+dot+1]=='.') break;
            degree=0;
            para++;
            while(1)
            {
              degree+=(USART2_RX_BUF[message+para]-0x30)*pow(10,dot-1);
              if(USART2_RX_BUF[message+para+1]=='.') para++;
              if(USART2_RX_BUF[message+para+1]==',')break;
              dot--;
              para++;
            }
          }
          else if(flag>9)
          {
            message+=para;
            break;
          }
        }
      }
    }
  }
  if(GPS_state==1)
  {
    GPS_Data.lat = lat;
    GPS_Data.lon = lon;
    GPS_Data.height = height;
    GPS_Data.velocity_n = velocity*cos(degree/57.3);
    GPS_Data.velocity_e = velocity*sin(degree/57.3);
  }
  else
  {
    GPS_Data.lat = 0;
    GPS_Data.lon = 0;
    GPS_Data.height = 0;
    GPS_Data.velocity_n = 0;
    GPS_Data.velocity_e = 0;
  }
}


