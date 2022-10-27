#include "led.h"


void LED_Configuration(void)
{
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//开启时钟
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//开启时钟
  GPIO_InitTypeDef  GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11; //配置引脚
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//配置为输出
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;//开漏输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Low_Speed;//慢速输出
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_1; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  PCout(0) = 1;
  PCout(1) = 1;
  PCout(12) = 1;
  LED = 1;
} 




















