#include "atgm336h.h"

void ATGM336H_Configuration(FunctionalState ATGM_State)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//ʹ��GPIOAʱ��
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//ʹ��GPIOCʱ��
  
  USART2_Configuration(9600,DISABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);//��ʼ��
  
  ATGM336H_NRST = 1;
  if(ATGM_State == ENABLE)ATGM336H_PWR = 1;
  else ATGM336H_PWR = 0;
}
