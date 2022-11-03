#include "fuse.h"

void FUSE_Configuration(void)
{
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//����ʱ��
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//����ʱ��
  GPIO_InitTypeDef  GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = FUSE1_Pin|FUSE2_Pin; //��������
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//����Ϊ���
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//��©���
  GPIO_InitStructure.GPIO_Speed = GPIO_Fast_Speed;//�������
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//����
  GPIO_Init(FUSE_Port, &GPIO_InitStructure);//��ʼ��
  FUSE1 = 0;
  FUSE2 = 0;
  
  GPIO_InitStructure.GPIO_Pin = TRIGGER_Pin; //��������
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//����Ϊ���
  GPIO_InitStructure.GPIO_Speed = GPIO_Fast_Speed;//�������
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(TRIGGER_Port, &GPIO_InitStructure);//��ʼ��
}
