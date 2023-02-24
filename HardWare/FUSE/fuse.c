#include "fuse.h"

void FUSE_Configuration(void)
{
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//开启时钟
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//开启时钟
  GPIO_InitTypeDef  GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = FUSE1_Pin|FUSE2_Pin; //配置引脚
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//配置为输出
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//开漏输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Fast_Speed;//慢速输出
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//上拉
  GPIO_Init(FUSE_Port, &GPIO_InitStructure);//初始化
  FUSE1 = 0;
  FUSE2 = 0;
  
  GPIO_InitStructure.GPIO_Pin = TRIGGER_Pin; //配置引脚
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//配置为输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Fast_Speed;//慢速输出
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(TRIGGER_Port, &GPIO_InitStructure);//初始化
}
