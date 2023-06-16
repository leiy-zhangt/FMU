#ifndef __DMA_H
#define __DMA_H

#include "sys.h" 
#include "usart.h"

extern DMA_InitTypeDef  DMA_USART4_InitStructure;

void DMA_Configuration(void);//配置DMA初始化设置
void DMA_USART4_Configuration(void);//配置DMA通道
  
#endif
