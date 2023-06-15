#include "dma.h"

void DMA_Configuration(void)
{
  DMA_InitTypeDef  DMA_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);//DMA1时钟使能 

  DMA_DeInit(DMA1_Stream2);
  while (DMA_GetCmdStatus(DMA1_Stream2) != DISABLE);//等待DMA可配置 
  /* 配置 DMA1 Stream2，USART4接收 */
  DMA_InitStructure.DMA_Channel            = DMA_Channel_4;               //通道选择
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&UART4->DR;            //DMA外设地址
  DMA_InitStructure.DMA_Memory0BaseAddr    = (u32)USART4_RX_BUF;      //DMA 存储器0地址
  DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralToMemory;  //外设到存储器模式
  DMA_InitStructure.DMA_BufferSize         = 11;       //数据传输量 
  DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;   //外设非增量模式
  DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;        //存储器增量模式
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //外设数据长度:8位
  DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;     //存储器数据长度:8位
  DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;             //使用普通模式 
  DMA_InitStructure.DMA_Priority           = DMA_Priority_Medium;         //中等优先级
  DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_1QuarterFull;
  DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;      //存储器突发单次传输
  DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;  //外设突发单次传输
  DMA_Init(DMA1_Stream2, &DMA_InitStructure);                             //初始化DMA Stream

  NVIC_InitStructure.NVIC_IRQChannel                   = DMA1_Stream2_IRQn ;//串口1发送中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;                 //抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;		          //子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd                = DISABLE;			  //IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);

  DMA_Cmd(DMA1_Stream2, ENABLE);

//  DMA_DeInit(DMA2_Stream7);
//  while (DMA_GetCmdStatus(DMA2_Stream7) != DISABLE);//等待DMA可配置 
//  /* 配置DMA2 Stream7，USART1发送 */
//  DMA_InitStructure.DMA_Channel            = DMA_Channel_4;               //通道选择
//  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART1->DR;            //DMA外设地址
//  DMA_InitStructure.DMA_Memory0BaseAddr    = (u32)DMA_USART1_TX_BUF;      //DMA 存储器0地址
//  DMA_InitStructure.DMA_DIR                = DMA_DIR_MemoryToPeripheral;  //存储器到外设模式
//  DMA_InitStructure.DMA_BufferSize         = DMA_USART1_TX_BUF_LEN;       //数据传输量 
//  DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;   //外设非增量模式
//  DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;        //存储器增量模式
//  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //外设数据长度:8位
//  DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;     //存储器数据长度:8位
//  DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;             //使用普通模式 
//  DMA_InitStructure.DMA_Priority           = DMA_Priority_Medium;         //中等优先级
//  DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable;         
//  DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_1QuarterFull;
//  DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;      //存储器突发单次传输
//  DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;  //外设突发单次传输
//  DMA_Init(DMA2_Stream7, &DMA_InitStructure);                             //初始化DMA Stream7

//  DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, ENABLE);							//DMA2传输完成中断
//  DMA_Cmd(DMA2_Stream7, DISABLE);											//不使能

}








