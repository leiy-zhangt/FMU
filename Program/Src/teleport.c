#include "teleport.h"
#include "string.h"
#include "printf.h"
#include "stdio.h"

//Teloport接收缓存数组
uint8_t TeleReceiveBuff[1024];
//Teloport接收端口
UART_HandleTypeDef *TeleHandle = &huart8;
//Teloport发送模式
PrintChannelSelect PrintChannel = TeleChannel;
//Teloport中断切换标志
SemaphoreHandle_t TeleSemaphore;
BaseType_t TeleHigherTaskSwitch;
//接收内容拼接处理
uint16_t TeleRecLen;
uint8_t *TeleRecAddr;
double TeleReceverData[80];

uint8_t SendBuff[1024];

void InfoPrint(PrintChannelSelect PrintChannel,char * info)//调试接口数据输出
{
	if(PrintChannel==DebugChannel)
	{
		printf("%s",info);
//		HAL_UART_Transmit_DMA(&huart1,info,strlen(info));
	}
	else if(PrintChannel==TeleChannel)
	{
		HAL_UART_Transmit_DMA(TeleHandle,info,strlen(info));
	}
}
