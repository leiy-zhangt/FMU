#include "teleport.h"
#include "string.h"
#include "printf.h"
#include "stdio.h"

PrintChannelSelect PrintChannel=DebugChannel;

void DebugPrint(PrintChannelSelect PrintChannel,char * info)//调试接口数据输出
{
	if(PrintChannel==DebugChannel)
	{
		printf("%s",info);
	}
	else if(PrintChannel==TeleChannel)
	{
		HAL_UART_Transmit_DMA(&TeleHandle,info,strlen(info));
	}
}
