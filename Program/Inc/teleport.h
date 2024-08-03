#ifndef __TELEPORT_H
#define __TELEPORT_H

#include "main.h"
#include "cmsis_os.h"

extern uint8_t TeleReceiveBuff[];

extern UART_HandleTypeDef *	TeleHandle;

extern SemaphoreHandle_t TeleSemaphore;
extern BaseType_t TeleHigherTaskSwitch;
//接收内容拼接处理
extern uint16_t TeleRecLen;
extern uint8_t *TeleRecAddr;
extern double TeleReceverData[];

typedef enum
{
	DebugChannel = 1,
	TeleChannel
}PrintChannelSelect;

extern PrintChannelSelect PrintChannel;

extern uint8_t SendBuff[];

void InfoPrint(PrintChannelSelect PrintChannel,char * info);

#endif
