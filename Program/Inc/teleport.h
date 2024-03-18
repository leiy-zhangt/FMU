#ifndef __TELEPORT_H
#define __TELEPORT_H

#include "main.h"


#define TeleHandle huart8

typedef enum
{
	DebugChannel = 1,
	TeleChannel
}PrintChannelSelect;

extern PrintChannelSelect PrintChannel;

void DebugPrint(PrintChannelSelect PrintChannel,char * info);

#endif
