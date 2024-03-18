#ifndef __CONTROL_H
#define __CONTROL_H

#include "fatfs.h"
#include "cmsis_os.h"

extern SemaphoreHandle_t ControlSemaphore;//控制率计算二值信号量
extern BaseType_t ControlHigherTaskSwitch;

typedef enum
{
	ServoChannel_1 = 0,
	ServoChannel_2,
	ServoChannel_3,
	ServoChannel_4,
	ServoChannel_5,
	ServoChannel_6,
	ServoChannel_7,
	ServoChannel_8,
}ServoChannel;

void ServoSet(ServoChannel channel,double angle);
void FixedWingControl(void);

#endif
