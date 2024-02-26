#ifndef __CONTROL_H
#define __CONTROL_H

#include "fatfs.h"
#include "cmsis_os.h"

extern SemaphoreHandle_t ControlSemaphore;//控制率计算二值信号量
extern BaseType_t ControlHigherTaskSwitch;

#endif
