#ifndef __FUSE_H
#define __FUSE_H

#include "sys.h"

#define FUSE_Port GPIOB
#define FUSE1_Pin GPIO_Pin_3
#define FUSE2_Pin GPIO_Pin_4

#define FUSE1 PBout(3)
#define FUSE2 PBout(4)

#define TRIGGER_Port GPIOA
#define TRIGGER_Pin GPIO_Pin_15

#define TRIGGER PAout(15)

void FUSE_Configuration(void);

#endif
