#ifndef __BUZZER_H
#define __BUZZER_H

#include "sys.h"

#define volume 80//蜂鸣器音量配置，范围为：0~999
#define BUZZER(state) TIM_Cmd(TIM4,state)

void BUZZER_Configuration(FunctionalState state);

#endif
