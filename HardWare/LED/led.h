#ifndef __LED_H
#define __LED_H	 
#include "sys.h"
#include "delay.h" 

#define LED PAout(11) //PA11

void LED_Configuration(void);	//LED初始化函数

#endif
