#ifndef __LED_H
#define __LED_H	 
#include "sys.h"
#include "delay.h" 

#define LED PAout(11) //PA11

#define LED_EN PAout(11)=0
#define LED_DIS PAout(11)=1

void LED_Configuration(void);	//LED初始化函数

#endif
