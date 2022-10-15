#ifndef __DELAY_H
#define __DELAY_H 			   
#include <sys.h>	  

#define SysTick_CTRL_ENABLE_MASK 0X01
#define SysTick_CTRL_DISABLE_MASK 0X00000000

void delay_ms(uint16_t nms); //延时nms毫秒，最大时间为65535ms
void delay_us(uint32_t nus); //延时nus纳秒，最大时间为4294967295us

#endif





























