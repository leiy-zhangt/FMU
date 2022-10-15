#include "delay.h"
				   
void delay_us(uint32_t us)
{
    SysTick->VAL=0X00;
    SysTick->LOAD = 21*us-1;
    SysTick->CTRL |= SysTick_CTRL_ENABLE_MASK;
    while(1)
    {
        if(((SysTick->CTRL)&(1<<16))) break;
    }
    SysTick->CTRL &= SysTick_CTRL_DISABLE_MASK;
}

void delay_ms(uint16_t ms)
{
    for(;ms!=0;ms--) delay_us(1000);
}


































