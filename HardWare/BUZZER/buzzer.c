#include "buzzer.h"

void BUZZER_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
	//开启时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 	//使能PORTB时钟	
    //配置舵机使能引脚
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;           
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;       
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;    
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;      
	GPIO_Init(GPIOA,&GPIO_InitStructure);  
    BUZZER = 0;
}
