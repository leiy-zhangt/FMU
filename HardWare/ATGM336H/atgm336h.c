#include "atgm336h.h"

void ATGM336H_Configuration(void)
{
//    GPIO_InitTypeDef  GPIO_InitStructure;
//    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//使能GPIOC时钟
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14; 
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
//    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
//    GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化
//    
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
//    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
//    GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化
//    
//    ATGM336H_PWR = 1;
//    ATGM336H_RST = 0;
//    delay_ms(50);
//    ATGM336H_RST = 1;
//    USART_MODE = Command_Receice;
//    ATGM336H_PWR = 0;
}
