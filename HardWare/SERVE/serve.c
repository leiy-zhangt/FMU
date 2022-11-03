#include "serve.h"

void SERVE_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE); 
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;           
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;       
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;    
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;      
	GPIO_Init(GPIOB,&GPIO_InitStructure);  
  SERVE_PWR = 1;
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;           
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;       
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;    
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;      
  GPIO_Init(GPIOC,&GPIO_InitStructure); 
  
  GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_TIM3);
  GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_TIM3);
  GPIO_PinAFConfig(GPIOC,GPIO_PinSource8,GPIO_AF_TIM3);
  GPIO_PinAFConfig(GPIOC,GPIO_PinSource9,GPIO_AF_TIM3);
  
  TIM_TimeBaseStructure.TIM_Prescaler=2099;  
  TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; 
  TIM_TimeBaseStructure.TIM_Period=9999;   
  TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 	
  TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);
 
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 
  TIM_OC1Init(TIM3, &TIM_OCInitStructure); 
  TIM_OC2Init(TIM3, &TIM_OCInitStructure); 
  TIM_OC3Init(TIM3, &TIM_OCInitStructure); 
  TIM_OC4Init(TIM3, &TIM_OCInitStructure);  
  
  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable); 
  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable); 
  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);   
  TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable); 
  TIM_ARRPreloadConfig(TIM3,ENABLE);	
  TIM_Cmd(TIM3, ENABLE);  
  
  TIM_SetCompare1(TIM3,1500);
  TIM_SetCompare2(TIM3,1500);
  TIM_SetCompare3(TIM3,1500);
  TIM_SetCompare4(TIM3,1500);
}
