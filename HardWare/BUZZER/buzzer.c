#include "buzzer.h"

void BUZZER_Configuration(FunctionalState state)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
  
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;       
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;    
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;     
	GPIO_Init(GPIOB,&GPIO_InitStructure);  
  
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource8,GPIO_AF_TIM4);
  
  TIM_TimeBaseStructure.TIM_Prescaler=209;  
  TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; 
  TIM_TimeBaseStructure.TIM_Period=999;   
  TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 	
  TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);
 
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 
  TIM_OC3Init(TIM4, &TIM_OCInitStructure);  
  
  TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);   
  TIM_ARRPreloadConfig(TIM4,ENABLE);	
  TIM_Cmd(TIM4,state);  
  
  TIM_SetCompare3(TIM4,volume);
}
