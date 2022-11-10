/*
指令分区：
0~9：BMI088模块
10~19：ADXL357模块
20~29:BMP388模块
30~39:BMI150模块
40~49:W25Q模块
50~59:Lora模块
60~69:GPS模块
70~99：测试部分
100~：工作部分
*/
#include "command.h"

uint8_t Command_State = 0;

void Command_Receive(uint8_t *buffer)
{
  if(strcmp(buffer,"BMI_START") == 0) Command_State = BMI_START;
  else if(strcmp(buffer,"BMI_STOP") == 0) Command_State = BMI_STOP;
  else if(strcmp(buffer,"AttitudeSolution_TEST") == 0) {Command_State = AttitudeSolution_TEST;AttitudeSolution_Ttst();}
  else if(strcmp(buffer,"Sample_STOP") == 0) {Command_State = Sample_STOP;}
  else Command_State = 0;
}

void AttitudeSolution_Ttst(void)
{
  q[0] = 1;
  q[1] = 0;
  q[2] = 0;
  q[3] = 0;
  Sample_Start();
}

void Sample_Start(void)
{
  sample_time = 0;
  TIM_ClearFlag(TIM2, TIM_FLAG_Update);
  TIM_SetCounter(TIM2,0);
  TIM_Cmd(TIM2,ENABLE);
}

