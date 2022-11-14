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

W25Q存储分区：
第0块：用于存放系统参数
  第0扇区：0~71Byte存放传感器稳态偏移量，72以后待定。
  第1扇区及以后，存放已经保存的数据量及页数。

*/
#include "command.h"

uint8_t Command_State = 0;

void Command_Receive(uint8_t *buffer)
{
  if(strcmp(buffer,"BMI_START") == 0) Command_State = BMI_START;
  else if(strcmp(buffer,"BMI_STOP") == 0) Command_State = BMI_STOP;
  else if(strcmp(buffer,"AttitudeSolution_TEST") == 0) {Command_State = AttitudeSolution_TEST;AttitudeSolution_Ttst();}
  else if(strcmp(buffer,"Sample_STOP") == 0) {Command_State = Sample_STOP;}
  else if(strcmp(buffer,"MotionOffset_Init") == 0) MotionOffset_Init();
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

void W25Q_DataConsult(void)
{
  uint32_t *addr;
  W25Q_DataReceive(0x1000,W25Q_buffer,256);
  addr = W25Q_buffer;
  for(uint8_t i = 0;i<64;i++)
  {
    if(&addr != 0xFFFFFFFF) W25Q_DataAddress[i] = &addr;
    else 
    {
      printf("%c dates have been stored!\r\n",i);
    }
    addr++;
  }
}
