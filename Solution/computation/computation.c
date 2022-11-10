#include "computation.h"

uint8_t sample_state;
float dt;//采样时间间隔
MotionDataStruct MotionData;
double sample_time = 0;
double dq[4],q[4];

void SampleFrequency_Configuration(SampleFrequency frequency)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  NVIC_InitTypeDef NVIC_InitStructure; 
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE); 
  
  TIM_TimeBaseStructure.TIM_Prescaler=81;  
  TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; 
  TIM_TimeBaseStructure.TIM_Period=frequency;   
  TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 	
  TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);

	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;//配置优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStructure);

  TIM_ClearFlag(TIM2, TIM_FLAG_Update);
  TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
  TIM_Cmd(TIM2,DISABLE);
  
  if(frequency == Frequency_1Hz) dt = 1;
  else if(frequency == Frequency_10Hz) dt = 0.1;
  else if(frequency == Frequency_50Hz) dt = 0.02;
  else if(frequency == Frequency_100Hz) dt = 0.01;
  else if(frequency == Frequency_200Hz) dt = 0.005;
}

void TIM2_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)  
	{ 
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    BMI088_Measure(&BMI088_Data);
    ADXL357_Measure(&ADXL357_Data);
    MotionData.acc_x = ADXL357_Data.acc_x;
    MotionData.acc_y = ADXL357_Data.acc_y;
    MotionData.acc_z = ADXL357_Data.acc_z;
    MotionData.gyr_x = BMI088_Data.gyr_x;
    MotionData.gyr_y = BMI088_Data.gyr_y;
    MotionData.gyr_z = BMI088_Data.gyr_z;
    MotionData.height = BMP388_HeightGet();
    MotionData.pressure = BMP388_Data.pre;
    sample_state = 0;
    sample_time += dt;
    LED = !LED;
	}
}

void AttitudeSolution(void)
{
  double w[3],q_norm,pitch,roll,yaw;
  w[0] = MotionData.gyr_x * PI /180;
  w[1] = MotionData.gyr_y * PI /180;
  w[2] = MotionData.gyr_z * PI /180;
  dq[0] = 0.5*(-w[1]*q[1]-w[0]*q[2]-w[2]*q[3]);
  dq[1] = 0.5*(w[0]*q[0]+w[2]*q[2]-w[1]*q[3]);
  dq[2] = 0.5*(w[1]*q[0]-w[2]*q[1]+w[0]*q[3]);
  dq[3] = 0.5*(w[2]*q[0]+w[1]*q[1]-w[0]*q[2]);
  q[0] = q[0] + dq[0]*dt;
  q[1] = q[1] + dq[1]*dt;
  q[2] = q[2] + dq[2]*dt;
  q[3] = q[3] + dq[3]*dt;
  q_norm = q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3];
  q_norm = sqrt(q_norm);
  q[0] = q[0]/q_norm;
  q[1] = q[1]/q_norm;
  q[2] = q[2]/q_norm;
  q[3] = q[3]/q_norm;
  pitch = asin(2*(q[0]*q[1]+q[2]*q[3]));//初始矩阵俯仰角
  yaw = atan2(2*(q[1]*q[2]-q[0]*q[3]),(pow(q[0],2)-pow(q[1],2)+pow(q[2],2)-pow(q[3],2)));//初始矩阵偏航角
  roll = atan2(-2*(q[1]*q[3]+q[0]*q[2]),pow(q[0],2)-pow(q[1],2)-pow(q[2],2)+pow(q[3],2));//初始矩阵滚转角
  printf("%0.4f  %0.4f  %0.4f  %0.4f\r\n",sample_time,pitch*180/PI,yaw*180/PI,roll*180/PI);
  sample_state = 1;
}

