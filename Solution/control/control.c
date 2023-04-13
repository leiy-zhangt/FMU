#include "control.h"

void Parafoil_Control(void)
{
  double IPI_x = -10;         //目标点位置
  double IPI_y = 100;
  double ptheta,vtheta,theta;
  static double angle[3];
  ptheta = atan2(IPI_y-MotionData.position_y,IPI_x-MotionData.position_x);
	vtheta = atan2(GPS_Data.velocity_n,GPS_Data.velocity_e);
	theta = vtheta-ptheta;
  angle[0] = ptheta;
  angle[1] = vtheta;
  if(theta < 0)
  {
    MotionData.serve[0] = -theta*57.3*0.87;
    MotionData.serve[1] = 0;
	  
  }
  else
  {
    MotionData.serve[0] = 0;
    MotionData.serve[1] = theta*57.3*0.87;
  }
  if(sample_number%100==0) 
  {
    USART_printf("s1:%+0.4f  s2v:%+0.4f  t:%+0.4f\r\n",MotionData.serve[0],MotionData.serve[1],theta*57.3);
  }
  MotionData.serve[0] = MotionData.serve[0]>20?20:MotionData.serve[0];
  MotionData.serve[1] = MotionData.serve[1]>20?20:MotionData.serve[1];
  Serve_1_Set(MotionData.serve[0]);
  Serve_2_Set(MotionData.serve[1]);
}

void Serve_1_Set(double angle)
{
  static double angle_offset = 5;
  angle = -angle + angle_offset;
  TIM_SetCompare1(TIM3,angle/90.0*1000+1500);
}

void Serve_2_Set(double angle)
{
  static double angle_offset = -15;
  angle = angle + angle_offset;
  TIM_SetCompare2(TIM3,angle/90.0*1000+1500);
}

void Serve_3_Set(double angle)
{
  static double angle_offset = 0;
  angle = -angle + angle_offset;
  TIM_SetCompare3(TIM3,angle/90.0*1000+1500);
}

void Serve_4_Set(double angle)
{
  static double angle_offset = 0;
  angle = angle + angle_offset;
  TIM_SetCompare4(TIM3,angle/90.0*1000+1500);
}

