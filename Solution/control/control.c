#include "control.h"

double ze_p;

void Control(void) 
{
  double U1,U2,U3,U4;//旋翼螺旋桨参数，代表升力、力矩
  double Kpz,Kiz,Kdz,Kpp,Kip,Kdp,Kpr,Kir,Pdr;//控制器系数
  double ze,ze_i,ze_d;
  ze = 1.5 - MotionData.height;
  ze_i += ze;
  ze_d = ze - ze_p;
  ze_p = ze;
  U1 = (Kpz*ze+Kiz*ze_i+Kdz*ze_d+0.75*g)/(cos(MotionData.roll)*cos(MotionData.pitch));
}

void Serve_1_Set(double angle)
{
  static double angle_offset = 0;
  angle = angle - angle_offset;
  angle>90?90:angle;
  MotionData.serve[0] = angle;
  TIM_SetCompare1(TIM3,angle/90.0*1000+1500);
}

void Serve_2_Set(double angle)
{
  static double angle_offset = 0;
  angle = angle - angle_offset;
  angle>90?90:angle;
  angle<-90?-90:angle;
  MotionData.serve[1] = angle;
  TIM_SetCompare2(TIM3,angle/90.0*1000+1500);
}

void Serve_3_Set(double angle)
{
  static double angle_offset = 0;
  angle = angle - angle_offset;
  angle>90?90:angle;
  angle<-90?-90:angle;
  MotionData.serve[1] = angle;
  TIM_SetCompare2(TIM3,angle/90.0*1000+1500);
}

void Serve_4_Set(double angle)
{
  static double angle_offset = 0;
  angle = angle - angle_offset;
  angle>90?90:angle;
  angle<-90?-90:angle;
  MotionData.serve[1] = angle;
  TIM_SetCompare2(TIM3,angle/90.0*1000+1500);
}
