#include "control.h"

double ze_p,pe_p,re_p;
double ze,ze_i,ze_d,pe,pe_i,pe_d,re,re_i,re_d;
double U1,U2,U3,f1,f2,f3,f4;//旋翼螺旋桨参数，代表升力、力矩
const double Kpz=4,Kiz=0,Kdz=0.2,Kpp=1,Kip=0.5,Kdp=00,Kpr=1,Kir=0.5,Kdr=0.5;//控制器系数
void Control(void) 
{
  //高度回路控制
  ze = 1 - MotionData.height;
  ze_i += ze*dt;
  ze_d = (ze - ze_p)/dt;
  ze_p = ze;
  U1 = (Kpz*ze+Kiz*ze_i+Kdz*ze_d+1*g)/(cos(MotionData.roll)*cos(MotionData.pitch));
  //俯仰控制回路
  pe = -MotionData.pitch;
  pe_i += pe*dt;
  pe_d = (pe - pe_p)/dt;
  pe_p = pe;
  U2 = Kpp*pe + Kip*pe_i + Kdp*pe_d;
  //滚转控制回路
  re = -MotionData.roll;
  re_i += re*dt;
  re_d = (re - re_p)/dt;
  re_p = re;
  U3 = Kpr*re + Kir*re_i + Kdr*re_d;
  //分配升力
  f1 = U1/4 + U2/4 - U3/4;
  f2 = U1/4 + U2/4 + U3/4;
  f3 = U1/4 - U2/4 + U3/4;
  f4 = U1/4 - U2/4 - U3/4;
  //求出偏角
  Serve_1_Set(f1);
  Serve_2_Set(f3);
  Serve_3_Set(f3);
  Serve_4_Set(f4);
}

void Serve_1_Set(double f)
{
  double serve;
  f = f<0?0:f;
  f = f>10?10:f;
  serve = sqrt(f/16);
  TIM_SetCompare1(TIM3,serve*1000+1000);
}

void Serve_2_Set(double f)
{
  double serve;
  f = f<0?0:f;
  f = f>10?10:f;
  serve = sqrt(f/16);
  TIM_SetCompare2(TIM3,serve*1000+1000);
}

void Serve_3_Set(double f)
{
  double serve;
  f = f<0?0:f;
  f = f>10?10:f;
  serve = sqrt(f/16);
  TIM_SetCompare3(TIM3,serve*1000+1000);
}

void Serve_4_Set(double f)
{
  double serve;
  f = f<0?0:f;
  f = f>10?10:f;
  serve = sqrt(f/16);
  TIM_SetCompare4(TIM3,serve*1000+1000);
}
