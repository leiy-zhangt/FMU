#include "control.h"

void Parafoil_Control(void)
{
  
}

void FixdWing_Control(void)
{
  static const double Kp=0.75,Kd=0.1;
  double serve;
  if(RemoteChannle[4]>1500)
  {
    Serve_1_Set(RemoteChannle[0]);
    Serve_2_Set(RemoteChannle[1]);
    Serve_3_Set(RemoteChannle[2]);
    Serve_4_Set(RemoteChannle[3]);
  }
  else
  {
    serve = -Kp*MotionData.roll*57.3-Kd*MotionData.gyr_y;
    serve = serve>45?45:serve;
    serve = serve<-45?-45:serve;
    Serve_1_Set(serve/45.0*500+1500);
    Serve_2_Set(RemoteChannle[1]);
    Serve_3_Set(RemoteChannle[2]);
    Serve_4_Set(RemoteChannle[3]);
  }
}

void Serve_1_Set(uint16_t angle)
{
  static uint16_t angle_offset = 0;
  TIM_SetCompare1(TIM3,angle+angle_offset);
}

void Serve_2_Set(uint16_t angle)
{
  static uint16_t angle_offset = 0;
  TIM_SetCompare2(TIM3,angle+angle_offset);
}

void Serve_3_Set(uint16_t angle)
{
  static uint16_t angle_offset = 0;
  TIM_SetCompare3(TIM3,angle+angle_offset);
}

void Serve_4_Set(uint16_t angle)
{
  static uint16_t angle_offset = 0;
  TIM_SetCompare4(TIM3,angle+angle_offset);
}

