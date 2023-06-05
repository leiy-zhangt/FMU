#include "control.h"

void Parafoil_Control(void)
{
  
}

//单通道稳定控制
void FixdWing_Control(void)
{
  static const double Kp=1,Kd=0.1,Kp_pitch=3,Kd_pitch=0.3;
  double serve,roll_e,serve_pitch,pitch_e;
  if(RemoteChannle[4]>1500)
  {
    Serve_1_Set(RemoteChannle[0]);
    Serve_2_Set(3000-RemoteChannle[1]);
    Serve_3_Set(RemoteChannle[2]);
    Serve_4_Set(3000-RemoteChannle[3]);
  }
  else
  {
    roll_e = (RemoteChannle[0]-1500)/500.0*30;
    pitch_e = (RemoteChannle[1]-1500)/500.0*45;
    serve_pitch = -Kp_pitch*(pitch_e-MotionData.pitch*57.3)-Kd_pitch*MotionData.gyr_x;
    serve_pitch = serve_pitch>45?45:serve_pitch;
    serve_pitch = serve_pitch<-45?-45:serve_pitch;
    serve = Kp*(roll_e-MotionData.roll*57.3)-Kd*MotionData.gyr_y;
    serve = serve>30?30:serve;
    serve = serve<-30?-30:serve;
    Serve_1_Set(serve/30.0*333+1500);
    Serve_2_Set(serve_pitch/45.0*500+1500);
    Serve_3_Set(RemoteChannle[2]);
    Serve_4_Set(3000-RemoteChannle[3]);
  }
}

//双通道稳定控制

//void FixdWing_Control(void)
//{
//  static const double Kp_roll=1,Kd_roll=0.1,Kp_pitch=1.5,Kd_pitch=0.1;
//  double serve_roll,roll_e,serve_pitch,pitch_e;
//  if(RemoteChannle[4]>1500)
//  {
//    Serve_1_Set(RemoteChannle[0]);
//    Serve_2_Set(3000-RemoteChannle[1]);
//    Serve_3_Set(RemoteChannle[2]);
//    Serve_4_Set(3000-RemoteChannle[3]);
//  }
//  else
//  {
//    roll_e = (RemoteChannle[0]-1500)/500*30;
//    serve_roll = Kp_roll*(roll_e-MotionData.roll*57.3)-Kd_roll*MotionData.gyr_y;
//    serve_roll = serve_roll>30?30:serve_roll;
//    serve_roll = serve_roll<-30?-30:serve_roll;
//    pitch_e = (RemoteChannle[1]-1500)/500*30;
//    serve_pitch = -Kp_pitch*(pitch_e-MotionData.pitch*57.3)-Kd_pitch*MotionData.gyr_x;
//    serve_pitch = serve_pitch>30?30:serve_pitch;
//    serve_pitch = serve_pitch<-30?-30:serve_pitch;
//    Serve_1_Set(serve_roll/30.0*333+1500);
//    Serve_2_Set(serve_pitch/30.0*333+1500);
//    Serve_3_Set(RemoteChannle[2]);
//    Serve_4_Set(RemoteChannle[3]);
//  }
//}





void Serve_1_Set(uint16_t angle)
{
  static uint16_t angle_offset = 0;
  TIM_SetCompare1(TIM3,angle+angle_offset);
}

void Serve_2_Set(uint16_t angle)
{
  static int16_t angle_offset = 200;
  TIM_SetCompare2(TIM3,angle+angle_offset);
}

void Serve_3_Set(uint16_t angle)
{
  static uint16_t angle_offset = 0;
  TIM_SetCompare3(TIM3,angle+angle_offset);
}

void Serve_4_Set(uint16_t angle)
{
  static int16_t angle_offset = -100;
  TIM_SetCompare4(TIM3,angle+angle_offset);
}

