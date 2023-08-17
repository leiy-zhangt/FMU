#include "control.h"

uint8_t ControlMode;
double control_pitch,control_roll,control_yaw,yaw_init,roll_e,pitch_e;

void Parafoil_Control(void)
{
  
}

//单通道稳定控制
//void FixdWing_Control(void)
//{
//  const double Kp=1,Kd=0.1,Kp_pitch=3,Kd_pitch=0.3;
//  double serve,roll_e,serve_pitch,pitch_e;
//  if(RemoteChannel[4]>1500)
//  {
//    ControlMode=0;
//    Serve_1_Set(RemoteChannel[0]);
//    Serve_2_Set(3000-RemoteChannel[1]);
//    Serve_3_Set(RemoteChannel[2]);
//    Serve_4_Set(3000-RemoteChannel[3]);
//  }
//  else
//  {
//    if(ControlMode==0)
//    {
//      ControlMode=1;
//    }
//    roll_e = (RemoteChannel[0]-1500)/500.0*30;
//    pitch_e = (RemoteChannel[1]-1500)/500.0*45;
//    serve_pitch = -Kp_pitch*(pitch_e-MotionData.pitch*57.3)-Kd_pitch*MotionData.gyr_x;
//    serve_pitch = serve_pitch>45?45:serve_pitch;
//    serve_pitch = serve_pitch<-45?-45:serve_pitch;
//    serve = Kp*(roll_e-MotionData.roll*57.3)-Kd*MotionData.gyr_y;
//    serve = serve>30?30:serve;
//    serve = serve<-30?-30:serve;
//    Serve_1_Set(serve/30.0*333+1500);
//    Serve_2_Set(serve_pitch/45.0*500+1500);
//    Serve_3_Set(RemoteChannel[2]);
//    Serve_4_Set(3000-RemoteChannel[3]);
//  }
//}

//双通道稳定控制

void FixdWing_Control(void)
{
  static const double Kp_roll=1,Kd_roll=0.1,Kp_pitch=1,Kd_pitch=0,Kp_yaw=1.5,Kd_yaw=0.1;
  static double serve_roll,serve_pitch,serve_yaw,yaw_init;
  uint16_t channel[5];
  for(uint8_t i=0;i<5;i++)channel[i]=RemoteChannel[i];
//  Serve_1_Set(RemoteChannel[0]);
//  Serve_2_Set(RemoteChannel[1]);
//  Serve_3_Set(RemoteChannel[2]);
//  Serve_4_Set(RemoteChannel[3]);
//  printf("%u %u %u %u %u\r\n",RemoteChannel[0],RemoteChannel[1],RemoteChannel[2],RemoteChannel[3],RemoteChannel[4]);
  if(channel[4]>1500)
  {
    ControlMode=0;
    ServeOutput[0]=channel[0];
    ServeOutput[1]=3000-channel[1];
    ServeOutput[2]=channel[2];
    ServeOutput[3]=3000-channel[3];
  }
  else
  {
    if(ControlMode == 0)
    {
      ControlMode=1;
      yaw_init=MotionData.yaw;
    }
    roll_e = (channel[0]-1500)/500.0*30;
    serve_roll = Kp_roll*(roll_e-MotionData.roll*57.3)-Kd_roll*MotionData.gyr_y;
    serve_roll = serve_roll>30?30:serve_roll;
    serve_roll = serve_roll<-30?-30:serve_roll;
    pitch_e = (channel[1]-1500)/500.0*30;
    serve_pitch = -Kp_pitch*(pitch_e-MotionData.pitch*57.3)-Kd_pitch*MotionData.gyr_x;
    serve_pitch = serve_pitch>60?60:serve_pitch;
    serve_pitch = serve_pitch<-60?-60:serve_pitch;
    yaw_init = yaw_init+(channel[3]-1500)/99999.0f;
    yaw_init = yaw_init>PI?yaw_init-PI:yaw_init;
    yaw_init = yaw_init<-PI?yaw_init+PI:yaw_init;
    serve_yaw = Kp_yaw*(AngleDifference(yaw_init,MotionData.yaw)*57.3)+Kd_yaw*MotionData.gyr_z;
    serve_yaw = serve_yaw>30?30:serve_yaw;
    serve_yaw = serve_yaw<-30?-30:serve_yaw;
    control_pitch=serve_pitch;
    control_roll=serve_roll;
    control_yaw=serve_yaw;
    
    ServeOutput[0]=serve_roll/30.0*333+1500;
    ServeOutput[1]=serve_pitch/60.0*666+1500;
    ServeOutput[2]=channel[2];
    ServeOutput[3]=3000-channel[3];
  }
  Serve_1_Set(ServeOutput[0]);
  Serve_2_Set(ServeOutput[1]);
  Serve_3_Set(ServeOutput[2]);
  Serve_4_Set(ServeOutput[3]);
  if(sample_number%20 == 0) DataStorage();
}

float AngleDifference(float angle_1,float angle_2)
{
  float angle=angle_1-angle_2;
  if(angle>PI) return angle-PI;
  else if(angle<-PI) return angle+PI;
  else return angle;
}



void Serve_1_Set(uint16_t angle)
{
  static int16_t angle_offset = 0;
  TIM_SetCompare1(TIM3,angle+angle_offset);
}

void Serve_2_Set(uint16_t angle)
{
  static int16_t angle_offset = 0;
  TIM_SetCompare2(TIM3,angle+angle_offset);
}

void Serve_3_Set(uint16_t angle)
{
  static int16_t angle_offset = 0;
  TIM_SetCompare3(TIM3,angle+angle_offset);
}

void Serve_4_Set(uint16_t angle)
{
  static int16_t angle_offset = 0;
  TIM_SetCompare4(TIM3,angle+angle_offset);
}

