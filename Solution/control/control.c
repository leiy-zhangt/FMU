#include "control.h"

//void norm(vector)
//{
//	int a = sqrt(pow(vector(0), 2) + pow(vector(1), 2) + pow(vector(2), 2));
//	return a;
//}

void guidace(MotionDataStruct MotionData)      //制导函数
{
	int IPI_x = 5000;         //目标点位置
	int IPI_y = 300;
  Sreve_1_Set(MotionData.pitch);
  Sreve_2_Set(MotionData.roll);
}

void Sreve_1_Set(double angle)
{
  static double angle_offset = 0;
  angle = angle - angle_offset;
  angle>90?90:angle;
  angle<-90?-90:angle;
  MotionData.serve[0] = angle;
  TIM_SetCompare1(TIM3,angle/90.0*1000+1500);
}

void Sreve_2_Set(double angle)
{
  static double angle_offset = 0;
  angle = angle - angle_offset;
  angle>90?90:angle;
  angle<-90?-90:angle;
  MotionData.serve[1] = angle;
  TIM_SetCompare2(TIM3,angle/90.0*1000+1500);
}