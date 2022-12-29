#include "control.h"

// void norm(vector)
// {
// 	int a = sqrt(pow(vector(0), 2) + pow(vector(1), 2) + pow(vector(2), 2));
// 	return a;
// }

// void guidace(MotionData)      //制导函数
// {
//  int IPI_x = 5000;         //目标点位置
//  int IPI_y = 300;
//  vector<int> xlv;          //从GPS获得的速度向量
//  xlv = (MotionData.velocity_x, MotionData.velocity_y);
//  vector<int> xlwz;        //从GPS获得位置，求解翼伞到目标点水平面向量
//  xlwz = (IPI_x - MotionData.position_x, IPI_y - MotionData.position_y);
//  int fx = MotionData.position_y / abs(yf);   //偏差方向
//  int pianhangwc = fx*arccos(xlv.dot(xlwz)/(norm(xlv)*norm(xlwz)));
//  int daleta = 20 * pianhangwc / 57.3;
//  return daleta;
// }

void guidace(MotionDataStruct MotionData)      //制导函数
{
  double IPI_x = 50;         //目标点位置
  double IPI_y = 1000;
  double theta = asin(abs(MotionData.position_x - IPI_x)/sqrt(pow(MotionData.position_x - IPI_x,2)+pow(MotionData.position_y - IPI_y,2)));
  if(MotionData.position_x - IPI_x > 0)
  {
    MotionData.serve[0] = theta*50;
    MotionData.serve[1] = 0;
  }
  else
  {
    MotionData.serve[0] = 0;
    MotionData.serve[1] = theta*50;
  }
}

void Serve_1_Set(double angle)
{
  static double angle_offset = 0;
  angle = angle - angle_offset;
  angle>90?90:angle;
  angle<-90?-90:angle;
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