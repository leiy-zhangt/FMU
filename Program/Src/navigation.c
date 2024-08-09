#include "navigation.h"
#include "math.h"
#include "control.h"
#include "imu.h"
#include "stdio.h"
#include "gnss.h"

uint8_t GNSSUpdate;
uint8_t FMUReturnFlag = 0;//无人机返航标志
FMURrturnDirection ReturnDire;//自动返回航向

NavigationDataStruct NevigationInitData;
NavigationDataStruct NevigationLastData;
NavigationDataStruct NevigationData;

IMUDateStruct	NavAttitudeData;

double p,r,y;//姿态角


void AttitudeSolution(double *pitch,double *roll,double *yaw,double gyr_x,double gyr_y,double gyr_z)  //对角速度进行处理，得到弧度值形式的姿态角
{
  double w[3],dq[4],q[4],q_norm,dt=AttiDt;
	double T_11,T_21,T_31,T_12,T_22,T_32,T_13,T_23,T_33;//转换矩阵
	double acc_norm;
  w[0] = gyr_x * 0.017452;
  w[1] = gyr_y * 0.017452;
  w[2] = gyr_z * 0.017452;
	p = (*pitch) * 0.017452;
	r = (*roll) * 0.017452;
	y = (*yaw) * 0.017452;
	T_11 = cos(r)*cos(y)-sin(r)*sin(p)*sin(y);
  T_21 = cos(r)*sin(y)+sin(r)*sin(p)*cos(y);
  T_31 = -sin(r)*cos(p);
  T_12 = -cos(p)*sin(y);
  T_22 = cos(p)*cos(y);
  T_32 = sin(p);
  T_13 = sin(r)*cos(y)+cos(r)*sin(p)*sin(y);
  T_23 = sin(r)*sin(y)-cos(r)*sin(p)*cos(y);
  T_33 = cos(r)*cos(p);
	q[0] = 0.5*sqrt(1+T_11+T_22+T_33);
  q[1] = 0.5*sqrt(1+T_11-T_22-T_33);
  q[2] = 0.5*sqrt(1-T_11+T_22-T_33);
  q[3] = 0.5*sqrt(1-T_11-T_22+T_33);
	if((T_32 - T_23)<0) q[1] = -q[1];
  if((T_13 - T_31)<0) q[2] = -q[2];
  if((T_21 - T_12)<0) q[3] = -q[3];
	dq[0] = 0.5*(-w[0]*q[1]-w[1]*q[2]-w[2]*q[3]);
  dq[1] = 0.5*(w[0]*q[0]+w[2]*q[2]-w[1]*q[3]);
  dq[2] = 0.5*(w[1]*q[0]-w[2]*q[1]+w[0]*q[3]);
  dq[3] = 0.5*(w[2]*q[0]+w[1]*q[1]-w[0]*q[2]);
  q[0] = q[0] + dq[0]*dt;
  q[1] = q[1] + dq[1]*dt;
  q[2] = q[2] + dq[2]*dt;
  q[3] = q[3] + dq[3]*dt;
  q_norm = sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
  q[0] = q[0]/q_norm;
  q[1] = q[1]/q_norm;
  q[2] = q[2]/q_norm;
  q[3] = q[3]/q_norm;
  p = asin(2*(q[0]*q[1]+q[2]*q[3]))*57.3;//初始矩阵俯仰角
  y = atan2(-2*(q[1]*q[2]-q[0]*q[3]),(pow(q[0],2)-pow(q[1],2)+pow(q[2],2)-pow(q[3],2)))*57.3;//初始矩阵偏航角
	r = atan2(-2*(q[1]*q[3]-q[0]*q[2]),pow(q[0],2)-pow(q[1],2)-pow(q[2],2)+pow(q[3],2))*57.3;//初始矩阵滚转角
	//对加速度进行低通滤波
	NavAttitudeData.acc_x = NavAttitudeData.acc_x*0.95 + IMUData.tran_acc_x*0.05;
	NavAttitudeData.acc_y = NavAttitudeData.acc_y*0.95 + IMUData.tran_acc_y*0.05;
	NavAttitudeData.acc_z = NavAttitudeData.acc_z*0.95 + IMUData.tran_acc_z*0.05;
	//对陀螺仪进行低通滤波
	NavAttitudeData.gyr_x = NavAttitudeData.gyr_x*0.95 + IMUData.tran_gyr_x*0.05;
	NavAttitudeData.gyr_y = NavAttitudeData.gyr_y*0.95 + IMUData.tran_gyr_y*0.05;
	NavAttitudeData.gyr_z = NavAttitudeData.gyr_z*0.95 + IMUData.tran_gyr_z*0.05;
	acc_norm = sqrt(pow(NavAttitudeData.acc_x,2)+pow(NavAttitudeData.acc_y,2)+pow(NavAttitudeData.acc_z,2));
  if((acc_norm>9)&&(acc_norm<11))
  {
    p = asin(NavAttitudeData.acc_y/acc_norm)*57.3*0.02 + p*0.98;
    r = atan2(-NavAttitudeData.acc_x,NavAttitudeData.acc_z)*57.3*0.02 + r*0.98;
  }
	if(r>90.0)
	{
		r = -(180.0-r);
		//俯仰角转换
		if(p>0) p=180-p;
		else p=-180-p;
		//滚转角转换
		if(y>0) y=y-180;
		else y=180+y;
	}
	else if(r<-90)
	{
		r = 180.0 + r;
		if(p>0) p=180-p;
		else p=-180-p;
		//滚转角转换
		if(y>0) y=y-180;
		else y=180+y;
	}
	y = 0.98*y+0.02*IMUData.tran_yaw;
	*pitch = p;
	*roll = r;
	*yaw = y;
}

void NevigayionSolutinInit(void)
{
	NavAttitudeData.pitch = IMUData.tran_pitch;
	NavAttitudeData.roll = IMUData.tran_roll;
	NavAttitudeData.yaw = IMUData.tran_yaw;
	NavAttitudeData.acc_x = IMUData.tran_acc_x;
	NavAttitudeData.acc_y = IMUData.tran_acc_y;
	NavAttitudeData.acc_z = IMUData.tran_acc_z;
	NavAttitudeData.gyr_x = IMUData.tran_gyr_x;
	NavAttitudeData.gyr_y = IMUData.tran_gyr_y;
	NavAttitudeData.gyr_z = IMUData.tran_gyr_z;
}


void NevigationSolution(void)
{
	static double a_n=0,a_e=0,a_u=0;
	static double v_n=0,v_e=0,v_u=0;
	static double p_n=0,p_e=0,p_u=0;
	static double T_11,T_21,T_31,T_12,T_22,T_32,T_13,T_23,T_33;
	T_11 = cos(IMUData.roll*0.0174532922)*cos(IMUData.yaw*0.0174532922)-sin(IMUData.roll*0.0174532922)*sin(IMUData.pitch*0.0174532922)*sin(IMUData.yaw*0.0174532922);
	T_21 = cos(IMUData.roll*0.0174532922)*sin(IMUData.yaw*0.0174532922)+sin(IMUData.roll*0.0174532922)*sin(IMUData.pitch*0.0174532922)*cos(IMUData.yaw*0.0174532922);
	T_31 = -sin(IMUData.roll*0.0174532922)*cos(IMUData.pitch*0.0174532922);
	T_12 = -cos(IMUData.pitch*0.0174532922)*sin(IMUData.yaw*0.0174532922);
	T_22 = cos(IMUData.pitch*0.0174532922)*cos(IMUData.yaw*0.0174532922);
	T_32 = sin(IMUData.pitch*0.0174532922);
	T_13 = sin(IMUData.roll*0.0174532922)*cos(IMUData.yaw*0.0174532922)+cos(IMUData.roll*0.0174532922)*sin(IMUData.pitch*0.0174532922)*sin(IMUData.yaw*0.0174532922);
	T_23 = sin(IMUData.roll*0.0174532922)*sin(IMUData.yaw*0.0174532922)-cos(IMUData.roll*0.0174532922)*sin(IMUData.pitch*0.0174532922)*cos(IMUData.yaw*0.0174532922);
	T_33 = cos(IMUData.roll*0.0174532922)*cos(IMUData.pitch*0.0174532922);
	a_e = T_11*IMUData.acc_x+T_12*IMUData.acc_y+T_13*IMUData.acc_z;
	a_n = T_21*IMUData.acc_x+T_22*IMUData.acc_y+T_23*IMUData.acc_z;
	a_u = T_31*IMUData.acc_x+T_32*IMUData.acc_y+T_33*IMUData.acc_z - 9.80665;
//	v_e += a_e*ControlDt;
//	v_n += a_n*ControlDt;
//	v_u += (a_u)*ControlDt;
//	p_e += v_e*ControlDt;
//	p_n += v_n*ControlDt;
//	p_u += v_u*ControlDt;
//	printf("%0.4f  %0.4f  %0.4f  %0.4f  %0.4f  %0.4f  %0.4f  %0.4f  %0.4f\r\n",a_e,a_n,a_u,v_e,v_n,v_u,p_e,p_n,p_u);
}

double Lon2Distance(double lon_1,double lon_2) //lon1-lon2的距离
{
	return (lon_1-lon_2)*100000;
}

double Lat2Distance(double lat_1,double lat_2) //lat1-lat2的距离
{
	return (lat_1-lat_2)*111320;
}

double Distance2Lon(double px_1,double px_2) //东向距离转换为经度
{
	return (px_1-px_2)*0.00001;
}

double Distance2Lat(double py_1,double py_2) //北向距离转换为纬度
{
	return (py_1-py_2)*0.000008983111;
}

FMURrturnDirection FMUReturnJudge(void)
{
	double dx,dy,distance,V_angle,R_angle,Diff_angle;
	distance = sqrt(pow(Lon2Distance(GNSSData.lon,GNSSData.lon_Init),2)+pow(Lat2Distance(GNSSData.lat,GNSSData.lat_Init),2));
	if(FMUReturnFlag == 0)
	{
		if(distance > FMUReturnDistance)
		{
			FMUReturnFlag = 1;
		}
	}
	else
	{
		R_angle = atan2(Lat2Distance(GNSSData.lat_Init,GNSSData.lat),Lon2Distance(GNSSData.lon_Init,GNSSData.lon))*57.3; 
		V_angle = IMUData.yaw;
		R_angle = R_angle<0?360+R_angle:R_angle;
		V_angle = V_angle<0?360+V_angle:V_angle;
		Diff_angle = V_angle - R_angle;
		Diff_angle = Diff_angle>180?Diff_angle-360:Diff_angle;
		Diff_angle = Diff_angle<-180?360+Diff_angle:Diff_angle;
		if(Diff_angle<20) 
		{
			if(distance<FMUReturnDistance) 
			{
				FMUReturnFlag = 0;
				return Return_NOTurn;
			}
		}
		if(Diff_angle>0) return Return_TurnLeft;
		else return Return_TurnRight;
	}
	return Return_NOTurn;
}




