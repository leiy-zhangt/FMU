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

