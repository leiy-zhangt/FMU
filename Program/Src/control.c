#include "control.h"
#include "main.h"
#include "tf.h"
#include "stdio.h"
#include "imu.h"
#include "gnss.h"
#include "receiver.h"
#include "taskinit.h"
#include "navigation.h"
#include "math.h"
#include "guide.h"

SemaphoreHandle_t ControlSemaphore=NULL;//控制率二值信号量
BaseType_t ControlHigherTaskSwitch;

double ControlTime;//飞控运行时间
const double ControlDt = 0.01;//飞控控制时间间隔

const double Kp_roll=2,Kd_roll=0.2,Kp_pitch=3,Kd_pitch=0.5,Ki_pitch = 1,Kp_yaw=1.5,Kd_yaw=0.1;//姿态控制参数
const double	Kp_height=4;//高度控制率参数
//const double Kp_roll=2,Kd_roll=0.2,Kp_pitch=0,Kd_pitch=0,Ki_pitch = 0.5,Kp_yaw=1.5,Kd_yaw=0.1,Kp_height=2;//姿态控制率参数
double expected_roll,expected_pitch,expected_yaw,expected_height;//各通道期望值
double servo_roll,servo_pitch,servo_yaw;//对应通道角度
double integtal_pitch=0;//俯仰角误差积分
double PitchNeutral=5,RollNeutral=0;//姿态角中立位置

FMUControlModeSelect FMUControlMode = FMU_Manual;//飞控工作模式选择
FMUControlModeSelect FMUControlModePrevious = FMU_Manual;

void ControlStart(void)//飞控开始工作初始化
{
	__HAL_TIM_SET_COUNTER(&htim6,0);
	FileCreate();
	ControlTime = 0;
	HAL_TIM_Base_Start_IT(&htim6);
}

void ControlUpdata(void)//飞控参数更新
{
	ControlTime += ControlDt;
	xSemaphoreGiveFromISR(ControlSemaphore,&ControlHigherTaskSwitch);
	portYIELD_FROM_ISR(ControlHigherTaskSwitch);
}

void ControlStop(void)//飞控结束工作
{
	HAL_TIM_Base_Stop_IT(&htim6);
	FileClose();
}


void ServoSet(ServoChannel channel,double angle)//
{
	uint8_t ServoDirection[8] = {1,1,0,0,1,0,0,0};
	int16_t ServoOffset[8] = {0,-50,0,0,0,20,80,0};
	int16_t angle_int16;
	switch(channel)
	{
		case ServoChannel_1:
			if(ServoDirection[ServoChannel_1]) angle = -angle;
			angle_int16 = angle*11.1+1500+ServoOffset[ServoChannel_1];
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,angle_int16);
			break;
		case ServoChannel_2:
			if(ServoDirection[ServoChannel_2]) angle = -angle;
			angle_int16 = angle*11.1+1500+ServoOffset[ServoChannel_2];
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,angle_int16);
			break;
		case ServoChannel_3:
			if(ServoDirection[ServoChannel_3]) angle = -angle;
			angle_int16 = angle*11.1+1500+ServoOffset[ServoChannel_3];
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,angle_int16);
			break;
		case ServoChannel_4:
			if(ServoDirection[ServoChannel_4]) angle = -angle;
			angle_int16 = angle*11.1+1500+ServoOffset[ServoChannel_4];
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,angle_int16);
			break;
		case ServoChannel_5:
			if(ServoDirection[ServoChannel_5]) angle = -angle;
			angle_int16 = angle*11.1+1500+ServoOffset[ServoChannel_5];
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,angle_int16);
			break;
		case ServoChannel_6:
			if(ServoDirection[ServoChannel_6]) angle = -angle;
			angle_int16 = angle*11.1+1500+ServoOffset[ServoChannel_6];
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,angle_int16);
			break;
		case ServoChannel_7:
			if(ServoDirection[ServoChannel_7]) angle = -angle;
			angle_int16 = angle*11.1+1500+ServoOffset[ServoChannel_7];
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,angle_int16);
			break;
		case ServoChannel_8:
			if(ServoDirection[ServoChannel_8]) angle = -angle;
			angle_int16 = angle*11.1+1500+ServoOffset[ServoChannel_8];
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,angle_int16);
			break;
	}
}

void FixedWingControl(void)
{
//	ReturnDire = FMUReturnJudge();
//	if(FMUReturnFlag)	FMUControlMode = FMU_Return;
	switch(FMUControlMode)
	{
		case FMU_Manual:
		{
			expected_roll = (ReceiverChannel[0]-ReceiverChannelNeutral[0])*0.09;
			expected_pitch = (ReceiverChannel[1]-ReceiverChannelNeutral[1])*0.09;
			expected_yaw = (ReceiverChannel[3]-ReceiverChannelNeutral[3])*0.045;
			servo_roll = expected_roll;
			servo_pitch = expected_pitch;
			servo_yaw = expected_yaw;
			ServoSet(ServoChannel_1,servo_roll);
			ServoSet(ServoChannel_5,servo_roll);
			ServoSet(ServoChannel_2,servo_pitch);
			ServoSet(ServoChannel_6,servo_pitch);
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,ReceiverChannel[2]);
			ServoSet(ServoChannel_4,servo_yaw);
			ServoSet(ServoChannel_7,servo_yaw);
			break;
		}
		case FMU_Stable:
		{
			//滚转与俯仰角期望值 0.09为45°
			expected_roll = (ReceiverChannel[0]-ReceiverChannelNeutral[0])*0.09+RollNeutral;
			expected_pitch = (ReceiverChannel[1]-ReceiverChannelNeutral[1])*0.09+PitchNeutral;
			expected_yaw = (ReceiverChannel[3]-ReceiverChannelNeutral[3])*0.045;
			//计算俯仰角误差积分
			integtal_pitch = integtal_pitch+(expected_pitch-NevAttitudeData.pitch)*ControlDt;
      integtal_pitch = integtal_pitch>20?20:integtal_pitch;
      integtal_pitch = integtal_pitch<-20?-20:integtal_pitch;
			//计算舵机角度
			servo_roll = Kp_roll*(expected_roll-NevAttitudeData.roll)-Kd_roll*NevAttitudeData.gyr_y;
			servo_roll = servo_roll>45?45:servo_roll;
			servo_roll = servo_roll<-45?-45:servo_roll;
			servo_pitch = Kp_pitch*(expected_pitch-NevAttitudeData.pitch)-Kd_pitch*NevAttitudeData.gyr_x+Ki_pitch*integtal_pitch;
			servo_pitch = servo_pitch>45?45:servo_pitch;
			servo_pitch = servo_pitch<-45?-45:servo_pitch;
			ServoSet(ServoChannel_1,servo_roll);
			ServoSet(ServoChannel_5,servo_roll);
			ServoSet(ServoChannel_2,servo_pitch);
			ServoSet(ServoChannel_6,servo_pitch);
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,ReceiverChannel[2]);
			ServoSet(ServoChannel_4,expected_yaw);
			ServoSet(ServoChannel_7,expected_yaw);
			break;
		}
		case FMU_Height:
		{
			//滚转与俯仰角期望值
			expected_roll = (ReceiverChannel[0]-ReceiverChannelNeutral[0])*0.09;
			expected_pitch = Kp_height*(expected_height-IMUData.height)+5+fabs(NevAttitudeData.roll)*0.5;
			//限制俯仰角上下限
			expected_pitch = expected_pitch>30?30:expected_pitch;
			expected_pitch = expected_pitch<-30?-30:expected_pitch;
			//计算俯仰角误差积分
			integtal_pitch = integtal_pitch+(expected_pitch-NevAttitudeData.pitch)*ControlDt;
      integtal_pitch = integtal_pitch>20?20:integtal_pitch;
      integtal_pitch = integtal_pitch<-20?-20:integtal_pitch;
			//计算舵机角度
			servo_roll = Kp_roll*(expected_roll-NevAttitudeData.roll)-Kd_roll*IMUData.gyr_y;
			servo_roll = servo_roll>30?30:servo_roll;
			servo_roll = servo_roll<-30?-30:servo_roll;
			servo_pitch = Kp_pitch*(expected_pitch-NevAttitudeData.pitch)-Kd_pitch*IMUData.gyr_x+Ki_pitch*integtal_pitch;
			servo_pitch = servo_pitch>45?45:servo_pitch;
			servo_pitch = servo_pitch<-45?-45:servo_pitch;
			ServoSet(ServoChannel_1,servo_roll);
			ServoSet(ServoChannel_5,servo_roll);
			ServoSet(ServoChannel_2,servo_pitch);
			ServoSet(ServoChannel_6,servo_pitch);
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,ReceiverChannel[2]);
			ServoSet(ServoChannel_4,expected_yaw);
			ServoSet(ServoChannel_7,expected_yaw);
			break;
		}
		case FMU_Path:
		{
			//路径计算参数
			GuidePe = Lon2Distance(GNSSData.lon,GuideInitPos.posx)*0.8281;
			GuidePn = Lat2Distance(GNSSData.lat,GuideInitPos.posy);
			GuideAngle = GNSSData.angle;
			if(GuideAngle<=180) GuideAngle = -GuideAngle*0.017452;
			else GuideAngle = (360 - GuideAngle)*0.017452;
			//滚转与俯仰角期望
//			PathInte = 0;
			expected_roll = guidence_roll(GuidePe,GuidePn,GuideAngle,&PathInte,&PathChangeJudge)*57.3;
			expected_pitch = Kp_height*(expected_height-IMUData.height)+5+fabs(NevAttitudeData.roll)*0.5;
//			expected_pitch = (ReceiverChannel[1]-ReceiverChannelNeutral[1])*0.09+PitchNeutral;
			//限制滚转角、俯仰角上下限
			expected_roll = expected_roll>20?20:expected_roll;
			expected_roll = expected_roll<-20?-20:expected_roll;
			expected_pitch = expected_pitch>30?30:expected_pitch;
			expected_pitch = expected_pitch<-30?-30:expected_pitch;
			//计算俯仰角误差积分
			integtal_pitch = integtal_pitch+(expected_pitch-NevAttitudeData.pitch)*ControlDt;
      integtal_pitch = integtal_pitch>20?20:integtal_pitch;
      integtal_pitch = integtal_pitch<-20?-20:integtal_pitch;
			//计算舵机角度
			servo_roll = Kp_roll*(expected_roll-NevAttitudeData.roll)-Kd_roll*IMUData.gyr_y;
			servo_roll = servo_roll>25?25:servo_roll;
			servo_roll = servo_roll<-25?-25:servo_roll;
			servo_pitch = Kp_pitch*(expected_pitch-NevAttitudeData.pitch)-Kd_pitch*IMUData.gyr_x+Ki_pitch*integtal_pitch;
			servo_pitch = servo_pitch>45?45:servo_pitch;
			ServoSet(ServoChannel_1,servo_roll);
			ServoSet(ServoChannel_5,servo_roll);
			ServoSet(ServoChannel_2,servo_pitch);
			ServoSet(ServoChannel_6,servo_pitch);
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,ReceiverChannel[2]);
			ServoSet(ServoChannel_4,expected_yaw);
			ServoSet(ServoChannel_7,expected_yaw);
		}
		case FMU_Return:
			break;
//		case FMU_Return:
//		{
//			//滚转与俯仰角期望值
//			if(ReturnDire == Return_TurnLeft) expected_roll = -FMUReturnRoll;
//			else expected_roll = FMUReturnRoll;
//			expected_height = IMUData.height_Init+FMUReturnHeight;
//			expected_pitch = Kp_height*(expected_height-IMUData.height);
//			//限制俯仰角上下限
//			expected_pitch = expected_pitch>20?20:expected_pitch;
//			expected_pitch = expected_pitch<-20?-20:expected_pitch;
//			//计算俯仰角误差积分
//			integtal_pitch = integtal_pitch+(expected_pitch-IMUData.pitch)*ControlDt;
//      integtal_pitch = integtal_pitch>10?10:integtal_pitch;
//      integtal_pitch = integtal_pitch<-10?-10:integtal_pitch;
//			//计算舵机角度
//			servo_roll = Kp_roll*(expected_roll-IMUData.roll)-Kd_roll*IMUData.gyr_y;
//			servo_roll = servo_roll>30?30:servo_roll;
//			servo_roll = servo_roll<-30?-30:servo_roll;
//			servo_pitch = Kp_pitch*(expected_pitch-IMUData.pitch)-Kd_pitch*IMUData.gyr_x+Ki_pitch*integtal_pitch;
//			servo_pitch = servo_pitch>25?25:servo_pitch;
//			servo_pitch = servo_pitch<-25?-25:servo_pitch;
//			ServoSet(ServoChannel_1,servo_roll);
//			ServoSet(ServoChannel_2,servo_pitch);
//			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,ReceiverChannel[2]);
//			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,ReceiverChannel[3]);
//			ServoSet(ServoChannel_1,servo_roll);
//			ServoSet(ServoChannel_2,servo_pitch);
//			break;
//		}
	}
	//飞行参数保存
	if(IMURet == IMU_OK) sprintf((char *)StorageBuff,"time: %0.2f %s ax: %0.2f ay: %0.2f az: %0.2f gx: %0.2f gy: %0.2f gz: %0.2f p: %0.2f r: %0.2f y: %0.2f pre: %0.2f h: %0.2f ",\
		ControlTime,"IMU OK!",IMUData.acc_x,IMUData.acc_y,IMUData.acc_z,IMUData.gyr_x,IMUData.gyr_y,IMUData.gyr_z,IMUData.pitch,IMUData.roll,IMUData.yaw,IMUData.pressure,IMUData.height);
	else sprintf((char *)StorageBuff,"time: %0.2f %s ax: %0.2f ay: %0.2f az: %0.2f gx: %0.2f gy: %0.2f gz: %0.2f p: %0.2f r: %0.2f y: %0.2f pre: %0.2f h: %0.2f ",
		ControlTime,"IMU ERR!",IMUData.acc_x,IMUData.acc_y,IMUData.acc_z,IMUData.gyr_x,IMUData.gyr_y,IMUData.gyr_z,IMUData.pitch,IMUData.roll,IMUData.yaw,IMUData.pressure,IMUData.height);
	f_printf(&SDFile,(char *)StorageBuff);
	if(GNSSRet == GNSS_FIX) sprintf((char *)StorageBuff,"%s lon: %0.10f lat: %0.10f h: %0.2f v: %0.2f v_e: %0.2f v_n: %0.2f angle: %0.2f ",\
		"GNSS FIX!",GNSSData.lon,GNSSData.lat,GNSSData.alt,GNSSData.velocity,GNSSData.velocity_e,GNSSData.velocity_n,GNSSData.angle);
	else sprintf((char *)StorageBuff,"%s lon: %0.10f lat: %0.10f h: %0.2f v: %0.2f v_e: %0.2f v_n: %0.2f amgle: %0.2f ",\
		"GNSS NOFIX!",GNSSData.lon,GNSSData.lat,GNSSData.alt,GNSSData.velocity,GNSSData.velocity_e,GNSSData.velocity_n,GNSSData.angle);
	f_printf(&SDFile,(char *)StorageBuff);
	if(ReceiverRet == Receiver_OK) sprintf((char *)StorageBuff,"%s mode1: %u mode2: %u expect_p: %0.4f expect_r: %0.4f expect_y: %0.4f expect_t: %u expect_height: %0.2f servo_p: %0.4f servo_r: %0.4f servo_y: %0.4f ","Receiver OK!",ReceiverChannel[5],ReceiverChannel[6],\
		expected_pitch,expected_roll,expected_yaw,ReceiverChannel[2],expected_height,servo_pitch,servo_roll,servo_yaw);
	else sprintf((char *)StorageBuff,"%s mode1: %u mode2: %u expect_p: %0.4f expect_r: %0.4f expect_y:%0.4f expect_t: %u expect_height: %0.2f servo_p: %0.4f servo_r: %0.4f servo_y: %0.4f ","Receiver ERR!",ReceiverChannel[5],ReceiverChannel[6],\
		expected_pitch,expected_roll,expected_yaw,ReceiverChannel[2],expected_height,servo_pitch,servo_roll,servo_yaw);
	f_printf(&SDFile,(char *)StorageBuff);
	sprintf((char *)StorageBuff,"sp: %0.2f sr: %0.2f sy: %0.2f ",NevAttitudeData.pitch,NevAttitudeData.roll,NevAttitudeData.yaw);
	f_printf(&SDFile,(char *)StorageBuff);
	sprintf((char *)StorageBuff,"lon0: %0.10f lat0: %0.10f pe: %0.4f pn: %0.4f judge: %d inte: %0.2f psi: %0.4f\n",GuideInitPos.posx,GuideInitPos.posy,GuidePe,GuidePn,PathChangeJudge,PathInte,GuideAngle);
	f_printf(&SDFile,(char *)StorageBuff);
}


