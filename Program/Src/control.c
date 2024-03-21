#include "control.h"
#include "main.h"
#include "tf.h"
#include "stdio.h"
#include "imu.h"
#include "gnss.h"
#include "receiver.h"
#include "taskinit.h"

SemaphoreHandle_t ControlSemaphore=NULL;//控制率二值信号量
BaseType_t ControlHigherTaskSwitch;

double ControlTime;//飞控运行时间
const double ControlDt = 0.01;//飞控控制时间间隔

const double Kp_roll=1.5,Kd_roll=0.15,Kp_pitch=1,Kd_pitch=0.1,Kp_yaw=1.5,Kd_yaw=0.1,Kp_height=3;//控制率参数
double expected_roll,expected_pitch,expected_yaw,expected_height;//各通道期望值
double servo_roll,servo_pitch,servo_yaw;//对应通道角度

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
	xSemaphoreGiveFromISR(ControlSemaphore,&GNSSHigherTaskSwitch);
	portYIELD_FROM_ISR(ControlHigherTaskSwitch);
}

void ControlStop(void)//飞控结束工作
{
	HAL_TIM_Base_Stop_IT(&htim6);
	FileClose();
}


void ServoSet(ServoChannel channel,double angle)//
{
	uint8_t ServoDirection[8] = {0,1,0,0,0,0,0,0};
	int16_t ServoOffset[8] = {0,0,0,0,0,0,0,0};
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
	switch(FMUControlMode)
	{
		case FMU_Manual:
		{
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,ReceiverChannel[0]);
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,3000-ReceiverChannel[1]);
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,ReceiverChannel[2]);
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,ReceiverChannel[3]);
			break;
		}
		case FMU_Stable:
		{
			//滚转与俯仰角期望值
			expected_roll = (ReceiverChannel[0]-1500)*0.12;
			expected_pitch = (ReceiverChannel[1]-1500)*0.12;
			//计算舵机角度
			servo_roll = Kp_roll*(expected_roll-IMUData.roll)-Kd_roll*IMUData.gyr_y;
			servo_roll = servo_roll>30?30:servo_roll;
			servo_roll = servo_roll<-30?-30:servo_roll;
			servo_pitch = Kp_pitch*(expected_pitch-IMUData.pitch)-Kd_pitch*IMUData.gyr_x;
			servo_pitch = servo_pitch>45?45:servo_pitch;
			servo_pitch = servo_pitch<-45?-45:servo_pitch;
			ServoSet(ServoChannel_1,servo_roll);
			ServoSet(ServoChannel_2,servo_pitch);
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,ReceiverChannel[2]);
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,ReceiverChannel[3]);
			break;
		}
		case FMU_Height:
		{
			//滚转与俯仰角期望值
			expected_roll = (ReceiverChannel[0]-1500)*0.12;
			expected_pitch = Kp_height*(expected_height-GNSSData.alt);
			//计算舵机角度
			servo_roll = Kp_roll*(expected_roll-IMUData.roll)+Kd_roll*IMUData.gyr_y;
			servo_roll = servo_roll>30?30:servo_roll;
			servo_roll = servo_roll<-30?-30:servo_roll;
			servo_pitch = Kp_pitch*(expected_pitch-IMUData.pitch)+Kd_pitch*IMUData.gyr_x;
			servo_pitch = servo_pitch>30?30:servo_pitch;
			servo_pitch = servo_pitch<-30?-30:servo_pitch;
			ServoSet(ServoChannel_1,servo_roll);
			ServoSet(ServoChannel_2,servo_pitch);
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,ReceiverChannel[2]);
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,ReceiverChannel[3]);
			break;
		}
	}
	//飞行参数保存，保存内容为:time IMUStatus ax ay az gx gy gz pitch roll yaw pre height GNSSStatus lon lat alt velocity v_e v_n ReceiverStatus ch[1-8] chout[1-8]
	if(IMURet == IMU_OK) sprintf((char *)StorageBuff,"time: %0.2f %s ax: %0.2f ay: %0.2f az: %0.2f gx: %0.2f gy: %0.2f gz: %0.2f p: %0.2f r: %0.2f y: %0.2f pre: %0.2f h: %0.2f ",\
		ControlTime,"IMU OK!",IMUData.acc_x,IMUData.acc_y,IMUData.acc_z,IMUData.gyr_x,IMUData.gyr_y,IMUData.gyr_z,IMUData.pitch,IMUData.roll,IMUData.yaw,IMUData.pressure,IMUData.height);
	else sprintf((char *)StorageBuff,"time: %0.2f %s ax: %0.2f ay: %0.2f az: %0.2f gx: %0.2f gy: %0.2f gz: %0.2f p: %0.2f r: %0.2f y: %0.2f pre: %0.2f h: %0.2f ",
		ControlTime,"IMU ERR!",IMUData.acc_x,IMUData.acc_y,IMUData.acc_z,IMUData.gyr_x,IMUData.gyr_y,IMUData.gyr_z,IMUData.pitch,IMUData.roll,IMUData.yaw,IMUData.pressure,IMUData.height);
	f_printf(&SDFile,(char *)StorageBuff);
	if(GNSSRet == GNSS_FIX) sprintf((char *)StorageBuff,"%s lon: %0.10f lat: %0.10f h: %0.2f v: %0.2f v_e: %0.2f v_n: %0.2f h: %0.2f ",\
		"GNSS FIX!",GNSSData.lon,GNSSData.lat,GNSSData.alt,GNSSData.velocity,GNSSData.velocity_e,GNSSData.velocity_n,expected_height);
	else sprintf((char *)StorageBuff,"%s lon: %0.10f lat: %0.10f h: %0.2f v: %0.2f v_e: %0.2f v_n: %0.2f h: %0.2f ",\
		"GNSS NOFIX!",GNSSData.lon,GNSSData.lat,GNSSData.alt,GNSSData.velocity,GNSSData.velocity_e,GNSSData.velocity_n,expected_height);
	f_printf(&SDFile,(char *)StorageBuff);
	if(ReceiverRet == Receiver_OK) sprintf((char *)StorageBuff,"%s c1: %u c2: %u c3: %u c4: %u c5: %u c6: %u c7: %u c8: %u\n","Receiver OK!",\
		ReceiverChannel[0],ReceiverChannel[1],ReceiverChannel[2],ReceiverChannel[3],ReceiverChannel[4],ReceiverChannel[5],ReceiverChannel[6],ReceiverChannel[7]);
	else sprintf((char *)StorageBuff,"%s c1: %u c2: %u c3: %u c4: %u c5: %u c6: %u c7: %u c8: %u\n","Receiver ERR!",\
		ReceiverChannel[0],ReceiverChannel[1],ReceiverChannel[2],ReceiverChannel[3],ReceiverChannel[4],ReceiverChannel[5],ReceiverChannel[6],ReceiverChannel[7]);
	f_printf(&SDFile,(char *)StorageBuff);
}





