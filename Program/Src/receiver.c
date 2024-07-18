#include "receiver.h"
#include "teleport.h"
#include "tf.h"
#include "control.h"
#include "taskinit.h"
#include "guide.h"

uint8_t ReceiverReceiveBuff[25];//Receiver接收缓存数组
uint8_t ReceiverFifoBuff[25];//Receiver数据处理数组，缓存数组接收数据后保存至处理数组中

uint16_t ReceiverChannel[16];//Receiver接收通道数据
uint16_t ReceiverChannelPrevious[16]={1500,1500,1000,1500,1000,1000,1000,1500,1500,1500,1500,1500,1500,1500,1500,1500};//Receiver接收通道之前数据
uint16_t ReceiverChannelNeutral[16] = {1500,1500,1000,1500,1000,1000,1000,1500,1500,1500,1500,1500,1500,1500,1500,1500};//Receiver接收通道中立数据

SemaphoreHandle_t ReceiverSemaphore=NULL;//Receiver二值信号量
BaseType_t ReceiverHigherTaskSwitch;

ReceiverStatus ReceiverDataConvert(uint8_t *ReceiverBuff)
{
	uint8_t n = 0;
	if(((ReceiverBuff[0] == 0x0F)||(ReceiverBuff[0] == 0x8F))&&((ReceiverBuff[24] == 0x00)||(ReceiverBuff[24] == 0x08))) 
	{
		if(ReceiverBuff[23]&0x0C) return Receiver_NOSignal;
		ReceiverChannel[0] = ((uint16_t)ReceiverBuff[1])|((uint16_t)((ReceiverBuff[2]&0x07)<<8));
		ReceiverChannel[1] = ((uint16_t)((ReceiverBuff[2]&0xf8)>>3))|(((uint16_t)(ReceiverBuff[3]&0x3f))<<5);
		ReceiverChannel[2] = ((uint16_t)((ReceiverBuff[3]&0xc0)>>6))|((((uint16_t)ReceiverBuff[4])<<2))|(((uint16_t)(ReceiverBuff[5]&0x01))<<10);
		ReceiverChannel[3] = ((uint16_t)((ReceiverBuff[5]&0xfe)>>1))|(((uint16_t)(ReceiverBuff[6]&0x0f))<<7);
		ReceiverChannel[4] = ((uint16_t)((ReceiverBuff[6]&0xf0)>>4))|(((uint16_t)(ReceiverBuff[7]&0x7f))<<4);
		ReceiverChannel[5] = ((uint16_t)((ReceiverBuff[7]&0x80)>>7))|(((uint16_t)ReceiverBuff[8])<<1)|(((uint16_t)(ReceiverBuff[9]&0x03))<<9);
		ReceiverChannel[6] = ((uint16_t)((ReceiverBuff[9]&0xfc)>>2))|(((uint16_t)(ReceiverBuff[10]&0x1f))<<6);
		ReceiverChannel[7] = ((uint16_t)((ReceiverBuff[10]&0xe0)>>5))|(((uint16_t)(ReceiverBuff[11]))<<3);
		ReceiverChannel[8] = ((uint16_t)ReceiverBuff[12])|(((uint16_t)(ReceiverBuff[13]&0x07))<<8);
		ReceiverChannel[9] = ((uint16_t)((ReceiverBuff[13]&0xf8)>>3))|(((uint16_t)(ReceiverBuff[14]&0x3f))<<5);
		ReceiverChannel[10] = ((uint16_t)((ReceiverBuff[14]&0xc0)>>6))|(((uint16_t)ReceiverBuff[15])<<2)|(((uint16_t)(ReceiverBuff[16]&0x01))<<10);
		ReceiverChannel[11] = ((uint16_t)((ReceiverBuff[16]&0xfe)>>1))|(((uint16_t)(ReceiverBuff[17]&0x0f))<<7);
		ReceiverChannel[12] = ((uint16_t)((ReceiverBuff[17]&0xf0)>>4))|(((uint16_t)(ReceiverBuff[18]&0x7f))<<4);
		ReceiverChannel[13] = ((uint16_t)((ReceiverBuff[18]&0x80)>>7))|(((uint16_t)ReceiverBuff[19])<<1)|(((uint16_t)(ReceiverBuff[20]&0x03))<<9);
		ReceiverChannel[14] = ((uint16_t)((ReceiverBuff[20]&0xfc)>>2))|(((uint16_t)(ReceiverBuff[21]&0x1f))<<6);
		ReceiverChannel[15] = ((uint16_t)((ReceiverBuff[21]&0xe0)>>5))|(((uint16_t)ReceiverBuff[22])<<3);
		for(n=0;n<16;n++)
		{
			ReceiverChannel[n] = (ReceiverChannel[n]-256)*0.6788867+1000;
		}
		return Receiver_OK;
	}
	else 
	{
		return Receiver_ERR;
	}
}

void ReceiverSolution(void)
{
	if(ReceiverChannel[4]<1400)//检测飞控停止工作信号
	{
		if(ReceiverChannelPrevious[4]>1450)
		{
			ControlStop();
		}
		expected_roll = (ReceiverChannel[0]-1500)*0.09;
		expected_pitch = (ReceiverChannel[1]-1500)*0.09;
		expected_yaw = (ReceiverChannel[3]-1500)*0.09;
		//滚转角输出
		ServoSet(ServoChannel_1,expected_roll);
		ServoSet(ServoChannel_5,expected_roll);
		//俯仰角输出
		ServoSet(ServoChannel_2,expected_pitch);
		ServoSet(ServoChannel_6,expected_pitch);
		//油门输出
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,ReceiverChannel[2]);
		//偏航角输出
		ServoSet(ServoChannel_4,expected_yaw);
		ServoSet(ServoChannel_7,expected_yaw);
	}
	else if(ReceiverChannel[4]>1600)//检测飞控开始工作信号
	{
		if(ReceiverChannelPrevious[4]<1550)
		{
			PitchNeutral=0;
			RollNeutral=0;
			ControlStart();
		}
	}
	
	//控制飞控飞行模式
	if(ReceiverChannel[5]<1400)
	{
		if(ReceiverChannel[6]<1400) FMUControlMode = FMU_Manual;
		else if(ReceiverChannel[6]<1600) 
		{
			FMUControlMode = FMU_Stable;
			if(FMUControlModePrevious != FMU_Stable) integtal_pitch = 0;
		}
		else 
		{
			FMUControlMode = FMU_Height;
			if(FMUControlModePrevious != FMU_Height) 
			{
				expected_height = IMUData.height;
//				integtal_pitch = 0;
			}
		}
	}
//	else if(ReceiverChannel[5]<1600)
	else
	{
		FMUControlMode = FMU_Path;
		if(FMUControlModePrevious != FMU_Path)
		{
			GuideInitPos.posx = GNSSData.lon;
			GuideInitPos.posy = GNSSData.lat;
			expected_height = IMUData.height;
			*PathChangeJudge = 1;
			*PathInte = 0;
//			integtal_pitch = 0;
		}
	}
	//进行遥控器归中校准
	if(ReceiverChannel[7]<1400) ;
	else if(ReceiverChannelPrevious[7]<1400)
	{
		memcpy(ReceiverChannelNeutral,ReceiverChannel,sizeof(ReceiverChannel));
	}
	
	
	//控制数传数据返回
//	if(ReceiverChannel[6]<1400) 
//	{
//		if(ReceiverChannelPrevious[6]>1450)
//		{
//			memcpy(ReceiverChannelNeutral,ReceiverChannel,sizeof(ReceiverChannel));
////			PitchNeutral = IMUData.pitch;
////			RollNeutral = IMUData.roll;
////			if(eTaskGetState(TeleportTransmit_TCB) != eSuspended) vTaskSuspend(TeleportTransmit_TCB);
//		}
//	}
//	else if(ReceiverChannel[6]>1600) 
//	{
//		if(ReceiverChannelPrevious[6]<1550)
//		{
//			if(eTaskGetState(TeleportTransmit_TCB) == eSuspended) vTaskResume(TeleportTransmit_TCB);
//		}
//	}
	//复制通道内容
	FMUControlModePrevious = FMUControlMode;
	memcpy(ReceiverChannelPrevious,ReceiverChannel,sizeof(ReceiverChannel));
}
