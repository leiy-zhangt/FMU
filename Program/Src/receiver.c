#include "receiver.h"

uint8_t ReceiverReceiveBuff[25];//Receiver接收缓存数组
uint8_t ReceiverFifoBuff[25];//Receiver数据处理数组，缓存数组接收数据后保存至处理数组中

uint16_t ReceiverChannel[16];//Receiver接收通道数据

SemaphoreHandle_t ReceiverSemaphore=NULL;//Receiver二值信号量
BaseType_t ReceiverHigherTaskSwitch;

ReceiverStatus ReceiverDataConvert(void)
{
	if((ReceiverFifoBuff[0] != 0x0F)||(ReceiverFifoBuff[24] != 0x08)) return Receiver_ERR;
	if(ReceiverFifoBuff[23] == 0x0D) return Receiver_NOSignal;
	ReceiverChannel[0] = ((uint16_t)ReceiverFifoBuff[1])|((uint16_t)((ReceiverFifoBuff[2]&0x07)<<8));
	ReceiverChannel[1] = ((uint16_t)((ReceiverFifoBuff[2]&0xf8)>>3))|(((uint16_t)(ReceiverFifoBuff[3]&0x3f))<<5);
	ReceiverChannel[2] = ((uint16_t)((ReceiverFifoBuff[3]&0xc0)>>6))|((((uint16_t)ReceiverFifoBuff[4])<<2))|(((uint16_t)(ReceiverFifoBuff[5]&0x01))<<10);
	ReceiverChannel[3] = ((uint16_t)((ReceiverFifoBuff[5]&0xfe)>>1))|(((uint16_t)(ReceiverFifoBuff[6]&0x0f))<<7);
	ReceiverChannel[4] = ((uint16_t)((ReceiverFifoBuff[6]&0xf0)>>4))|(((uint16_t)(ReceiverFifoBuff[7]&0x7f))<<4);
	ReceiverChannel[5] = ((uint16_t)((ReceiverFifoBuff[7]&0x80)>>7))|(((uint16_t)ReceiverFifoBuff[8])<<1)|(((uint16_t)(ReceiverFifoBuff[9]&0x03))<<9);
	ReceiverChannel[6] = ((uint16_t)((ReceiverFifoBuff[9]&0xfc)>>2))|(((uint16_t)(ReceiverFifoBuff[10]&0x1f))<<6);
	ReceiverChannel[7] = ((uint16_t)((ReceiverFifoBuff[10]&0xe0)>>5))|(((uint16_t)(ReceiverFifoBuff[11]))<<3);
	ReceiverChannel[8] = ((uint16_t)ReceiverFifoBuff[12])|(((uint16_t)(ReceiverFifoBuff[13]&0x07))<<8);
	ReceiverChannel[9] = ((uint16_t)((ReceiverFifoBuff[13]&0xf8)>>3))|(((uint16_t)(ReceiverFifoBuff[14]&0x3f))<<5);
	ReceiverChannel[10] = ((uint16_t)((ReceiverFifoBuff[14]&0xc0)>>6))|(((uint16_t)ReceiverFifoBuff[15])<<2)|(((uint16_t)(ReceiverFifoBuff[16]&0x01))<<10);
	ReceiverChannel[11] = ((uint16_t)((ReceiverFifoBuff[16]&0xfe)>>1))|(((uint16_t)(ReceiverFifoBuff[17]&0x0f))<<7);
	ReceiverChannel[12] = ((uint16_t)((ReceiverFifoBuff[17]&0xf0)>>4))|(((uint16_t)(ReceiverFifoBuff[18]&0x7f))<<4);
	ReceiverChannel[13] = ((uint16_t)((ReceiverFifoBuff[18]&0x80)>>7))|(((uint16_t)ReceiverFifoBuff[19])<<1)|(((uint16_t)(ReceiverFifoBuff[20]&0x03))<<9);
	ReceiverChannel[14] = ((uint16_t)((ReceiverFifoBuff[20]&0xfc)>>2))|(((uint16_t)(ReceiverFifoBuff[21]&0x1f))<<6);
	ReceiverChannel[15] = ((uint16_t)((ReceiverFifoBuff[21]&0xe0)>>5))|(((uint16_t)ReceiverFifoBuff[22])<<3);
	return Receiver_OK;
}

