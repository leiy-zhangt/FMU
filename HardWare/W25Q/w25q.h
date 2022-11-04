#ifndef __W25Q_H
#define __W25Q_H			    
#include "sys.h" 
#include "spi.h"
#include "stdio.h"
#include "delay.h"	

#define Start_Page 65 //���ݳ�ʼ����ҳ

#define W25Q_Port  GPIOC
#define W25Q_CS_Pin  GPIO_Pin_12

#define	W25Q_CS PCout(12)  	//W25Q��Ƭѡ�ź�

extern uint8_t W25Q_buffer[4096];


void W25Q_Configuration(void);
void W25Q_WriteEnable(void);  //дʹ��
FlagStatus W25Q_CheckBusy(void); //���BUSYλ
void W25Q_WaitBusy(void);  //�ȴ�BUSYλ
void W25Q_SectorErase(uint32_t sector);//����0~16384��
void W25Q_BlockErase(uint32_t block); //����0~1023�� 
void W25Q_ChipErase(void); //��������оƬ
void W25Q_DataStorage(uint32_t addr,uint8_t *buffer,uint16_t length); //������д������
void W25Q_WriteInstruction(uint8_t instuction); //��Ĵ���д������
void  W25Q_DataReceive(uint32_t addr,uint8_t *buffer,uint32_t length); //��Page�ж�ȡ����
void W25Q_WriteSatusRegister(uint8_t reg,uint8_t data);//��״̬�Ĵ���д������
#endif
















