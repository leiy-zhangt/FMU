#ifndef __W25Q_H
#define __W25Q_H			    
#include "sys.h" 
#include "spi.h"
#include "delay.h"	

#define Start_Page 65 //���ݳ�ʼ����ҳ

#define W25Q_Port  GPIOC
#define W25Q_CS_Pin  GPIO_Pin_12

#define	W25Q_CS PCout(12)  	//W25Q��Ƭѡ�ź�


void W25Q_Configuration(void);
void W25Q_WriteEnable(void);  //дʹ��
FlagStatus W25Q_CheckBusy(void); //���BUSYλ
void W25Q_WaitBusy(void);  //�ȴ�BUSYλ
void W25Q_BlockErase(uint16_t block); //����0~1023��  Bloc = Page/64
void W25Q_ChipErase(void); //��������оƬ
void W25Q_DataStorage(uint32_t addr,uint8_t *buffer); //��Pageд������
void W25Q_WriteInstruction(uint8_t instuction); //��Ĵ���д������
uint8_t W25Q_ReadData(uint8_t addr);//�ӼĴ�����ȡ����
void  W25Q_DataReceive(uint8_t *buffer,uint16_t page); //��Page�ж�ȡ����
void W25Q_WriteSatusRegister(uint8_t reg,uint8_t data);//��״̬�Ĵ���д������
#endif
















