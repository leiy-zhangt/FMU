#ifndef __W25Q_H
#define __W25Q_H			    
#include "sys.h" 
#include "spi.h"
#include "stdio.h"
#include "delay.h"	

#define Start_Page 65 //数据初始保存页

#define W25Q_Port  GPIOC
#define W25Q_CS_Pin  GPIO_Pin_12

#define	W25Q_CS PCout(12)  	//W25Q的片选信号

extern uint8_t W25Q_buffer[4096];


void W25Q_Configuration(void);
void W25Q_WriteEnable(void);  //写使能
FlagStatus W25Q_CheckBusy(void); //检查BUSY位
void W25Q_WaitBusy(void);  //等待BUSY位
void W25Q_SectorErase(uint32_t sector);//擦除0~16384块
void W25Q_BlockErase(uint32_t block); //擦除0~1023块 
void W25Q_ChipErase(void); //擦除整个芯片
void W25Q_DataStorage(uint32_t addr,uint8_t *buffer,uint16_t length); //向扇区写入数据
void W25Q_WriteInstruction(uint8_t instuction); //向寄存器写入数据
void  W25Q_DataReceive(uint32_t addr,uint8_t *buffer,uint32_t length); //从Page中读取数据
void W25Q_WriteSatusRegister(uint8_t reg,uint8_t data);//向状态寄存器写入数据
#endif
















