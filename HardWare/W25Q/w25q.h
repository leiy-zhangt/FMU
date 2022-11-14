#ifndef __W25Q_H
#define __W25Q_H			    
#include "sys.h" 
#include "spi.h"
#include "stdio.h"
#include "delay.h"	
#include "computation.h"


#define W25Q_Port  GPIOC
#define W25Q_CS_Pin  GPIO_Pin_12

#define	W25Q_CS PCout(12)  	//W25Q片选引脚

extern uint8_t W25Q_buffer[4096];
extern uint32_t W25Q_DataAddress[64];


void W25Q_Configuration(void);
void W25Q_WriteEnable(void);  //写使能
FlagStatus W25Q_CheckBusy(void); //检查BUSY标志位
void W25Q_WaitBusy(void);  //等待W25QBUSY标志位清零
void W25Q_SectorErase(uint32_t sector);//擦除扇区，范围为0~16384
void W25Q_BlockErase(uint32_t block); //擦除块，范围为0~1023
void W25Q_ChipErase(void); //W25Q整块擦除
void W25Q_DataStorage(uint32_t addr,uint8_t *buffer,uint16_t length); //W25Q按页写入，数据量不得超过256Byte
void W25Q_WriteInstruction(uint8_t instuction); //向W25Q发送指令
void W25Q_DataReceive(uint32_t addr,uint8_t *buffer,uint32_t length); //从地址中读出数据
void W25Q_WriteSatusRegister(uint8_t reg,uint8_t data);//向状态寄存器写入数据
#endif
















