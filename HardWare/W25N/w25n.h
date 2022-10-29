#ifndef __W25N_H
#define __W25N_H			    
#include "sys.h" 
#include "spi.h"
#include "delay.h"	

#define Start_Page 65 //数据初始保存页

#define W25N_Port  GPIOA
#define W25N_CS_Pin  GPIO_Pin_11
#define W25N_WP_Pin  GPIO_Pin_12
#define W25N_HOLD_Pin  GPIO_Pin_15

#define	W25N_CS PAout(11)  	//W25Q的片选信号
#define	W25N_WP PAout(12)  	//W25Q的写入保护
#define	W25N_HOLD PAout(15)  	//W25Q的保持

extern uint16_t page; 

void W25N_Configuration(void);
void W25N_WriteEnable(void);  //写使能
FlagStatus W25N_CheckBusy(void); //检查BUSY位
void W25N_WaitBusy(void);  //等待BUSY位
void W25N_BlockErase(uint16_t block); //擦除0~1023块  Bloc = Page/64
void W25N_ChipErase(void); //擦除整个芯片
void W25N_DataWrirte(u8 *buffer,uint16_t page); //向Page写入数据
void W25N_SendData(u8 addr,u8 data); //向寄存器写入数据
void  W25N_DataReceive(u8 *buffer,uint16_t page); //从Page中读取数据
#endif
















