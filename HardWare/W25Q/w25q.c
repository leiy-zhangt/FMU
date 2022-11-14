#include "w25q.h"    
/*
W25Q一页为256byte,一个扇区为64Kbyte
*/

uint8_t W25Q_buffer[4096];
uint32_t W25Q_DataAddress[64];

void W25Q_Configuration(void)
{ 
  //W25Q配置
  W25Q_WriteInstruction(0xB7);//使用32位地址模式
  W25Q_WriteInstruction(0x98);//全部块解锁
}  

void W25Q_WriteEnable(void)
{ 
  W25Q_CS = 0;
  SPI_ReadWriteByte(SPI1,0x06);
  W25Q_CS = 1;
  delay_us(1);
  W25Q_CS = 0;
  delay_us(1);
  SPI_ReadWriteByte(SPI1,0x05);
  while((SPI_ReadWriteByte(SPI1,0x00)&0x02) == 0);
  W25Q_CS = 1;
  delay_us(1);
}

FlagStatus W25Q_CheckBusy(void)
{
    W25Q_CS = 0;
    delay_us(1);
    SPI_ReadWriteByte(SPI1,0X05);
    if(SPI_ReadWriteByte(SPI1,0X00)&0X01)
    {
        W25Q_CS = 1;
        return SET;
    }
    else 
    {
        W25Q_CS = 1;
        return RESET;
    }  
}

void W25Q_WaitBusy(void)
{
  W25Q_CS = 0;
  delay_us(1);
  SPI_ReadWriteByte(SPI1,0x05);
  while(SPI_ReadWriteByte(SPI1,0x00)&0x01);
  W25Q_CS = 1;
  delay_us(1);
}

void W25Q_Reset(void)
{
  W25Q_WriteInstruction(0x66);
  W25Q_WriteInstruction(0x99);
  delay_ms(1);
}

void W25Q_SectorErase(uint32_t sector)
{
  uint32_t addr = sector*4096;
  W25Q_WaitBusy();
  W25Q_WriteEnable();
  W25Q_CS = 0;
  delay_us(1);
  SPI_ReadWriteByte(SPI1,0x21);
  SPI_ReadWriteByte(SPI1,addr>>24);
  SPI_ReadWriteByte(SPI1,addr>>16);
  SPI_ReadWriteByte(SPI1,addr>>8);
  SPI_ReadWriteByte(SPI1,addr);
  W25Q_CS = 1;
  delay_us(1);
  W25Q_WaitBusy();
}

void W25Q_BlockErase(uint32_t block)
{
  uint32_t addr = block*65536;
  W25Q_WaitBusy();
  W25Q_WriteEnable();
  W25Q_CS = 0;
  delay_us(1);
  SPI_ReadWriteByte(SPI1,0xDC);
  SPI_ReadWriteByte(SPI1,addr>>24);
  SPI_ReadWriteByte(SPI1,addr>>16);
  SPI_ReadWriteByte(SPI1,addr>>8);
  SPI_ReadWriteByte(SPI1,addr);
  W25Q_CS = 1;
  delay_us(1);
  W25Q_WaitBusy();
}

void W25Q_ChipErase(void)
{
  W25Q_WaitBusy();
  W25Q_WriteEnable();
  W25Q_CS = 0;
  delay_us(1);
  SPI_ReadWriteByte(SPI1,0x60);
  W25Q_CS = 1;
  delay_us(1);
  W25Q_WaitBusy();
}

void W25Q_DataStorage(uint32_t addr,uint8_t *buffer,uint16_t length)
{
  uint16_t i;
  W25Q_WaitBusy();
  W25Q_WriteEnable();
  W25Q_CS = 0;
  delay_us(1);
  SPI_ReadWriteByte(SPI1,0x12);
  SPI_ReadWriteByte(SPI1,addr>>24);
  SPI_ReadWriteByte(SPI1,addr>>16);
  SPI_ReadWriteByte(SPI1,addr>>8);
  SPI_ReadWriteByte(SPI1,addr);
  for(i=0;i<length;i++)
  {
      SPI_ReadWriteByte(SPI1,buffer[i]);
  }
  W25Q_CS = 1;
  delay_us(1);
}

void W25Q_WriteInstruction(uint8_t instuction)
{
  W25Q_WriteEnable();
  W25Q_CS = 0;
  SPI_ReadWriteByte(SPI1,instuction);
  W25Q_CS = 1;
  delay_us(1);
  W25Q_WaitBusy();
}


void W25Q_DataReceive(uint32_t addr,uint8_t *buffer,uint32_t length)
{
  uint32_t i;
  W25Q_CS = 0;
  delay_us(1);
  SPI_ReadWriteByte(SPI1,0x13);
  SPI_ReadWriteByte(SPI1,addr>>24);
  SPI_ReadWriteByte(SPI1,addr>>16);
  SPI_ReadWriteByte(SPI1,addr>>8);
  SPI_ReadWriteByte(SPI1,addr);
  for(i=0;i<length;i++)
  {
    *buffer = SPI_ReadWriteByte(SPI1,0x00);
    buffer++;
  }
  W25Q_CS = 1;
  delay_us(1);
}

void W25Q_WriteSatusRegister(uint8_t reg,uint8_t data)
{
  W25Q_CS = 0;
  delay_us(1);
  SPI_ReadWriteByte(SPI1,0x50);
  W25Q_CS = 1;
  delay_us(1);
  W25Q_CS = 0;
  delay_us(1);
  SPI_ReadWriteByte(SPI1,reg);
  SPI_ReadWriteByte(SPI1,data);
  W25Q_CS = 1;
  delay_us(1);
}
