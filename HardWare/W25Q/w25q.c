#include "w25q.h"    
/*
W25Q一页为256byte,一个扇区为16Kbyte
*/
void W25Q_Configuration(void)
{ 
    //W25Q初始化配置
  uint8_t res;
  W25Q_WriteInstruction(0xB7);
  res = W25Q_ReadData(0x9f);
  res = W25Q_ReadData(0x05);
  res = W25Q_ReadData(0x35);
  res = W25Q_ReadData(0x15);
//  W25Q_WriteInstruction(0XA0,0X00);
//  W25Q_WriteInstruction(0XB0,0X00);
}  

void W25Q_WriteEnable(void)
{  
  W25Q_CS = 0;
  delay_us(1);
  SPI_ReadWriteByte(SPI1,0X06);
  W25Q_CS = 1;
  delay_us(1);
}

FlagStatus W25Q_CheckBusy(void)
{
    W25Q_CS = 0;
    delay_us(1);
    SPI_ReadWriteByte(SPI1,0X05);
    SPI_ReadWriteByte(SPI1,0XC0);
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
    SPI_ReadWriteByte(SPI1,0xC0);
    while(SPI_ReadWriteByte(SPI1,0X00)&0X01);
    W25Q_CS = 1;
  delay_us(1);
}

void W25Q_Reset(void)
{
    W25Q_CS = 0;
    delay_us(1);
    while(SPI_ReadWriteByte(SPI1,0xFF));
    W25Q_CS = 1;
}

void W25Q_BlockErase(uint16_t block)
{
    uint16_t page = block*64;
    W25Q_WriteEnable();
    W25Q_CS = 0;
    delay_us(1);
    SPI_ReadWriteByte(SPI1,0XD8);
    SPI_ReadWriteByte(SPI1,0X00);
    SPI_ReadWriteByte(SPI1,page>>8);
    SPI_ReadWriteByte(SPI1,page);
    W25Q_CS = 1;
    W25Q_WaitBusy();
}

void W25Q_ChipErase(void)
{
  W25Q_CS = 0;
  delay_us(1);
  SPI_ReadWriteByte(SPI1,0x60);
  W25Q_CS = 1;
  delay_us(1);
}

void W25Q_DataStorage(uint32_t addr,uint8_t *buffer)
{
  static uint16_t length;
  W25Q_WaitBusy();
  W25Q_WriteEnable();
  W25Q_CS = 0;
  SPI_ReadWriteByte(SPI1,0x12);
  SPI_ReadWriteByte(SPI1,addr>>24);
  SPI_ReadWriteByte(SPI1,addr>>16);
  SPI_ReadWriteByte(SPI1,addr>>8);
  SPI_ReadWriteByte(SPI1,addr);
  for(length=0;length<256;length++)
  {
      SPI_ReadWriteByte(SPI1,buffer[length]);
  }
  W25Q_CS = 1;
  //W25Q_WriteEnable();
//  delay_us(5);
//  W25Q_CS = 0;
//  SPI_ReadWriteByte(SPI1,0X10);
//  SPI_ReadWriteByte(SPI1,0X00);
//  SPI_ReadWriteByte(SPI1,page>>8);
//  SPI_ReadWriteByte(SPI1,page);
//  W25Q_CS = 1;
    
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



uint8_t W25Q_ReadData(uint8_t addr)
{
  uint8_t res;
  W25Q_CS = 0;
  SPI_ReadWriteByte(SPI1,addr);
  res = SPI_ReadWriteByte(SPI1,0x00);
  W25Q_CS = 1;
  delay_us(1);
  return res;
}

void  W25Q_DataReceive(u8 *buffer,uint16_t page)
{
    static uint16_t i;
    W25Q_CS = 0;
    delay_us(1);
    SPI_ReadWriteByte(SPI1,0X13);
    SPI_ReadWriteByte(SPI1,0X00);
    SPI_ReadWriteByte(SPI1,page>>8);
    SPI_ReadWriteByte(SPI1,page);
    W25Q_CS = 1;
    W25Q_WaitBusy();
    W25Q_CS = 0;
    delay_us(1);
    SPI_ReadWriteByte(SPI1,0X03);
    SPI_ReadWriteByte(SPI1,0X00);
    SPI_ReadWriteByte(SPI1,0X00);
    SPI_ReadWriteByte(SPI1,0X00);
    for(i=0;i<2048;i++)
    {
        buffer[i] = SPI_ReadWriteByte(SPI1,0X00);
    }
    W25Q_CS = 1;
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
