#include "w25n.h"    

//2KByte为一个Page，总共65536页
//64Page为1个Block，总共1024块
//第64页为已保存的数据量
//第1页为传感器偏移量

uint16_t page=0;

void W25N_Configuration(void)
{ 
    GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能GPIOA时钟
    GPIO_InitStructure.GPIO_Pin = W25N_CS_Pin|W25N_WP_Pin|W25N_HOLD_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//输出
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(W25N_Port,&GPIO_InitStructure);//初始化
    W25N_CS = 1;
    W25N_HOLD = 1;
    W25N_WP = 1;
    //W25N初始化配置
    W25N_SendData(0XA0,0X00);
    W25N_SendData(0XB0,0X00);
}  

void W25N_WriteEnable(void)
{  
    W25N_CS = 0;
    delay_us(1);
    SPI_ReadWriteByte(SPI1,0X06);
    W25N_CS = 1;
}

FlagStatus W25N_CheckBusy(void)
{
    W25N_CS = 0;
    delay_us(1);
    SPI_ReadWriteByte(SPI1,0X05);
    SPI_ReadWriteByte(SPI1,0XC0);
    if(SPI_ReadWriteByte(SPI1,0X00)&0X01)
    {
        W25N_CS = 1;
        return SET;
    }
    else 
    {
        W25N_CS = 1;
        return RESET;
    }  
}

void W25N_WaitBusy(void)
{
    W25N_CS = 0;
    delay_us(1);
    SPI_ReadWriteByte(SPI1,0X05);
    SPI_ReadWriteByte(SPI1,0XC0);
    while(SPI_ReadWriteByte(SPI1,0X00)&0X01);
    W25N_CS = 1;
}

void W25N_Reset(void)
{
    W25N_CS = 0;
    delay_us(1);
    while(SPI_ReadWriteByte(SPI1,0XFF));
    W25N_CS = 1;
}

void W25N_BlockErase(uint16_t block)
{
    uint16_t page = block*64;
    W25N_WriteEnable();
    W25N_CS = 0;
    delay_us(1);
    SPI_ReadWriteByte(SPI1,0XD8);
    SPI_ReadWriteByte(SPI1,0X00);
    SPI_ReadWriteByte(SPI1,page>>8);
    SPI_ReadWriteByte(SPI1,page);
    W25N_CS = 1;
    W25N_WaitBusy();
}

void W25N_ChipErase(void)
{
    uint16_t block=0;
    while(block<1024) W25N_BlockErase(block++);
}

void W25N_DataWrirte(u8 *buffer,uint16_t page)
{
    static uint16_t length;
    W25N_WriteEnable();
    W25N_CS = 0;
    SPI_ReadWriteByte(SPI1,0X02);
    SPI_ReadWriteByte(SPI1,0X00);
    SPI_ReadWriteByte(SPI1,0X00);
    for(length=0;length<2048;length++)
    {
        SPI_ReadWriteByte(SPI1,buffer[length]);
    }
    W25N_CS = 1;
    //W25N_WriteEnable();
    delay_us(5);
    W25N_CS = 0;
    SPI_ReadWriteByte(SPI1,0X10);
    SPI_ReadWriteByte(SPI1,0X00);
    SPI_ReadWriteByte(SPI1,page>>8);
    SPI_ReadWriteByte(SPI1,page);
    W25N_CS = 1;
    W25N_WaitBusy();
}

void W25N_SendData(u8 addr,u8 data)
{
    W25N_WriteEnable();
    W25N_CS = 0;
    SPI_ReadWriteByte(SPI1,0X1F);
    SPI_ReadWriteByte(SPI1,addr);
    SPI_ReadWriteByte(SPI1,data);
    W25N_CS = 1;
}

void  W25N_DataReceive(u8 *buffer,uint16_t page)
{
    static uint16_t i;
    W25N_CS = 0;
    delay_us(1);
    SPI_ReadWriteByte(SPI1,0X13);
    SPI_ReadWriteByte(SPI1,0X00);
    SPI_ReadWriteByte(SPI1,page>>8);
    SPI_ReadWriteByte(SPI1,page);
    W25N_CS = 1;
    W25N_WaitBusy();
    W25N_CS = 0;
    delay_us(1);
    SPI_ReadWriteByte(SPI1,0X03);
    SPI_ReadWriteByte(SPI1,0X00);
    SPI_ReadWriteByte(SPI1,0X00);
    SPI_ReadWriteByte(SPI1,0X00);
    for(i=0;i<2048;i++)
    {
        buffer[i] = SPI_ReadWriteByte(SPI1,0X00);
    }
    W25N_CS = 1;
}












