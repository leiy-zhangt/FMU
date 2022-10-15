#include "iic.h"

#if EN_I2C1

void I2C1_Configuration(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	I2C_InitTypeDef I2C_InitStructure;
	//开启时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);
	//配置引脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Pin = I2C1_SCL|I2C1_SDA;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(I2C1_Port,&GPIO_InitStructure);
	//配置复用模式
	GPIO_PinAFConfig(I2C1_Port,I2C1_Pinsourse_SCL,GPIO_AF_I2C1);
	GPIO_PinAFConfig(I2C1_Port,I2C1_Pinsourse_SDA,GPIO_AF_I2C1);
	//配置I2C参数
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = I2C1_Speed;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_OwnAddress1 = 0X11;
  I2C_Init(I2C1,&I2C_InitStructure);
  I2C_Cmd(I2C1,ENABLE);
}
#endif

#ifdef EN_I2C2

void I2C2_Configuration(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	I2C_InitTypeDef I2C_InitStructure;
	//开启时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2,ENABLE);
	//配置引脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Pin = I2C2_SCL|I2C2_SDA;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(I2C2_Port,&GPIO_InitStructure);
	//配置复用模式
	GPIO_PinAFConfig(I2C2_Port,I2C2_Pinsourse_SCL,GPIO_AF_I2C2);
	GPIO_PinAFConfig(I2C2_Port,I2C2_Pinsourse_SDA,GPIO_AF_I2C2);
	//配置I2C参数
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = I2C2_Speed;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_OwnAddress1 = 0X11;
  I2C_Init(I2C2,&I2C_InitStructure);
  I2C_Cmd(I2C2,ENABLE);
}

#endif


void I2C_SendByte(I2C_TypeDef* I2C,uint8_t SlaveAddr,uint8_t WriteAddr,uint8_t Data)
{
  I2C_GenerateSTART(I2C,ENABLE);
  while(!I2C_CheckEvent(I2C,I2C_EVENT_MASTER_MODE_SELECT));
  I2C_Send7bitAddress(I2C,SlaveAddr,I2C_Direction_Transmitter);
  while(!I2C_CheckEvent(I2C,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
  I2C_SendData(I2C,WriteAddr);
  while(!I2C_CheckEvent(I2C,I2C_EVENT_MASTER_BYTE_TRANSMITTED));    
  I2C_SendData(I2C,Data);
  while(!I2C_CheckEvent(I2C,I2C_EVENT_MASTER_BYTE_TRANSMITTED));
  I2C_GenerateSTOP(I2C,ENABLE);
}

void I2C_SendBuffer(I2C_TypeDef* I2C,uint8_t SlaveAddr,uint8_t WriteAddr,uint8_t *buffer,uint16_t length)
{
  while(I2C_GetFlagStatus(I2C,I2C_FLAG_BUSY));
  I2C_GenerateSTART(I2C,ENABLE);
  while(!I2C_CheckEvent(I2C,I2C_EVENT_MASTER_MODE_SELECT));	//清除 EV5
	I2C_Send7bitAddress(I2C,SlaveAddr, I2C_Direction_Transmitter); //写入器件地址
	while(!I2C_CheckEvent(I2C,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));//清除 EV6
  I2C_SendData(I2C,WriteAddr); //内部功能地址
	while(!I2C_CheckEvent(I2C,I2C_EVENT_MASTER_BYTE_TRANSMITTED));//移位寄存器非空，数据寄存器已空，产生EV8，发送数据到DR既清除该事件
	while(length--){ //循环发送数据	
		I2C_SendData(I2C,*buffer); //发送数据
		buffer++; //数据指针移位
		while (!I2C_CheckEvent(I2C,I2C_EVENT_MASTER_BYTE_TRANSMITTED));//清除EV8
	}
	I2C_GenerateSTOP(I2C,ENABLE);//产生停止信号
}

uint8_t I2C_ReadByte(I2C_TypeDef* I2C,uint8_t SlaveAddr,uint8_t ReadAddr)      //I2C读取一个字节
{ 
  uint8_t res;
	while(I2C_GetFlagStatus(I2C,I2C_FLAG_BUSY));
	I2C_GenerateSTART(I2C,ENABLE);
	while(!I2C_CheckEvent(I2C,I2C_EVENT_MASTER_MODE_SELECT));
	I2C_Send7bitAddress(I2C,SlaveAddr,I2C_Direction_Transmitter); 
	while(!I2C_CheckEvent(I2C,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	I2C_SendData(I2C,ReadAddr);
	while(!I2C_CheckEvent(I2C,I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	I2C_GenerateSTART(I2C,ENABLE);
	while(!I2C_CheckEvent(I2C,I2C_EVENT_MASTER_MODE_SELECT));
	I2C_Send7bitAddress(I2C,SlaveAddr,I2C_Direction_Receiver);
	while(!I2C_CheckEvent(I2C,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
	I2C_AcknowledgeConfig(I2C,DISABLE); //最后有一个数据时关闭应答位
	I2C_GenerateSTOP(I2C,ENABLE);	//最后一个数据时使能停止位
  while(!I2C_CheckEvent(I2C,I2C_EVENT_MASTER_BYTE_RECEIVED));
  res = I2C_ReceiveData(I2C);
  I2C_AcknowledgeConfig(I2C,ENABLE);
	return res;
}

void I2C_ReadBuffer(I2C_TypeDef* I2C,uint8_t SlaveAddr,uint8_t ReadAddr,uint8_t* buffer,uint16_t length)
{ //I2C读取数据串（器件地址，寄存器，内部地址，数量）
  while(I2C_GetFlagStatus(I2C,I2C_FLAG_BUSY));
  I2C_GenerateSTART(I2C,ENABLE);//开启信号
  while(!I2C_CheckEvent(I2C,I2C_EVENT_MASTER_MODE_SELECT));	//清除 EV5
  I2C_Send7bitAddress(I2C,SlaveAddr,I2C_Direction_Transmitter); //写入器件地址
  while(!I2C_CheckEvent(I2C,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));//清除 EV6
  //	I2C_Cmd(I2C,ENABLE);
  I2C_SendData(I2C,ReadAddr); //发送读的地址
  while(!I2C_CheckEvent(I2C,I2C_EVENT_MASTER_BYTE_TRANSMITTED)); //清除 EV8
  I2C_GenerateSTART(I2C,ENABLE); //开启信号
  while(!I2C_CheckEvent(I2C,I2C_EVENT_MASTER_MODE_SELECT)); //清除 EV5
  I2C_Send7bitAddress(I2C,SlaveAddr,I2C_Direction_Receiver); //将器件地址传出，主机为读
  while(!I2C_CheckEvent(I2C,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)); //清除EV6
  while(length)
  {
    if(length == 1){ //只剩下最后一个数据时进入 if 语句
      I2C_AcknowledgeConfig(I2C,DISABLE); //最后有一个数据时关闭应答位
      I2C_GenerateSTOP(I2C,ENABLE);	//最后一个数据时使能停止位
    }
    if(I2C_CheckEvent(I2C,I2C_EVENT_MASTER_BYTE_RECEIVED)){ //读取数据
      *buffer = I2C_ReceiveData(I2C);//调用库函数将数据取出到 pBuffer
      buffer++; //指针移位
      length--; //字节数减 1 
    }
  }
  I2C_AcknowledgeConfig(I2C,ENABLE);
}

