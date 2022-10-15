#ifndef __IIC_H
#define __IIC_H

#include "sys.h"

#define EN_I2C1 1		//使能I2C1
#define EN_I2C2 1		//使能I2C2

#define I2C1_Speed 350000
#define I2C2_Speed 350000
#define I2C1_Port GPIOB
#define I2C2_Port GPIOB
#define I2C1_GPIO_CLK RCC_AHB1Periph_GPIOB 
#define I2C2_GPIO_CLK RCC_AHB1Periph_GPIOB 
#define I2C1_SCL GPIO_Pin_6
#define I2C1_SDA GPIO_Pin_7
#define I2C1_Pinsourse_SCL GPIO_PinSource6
#define I2C1_Pinsourse_SDA GPIO_PinSource7
#define I2C2_SCL GPIO_Pin_10
#define I2C2_SDA GPIO_Pin_11
#define I2C2_Pinsourse_SCL GPIO_PinSource10
#define I2C2_Pinsourse_SDA GPIO_PinSource11

void I2C1_Configuration(void);
void I2C2_Configuration(void);

void I2C_SendByte(I2C_TypeDef* I2C,uint8_t SlaverAddr,uint8_t WriteAddr,uint8_t Data);
void I2C_SendBuffer(I2C_TypeDef* I2C,uint8_t SlaveAddr,uint8_t WriteAddr,uint8_t *buffer,u16 length);
uint8_t I2C_ReadByte(I2C_TypeDef* I2C,uint8_t SlaveAddr,uint8_t ReadAddr);
void I2C_ReadBuffer(I2C_TypeDef* I2C,uint8_t SlaveAddr,uint8_t ReadAddr,uint8_t* buffer,u16 length);

#endif

