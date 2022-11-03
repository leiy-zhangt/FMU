#include "lora.h"

void LORA_Configuration(uint16_t lora_addr,int32_t bound)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_6; //��������
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//����Ϊ���
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//��©���
  GPIO_InitStructure.GPIO_Speed = GPIO_Low_Speed;//�������
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��
 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; //��������
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//����Ϊ���
  GPIO_InitStructure.GPIO_Speed = GPIO_Low_Speed;//�������
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��
  LORA_NRST = 1;
  LORA_M0 = 0;
  LORA_M1 = 0;
  delay_ms(100);
  USART3_Configuration(9600,ENABLE);
  while(LORA_Status == 0);
  LORA_M0 = 0;
  LORA_M1 = 1;
  delay_ms(10);
  while(LORA_Status == 0);
  //��������ģʽ
  LORA_WriteCmd(0x00,lora_addr>>8);
  delay_ms(50);
  LORA_WriteCmd(0x01,lora_addr);//���õ�ַ
  delay_ms(50);
  LORA_WriteCmd(0x03,0xA7);//���ò��������ٶ�
  delay_ms(50);
  LORA_WriteCmd(0x04,0x00);//�����ŵ�
  delay_ms(50);//������ģʽ
  LORA_WriteCmd(0x05,Lora_channel);//�����ŵ�
  delay_ms(50);//������ģʽ
  while(LORA_Status == 0);
  LORA_M0 = 0;
  LORA_M1 = 0;
  delay_ms(1);
  while(LORA_Status == 0);
  USART3_Configuration(bound,ENABLE);
}

void LORA_WriteCmd(uint8_t addr,uint8_t cmd)
{
  while (USART_GetFlagStatus(USART3,USART_FLAG_TC) == RESET);
  USART_SendData(USART3,0xC0);
  while (USART_GetFlagStatus(USART3,USART_FLAG_TC) == RESET);
  USART_SendData(USART3,addr);
  while (USART_GetFlagStatus(USART3,USART_FLAG_TC) == RESET);
  USART_SendData(USART3,0x01);
  while (USART_GetFlagStatus(USART3,USART_FLAG_TC) == RESET);
  USART_SendData(USART3,cmd);
  while (USART_GetFlagStatus(USART3,USART_FLAG_TC) == RESET);
}
