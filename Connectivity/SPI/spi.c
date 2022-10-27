#include "spi.h"

#if EN_SPI1
void SPI1_Configuration(void)
{	 
  GPIO_InitTypeDef  GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure;
  //开启时钟
  RCC_AHB1PeriphClockCmd(SPI1_GPIO_CLK, ENABLE);//使能GPIOB时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);//使能SPI1时钟
  //GPIO初始化设置
  GPIO_InitStructure.GPIO_Pin = SPI1_SCK|SPI1_MISO|SPI1_MOSI;//PB3~5复用功能输出	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//无上下拉电阻
  GPIO_Init(SPI1_Port, &GPIO_InitStructure);//初始化

  GPIO_PinAFConfig(SPI1_Port,SPI1_Pinsourse_SCK,GPIO_AF_SPI1); //PB3复用为 SPI1
  GPIO_PinAFConfig(SPI1_Port,SPI1_Pinsourse_MISO,GPIO_AF_SPI1); //PB4复用为 SPI1
  GPIO_PinAFConfig(SPI1_Port,SPI1_Pinsourse_MOSI,GPIO_AF_SPI1); //PB5复用为 SPI1
  
  //这里只针对SPI口初始化
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1,ENABLE);//复位SPI1
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1,DISABLE);//停止复位SPI1

  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//设置SPI工作模式:设置为主SPI
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//设置SPI的数据大小:SPI发送接收8位帧结构
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;		//串行同步时钟的空闲状态为高电平
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;	//串行同步时钟的第一个跳变沿（上升或下降）数据被采样
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;		//定义波特率预分频的值:波特率预分频值为16
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
  SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRC值计算的多项式
  SPI_Init(SPI1, &SPI_InitStructure);  //根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器
  
  SPI_Cmd(SPI1, ENABLE); //使能SPI外设

	 SPI_CS_Configuration();//初始化片选引脚
}   
//SPI1速度设置函数
//SPI速度=fAPB2/分频系数
//@ref SPI_BaudRate_Prescaler:SPI_BaudRatePrescaler_2~SPI_BaudRatePrescaler_256  
//fAPB2时钟一般为84Mhz：
void SPI1_SetSpeed(u8 SPI_BaudRatePrescaler)
{
  assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));//判断有效性
	SPI1->CR1&=0XFFC7;//位3-5清零，用来设置波特率
	SPI1->CR1|=SPI_BaudRatePrescaler;	//设置SPI1速度 
	SPI_Cmd(SPI1,ENABLE); //使能SPI1


}   
#endif

#if EN_SPI2
void SPI2_Configuration(void)
{	 
  GPIO_InitTypeDef  GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure;
  //开启时钟
  RCC_AHB1PeriphClockCmd(SPI2_GPIO_CLK, ENABLE);//使能GPIOB时钟
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);//使能SPI1时钟
  //GPIO初始化设置
  GPIO_InitStructure.GPIO_Pin = SPI2_SCK|SPI2_MISO|SPI2_MOSI;//PB3~5复用功能输出	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(SPI2_Port, &GPIO_InitStructure);//初始化

  GPIO_PinAFConfig(SPI2_Port,SPI2_Pinsourse_SCK,GPIO_AF_SPI2); //PB3复用为 SPI1
  GPIO_PinAFConfig(SPI2_Port,SPI2_Pinsourse_MISO,GPIO_AF_SPI2); //PB4复用为 SPI1
  GPIO_PinAFConfig(SPI2_Port,SPI2_Pinsourse_MOSI,GPIO_AF_SPI2); //PB5复用为 SPI1

  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//设置SPI工作模式:设置为主SPI
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//设置SPI的数据大小:SPI发送接收8位帧结构
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;		//串行同步时钟的空闲状态为高电平
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;	//串行同步时钟的第二个跳变沿（上升或下降）数据被采样
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;		//定义波特率预分频的值:波特率预分频值为256
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_LSB;	//指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
  SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRC值计算的多项式
  SPI_Init(SPI2, &SPI_InitStructure);  //根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器
  SPI_Cmd(SPI2, ENABLE); //使能SPI外设		 
}   
#endif

uint16_t SPI_ReadWriteByte(SPI_TypeDef*SPI,uint16_t Data)
{		 			 
  while (SPI_I2S_GetFlagStatus(SPI,SPI_I2S_FLAG_TXE) == RESET){}//等待发送区空  
  SPI_I2S_SendData(SPI, Data); //通过外设SPIx发送一个byte  数据	
  while (SPI_I2S_GetFlagStatus(SPI,SPI_I2S_FLAG_RXNE) == RESET){} //等待接收完一个byte  
  return SPI_I2S_ReceiveData(SPI); //返回通过SPIx最近接收的数据			    
}

void SPI_CS_Configuration(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  
   //配置ADXL357片选引脚
  GPIO_InitStructure.GPIO_Pin = ADXL_CS_Pin; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  ADXL_CS = 1;
  
  //配置BMI088片选引脚
  GPIO_InitStructure.GPIO_Pin = ACC_CS_Pin|GYR_CS_Pin; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  ACC_CS = 1;
  GYR_CS = 1;
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_12|GPIO_Pin_1; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  PCout(1) = 1;
  PCout(12) = 1;
  BMM_CS = 1;
  
}







