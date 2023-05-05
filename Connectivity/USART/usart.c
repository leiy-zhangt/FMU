#include "usart.h"
	  	 
//使UASRT串口可用printf函数发送
//在usart.h文件里可更换使用printf函数的串口号	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE {
	int handle; 
}; 
FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x){ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f){      
	while((USART_n->SR&0X40)==0);//循环发送,直到发送完毕   
    USART_n->DR = (u8) ch;      
	return ch;
}
#endif 

USART_TypeDef * USART_X=USART1;

void USART_printf(char *fmt, ...){ 
	char buffer[USART_REC_LEN+1];  // 数据长度
	uint8_t i = 0;	
	va_list arg_ptr;
	va_start(arg_ptr, fmt);  
	vsnprintf(buffer, USART_REC_LEN+1, fmt, arg_ptr);
	while ((i < USART_REC_LEN) && (i < strlen(buffer))){
        USART_SendData(USART_X, (uint8_t) buffer[i++]);
        while (USART_GetFlagStatus(USART_X, USART_FLAG_TC) == RESET); 
	}
	va_end(arg_ptr);
}

/*
USART1串口相关程序
*/
 
#if EN_USART1   //USART1使用与屏蔽选择
uint8_t USART1_RX_BUF[USART1_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
uint16_t USART1_RX_STA=0;       //接收状态标记	  

/*
USART1专用的printf函数
当同时开启2个以上串口时，printf函数只能用于其中之一，其他串口要自创独立的printf函数
调用方法：USART1_printf("123"); //向USART2发送字符123
*/
void USART1_printf (char *fmt, ...){ 
	char buffer[USART1_REC_LEN+1];  // 数据长度
	uint8_t i = 0;	
	va_list arg_ptr;
	va_start(arg_ptr, fmt);  
	vsnprintf(buffer, USART1_REC_LEN+1, fmt, arg_ptr);
	while ((i < USART1_REC_LEN) && (i < strlen(buffer))){
        USART_SendData(USART1, (uint8_t) buffer[i++]);
        while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET); 
	}
	va_end(arg_ptr);
}

void USART1_Configuration(uint32_t bound,FunctionalState ITStatus){ //串口1初始化并启动
  //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;	 
  
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);	//使能USART1时钟
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);  //使能GPIOA时钟
  
  //USART 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//一般设置为9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART1, &USART_InitStructure); //初始化串口
  USART_ITConfig(USART1, USART_IT_RXNE, ITStatus);//开启ENABLE/关闭DISABLE中断
  
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9复用为USART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10复用为USART1
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //选中GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA9，PA10
  //Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ITStatus;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器 

  USART_Cmd(USART1, ENABLE);                    //使能串口 
}

void USART1_IRQHandler(void){ //串口1中断服务程序（固定的函数名不能修改）	
	uint8_t Res;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)   //接收中断(接收到的数据必须是0x0d 0x0a结尾)	
  { 
    USART_ClearITPendingBit(USART1, USART_IT_RXNE);
		Res=USART_ReceiveData(USART1);//读取接收到的数据
//		printf("%c",Res); //把收到的数据发送回电脑
    if((USART1_RX_STA&0x8000)==0)//接收未完成
		{
			if(USART1_RX_STA&0x4000)//接收到了0x0d
			{
				if(Res!=0x0A)USART1_RX_STA=0;//接收错误,重新开始
				else USART1_RX_STA|=0x8000;	//接收完成了 
			}
			else //还没收到0X0D
			{	
				if(Res==0x0D)USART1_RX_STA|=0x4000;
				else
				{
					USART1_RX_BUF[USART1_RX_STA&0x3FFF]=Res ;
					USART1_RX_STA++;
					if(USART1_RX_STA>(USART1_REC_LEN-1))USART1_RX_STA=0;//接收数据错误,重新开始接收	  
				}		 
			}
		} 
    if((USART1_RX_STA&0x8000))
    {
      USART1_RX_BUF[USART1_RX_STA&0x3FFF] = 0;
      USART1_RX_STA = 0;
      USART_X = USART1;
      Command_Receive(USART1_RX_BUF);
    }
	} 
} 

#endif	

/*
USART2串口相关程序
*/
#if EN_USART2   //USART2使用与屏蔽选择
uint8_t USART2_RX_BUF[USART2_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
uint16_t USART2_RX_STA=0;       //接收状态标记	  

/*
USART2专用的printf函数
当同时开启2个以上串口时，printf函数只能用于其中之一，其他串口要自创独立的printf函数
调用方法：USART2_printf("123"); //向USART2发送字符123
*/
void USART2_printf (char *fmt, ...){ 
	char buffer[USART2_REC_LEN+1];  // 数据长度
	uint8_t i = 0;	
	va_list arg_ptr;
	va_start(arg_ptr, fmt);  
	vsnprintf(buffer, USART2_REC_LEN+1, fmt, arg_ptr);
	while ((i < USART2_REC_LEN) && (i < strlen(buffer))){
        USART_SendData(USART2, (uint8_t) buffer[i++]);
        while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET); 
	}
	va_end(arg_ptr);
}


void USART2_Configuration(uint32_t bound,FunctionalState ITStatus){ //串口1初始化并启动
  //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;	 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);	//使能USART1时钟
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);  
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2); //GPIOA2复用为USART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2); //GPIOA3复用为USART1
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; //GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure); 
  //Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//抢占优先级5
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//子优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ITStatus;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器 
  //USART 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//一般设置为9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART2, &USART_InitStructure); //初始化串口
  USART_ITConfig(USART2, USART_IT_RXNE, ITStatus);//开启ENABLE/关闭DISABLE中断
  USART_Cmd(USART2, ENABLE);                    //使能串口 
}

void USART2_IRQHandler(void){ //串口2中断服务程序（固定的函数名不能修改）
	uint8_t Res;
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)   //接收中断(接收到的数据必须是0x0d 0x0a结尾)	
  { 
		Res=USART_ReceiveData(USART2);//读取接收到的数据
//		printf("%c",Res); //把收到的数据发送回电脑
    if((USART2_RX_STA&0x8000)==0)//接收未完成
		{
			if(USART2_RX_STA&0x4000)//接收到了0x0d
			{
				if(Res!=0x0A)USART2_RX_STA=0;//接收错误,重新开始
				else USART2_RX_STA|=0x8000;	//接收完成了 
			}
			else //还没收到0X0D
			{	
				if(Res==0x0D)USART2_RX_STA|=0x4000;
				else
				{
					USART2_RX_BUF[USART2_RX_STA&0x3FFF]=Res ;
					USART2_RX_STA++;
					if(USART2_RX_STA>(USART2_REC_LEN-1))USART2_RX_STA=0;//接收数据错误,重新开始接收	  
				}		 
			}
		} 
    if((USART2_RX_STA&0x8000))
    {
      if(GPS_status)
      {
        USART2_RX_STA = 0;
        if(USART2_RX_BUF[0]==0x24)
        {
          if(GPS_status) GPS_Solution(USART2_RX_BUF);
        }
      }
    }
	}
  USART_ClearITPendingBit(USART2, USART_IT_RXNE);  
}

#endif	


#if EN_USART3   //如果使能了接收
uint8_t USART3_RX_BUF[USART3_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
uint16_t USART3_RX_STA=0;       //接收状态标记	  

/*
USART3专用的printf函数
当同时开启2个以上串口时，printf函数只能用于其中之一，其他串口要自创独立的printf函数
调用方法：USART3_printf("123"); //向USART3发送字符123
*/
void USART3_printf (char *fmt, ...){ 
	char buffer[USART3_REC_LEN+1];  // 数据长度
	uint8_t i = 0;	
	va_list arg_ptr;
	va_start(arg_ptr, fmt);  
	vsnprintf(buffer, USART3_REC_LEN+1, fmt, arg_ptr);
	while ((i < USART3_REC_LEN) && (i < strlen(buffer))){
        USART_SendData(USART3, (uint8_t) buffer[i++]);
        while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET); 
	}
	va_end(arg_ptr);
}

void USART3_Configuration(uint32_t bound,FunctionalState ITStatus){ //串口1初始化并启动
  //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;	 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);	//使能USART1时钟
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);  
  
  USART_InitStructure.USART_BaudRate = bound;//一般设置为9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;	//收发模式
  USART_ITConfig(USART3, USART_IT_RXNE, ITStatus);//开启ENABLE/关闭DISABLE中断
  USART_Init(USART3, &USART_InitStructure); //初始化串口
  
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3); //GPIOB10复用为USART3
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3); //GPIOB11复用为USART3
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; //选中GPIOB10与GPIOB11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOB,&GPIO_InitStructure); //初始化PB10，PB11
  //Usart3 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ITStatus;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器 
  //USART 初始化设置
	
  USART_ITConfig(USART3, USART_IT_RXNE, ITStatus);//开启ENABLE/关闭DISABLE中断
  USART_Cmd(USART3, ENABLE);                    //使能串口 
}

//串口3中断服务程序（固定的函数名不能修改）
//void USART3_IRQHandler(void){ 	
//  uint8_t Res;
//	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET){  //接收中断(接收到的数据必须是0x0d 0x0a结尾)		
//		Res=USART_ReceiveData(USART3);//读取接收到的数据
//		printf("%c",Res); //把收到的数据发送回电脑
//    if((USART3_RX_STA&0x8000)==0)//接收未完成
//		{
//			if(USART3_RX_STA&0x4000)//接收到了0x0d
//			{
//				if(Res!=0x0a)USART3_RX_STA=0;//接收错误,重新开始
//				else USART3_RX_STA|=0x8000;	//接收完成了 
//			}
//			else //还没收到0X0D
//			{	
//				if(Res==0x0d)USART3_RX_STA|=0x4000;
//				else
//				{
//					USART3_RX_BUF[USART3_RX_STA&0X3FFF]=Res ;
//					USART3_RX_STA++;
//					if(USART3_RX_STA>(USART3_REC_LEN-1))USART3_RX_STA=0;//接收数据错误,重新开始接收	  
//				}		 
//			}
//		}   	
//	} 
//} 

void USART3_IRQHandler(void){ //串口1中断服务程序（固定的函数名不能修改）	
	uint8_t Res;
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)   //接收中断(接收到的数据必须是0x0d 0x0a结尾)	
  { 
    USART_ClearITPendingBit(USART3, USART_IT_RXNE);
		Res=USART_ReceiveData(USART3);//读取接收到的数据
//		printf("%c",Res); //把收到的数据发送回电脑
    if((USART3_RX_STA&0x8000)==0)//接收未完成
		{
			if(USART3_RX_STA&0x4000)//接收到了0x0d
			{
				if(Res!=0x0A)USART3_RX_STA=0;//接收错误,重新开始
				else USART3_RX_STA|=0x8000;	//接收完成了 
			}
			else //还没收到0X0D
			{	
				if(Res==0x0D)USART3_RX_STA|=0x4000;
				else
				{
					USART3_RX_BUF[USART3_RX_STA&0x3FFF]=Res ;
					USART3_RX_STA++;
					if(USART3_RX_STA>(USART3_REC_LEN-1))USART3_RX_STA=0;//接收数据错误,重新开始接收	  
				}		 
			}
		} 
    if((USART3_RX_STA&0x8000))
    {
      USART3_RX_BUF[USART3_RX_STA&0x3FFF] = 0;
      USART3_RX_STA = 0;
      USART_X = USART3;
      Command_Receive(USART3_RX_BUF);
    }
	} 
} 

#endif	

#if EN_USART4   //如果使能了接收
uint8_t USART4_RX_BUF[USART4_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
uint16_t USART4_RX_STA=0;       //接收状态标记	  

/*
USART3专用的printf函数
当同时开启2个以上串口时，printf函数只能用于其中之一，其他串口要自创独立的printf函数
调用方法：USART3_printf("123"); //向USART3发送字符123
*/
void USART4_printf (char *fmt, ...){ 
	char buffer[USART4_REC_LEN+1];  // 数据长度
	uint8_t i = 0;	
	va_list arg_ptr;
	va_start(arg_ptr, fmt);  
	vsnprintf(buffer, USART4_REC_LEN+1, fmt, arg_ptr);
	while ((i < USART4_REC_LEN) && (i < strlen(buffer))){
        USART_SendData(UART4, (uint8_t) buffer[i++]);
        while (USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET); 
	}
	va_end(arg_ptr);
}

void USART4_Configuration(uint32_t bound,FunctionalState ITStatus){ //串口1初始化并启动
  //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;	 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);	//使能USART1时钟
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);  
  
  USART_InitStructure.USART_BaudRate = bound;//一般设置为9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_2;//二个停止位
	USART_InitStructure.USART_Parity = USART_Parity_Even;//偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;	//收发模式
  USART_ITConfig(UART4, USART_IT_RXNE, ITStatus);//开启ENABLE/关闭DISABLE中断
  USART_Init(UART4, &USART_InitStructure); //初始化串口
  
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_UART4); //GPIOB10复用为USART3
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_UART4); //GPIOB11复用为USART3
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; //选中GPIOB10与GPIOB11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOC,&GPIO_InitStructure); //初始化PB10，PB11
  //Usart3 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ITStatus;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器 
  //USART 初始化设置
	
  USART_ITConfig(UART4, USART_IT_RXNE, ITStatus);//开启ENABLE/关闭DISABLE中断
  USART_Cmd(UART4, ENABLE);                    //使能串口 
}

void UART4_IRQHandler(void){ //串口1中断服务程序（固定的函数名不能修改）
  uint8_t Res;
	if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)   //接收中断(接收到的数据必须是0x0d 0x0a结尾)	
  { 
    USART_ClearITPendingBit(UART4, USART_IT_RXNE);
		Res=USART_ReceiveData(UART4);//读取接收到的数据
    if((USART4_RX_STA&0x8000)==0)//接收未完成
		{
			if(USART4_RX_STA&0x4000)//接收到了0x0d
			{
				if(Res!=0x0A)USART4_RX_STA=0;//接收错误,重新开始
				else USART4_RX_STA|=0x8000;	//接收完成了 
			}
			else //还没收到0X0D
			{	
				if(Res==0x0D)USART4_RX_STA|=0x4000;
				else
				{
					USART4_RX_BUF[USART4_RX_STA&0x3FFF]=Res ;
					USART4_RX_STA++;
					if(USART4_RX_STA>(USART4_REC_LEN-1))USART4_RX_STA=0;//接收数据错误,重新开始接收	  
				}		 
			}
		} 
    if((USART4_RX_STA&0x8000))
    {
      USART4_RX_STA = 0;
      if(USART4_RX_BUF[0] == 0)
      {
        for(uint8_t n=0;n<5;n++)
        {
          RemoteChannle[n] = ((USART4_RX_BUF[2*n+1]*100+USART4_RX_BUF[2*(n+1)])-352)*0.744f+1000;
        }
      }
    }
  }
} 

#endif	

/*
a符号的作用：
%d 十进制有符号整数
%u 十进制无符号整数
%f 浮点数
%s 字符串
%c 单个字符
%p 指针的值
%e 指数形式的浮点数
%x, %X 无符号以十六进制表示的整数
%o 无符号以八进制表示的整数
%g 自动选择合适的表示法
%p 输出地址符
*/
