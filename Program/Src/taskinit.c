#include "taskinit.h"

//LEDTwink函数声明
BaseType_t LEDTwink_Ret;
UBaseType_t LEDTwink_Prio=2;
TaskHandle_t *LEDTwink_TCB;

void LEDTwink(void)
{
	while(1)
	{
		HAL_GPIO_TogglePin(SIGNAL_GPIO_Port,SIGNAL_Pin);
		HAL_GPIO_TogglePin(RS232_IO1_GPIO_Port,RS232_IO1_Pin);
//		HAL_GPIO_WritePin(RS232_IO1_GPIO_Port,RS232_IO1_Pin,GPIO_PIN_SET);
		HAL_Delay(1000000);
		vTaskDelay(100);
		
//		HAL_GPIO_WritePin(RS232_IO1_GPIO_Port,RS232_IO1_Pin,GPIO_PIN_RESET);
//		vTaskDelay(100);
	}
}

//SDwrite函数声明
BaseType_t SDWrite_Ret;
UBaseType_t SDWrite_Prio=10;
TaskHandle_t *SDWrite_TCB;

void SDwrite(void)
{
	uint32_t num=0;
	uint8_t string[300];
	while(1)
	{
		vTaskList(string);
		printf("%s",string);
		vTaskGetRunTimeStats(string);
		printf("%s",string);
		vTaskDelay(500);
	}
}
