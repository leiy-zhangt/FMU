#include "taskinit.h"
#include "stdio.h"
//TaskCreate函数声明
void TaskCreate(void)
{
//	taskENTER_CRITICAL();
	//Create LEDTwink
	LEDTwink_Ret = xTaskCreate((TaskFunction_t)LEDTwink,"LEDTwink",32,(void *)1,LEDTwink_Prio,(TaskHandle_t *)(&LEDTwink_TCB));
	if(LEDTwink_Ret == pdPASS) printf("LEDTwink creat successfully!\r\n");
	else printf("LEDTwink creat failed!\r\n");
	//Create TaskMonitor
	TaskMonitor_Ret = xTaskCreate((TaskFunction_t)TaskMonitor,"TaskMonitor",200,(void *)1,TaskMonitor_Prio,(TaskHandle_t *)(&TaskMonitor_TCB));
	if(TaskMonitor_Ret == pdPASS) printf("TaskMonitor creat successfully!\r\n");
	else printf("TaskMonitor creat failed!\r\n");
	//Create SDWrite
	SDWrite_Ret = xTaskCreate((TaskFunction_t)SDWrite,"SDWrite",200,(void *)1,SDWrite_Prio,(TaskHandle_t *)(&SDWrite_TCB));
	if(SDWrite_Ret == pdPASS) printf("SDWrite creat successfully!\r\n");
	else printf("SDWrite creat failed!\r\n");
	//EXIT CRITICAL
//	vTaskSuspend(NULL);
//	taskEXIT_CRITICAL();
	vTaskStartScheduler();
	while(1) ;
}

//LEDTwink函数声明
BaseType_t LEDTwink_Ret;
UBaseType_t LEDTwink_Prio=8;
TaskHandle_t LEDTwink_TCB;

void LEDTwink(void *pvParameters)
{
	while(1)
	{
		HAL_GPIO_TogglePin(SIGNAL_GPIO_Port,SIGNAL_Pin);
		vTaskDelay(125);
	}
}

//TaskMonitor函数声明
BaseType_t TaskMonitor_Ret;
UBaseType_t TaskMonitor_Prio=5;
TaskHandle_t TaskMonitor_TCB;

void TaskMonitor(void *pvParameters)
{
	char InfoBuffer[400];
	while(1)
	{
		vTaskList(InfoBuffer);
		printf("---------------------------------------------\r\n");
		printf("%s",InfoBuffer);
		printf("---------------------------------------------\r\n");
		vTaskGetRunTimeStats(InfoBuffer);
		printf("%s",InfoBuffer);
		printf("---------------------------------------------\r\n");
		vTaskDelay(500);
	}
}

//SDwrite函数声明

BaseType_t SDWrite_Ret;
UBaseType_t SDWrite_Prio=2;
TaskHandle_t SDWrite_TCB;

void SDWrite(void)
{
	FRESULT SDRet;
	uint32_t SDprint = 1;
	SDRet = f_mount(&SDFatFS,SDPath,1);
	if(SDRet != FR_OK)
	{
		switch(SDRet)
		{
			case FR_NO_FILESYSTEM:
				SDRet = f_mkfs(SDPath,FM_FAT32,0,work,_MAX_SS);
				if(SDRet==FR_OK) 
				{
					SDRet = f_mount(&SDFatFS,SDPath,1);
					if(SDRet == FR_OK) printf("SD mount successfully!\r\n");
					else printf("SD mount successfully!\r\n");
				}
				break;
			default:
				printf("SD mount failed!\r\n");
				while(1) ;
		}
	}
	else if(SDRet == FR_OK) printf("SD mount successfully!\r\n");
	vTaskDelay(100);
	SDRet = f_open(&SDFile,"sdtest.txt",FA_WRITE|FA_CREATE_ALWAYS);
	if(SDRet == FR_OK) printf("SD open successfully!\r\n");
	while(1)
	{
//		taskENTER_CRITICAL();
		f_printf(&SDFile,"SD write test:%d!\n",SDprint++);
//		printf("SD write %d!\r\n",SDprint);
		if(SDprint%10 == 0) f_sync(&SDFile);
//		taskEXIT_CRITICAL();
//		vTaskDelay(100);
//		for(num=0;num<9999999;num++)
//		{
//		}
		vTaskDelay(10);
	}
}



