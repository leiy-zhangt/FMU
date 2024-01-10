#include "taskinit.h"
#include "stdio.h"
#include "imu.h"
//TaskCreate函数声明
void TaskCreate(void)
{
	//Create LEDTwink
	LEDTwink_Ret = xTaskCreate((TaskFunction_t)LEDTwink,"LEDTwink",32,(void *)1,LEDTwink_Prio,(TaskHandle_t *)(&LEDTwink_TCB));
	if(LEDTwink_Ret == pdPASS) printf("LEDTwink creat successfully!\r\n");
	else printf("LEDTwink creat failed!\r\n");
	//Create TaskMonitor
//	TaskMonitor_Ret = xTaskCreate((TaskFunction_t)TaskMonitor,"TaskMonitor",200,(void *)1,TaskMonitor_Prio,(TaskHandle_t *)(&TaskMonitor_TCB));
//	if(TaskMonitor_Ret == pdPASS) printf("TaskMonitor creat successfully!\r\n");
//	else printf("TaskMonitor creat failed!\r\n");
	//Create SDWrite
//	SDWrite_Ret = xTaskCreate((TaskFunction_t)SDWrite,"SDWrite",200,(void *)1,SDWrite_Prio,(TaskHandle_t *)(&SDWrite_TCB));
//	if(SDWrite_Ret == pdPASS) printf("SDWrite creat successfully!\r\n");
//	else printf("SDWrite creat failed!\r\n");
	//Create LEDTwink
	IMUReceive_Ret = xTaskCreate((TaskFunction_t)IMUReceive,"IMUReceive",196,(void *)1,IMUReceive_Prio,(TaskHandle_t *)(&IMUReceive_TCB));
	if(IMUReceive_Ret == pdPASS) printf("IMUReceive creat successfully!\r\n");
	else printf("IMUReceive creat failed!\r\n");
	
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
		vTaskDelay(250);
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
		taskENTER_CRITICAL();
		printf("---------------------------------------------\r\n");
		printf("%s",InfoBuffer);
		printf("---------------------------------------------\r\n");
		taskEXIT_CRITICAL();
		vTaskGetRunTimeStats(InfoBuffer);
		taskENTER_CRITICAL();
		printf("%s",InfoBuffer);
		printf("---------------------------------------------\r\n");
		taskEXIT_CRITICAL();
		vTaskDelay(1000);
	}
}

//SDwrite函数声明
BaseType_t SDWrite_Ret;
UBaseType_t SDWrite_Prio=2;
TaskHandle_t SDWrite_TCB;

void SDWrite(void *pvParameters)
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
		f_printf(&SDFile,"SD write test:%d!\n",SDprint++);
		if(SDprint%10 == 0) f_sync(&SDFile);
		vTaskDelay(10);
	}
}

//IMUReceive函数声明
BaseType_t IMUReceive_Ret;
UBaseType_t IMUReceive_Prio=9;
TaskHandle_t IMUReceive_TCB;

void IMUReceive(void *pvParameters)
{
	IMUStatus IMURet;
	IMUSemaphore = xSemaphoreCreateBinary();
	HAL_UART_Receive_DMA(&huart2,IMUReceiveBuff,55);
	__HAL_UART_ENABLE_IT(&huart2,UART_IT_IDLE);
	while(1)
	{
		xSemaphoreTake(IMUSemaphore,portMAX_DELAY);
		HAL_GPIO_WritePin(RS232_IO1_GPIO_Port,RS232_IO1_Pin,GPIO_PIN_SET);
		IMURet = IMUDataConvert(IMUFifoBuff);
		if(IMURet == IMU_OK)
		{
			printf("%0.4f  %0.4f  %0.4f  ",IMUData.acc_x,IMUData.acc_y,IMUData.acc_z);
			printf("%0.4f  %0.4f  %0.4f  ",IMUData.gyr_x,IMUData.gyr_y,IMUData.gyr_z);
			printf("%0.4f  %0.4f  %0.4f  ",IMUData.pitch,IMUData.roll,IMUData.yaw);
			printf("%0.4f  %0.4f  ",IMUData.pressure,IMUData.height);
			printf("%0.4f  %0.4f  %0.4f  %0.4f\r\n",IMUData.quaternion[0],IMUData.quaternion[1],IMUData.quaternion[2],IMUData.quaternion[3]);
		}
		else 
		{
			uint8_t i;
			for(i=0;i<55;i++) printf("%hx ",IMUFifoBuff[i]);
			printf("\r\n");
		}
		HAL_GPIO_WritePin(RS232_IO1_GPIO_Port,RS232_IO1_Pin,GPIO_PIN_RESET);
//		vTaskDelay(2);
	}
}

