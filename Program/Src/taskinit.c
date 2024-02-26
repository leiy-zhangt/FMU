#include "taskinit.h"
#include "stdio.h"
#include "imu.h"
#include "gnss.h"
#include "receiver.h"
#include "string.h"

EventGroupHandle_t FMUCheck_Status; 

//TaskCreate函数声明
void TaskCreate(void)
{
	FMUCheck_Status = xEventGroupCreate();//创建自检事件标志位
	ReceiverSemaphore = xSemaphoreCreateBinary();
	IMUSemaphore = xSemaphoreCreateBinary();
	GNSSSemaphore = xSemaphoreCreateBinary();
	//Create LEDTwink
	LEDTwink_Ret = xTaskCreate((TaskFunction_t)LEDTwink,"LEDTwink",32,(void *)1,LEDTwink_Prio,(TaskHandle_t *)(&LEDTwink_TCB));
	if(LEDTwink_Ret == pdPASS) printf("LEDTwink creat successfully!\r\n");
	else printf("LEDTwink creat failed!\r\n");
	//Create FMUCheck
//	FMUCheck_Ret = xTaskCreate((TaskFunction_t)FMUCheck,"FMUCheck",64,(void *)1,FMUCheck_Prio,(TaskHandle_t *)(&FMUCheck_TCB));
//	if(FMUCheck_Ret == pdPASS) printf("FMUCheck creat successfully!\r\n");
//	else printf("FMUCheck creat failed!\r\n");
	//Create TaskMonitor
//	TaskMonitor_Ret = xTaskCreate((TaskFunction_t)TaskMonitor,"TaskMonitor",200,(void *)1,TaskMonitor_Prio,(TaskHandle_t *)(&TaskMonitor_TCB));
//	if(TaskMonitor_Ret == pdPASS) printf("TaskMonitor creat successfully!\r\n");
//	else printf("TaskMonitor creat failed!\r\n");
//	//Create SDWrite
//	SDWrite_Ret = xTaskCreate((TaskFunction_t)SDWrite,"SDWrite",200,(void *)1,SDWrite_Prio,(TaskHandle_t *)(&SDWrite_TCB));
//	if(SDWrite_Ret == pdPASS) printf("SDWrite creat successfully!\r\n");
//	else printf("SDWrite creat failed!\r\n");
	//Create IMUReceive
	IMUReceive_Ret = xTaskCreate((TaskFunction_t)IMUReceive,"IMUReceive",128,(void *)1,IMUReceive_Prio,(TaskHandle_t *)(&IMUReceive_TCB));
	if(IMUReceive_Ret == pdPASS) printf("IMUReceive creat successfully!\r\n");
	else printf("IMUReceive creat failed!\r\n");
//	//Create GNSSReceive
//	GNSSReceive_Ret = xTaskCreate((TaskFunction_t)GNSSReceive,"GNSSReceive",196,(void *)1,GNSSReceive_Prio,(TaskHandle_t *)(&GNSSReceive_TCB));
//	if(GNSSReceive_Ret == pdPASS) printf("GNSSReceive creat successfully!\r\n");
//	else printf("GNSSReceive creat failed!\r\n");
//	//Create ReceiverReceive
//	ReceiverReceive_Ret = xTaskCreate((TaskFunction_t)ReceiverReceive,"ReceiverReceive",96,(void *)1,ReceiverReceive_Prio,(TaskHandle_t *)(&ReceiverReceive_TCB));
//	if(ReceiverReceive_Ret == pdPASS) printf("ReceiverReceive creat successfully!\r\n");
//	else printf("ReceiverReceive creat failed!\r\n");
	//Start
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

//FMUCheck函数声明
BaseType_t FMUCheck_Ret;
UBaseType_t FMUCheck_Prio=22;
TaskHandle_t FMUCheck_TCB;

void FMUCheck(void *pvParameters)
{
	GNSSStatus GNSSRet;
//	FMUCheck_Status = xEventGroupCreate();
	xEventGroupClearBits(FMUCheck_Status,0xFFFF);
	while(1)
	{
		xSemaphoreTake(GNSSSemaphore,portMAX_DELAY);
		GNSSRet = GNSSDataConvert(GNSSFifoBuff);
		if(GNSSRet == GNSS_OK) 
		{
			xEventGroupSetBits(FMUCheck_Status,0xFF);
			vTaskSuspend(NULL);
		}
	}
}

//TaskMonitor函数声明
BaseType_t TaskMonitor_Ret;
UBaseType_t TaskMonitor_Prio=2;
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
		vTaskDelay(5000);
		vTaskGetRunTimeStats(InfoBuffer);
		taskENTER_CRITICAL();
		printf("%s",InfoBuffer);
		printf("---------------------------------------------\r\n");
		taskEXIT_CRITICAL();
		vTaskDelay(5000);
	}
}

//SDwrite函数声明
BaseType_t SDWrite_Ret;
UBaseType_t SDWrite_Prio=4;
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
//	IMUSemaphore = xSemaphoreCreateBinary();
	HAL_UART_Receive_DMA(&huart2,IMUReceiveBuff,55);
	__HAL_UART_ENABLE_IT(&huart2,UART_IT_IDLE);
//	xEventGroupWaitBits(FMUCheck_Status,0x01,pdFALSE,pdTRUE,portMAX_DELAY);
	while(1)
	{
		xSemaphoreTake(IMUSemaphore,portMAX_DELAY);
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
//		vTaskDelay(2);
	}
}

//GNSSReceive函数声明
BaseType_t GNSSReceive_Ret;
UBaseType_t GNSSReceive_Prio=8;
TaskHandle_t GNSSReceive_TCB;

void GNSSReceive(void *pvParameters)
{
	GNSSStatus GNSSRet;
//	GNSSSemaphore = xSemaphoreCreateBinary();
	GNSSInit();//初始化GNSS串口波特率
	HAL_UART_Receive_DMA(GNSSHandle,GNSSReceiveBuff,1024);
	__HAL_UART_ENABLE_IT(GNSSHandle,UART_IT_IDLE);
	xEventGroupWaitBits(FMUCheck_Status,0x02,pdFALSE,pdTRUE,portMAX_DELAY);
	while(1)
	{
		xSemaphoreTake(GNSSSemaphore,portMAX_DELAY);
		GNSSRet = GNSSDataConvert(GNSSFifoBuff);
		if(GNSSRet != GNSS_Data_ERR)
		{
//			printf("%s",GNSSFifoBuff);
//			printf("%0.8f  %0.8f  %0.4f  ",GNSSData.lat,GNSSData.lon,GNSSData.alt);
//			printf("%0.4f  %0.4f  %0.4f\r\n",GNSSData.velocity,GNSSData.velocity_e,GNSSData.velocity_n);
		}
//		vTaskDelay(2);
	}
}

//ReceiverReceive函数声明
BaseType_t ReceiverReceive_Ret;
UBaseType_t ReceiverReceive_Prio=20;
TaskHandle_t ReceiverReceive_TCB;

void ReceiverReceive(void *pvParameters)
{
	ReceiverStatus ReceiverRet;
	uint8_t i;
//	ReceiverSemaphore = xSemaphoreCreateBinary();
	HAL_UART_Receive_DMA(&huart5,ReceiverReceiveBuff,25);
	__HAL_UART_ENABLE_IT(&huart5,UART_IT_IDLE);
	while(1)
	{
		xSemaphoreTake(ReceiverSemaphore,portMAX_DELAY);
		ReceiverRet = ReceiverDataConvert();
		if(ReceiverRet == Receiver_ERR) 
		{
			for(i=0;i<55;i++) printf("%hx ",ReceiverFifoBuff[i]);
			printf("Receiver is err!\r\n");
		}
		if(ReceiverRet == Receiver_NOSignal) printf("Receiver is no signal!\r\n");
//		if(ReceiverRet == Receiver_OK)
//		{
//			for(i=0;i<16;i++) printf("%d  ",ReceiverChannel[i]);
//			printf("\r\n");
//		}
//		vTaskDelay(2);
	}
}

