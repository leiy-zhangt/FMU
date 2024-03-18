#include "taskinit.h"
#include "main.h"


EventGroupHandle_t FMUCheckEvent; 

IMUStatus IMURet;
GNSSStatus GNSSRet;
ReceiverStatus ReceiverRet;

//TaskCreate函数声明
void TaskCreate(void)
{
	FMUCheckEvent = xEventGroupCreate();//创建自检事件标志位
	ReceiverSemaphore = xSemaphoreCreateBinary();
	IMUSemaphore = xSemaphoreCreateBinary();
	GNSSSemaphore = xSemaphoreCreateBinary();
	ControlSemaphore = xSemaphoreCreateBinary();
	//Create LEDTwink
	LEDTwink_Ret = xTaskCreate((TaskFunction_t)LEDTwink,"LEDTwink",32,(void *)1,LEDTwink_Prio,(TaskHandle_t *)(&LEDTwink_TCB));
	if(LEDTwink_Ret == pdPASS) printf("LEDTwink creat successfully!\r\n");
	else printf("LEDTwink creat failed!\r\n");
	//Create FMUCheck
	FMUCheck_Ret = xTaskCreate((TaskFunction_t)FMUCheck,"FMUCheck",128,(void *)1,FMUCheck_Prio,(TaskHandle_t *)(&FMUCheck_TCB));
	if(FMUCheck_Ret == pdPASS) printf("FMUCheck creat successfully!\r\n");
	else printf("FMUCheck creat failed!\r\n");
	//Create FMUControlCalculation
	FMUControlCalculation_Ret = xTaskCreate((TaskFunction_t)FMUControlCalculation,"FMUControlCalculation",256,(void *)1,FMUControlCalculation_Prio,(TaskHandle_t *)(&FMUControlCalculation_TCB));
	if(FMUControlCalculation_Ret == pdPASS) printf("FMUControlCalculation creat successfully!\r\n");
	else printf("FMUControlCalculation creat failed!\r\n");
	//Create TaskMonitor
	TaskMonitor_Ret = xTaskCreate((TaskFunction_t)TaskMonitor,"TaskMonitor",256,(void *)1,TaskMonitor_Prio,(TaskHandle_t *)(&TaskMonitor_TCB));
	if(TaskMonitor_Ret == pdPASS) printf("TaskMonitor creat successfully!\r\n");
	else printf("TaskMonitor creat failed!\r\n");
	//Create SDWrite
	SDWrite_Ret = xTaskCreate((TaskFunction_t)SDWrite,"SDWrite",200,(void *)1,SDWrite_Prio,(TaskHandle_t *)(&SDWrite_TCB));
	if(SDWrite_Ret == pdPASS) printf("SDWrite creat successfully!\r\n");
	else printf("SDWrite creat failed!\r\n");
	//Create IMUReceive
	IMUReceive_Ret = xTaskCreate((TaskFunction_t)IMUReceive,"IMUReceive",128,(void *)1,IMUReceive_Prio,(TaskHandle_t *)(&IMUReceive_TCB));
	if(IMUReceive_Ret == pdPASS) printf("IMUReceive creat successfully!\r\n");
	else printf("IMUReceive creat failed!\r\n");
	//Create GNSSReceive
	GNSSReceive_Ret = xTaskCreate((TaskFunction_t)GNSSReceive,"GNSSReceive",196,(void *)1,GNSSReceive_Prio,(TaskHandle_t *)(&GNSSReceive_TCB));
	if(GNSSReceive_Ret == pdPASS) printf("GNSSReceive creat successfully!\r\n");
	else printf("GNSSReceive creat failed!\r\n");
	//Create ReceiverReceive
	ReceiverReceive_Ret = xTaskCreate((TaskFunction_t)ReceiverReceive,"ReceiverReceive",96,(void *)1,ReceiverReceive_Prio,(TaskHandle_t *)(&ReceiverReceive_TCB));
	if(ReceiverReceive_Ret == pdPASS) printf("ReceiverReceive creat successfully!\r\n");
	else printf("ReceiverReceive creat failed!\r\n");
	//Start
	vTaskStartScheduler();
	while(1) ;
}

//LEDTwink函数声明
BaseType_t LEDTwink_Ret;
UBaseType_t LEDTwink_Prio=2;
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
UBaseType_t FMUCheck_Prio=30;
TaskHandle_t FMUCheck_TCB;

void FMUCheck(void *pvParameters)
{
	GNSSStatus GNSSRet;
	xEventGroupClearBits(FMUCheckEvent,0xFFFF);
	while(1)
	{
		xSemaphoreTake(GNSSSemaphore,portMAX_DELAY);
		GNSSRet = GNSSDataConvert(GNSSFifoBuff);
		if(GNSSRet == GNSS_FIX) 
		{
			DebugPrint(PrintChannel,"GNSS is ready!\r\n");
			xEventGroupSetBits(FMUCheckEvent,0xFF);
			HAL_TIM_Base_Start(&htim5);
			vTaskSuspend(NULL);
		}
		else
		{
			DebugPrint(PrintChannel,"GNSS is preparing!\r\n");
		}
	}
}

//FMUControlCalculation函数声明
BaseType_t FMUControlCalculation_Ret;
UBaseType_t FMUControlCalculation_Prio=20;
TaskHandle_t FMUControlCalculation_TCB;

void FMUControlCalculation(void *pvParameters)
{
	xEventGroupWaitBits(FMUCheckEvent,0x08,pdFALSE,pdTRUE,portMAX_DELAY);
	while(1)
	{
//		xSemaphoreTake(ControlSemaphore,portMAX_DELAY);
//		HAL_GPIO_TogglePin(SIGNAL_GPIO_Port,SIGNAL_Pin);
		vTaskDelay(250);
	}
}

//TaskMonitor函数声明
BaseType_t TaskMonitor_Ret;
UBaseType_t TaskMonitor_Prio=3;
TaskHandle_t TaskMonitor_TCB;

void TaskMonitor(void *pvParameters)
{
	char InfoBuffer[600];
	xEventGroupWaitBits(FMUCheckEvent,0x10,pdFALSE,pdTRUE,portMAX_DELAY);
	while(1)
	{
		vTaskList(InfoBuffer);
		taskENTER_CRITICAL();
		DebugPrint(PrintChannel,"---------------------------------------------\r\n");
		DebugPrint(PrintChannel,InfoBuffer);
		DebugPrint(PrintChannel,"---------------------------------------------\r\n");
		taskEXIT_CRITICAL();
		vTaskDelay(5000);
		vTaskGetRunTimeStats(InfoBuffer);
		taskENTER_CRITICAL();
		DebugPrint(PrintChannel,"---------------------------------------------\r\n");
		DebugPrint(PrintChannel,InfoBuffer);
		DebugPrint(PrintChannel,"---------------------------------------------\r\n");
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
	uint32_t SDprint = 1;
	xEventGroupWaitBits(FMUCheckEvent,0x10,pdFALSE,pdTRUE,portMAX_DELAY);
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
					if(SDRet == FR_OK) DebugPrint(PrintChannel,"SD mount successfully!\r\n");
					else DebugPrint(PrintChannel,"SD mount successfully!\r\n");
				}
				break;
			default:
				DebugPrint(PrintChannel,"SD mount failed!\r\n");
				while(1) ;
		}
	}
	else if(SDRet == FR_OK) DebugPrint(PrintChannel,"SD mount successfully!\r\n");
	FileCreate();
	while(1)
	{

	}
}

//IMUReceive函数声明
BaseType_t IMUReceive_Ret;
UBaseType_t IMUReceive_Prio=9;
TaskHandle_t IMUReceive_TCB;

void IMUReceive(void *pvParameters)
{
	HAL_UART_Receive_DMA(&huart2,IMUReceiveBuff,55);
	__HAL_UART_ENABLE_IT(&huart2,UART_IT_IDLE);
	xEventGroupWaitBits(FMUCheckEvent,0x10,pdFALSE,pdTRUE,portMAX_DELAY);
	while(1)
	{
		xSemaphoreTake(IMUSemaphore,portMAX_DELAY);
		IMURet = IMUDataConvert(IMUFifoBuff);
		if(IMURet == IMU_OK)
		{
//			printf("%0.4f  %0.4f  %0.4f  ",IMUData.acc_x,IMUData.acc_y,IMUData.acc_z);
//			printf("%0.4f  %0.4f  %0.4f  ",IMUData.gyr_x,IMUData.gyr_y,IMUData.gyr_z);
//			printf("%0.4f  %0.4f  %0.4f  ",IMUData.pitch,IMUData.roll,IMUData.yaw);
//			printf("%0.4f  %0.4f  ",IMUData.pressure,IMUData.height);
//			printf("%0.4f  %0.4f  %0.4f  %0.4f\r\n",IMUData.quaternion[0],IMUData.quaternion[1],IMUData.quaternion[2],IMUData.quaternion[3]);
		}
		else 
		{
			DebugPrint(PrintChannel,"IMU error!\r\n");
		}
	}
}

//GNSSReceive函数声明
BaseType_t GNSSReceive_Ret;
UBaseType_t GNSSReceive_Prio=8;
TaskHandle_t GNSSReceive_TCB;

void GNSSReceive(void *pvParameters)
{
	GNSSInit();//初始化GNSS串口波特率
	HAL_UART_Receive_DMA(GNSSHandle,GNSSReceiveBuff,1024);
	__HAL_UART_ENABLE_IT(GNSSHandle,UART_IT_IDLE);
	xEventGroupWaitBits(FMUCheckEvent,0x02,pdFALSE,pdTRUE,portMAX_DELAY);
	while(1)
	{
		xSemaphoreTake(GNSSSemaphore,portMAX_DELAY);
		GNSSRet = GNSSDataConvert(GNSSFifoBuff);
		if(GNSSRet == GNSS_FIX)
		{
//			DebugPrint(PrintChannel,"IMU error!\r\n");
		}
		else DebugPrint(PrintChannel,"GNSS error!\r\n");
	}
}

//ReceiverReceive函数声明
BaseType_t ReceiverReceive_Ret;
UBaseType_t ReceiverReceive_Prio=20;
TaskHandle_t ReceiverReceive_TCB;

void ReceiverReceive(void *pvParameters)
{
	uint8_t i;
	HAL_UART_Receive_DMA(&huart5,ReceiverReceiveBuff,25);
	__HAL_UART_ENABLE_IT(&huart5,UART_IT_IDLE);
	xEventGroupWaitBits(FMUCheckEvent,0x04,pdFALSE,pdTRUE,portMAX_DELAY);
	while(1)
	{
		xSemaphoreTake(ReceiverSemaphore,portMAX_DELAY);
		ReceiverRet = ReceiverDataConvert();
		if(ReceiverRet == Receiver_OK)
		{

		}
		else  DebugPrint(PrintChannel,"Receiver err!\r\n");
	}
}

