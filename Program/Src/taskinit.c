#include "taskinit.h"
#include "main.h"
#include "tf.h"


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
//	if(LEDTwink_Ret == pdPASS) InfoPrint(PrintChannel,"LEDTwink creat successfully!\r\n");
//	else InfoPrint(PrintChannel,"LEDTwink creat failed!\r\n");
	//Create FMUCheck
	FMUCheck_Ret = xTaskCreate((TaskFunction_t)FMUCheck,"FMUCheck",128,(void *)1,FMUCheck_Prio,(TaskHandle_t *)(&FMUCheck_TCB));
//	if(FMUCheck_Ret == pdPASS) InfoPrint(PrintChannel,"FMUCheck creat successfully!\r\n");
//	else InfoPrint(PrintChannel,"FMUCheck creat failed!\r\n");
	//Create FMUControlCalculation
	FMUControlCalculation_Ret = xTaskCreate((TaskFunction_t)FMUControlCalculation,"FMUControlCalculation",256,(void *)1,FMUControlCalculation_Prio,(TaskHandle_t *)(&FMUControlCalculation_TCB));
//	if(FMUControlCalculation_Ret == pdPASS) InfoPrint(PrintChannel,"FMUControlCalculation creat successfully!\r\n");
//	else InfoPrint(PrintChannel,"FMUControlCalculation creat failed!\r\n");
	//Create TaskMonitor
	TaskMonitor_Ret = xTaskCreate((TaskFunction_t)TaskMonitor,"TaskMonitor",256,(void *)1,TaskMonitor_Prio,(TaskHandle_t *)(&TaskMonitor_TCB));
//	if(TaskMonitor_Ret == pdPASS) InfoPrint(PrintChannel,"TaskMonitor creat successfully!\r\n");
//	else InfoPrint(PrintChannel,"TaskMonitor creat failed!\r\n");
	//Create SDWrite
	SDWrite_Ret = xTaskCreate((TaskFunction_t)SDWrite,"SDWrite",200,(void *)1,SDWrite_Prio,(TaskHandle_t *)(&SDWrite_TCB));
//	if(SDWrite_Ret == pdPASS) InfoPrint(PrintChannel,"SDWrite creat successfully!\r\n");
//	else InfoPrint(PrintChannel,"SDWrite creat failed!\r\n");
	//Create IMUReceive
	IMUReceive_Ret = xTaskCreate((TaskFunction_t)IMUReceive,"IMUReceive",128,(void *)1,IMUReceive_Prio,(TaskHandle_t *)(&IMUReceive_TCB));
//	if(IMUReceive_Ret == pdPASS) InfoPrint(PrintChannel,"IMUReceive creat successfully!\r\n");
//	else InfoPrint(PrintChannel,"IMUReceive creat failed!\r\n");
	//Create GNSSReceive
	GNSSReceive_Ret = xTaskCreate((TaskFunction_t)GNSSReceive,"GNSSReceive",196,(void *)1,GNSSReceive_Prio,(TaskHandle_t *)(&GNSSReceive_TCB));
//	if(GNSSReceive_Ret == pdPASS) InfoPrint(PrintChannel,"GNSSReceive creat successfully!\r\n");
//	else InfoPrint(PrintChannel,"GNSSReceive creat failed!\r\n");
	//Create ReceiverReceive
	ReceiverReceive_Ret = xTaskCreate((TaskFunction_t)ReceiverReceive,"ReceiverReceive",256,(void *)1,ReceiverReceive_Prio,(TaskHandle_t *)(&ReceiverReceive_TCB));
//	if(ReceiverReceive_Ret == pdPASS) InfoPrint(PrintChannel,"ReceiverReceive creat successfully!\r\n");
//	else InfoPrint(PrintChannel,"ReceiverReceive creat failed!\r\n");
	//Create TeleportTransmit
	TeleportTransmit_Ret = xTaskCreate((TaskFunction_t)TeleportTransmit,"TeleportTransmit",196,(void *)1,TeleportTransmit_Prio,(TaskHandle_t *)(&TeleportTransmit_TCB));
//	if(TeleportTransmit_Ret == pdPASS) InfoPrint(PrintChannel,"TeleportTransmit creat successfully!\r\n");
//	else InfoPrint(PrintChannel,"TeleportTransmit creat failed!\r\n");
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
	xEventGroupClearBits(FMUCheckEvent,0xFFFF);
	//调试时使用禁用GPS
	xEventGroupSetBits(FMUCheckEvent,0xFF);
	vTaskSuspend(NULL);
	//
	while(1)
	{
		xSemaphoreTake(GNSSSemaphore,portMAX_DELAY);
		GNSSRet = GNSSDataConvert(GNSSFifoBuff);
		if(GNSSRet == GNSS_FIX) 
		{
			InfoPrint(PrintChannel,"GNSS is ready!\r\n");
			xEventGroupSetBits(FMUCheckEvent,0xFF);
			vTaskSuspend(NULL);
		}
		else
		{
			InfoPrint(PrintChannel,"GNSS is preparing!\r\n");
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
		xSemaphoreTake(ControlSemaphore,portMAX_DELAY);
		HAL_GPIO_WritePin(TRIGGER_GPIO_Port,TRIGGER_Pin,GPIO_PIN_SET);
		FixedWingControl();
		HAL_GPIO_WritePin(TRIGGER_GPIO_Port,TRIGGER_Pin,GPIO_PIN_RESET);
	}
}

//TaskMonitor函数声明
BaseType_t TaskMonitor_Ret;
UBaseType_t TaskMonitor_Prio=30;
TaskHandle_t TaskMonitor_TCB;

void TaskMonitor(void *pvParameters)
{
	char InfoBuffer[600];
	xEventGroupWaitBits(FMUCheckEvent,0x10,pdFALSE,pdTRUE,portMAX_DELAY);
	while(1)
	{
		vTaskList(InfoBuffer);
		taskENTER_CRITICAL();
		InfoPrint(DebugChannel,"---------------------------------------------\r\n");
		InfoPrint(DebugChannel,InfoBuffer);
		InfoPrint(DebugChannel,"---------------------------------------------\r\n");
		taskEXIT_CRITICAL();
		vTaskDelay(5000);
		vTaskGetRunTimeStats(InfoBuffer);
		taskENTER_CRITICAL();
		InfoPrint(DebugChannel,"---------------------------------------------\r\n");
		InfoPrint(DebugChannel,InfoBuffer);
		InfoPrint(DebugChannel,"---------------------------------------------\r\n");
		taskEXIT_CRITICAL();
		vTaskDelay(5000);
	}
}

//SDwrite函数声明
BaseType_t SDWrite_Ret;
UBaseType_t SDWrite_Prio=25;
TaskHandle_t SDWrite_TCB;

void SDWrite(void *pvParameters)
{
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
					if(SDRet == FR_OK) InfoPrint(PrintChannel,"TF mount successfully!\r\n");
					else InfoPrint(PrintChannel,"TF mount successfully!\r\n");
				}
				break;
			default:
				InfoPrint(PrintChannel,"TF mount failed!\r\n");
				while(1) ;
		}
	}
	else if(SDRet == FR_OK) InfoPrint(PrintChannel,"TF mount successfully!\r\n");
	vTaskSuspend(NULL);
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
//			InfoPrint(PrintChannel,"%0.4f  %0.4f  %0.4f  ",IMUData.acc_x,IMUData.acc_y,IMUData.acc_z);
//			InfoPrint(PrintChannel,"%0.4f  %0.4f  %0.4f  ",IMUData.gyr_x,IMUData.gyr_y,IMUData.gyr_z);
//			InfoPrint(PrintChannel,"%0.4f  %0.4f  %0.4f  ",IMUData.pitch,IMUData.roll,IMUData.yaw);
//			InfoPrint(PrintChannel,"%0.4f  %0.4f  ",IMUData.pressure,IMUData.height);
//			InfoPrint(PrintChannel,"%0.4f  %0.4f  %0.4f  %0.4f\r\n",IMUData.quaternion[0],IMUData.quaternion[1],IMUData.quaternion[2],IMUData.quaternion[3]);
		}
		else 
		{
			InfoPrint(PrintChannel,"IMU error!\r\n");
		}
	}
}

//GNSSReceive函数声明
BaseType_t GNSSReceive_Ret;
UBaseType_t GNSSReceive_Prio=17;
TaskHandle_t GNSSReceive_TCB;

void GNSSReceive(void *pvParameters)
{
	GNSSInit();//初始化GNSS串口波特率
	HAL_UART_Receive_DMA(GNSSHandle,GNSSReceiveBuff,1024);
	__HAL_UART_ENABLE_IT(GNSSHandle,UART_IT_IDLE);
	xEventGroupWaitBits(FMUCheckEvent,0x02,pdFALSE,pdTRUE,portMAX_DELAY);
	while(1)
	{
		xSemaphoreTake(GNSSSemaphore,1500);
		GNSSRet = GNSSDataConvert(GNSSFifoBuff);
		if(GNSSRet == GNSS_FIX)
		{
			
		}
		else 
		{
			InfoPrint(PrintChannel,"GNSS error!\r\n");
		}
	}
}

//ReceiverReceive函数声明
BaseType_t ReceiverReceive_Ret;
UBaseType_t ReceiverReceive_Prio=26;
TaskHandle_t ReceiverReceive_TCB;

void ReceiverReceive(void *pvParameters)
{
	HAL_UART_Receive_DMA(&huart5,ReceiverReceiveBuff,25);
	__HAL_UART_ENABLE_IT(&huart5,UART_IT_IDLE);
	xEventGroupWaitBits(FMUCheckEvent,0x04,pdFALSE,pdTRUE,portMAX_DELAY);
	while(1)
	{
		xSemaphoreTake(ReceiverSemaphore,50);
		ReceiverRet = ReceiverDataConvert(ReceiverFifoBuff);
		if(ReceiverRet == Receiver_OK)
		{
			ReceiverSolution();
		}
		else if(ReceiverRet == Receiver_ERR)
		{
			InfoPrint(PrintChannel,"Receiver err!\r\n");
		}
		else if(ReceiverRet == Receiver_NOSignal)
		{
			InfoPrint(PrintChannel,"Receiver no signal!\r\n");
		}
	}
}

//TeleportTransmit函数声明
BaseType_t TeleportTransmit_Ret;
UBaseType_t TeleportTransmit_Prio=10;
TaskHandle_t TeleportTransmit_TCB;

void TeleportTransmit(void *pvParameters)
{
	uint32_t voltage_uint32,current_uint32;
	double voltage,current;
	uint8_t ControlMode[10];
	xEventGroupWaitBits(FMUCheckEvent,0x04,pdFALSE,pdTRUE,portMAX_DELAY);
	while(1)
	{
		HAL_ADC_Start(&hadc3);
		HAL_ADC_PollForConversion(&hadc3,0x10);
		current_uint32 = HAL_ADC_GetValue(&hadc3);
		HAL_ADC_Start(&hadc3);
		HAL_ADC_PollForConversion(&hadc3,0x10);
		voltage_uint32 = HAL_ADC_GetValue(&hadc3);
		voltage = voltage_uint32*0.00042802;
		current = current_uint32*0.00005355;
		HAL_ADC_Stop(&hadc3);
		switch(FMUControlMode)
		{
			case FMU_Manual:
				sprintf(ControlMode,"Manual");
				break;
			case FMU_Stable:
				sprintf(ControlMode,"Stable");
				break;
			case FMU_Height:
				sprintf(ControlMode,"Height");
				break;
		}
		sprintf(SendBuff,"%s  p:%0.2f  r:%0.2f  y:%0.2f  h:%0.2f  lat:%0.8f  lon:%0.8f  s:%0.2f  v:%0.2f\r\n",ControlMode,IMUData.pitch,IMUData.roll,IMUData.yaw,GNSSData.alt,GNSSData.lat,GNSSData.lon,GNSSData.velocity,voltage);
		InfoPrint(PrintChannel,SendBuff);
		vTaskDelay(1000);
	}
}

