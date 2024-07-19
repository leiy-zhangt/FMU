#include "taskinit.h"
#include "main.h"
#include "tf.h"
#include "navigation.h"
#include "teleport.h"
#include "ms5525.h"
#include "airspeed.h"
#include "guide.h"


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
	if(LEDTwink_Ret == pdPASS) InfoPrint(PrintChannel,"LEDTwink creat successfully!\r\n");
	else InfoPrint(PrintChannel,"LEDTwink creat failed!\r\n");
	//Create FMUCheck
	FMUCheck_Ret = xTaskCreate((TaskFunction_t)FMUCheck,"FMUCheck",128,(void *)1,FMUCheck_Prio,(TaskHandle_t *)(&FMUCheck_TCB));
	if(FMUCheck_Ret == pdPASS) InfoPrint(PrintChannel,"FMUCheck creat successfully!\r\n");
	else InfoPrint(PrintChannel,"FMUCheck creat failed!\r\n");
	//Create FMUControlCalculation
	FMUControlCalculation_Ret = xTaskCreate((TaskFunction_t)FMUControlCalculation,"FMUControlCalculation",256,(void *)1,FMUControlCalculation_Prio,(TaskHandle_t *)(&FMUControlCalculation_TCB));
	if(FMUControlCalculation_Ret == pdPASS) InfoPrint(PrintChannel,"FMUControlCalculation creat successfully!\r\n");
	else InfoPrint(PrintChannel,"FMUControlCalculation creat failed!\r\n");
	//Create TaskMonitor
//	TaskMonitor_Ret = xTaskCreate((TaskFunction_t)TaskMonitor,"TaskMonitor",256,(void *)1,TaskMonitor_Prio,(TaskHandle_t *)(&TaskMonitor_TCB));
//	if(TaskMonitor_Ret == pdPASS) InfoPrint(PrintChannel,"TaskMonitor creat successfully!\r\n");
//	else InfoPrint(PrintChannel,"TaskMonitor creat failed!\r\n");
	//Create SDWrite
	SDWrite_Ret = xTaskCreate((TaskFunction_t)SDWrite,"SDWrite",200,(void *)1,SDWrite_Prio,(TaskHandle_t *)(&SDWrite_TCB));
	if(SDWrite_Ret == pdPASS) InfoPrint(PrintChannel,"SDWrite creat successfully!\r\n");
	else InfoPrint(PrintChannel,"SDWrite creat failed!\r\n");
	//Create IMUReceive
	IMUReceive_Ret = xTaskCreate((TaskFunction_t)IMUReceive,"IMUReceive",192,(void *)1,IMUReceive_Prio,(TaskHandle_t *)(&IMUReceive_TCB));
	if(IMUReceive_Ret == pdPASS) InfoPrint(PrintChannel,"IMUReceive creat successfully!\r\n");
	else InfoPrint(PrintChannel,"IMUReceive creat failed!\r\n");
	//Create GNSSReceive
	GNSSReceive_Ret = xTaskCreate((TaskFunction_t)GNSSReceive,"GNSSReceive",196,(void *)1,GNSSReceive_Prio,(TaskHandle_t *)(&GNSSReceive_TCB));
	if(GNSSReceive_Ret == pdPASS) InfoPrint(PrintChannel,"GNSSReceive creat successfully!\r\n");
	else InfoPrint(PrintChannel,"GNSSReceive creat failed!\r\n");
	//Create ReceiverReceive
	ReceiverReceive_Ret = xTaskCreate((TaskFunction_t)ReceiverReceive,"ReceiverReceive",256,(void *)1,ReceiverReceive_Prio,(TaskHandle_t *)(&ReceiverReceive_TCB));
	if(ReceiverReceive_Ret == pdPASS) InfoPrint(PrintChannel,"ReceiverReceive creat successfully!\r\n");
	else InfoPrint(PrintChannel,"ReceiverReceive creat failed!\r\n");
	//Create TeleportTransmit
	TeleportTransmit_Ret = xTaskCreate((TaskFunction_t)TeleportTransmit,"TeleportTransmit",196,(void *)1,TeleportTransmit_Prio,(TaskHandle_t *)(&TeleportTransmit_TCB));
	if(TeleportTransmit_Ret == pdPASS) InfoPrint(PrintChannel,"TeleportTransmit creat successfully!\r\n");
	else InfoPrint(PrintChannel,"TeleportTransmit creat failed!\r\n");
	//Create AirSpeedMeasure
	AirSpeedMeasure_Ret = xTaskCreate((TaskFunction_t)AirSpeedMeasure,"AirSpeedMeasure",256,(void *)1,AirSpeedMeasure_Prio,(TaskHandle_t *)(&AirSpeedMeasure_TCB));
	if(AirSpeedMeasure_Ret == pdPASS) InfoPrint(PrintChannel,"AirSpeedMeasure creat successfully!\r\n");
	else InfoPrint(PrintChannel,"AirSpeedMeasure creat failed!\r\n");
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
//	xEventGroupSetBits(FMUCheckEvent,0xFF);
//	vTaskSuspend(NULL);
	//
	while(1)
	{
		xSemaphoreTake(GNSSSemaphore,portMAX_DELAY);
		GNSSRet = GNSSDataConvert(GNSSFifoBuff);
		if(GNSSRet == GNSS_FIX) 
		{
			InfoPrint(PrintChannel,"GNSS is ready!\r\n");
			GNSSData.alt_Init = GNSSData.alt;
			GNSSData.lat_Init = GNSSData.lat;
			GNSSData.lon_Init = GNSSData.lon;
			
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
//		HAL_GPIO_WritePin(TRIGGER_GPIO_Port,TRIGGER_Pin,GPIO_PIN_SET);
//		NevigationSolution();
		FixedWingControl();
//		HAL_GPIO_WritePin(TRIGGER_GPIO_Port,TRIGGER_Pin,GPIO_PIN_RESET);
	}
}

//TaskMonitor函数声明
BaseType_t TaskMonitor_Ret;
UBaseType_t TaskMonitor_Prio=30;
TaskHandle_t TaskMonitor_TCB;

void TaskMonitor(void *pvParameters)
{
	char InfoBuffer[600];
//	xEventGroupWaitBits(FMUCheckEvent,0x10,pdFALSE,pdTRUE,portMAX_DELAY);
	while(1)
	{
		taskENTER_CRITICAL();
		vTaskList(InfoBuffer);
		InfoPrint(DebugChannel,"---------------------------------------------\r\n");
		InfoPrint(DebugChannel,InfoBuffer);
		InfoPrint(DebugChannel,"---------------------------------------------\r\n");
		taskEXIT_CRITICAL();
		vTaskDelay(5000);
		taskENTER_CRITICAL();
		vTaskGetRunTimeStats(InfoBuffer);
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
	else if(SDRet == FR_OK) 
	{
		InfoPrint(PrintChannel,"TF mount successfully!\r\n");
//		f_open(&SDFile,"TF test.txt",FA_WRITE|FA_CREATE_ALWAYS);
//		f_printf(&SDFile,"TF write test!\n");
//		f_close(&SDFile);
	}
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
	xEventGroupWaitBits(FMUCheckEvent,0x10,pdFALSE,pdTRUE,portMAX_DELAY);
	HAL_UART_Receive_DMA(&huart2,IMUReceiveBuff,55);
	__HAL_UART_ENABLE_IT(&huart2,UART_IT_IDLE);
	//初始化IMU高度
	while(1)
	{
		xSemaphoreTake(IMUSemaphore,portMAX_DELAY);
		IMURet = IMUDataConvert(IMUFifoBuff);
		if(IMURet == IMU_OK)
		{
			NevigayionSolutinInit();
			IMUData.height_Init = IMUData.height;
			break;
		}
	}
	while(1)
	{
		xSemaphoreTake(IMUSemaphore,portMAX_DELAY);
		IMURet = IMUDataConvert(IMUFifoBuff);
		if(IMURet == IMU_OK)
		{
//			printf("%0.4f  %0.4f  %0.4f  ",IMUData.tran_acc_x,IMUData.tran_acc_y,IMUData.tran_acc_z);
//			printf("%0.4f  %0.4f  %0.4f  ",IMUData.tran_gyr_x,IMUData.tran_gyr_y,IMUData.tran_gyr_z);
//			printf("%0.4f  %0.4f  %0.4f  ",IMUData.tran_pitch,IMUData.tran_roll,IMUData.tran_yaw);
//			printf("%0.4f  %0.4f  %0.4f\r\n",NevAttitudeData.tran_pitch,NevAttitudeData.tran_roll,NevAttitudeData.tran_yaw);
//			printf("%0.4f  %0.4f  ",IMUData.pressure,IMUData.height);
//			printf("%0.4f  %0.4f  %0.4f  %0.4f\r\n",IMUData.quaternion[0],IMUData.quaternion[1],IMUData.quaternion[2],IMUData.quaternion[3]);
			//体坐标系到惯性坐标系
//			printf("%0.4f  %0.4f  %0.4f  %0.4f  %0.4f  %0.4f\r\n",a_e,a_n,a_u,p_e,p_n,p_u);
		}
		else 
		{
//			InfoPrint(PrintChannel,"IMU error!\r\n");
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
//			InfoPrint(PrintChannel,"GNSS error!\r\n");
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
		xSemaphoreTake(ReceiverSemaphore,100);
//		xSemaphoreTake(ReceiverSemaphore,portMAX_DELAY);
		ReceiverRet = ReceiverDataConvert(ReceiverFifoBuff);
		if(ReceiverRet == Receiver_OK)
		{
			ReceiverSolution();
//			printf("%d  %d  %d  %d  %d  %d  %d  %d\r\n",ReceiverChannel[0],ReceiverChannel[1],ReceiverChannel[2],ReceiverChannel[3],ReceiverChannel[4],ReceiverChannel[5],ReceiverChannel[6],ReceiverChannel[7]);
		}
		else if(ReceiverRet == Receiver_ERR)
		{
//			InfoPrint(DebugChannel,"Receiver err!\r\n");
		}
		else if(ReceiverRet == Receiver_NOSignal)
		{
//			InfoPrint(DebugChannel,"Receiver no signal!\r\n");
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
				sprintf((char*)ControlMode,"Manual");
				break;
			case FMU_Stable:
				sprintf((char*)ControlMode,"Stable");
				break;
			case FMU_Height:
				sprintf((char*)ControlMode,"Height");
				break;
			case FMU_Path:
				sprintf((char*)ControlMode,"Path");
				break;
			case FMU_Return:
				sprintf((char*)ControlMode,"Return");
				break;
		}
//		sprintf(SendBuff,"%s p: %0.2f r: %0.2f y: %0.2f h_e: %0.2f h: %0.2f lon: %0.8f lat: %0.8f s: %0.2f v: %0.2f\r\n",ControlMode,NevAttitudeData.pitch,NevAttitudeData.roll,NevAttitudeData.yaw,expected_height,IMUData.height - IMUData.height_Init,GNSSData.lon,GNSSData.lat,GNSSData.velocity,voltage);
		sprintf((char*)SendBuff,"%s lon: %0.8f lat: %0.8f v: %0.2f lon0: %0.8f lat0: %0.8f j: %d i:%0.4f\r\n",ControlMode,GNSSData.lon,GNSSData.lat,GNSSData.velocity,GuideInitPos.posx,GuideInitPos.posy,PathChangeJudge,PathInte);
		InfoPrint(PrintChannel,(char*)SendBuff);
		vTaskDelay(1000);
	}
}

//AirSpeedMeasure函数声明
BaseType_t AirSpeedMeasure_Ret;
UBaseType_t AirSpeedMeasure_Prio=12;
TaskHandle_t AirSpeedMeasure_TCB;

void AirSpeedMeasure(void *pvParameters)
{
	AirSpeedCalibration();
	while(1)
	{
		MS5525_Ret = AirSpeedGet();
		DiffPressure = (MS5525_TotalData.pre - MS5525_TotalData.pre_init) - (MS5525_StaticData.pre - MS5525_StaticData.pre_init);
		DiffPressure = DiffPressure>0?DiffPressure:0;
		AirSpeedData.AirSpeed = calc_IAS_corrected(0,0,DiffPressure,MS5525_StaticData.pre,MS5525_StaticData.temp);
		if(MS5525_Ret == MS5525_OK)
		{
			printf("%0.4f  %0.4f\r\n",DiffPressure,AirSpeedData.AirSpeed);
		}
		vTaskDelay(1000);
	}
}

