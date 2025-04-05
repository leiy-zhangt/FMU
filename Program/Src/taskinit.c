#include "taskinit.h"
#include "main.h"
#include "tf.h"
#include "navigation.h"
#include "teleport.h"
#include "ms5525.h"
#include "airspeed.h"


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
	TeleSemaphore = xSemaphoreCreateBinary();
	//Create LEDTwink
	LEDTwink_Ret = xTaskCreate((TaskFunction_t)LEDTwink,"LEDTwink",32,(void *)1,LEDTwink_Prio,(TaskHandle_t *)(&LEDTwink_TCB));
	if(LEDTwink_Ret == pdPASS) InfoPrint(PrintChannel,"LEDTwink creat successfully!\r\n");
	else InfoPrint(PrintChannel,"LEDTwink creat failed!\r\n");
	//Create IMUReceive
	IMUReceive_Ret = xTaskCreate((TaskFunction_t)IMUReceive,"IMUReceive",192,(void *)1,IMUReceive_Prio,(TaskHandle_t *)(&IMUReceive_TCB));
	if(IMUReceive_Ret == pdPASS) InfoPrint(PrintChannel,"IMUReceive creat successfully!\r\n");
	else InfoPrint(PrintChannel,"IMUReceive creat failed!\r\n");
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

//IMUReceive函数声明
BaseType_t IMUReceive_Ret;
UBaseType_t IMUReceive_Prio=23;
TaskHandle_t IMUReceive_TCB;

void IMUReceive(void *pvParameters)
{
	HAL_UART_Receive_DMA(&huart2,IMUReceiveBuff,55);
	__HAL_UART_ENABLE_IT(&huart2,UART_IT_IDLE);
	while(1)
	{
		xSemaphoreTake(IMUSemaphore,portMAX_DELAY);
		IMURet = IMUDataConvert(IMUFifoBuff);
		if(IMURet == IMU_OK)
		{
//			printf("%0.4f  %0.4f  %0.4f  ",IMUData.tran_acc_x,IMUData.tran_acc_y,IMUData.tran_acc_z);
//			printf("%0.4f  %0.4f  %0.4f  ",IMUData.tran_gyr_x,IMUData.tran_gyr_y,IMUData.tran_gyr_z);
//			printf("%0.4f  %0.4f  %0.4f  \r\n",IMUData.tran_pitch,IMUData.tran_roll,IMUData.tran_yaw);
//			printf("%0.4f  %0.4f  %0.4f\r\n",NavAttitudeData.tran_pitch,NavAttitudeData.tran_roll,NavAttitudeData.tran_yaw);
//			printf("p=%0.4f,r=%0.4f,y=%0.4f,sp=%0.4f,sr=%0.4f,sy=%0.4f\r\n",IMUData.tran_pitch,IMUData.tran_roll,IMUData.tran_yaw,NavAttitudeData.tran_pitch,NavAttitudeData.tran_roll,NavAttitudeData.tran_yaw);
//			printf("p=%0.4f,r=%0.4f,y=%0.4f,sp=%0.4f,sr=%0.4f,sy=%0.4f\r\n",IMUData.tran_pitch,IMUData.tran_roll,IMUData.tran_yaw,NavAttitudeData.pitch,NavAttitudeData.roll,NavAttitudeData.yaw);
//			printf("p=%0.4f,r=%0.4f,y=%0.4f,sp=%0.4f,sr=%0.4f,sy=%0.4f\r\n",IMUData.pitch,IMUData.roll,IMUData.yaw,NavAttitudeData.pitch,NavAttitudeData.roll,NavAttitudeData.yaw);
//			printf("%0.4f  %0.4f  ",IMUData.pressure,IMUData.height);
//			printf("%0.4f  %0.4f  %0.4f  %0.4f\r\n",IMUData.quaternion[0],IMUData.quaternion[1],IMUData.quaternion[2],IMUData.quaternion[3]);
			//体坐标系到惯性坐标系
//			printf("%0.4f  %0.4f  %0.4f  %0.4f  %0.4f  %0.4f\r\n",a_e,a_n,a_u,p_e,p_n,p_u);
			ModelRocketControl();
		}
		else 
		{
//			InfoPrint(PrintChannel,"IMU error!\r\n");
		}
	}
}
