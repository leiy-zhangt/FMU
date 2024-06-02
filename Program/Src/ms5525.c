#include "ms5525.h"
#include "cmsis_os.h"

const uint8_t MS5525_Q1 = 16;
const uint8_t MS5525_Q2 = 17;
const uint8_t MS5525_Q3 = 6;
const uint8_t MS5525_Q4 = 5;
const uint8_t MS5525_Q5 = 7;
const uint8_t MS5525_Q6 = 22;

//I2C接收Buff
uint8_t MS5525_Buff[8];
//I2C命令
uint8_t MS5525_CMD;
//风速计数据结构体
MS5525_DataStruct MS5525_StaticData;
MS5525_DataStruct MS5525_TotalData;
//MS5525返回状态值
MS5525_Status MS5525_Ret;


void MS5525_Reset(uint8_t addr)
{
	MS5525_CMD = 0x1E;
	HAL_I2C_Master_Transmit(&hi2c1,addr<<1,&MS5525_CMD,1,0xFFFF);
	HAL_Delay(3000);
}

void MS5525_Calibration(uint8_t addr,MS5525_DataStruct *MS5525_Data)
{
	uint8_t i;
	MS5525_CMD = 0xA0;
	for(i=0;i<8;i++)
	{
		HAL_I2C_Master_Transmit(&hi2c1,addr<<1,&MS5525_CMD,1,0xFFFF);
		HAL_I2C_Master_Receive(&hi2c1,addr<<1,MS5525_Buff,2,0xFFFF);
		MS5525_CMD += 2;
		MS5525_Data->MS5525_C[i] = ((uint16_t)MS5525_Buff[0]<<8)|MS5525_Buff[1];
	}
	MS5525_Data->MS5525_Tref = (int64_t)MS5525_Data->MS5525_C[5] * (1UL << MS5525_Q5);
}

// If the conversion is not executed before the ADC read command, or the ADC read command is repeated, it will give 0 as the output
// result. If the ADC read command is sent during conversion the result will be 0, the conversion will not stop and
// the final result will be wrong. Conversion sequence sent during the already started conversion process will yield
// incorrect result as well.

MS5525_Status MS5525_Measure(void)
{
	volatile uint8_t MeasureFlag;
	MS5525_StaticData.MS5525_D1 = 0;
	MS5525_TotalData.MS5525_D1 = 0;
	MS5525_StaticData.MS5525_D2 = 0;
	MS5525_TotalData.MS5525_D2 = 0;
	MS5525_CMD = 0x48;
	MeasureFlag = 0;
//	taskENTER_CRITICAL();
	HAL_I2C_Master_Transmit(&hi2c1,0x76<<1,&MS5525_CMD,1,0xFFFF);
	HAL_I2C_Master_Transmit(&hi2c1,0x77<<1,&MS5525_CMD,1,0xFFFF);
//	taskEXIT_CRITICAL();
	HAL_Delay(10000);
	vTaskDelay(10);
	MS5525_StaticData.MS5525_D1 = MS5525_GetValue(0x76);
	MS5525_TotalData.MS5525_D1 = MS5525_GetValue(0x77);
	if(MS5525_StaticData.MS5525_D1 != 0) MeasureFlag |= 0x01;
	if(MS5525_TotalData.MS5525_D1 != 0) MeasureFlag |= 0x02;
	if((MeasureFlag&0x03) != 0x03) return MS5525_ERR; 
	MS5525_CMD = 0x58;
	MeasureFlag = 0;
	HAL_I2C_Master_Transmit(&hi2c1,0x76<<1,&MS5525_CMD,1,0xFFFF);
	HAL_I2C_Master_Transmit(&hi2c1,0x77<<1,&MS5525_CMD,1,0xFFFF);
	HAL_Delay(10000);
	vTaskDelay(10);
//	taskENTER_CRITICAL();
	MS5525_StaticData.MS5525_D2 = MS5525_GetValue(0x76);
	MS5525_TotalData.MS5525_D2 = MS5525_GetValue(0x77);
//	taskEXIT_CRITICAL();
	if(MS5525_StaticData.MS5525_D2 != 0) MeasureFlag |= 0x01;
	if(MS5525_TotalData.MS5525_D2 != 0) MeasureFlag |= 0x02;
	if((MeasureFlag&0x03) != 0x03) return MS5525_ERR;
	else return MS5525_OK;
}

uint32_t MS5525_GetValue(uint8_t addr)
{
	MS5525_CMD = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1,addr<<1,&MS5525_CMD,1,0xFFFF);
	HAL_I2C_Master_Receive(&hi2c1,addr<<1,MS5525_Buff,3,0xFFFF);
	return ((uint16_t)MS5525_Buff[0] << 16) | ((uint16_t)MS5525_Buff[1] << 8) | MS5525_Buff[2];
}

void MS5525_Converse(MS5525_DataStruct *MS5525_Data)
{
	int64_t MS5525_dT;
	int64_t MS5525_TEMP;
	int64_t MS5525_OFF;
	int64_t MS5525_SENS;
	int64_t MS5525_P;
//	float diff_press_PSI;
//	float PSI_to_Pa;
//	float diff_press_pa;
//	float temperature_c;
	// Difference between actual and reference temperature
	//  dT = D2 - Tref
	MS5525_dT = MS5525_Data->MS5525_D2 - MS5525_Data->MS5525_Tref;

	// Measured temperature
	//  TEMP = 20°C + dT * TEMPSENS
	MS5525_TEMP = 2000 + (MS5525_dT * (int64_t)(MS5525_Data->MS5525_C[6])) / (1UL << MS5525_Q6);

	// Offset at actual temperature
	//  OFF = OFF_T1 + TCO * dT
	MS5525_OFF = (int64_t)(MS5525_Data->MS5525_C[2]) * (1UL << MS5525_Q2) + ((int64_t)(MS5525_Data->MS5525_C[4]) * MS5525_dT) / (1UL << MS5525_Q4);

	// Sensitivity at actual temperature
	//  SENS = SENS_T1 + TCS * dT
	MS5525_SENS = (int64_t)(MS5525_Data->MS5525_C[1]) * (1UL << MS5525_Q1) + ((int64_t)(MS5525_Data->MS5525_C[3]) * MS5525_dT) / (1UL << MS5525_Q3);

	// Temperature Compensated Pressure (example 24996 = 2.4996 psi)
	//  P = D1 * SENS - OFF
	MS5525_P = (MS5525_Data->MS5525_D1 * MS5525_SENS / (1UL << 21) - MS5525_OFF) / (1UL << 15);
	// 1 PSI = 6894.76 Pascals
	MS5525_Data->pre = MS5525_P * 0.0001 * 6894.757;
	MS5525_Data->temp = MS5525_TEMP * 0.01;
}

//			uint16_t MS5525_C[8];
//			int64_t MS5525_Tref;
//			int32_t MS5525_Temp;
//			int32_t MS5525_Pre;

//			uint32_t MS5525_D1;
//			uint32_t MS5525_D2;
//			
//			double pre,temp;
//		ServoSet(ServoChannel_1,30);
//		ServoSet(ServoChannel_2,30);
//		ServoSet(ServoChannel_4,30);
//		HAL_Delay(100000);
//		ServoSet(ServoChannel_1,0);
//		ServoSet(ServoChannel_2,0);
//		ServoSet(ServoChannel_4,0);
//		HAL_Delay(100000);
		
//		static uint8_t res[3],i,tran[2];
//		static uint16_t C[8]; 
//		static uint8_t send = 0xA0;
//		send = 0xA0;
//		for(i=0;i<8;i++)
//		{
//			HAL_I2C_Master_Transmit(&hi2c1,0x77<<1,&send,1,0xFFFF);
//			HAL_I2C_Master_Receive(&hi2c1,0x77<<1,res,2,0xFFFF);
//			send = send + 2;
//			C[i] = ((uint16_t)res[0]<<8)|res[1];
//		}
//		memcpy(MS5525_C,C,16);
//		MS5525_Tref = (int64_t)MS5525_C[5] * (1UL << MS5525_Q5);
//		while(1)
//		{
//			send = 0x48;
//			HAL_I2C_Master_Transmit(&hi2c1,0x77<<1,&send,1,0xFFFF);
//			HAL_Delay(10000);
//			send = 0x00;
//			HAL_I2C_Master_Transmit(&hi2c1,0x77<<1,&send,1,0xFFFF);
//			HAL_I2C_Master_Receive(&hi2c1,0x77<<1,res,3,0xFFFF);
//			MS5525_D1 = ((uint16_t)res[0] << 16) | ((uint16_t)res[1] << 8) | res[2];
//			send = 0x58;
//			HAL_I2C_Master_Transmit(&hi2c1,0x77<<1,&send,1,0xFFFF);
//			HAL_Delay(10000);
//			send = 0x00;
//			HAL_I2C_Master_Transmit(&hi2c1,0x77<<1,&send,1,0xFFFF);
//			HAL_I2C_Master_Receive(&hi2c1,0x77<<1,res,3,0xFFFF);
//			MS5525_D2 = ((uint16_t)res[0] << 16) | ((uint16_t)res[1] << 8) | res[2];
//			// Difference between actual and reference temperature
//			//  dT = D2 - Tref
//			int64_t MS5525_dT = MS5525_D2 - MS5525_Tref;

//			// Measured temperature
//			//  TEMP = 20°C + dT * TEMPSENS
//			volatile int64_t MS5525_TEMP = 2000 + (MS5525_dT * (int64_t)(MS5525_C[6])) / (1UL << MS5525_Q6);

//			// Offset at actual temperature
//			//  OFF = OFF_T1 + TCO * dT
//			volatile int64_t MS5525_OFF = (int64_t)(MS5525_C[2]) * (1UL << MS5525_Q2) + ((int64_t)(MS5525_C[4]) * MS5525_dT) / (1UL << MS5525_Q4);

//			// Sensitivity at actual temperature
//			//  SENS = SENS_T1 + TCS * dT
//			volatile int64_t MS5525_SENS = (int64_t)(MS5525_C[1]) * (1UL << MS5525_Q1) + ((int64_t)(MS5525_C[3]) * MS5525_dT) / (1UL << MS5525_Q3);

//			// Temperature Compensated Pressure (example 24996 = 2.4996 psi)
//			//  P = D1 * SENS - OFF
//			volatile int64_t MS5525_P = (MS5525_D1 * MS5525_SENS / (1UL << 21) - MS5525_OFF) / (1UL << 15);

//			float diff_press_PSI = MS5525_P * 0.0001f;

//			// 1 PSI = 6894.77 Pascals
//			float PSI_to_Pa = 6894.757f;
//			float diff_press_pa = diff_press_PSI * PSI_to_Pa;

//			float temperature_c = MS5525_TEMP * 0.01f;	
//			float v = calc_IAS_corrected(0.2,1.5,diff_press_pa+37,96000,temperature_c);
//			printf("pre:%f  tem:%f\r\n",diff_press_pa,temperature_c);



