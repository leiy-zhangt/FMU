#include "bmp388.h"

//最高位为0：写  最高位为1：读
//气压的数据需要温度进行修正，因此在测量气压之前需要先测量温度

double height_init = 0;

BMP388_Calibration_DataStruct BMP388_Calibration_Data;
BMP388_Calibration_QuantizedDataStruct BMP388_Calibration_QuantizedData;
BMP388_DataStruct BMP388_Data;

void BMP388_Configuration(void)
{
  BMP388_Calibration(&BMP388_Calibration_Data,&BMP388_Calibration_QuantizedData);//获取校正系数
  //BMP388工作模式配置
  BMP388_WriteData(0x7E,0xB6);//复位    
  delay_ms(1);
  BMP388_WriteData(0x1C,0x01);//配置过采样
  BMP388_WriteData(0x1D,0x01);//配置输出速率为100Hz
  BMP388_WriteData(0x1F,0x02);//配置滤波IIR滤波器为1
  BMP388_WriteData(0x1B,0x33);//配置工作模式为正常工作
  delay_ms(20);
  height_init = 0;
  height_init = BMP388_HeightGet();
}

void BMP388_WriteData(uint8_t addr,uint8_t data)
{
    BMP_CmdRd;
    delay_us(1);
    BMP_CS = 0;
    delay_us(1);
    SPI_ReadWriteByte(SPI1,addr&0x7F);
    SPI_ReadWriteByte(SPI1,data);
    BMP_CS = 1;
    delay_us(1);
}

uint8_t BMP388_ReadData(uint8_t addr)
{
  static uint8_t res;
  BMP_CS = 0;
  delay_us(1);
  SPI_ReadWriteByte(SPI1,addr|0x80);
  SPI_ReadWriteByte(SPI1,0x00);
  res = SPI_ReadWriteByte(SPI1,0x00);
  BMP_CS = 1;
  delay_us(1);
  return res;
}

void BMP388_ReadBuffer(uint8_t addr,uint8_t *buffer,uint8_t length)
{
    BMP_CS = 0;
    delay_us(1);
    SPI_ReadWriteByte(SPI1,addr|0x80);
    SPI_ReadWriteByte(SPI1,0x00);
    for(;length>0;length--)
    {
        *buffer = SPI_ReadWriteByte(SPI1,0x00);
        buffer++;
    }
    BMP_CS = 1;
    delay_us(1);
}

uint8_t BMP388_StatusGet(void)
{
    return BMP388_ReadData(0x03);
}


void BMP388_TemperatureGet(BMP388_DataStruct *BMP388_Data)
{
  double partial_data1;
  double partial_data2;
  while(BMP388_ReadData(0x03)&0x20 == 0);
  BMP388_ReadBuffer(0x07,BMP388_Data->buffer,3);
  BMP388_Data->uncomp_data = ((int32_t)(BMP388_Data->buffer[2])<<16)|((int32_t)(BMP388_Data->buffer[1])<<8)|((int32_t)(BMP388_Data->buffer[0]));
  partial_data1 = (double)(BMP388_Data->uncomp_data - BMP388_Calibration_QuantizedData.par_t1);
  partial_data2 = (double)(partial_data1 * BMP388_Calibration_QuantizedData.par_t2);
  BMP388_Calibration_QuantizedData.t_lin = partial_data2 + (partial_data1 * partial_data1) * BMP388_Calibration_QuantizedData.par_t3;
  BMP388_Data->tem = BMP388_Calibration_QuantizedData.t_lin;
}


void BMP388_PressureGet(BMP388_DataStruct *BMP388_Data)
{
  double partial_data1;
  double partial_data2;
  double partial_data3;
  double partial_data4;
  double partial_out1;
  double partial_out2;
  BMP388_TemperatureGet(BMP388_Data);
  while(BMP388_ReadData(0x03)&0x10 == 0);
  BMP388_ReadBuffer(0x04,BMP388_Data->buffer,3);
  BMP388_Data->uncomp_data = ((uint32_t)(BMP388_Data->buffer[2])<<16)|((uint32_t)(BMP388_Data->buffer[1])<<8)|(BMP388_Data->buffer[0]);
  partial_data1 = BMP388_Calibration_QuantizedData.par_p6 * BMP388_Calibration_QuantizedData.t_lin;
  partial_data2 = BMP388_Calibration_QuantizedData.par_p7 * pow(BMP388_Calibration_QuantizedData.t_lin, 2);
  partial_data3 = BMP388_Calibration_QuantizedData.par_p8 * pow(BMP388_Calibration_QuantizedData.t_lin, 3);
  partial_out1 = BMP388_Calibration_QuantizedData.par_p5 + partial_data1 + partial_data2 + partial_data3;
  partial_data1 = BMP388_Calibration_QuantizedData.par_p2 * BMP388_Calibration_QuantizedData.t_lin;
  partial_data2 = BMP388_Calibration_QuantizedData.par_p3 * pow(BMP388_Calibration_QuantizedData.t_lin, 2);
  partial_data3 = BMP388_Calibration_QuantizedData.par_p4 * pow(BMP388_Calibration_QuantizedData.t_lin, 3);
  partial_out2 = BMP388_Data->uncomp_data *(BMP388_Calibration_QuantizedData.par_p1 + partial_data1 + partial_data2 + partial_data3);
  partial_data1 = pow((double)BMP388_Data->uncomp_data,2);
  partial_data2 = BMP388_Calibration_QuantizedData.par_p9 + BMP388_Calibration_QuantizedData.par_p10 * BMP388_Calibration_QuantizedData.t_lin;
  partial_data3 = partial_data1 * partial_data2;
  partial_data4 = partial_data3 + pow((double)BMP388_Data->uncomp_data,3) * BMP388_Calibration_QuantizedData.par_p11;
  BMP388_Data->pre = partial_out1 + partial_out2 + partial_data4;
}

double BMP388_HeightGet(void)
{
  BMP388_PressureGet(&BMP388_Data);
  return 288.15*(1-pow(BMP388_Data.pre/101325,0.0065*287.05287/g))/0.0065 - height_init;
//  return 288.15*(1-pow(BMP388_Data.pre/101325,0.0065*287.05287/g))/0.0065;
}

double BMP388_HeightCalibration()
{
  double T = 0.01, tau = 0.1;
  double alpha = T / (T + tau);
  return alpha*BMP388_HeightGet()+(1-alpha)*MotionData.height;
}


void BMP388_Calibration(BMP388_Calibration_DataStruct *data_int,BMP388_Calibration_QuantizedDataStruct *data_float)
{
    uint8_t buffer[21];
    double temp_var;
    BMP388_ReadBuffer(0x31,buffer,21);
    data_int->par_t1 = (((uint16_t)buffer[1])<<8)|buffer[0];
    data_int->par_t2 = (((uint16_t)buffer[3])<<8)|buffer[2];
    data_int->par_t3 = buffer[4];
    data_int->par_p1 = (((int16_t)buffer[6])<<8)|buffer[5];
    data_int->par_p2 = (((int16_t)buffer[8])<<8)|buffer[7];
    data_int->par_p3 = buffer[9];
    data_int->par_p4 = buffer[10];
    data_int->par_p5 = (((uint16_t)buffer[12])<<8)|buffer[11];
    data_int->par_p6 = (((uint16_t)buffer[14])<<8)|buffer[13];
    data_int->par_p7 = buffer[15];
    data_int->par_p8 = buffer[16];
    data_int->par_p9 = (((int16_t)buffer[18])<<8)|buffer[17];
    data_int->par_p10 = buffer[19];
    data_int->par_p11 = buffer[20];
    
    temp_var = 0.00390625;
    data_float->par_t1 = ((double)data_int->par_t1 / temp_var);
    temp_var = 1073741824.0;
    data_float->par_t2 = ((double)data_int->par_t2 / temp_var);
    temp_var = 281474976710656.0;
    data_float->par_t3 = ((double)data_int->par_t3 / temp_var);
    temp_var = 1048576.0;
    data_float->par_p1 = ((double)(data_int->par_p1 - (16384)) / temp_var);
    temp_var = 536870912.0;
    data_float->par_p2 = ((double)(data_int->par_p2 - (16384)) / temp_var);
    temp_var = 4294967296.0;
    data_float->par_p3 = ((double)data_int->par_p3 / temp_var);
    temp_var = 137438953472.0;
    data_float->par_p4 = ((double)data_int->par_p4 / temp_var);
    temp_var = 0.125;
    data_float->par_p5 = ((double)data_int->par_p5 / temp_var);
    temp_var = 64.0;
    data_float->par_p6 = ((double)data_int->par_p6 / temp_var);
    temp_var = 256.0;
    data_float->par_p7 = ((double)data_int->par_p7 / temp_var);
    temp_var = 32768.0;
    data_float->par_p8 = ((double)data_int->par_p8 / temp_var);
    temp_var = 281474976710656.0;
    data_float->par_p9 = ((double)data_int->par_p9 / temp_var);
    temp_var = 281474976710656.0;
    data_float->par_p10 = ((double)data_int->par_p10 / temp_var);
    temp_var = 36893488147419103232.0;
    data_float->par_p11 = ((double)data_int->par_p11 / temp_var);
}

