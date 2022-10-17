#ifndef __BMM150_H
#define __BMM150_H

#include "sys.h"
#include "spi.h"
#include "delay.h"

#define BMM_CS PCout(0)

//校准寄存器宏定义
#define BMM150_DIG_X1                             UINT8_C(0x5D)
#define BMM150_DIG_Y1                             UINT8_C(0x5E)
#define BMM150_DIG_Z4_LSB                         UINT8_C(0x62)
#define BMM150_DIG_Z4_MSB                         UINT8_C(0x63)
#define BMM150_DIG_X2                             UINT8_C(0x64)
#define BMM150_DIG_Y2                             UINT8_C(0x65)
#define BMM150_DIG_Z2_LSB                         UINT8_C(0x68)
#define BMM150_DIG_Z2_MSB                         UINT8_C(0x69)
#define BMM150_DIG_Z1_LSB                         UINT8_C(0x6A)
#define BMM150_DIG_Z1_MSB                         UINT8_C(0x6B)
#define BMM150_DIG_XYZ1_LSB                       UINT8_C(0x6C)
#define BMM150_DIG_XYZ1_MSB                       UINT8_C(0x6D)
#define BMM150_DIG_Z3_LSB                         UINT8_C(0x6E)
#define BMM150_DIG_Z3_MSB                         UINT8_C(0x6F)
#define BMM150_DIG_XY2                            UINT8_C(0x70)
#define BMM150_DIG_XY1                            UINT8_C(0x71)

typedef struct
{
  uint8_t buffer[8];
  int16_t data_x_int;
  int16_t data_y_int;
  int16_t data_z_int;
  uint16_t data_rhall_uint;
  double data_x;
  double data_y;
  double data_z;
}BMM150_DataStruct;

typedef struct
{
  int8_t digX1;    /**< trim x1 data */
  int8_t digY1;    /**< trim y1 data */
  int8_t digX2;    /**< trim x2 data */
  int8_t digY2;    /**< trim y2 data */
  uint16_t digZ1;  /**< trim z1 data */
  int16_t digZ2;   /**< trim z2 data */
  int16_t digZ3;   /**< trim z3 data */
  int16_t digZ4;   /**< trim z4 data */
  uint8_t digXY1;  /**< trim xy1 data */
  int8_t digXY2;   /**< trim xy2 data */
  uint16_t digXYZ1;/**< trim xyz1 data */
}BMM150_TrimStruct;

extern BMM150_DataStruct BMM150_Data;
extern BMM150_TrimStruct BMM150_Trim;

void BMM150_Configuration(void);//BMM150初始化函数
void BMM150_SendData(uint8_t addr,uint8_t data);//BMM150单字节发送函数
uint8_t BMM150_ReadData(uint8_t addr);//BMM150单字节接收函数
void BMM150_ReadBuffer(uint8_t addr,uint8_t *buffer,uint8_t length);//BMM150多字节接收函数
void BMM150_MeasureGet(BMM150_DataStruct *BMM150_Data);
void BMM150_Trim_Get(BMM150_TrimStruct *BMM150_Trim);//获得校准系数
//double BMM150_CompensateX(BMM150_DataStruct *BMM150_Data,BMM150_TrimStruct *BMM150_Trim);//x通道补偿
//double BMM150_CompensateY(BMM150_DataStruct *BMM150_Data,BMM150_TrimStruct *BMM150_Trim);//y通道补偿
//double BMM150_CompensateZ(BMM150_DataStruct *BMM150_Data,BMM150_TrimStruct *BMM150_Trim);//z通道补偿

#endif 


