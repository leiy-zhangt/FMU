#ifndef __ATGM336H_H
#define __ATGM336H_H

#include "sys.h"
#include "delay.h"
#include "usart.h"

#define ATGM336H_PWR PAout(4)
#define ATGM336H_NRST PCout(15)

#define R 6371393

typedef struct
{
  double lat,lon,height;
  double velocity_n,velocity_e,velocity_course;
  double lat_init,lon_init,height_init;
}GPS_DataStruct;


extern uint8_t GPS_state;
extern uint8_t GPS_init;
extern GPS_DataStruct GPS_Data;

void ATGM336H_Configuration(FunctionalState ATGM_State);
ErrorStatus GPS_WaitReady(void);//等待GPS就绪
void Coordinate2Position(void);//坐标转换公式
void NMEASolution(void);//NEMA报文解析函数

#endif
