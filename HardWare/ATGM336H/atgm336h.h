#ifndef __ATGM336H_H
#define __ATGM336H_H

#include "sys.h"
#include "delay.h"
#include "usart.h"

#define ATGM336H_PWR PCout(13)
#define ATGM336H_RST PCout(14)
#define ATGM336H_PPS PCin(15)

void ATGM336H_Configuration(void);
#endif
