#ifndef __ATGM336H_H
#define __ATGM336H_H

#include "sys.h"
#include "delay.h"
#include "usart.h"

#define ATGM336H_PWR PAout(4)
#define ATGM336H_NRST PCout(15)

void ATGM336H_Configuration(FunctionalState ATGM_State);
#endif
