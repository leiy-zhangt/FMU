#ifndef __COMMAND_H
#define __COMMAND_H

#include "sys.h"
#include "computation.h"
#include "string.h"
#include "led.h" 
#include "serve.h"
#include "buzzer.h"
#include "bmm150.h"
#include "bmi088.h"
#include "adxl357.h"
#include "bmp388.h"
#include "fuse.h"
#include "w25q.h"
#include "lora.h"
#include "atgm336h.h"

#define BMI_START 1
#define BMI_STOP 2

#define AttitudeSolution_TEST 70
#define Sample_STOP 71

extern uint8_t Command_State;

void Command_Receive(uint8_t *buffer);
void AttitudeSolution_Ttst(void);
void Sample_Start(void);

#endif

