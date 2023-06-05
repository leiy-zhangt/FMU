#ifndef __CONTROL_H
#define __CONTROL_H

#include "computation.h"
#include "serve.h"

void Parafoil_Control(void);
void FixdWing_Control(void);

void Serve_1_Set(uint16_t angle);
void Serve_2_Set(uint16_t angle);
void Serve_3_Set(uint16_t angle);
void Serve_4_Set(uint16_t angle);

#endif

