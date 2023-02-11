#ifndef __CONTROL_H
#define __CONTROL_H

#include "computation.h"
#include "serve.h"

extern double ze_p;

void Control(void);
void Serve_1_Set(double angle);
void Serve_2_Set(double angle);
void Serve_3_Set(double angle);
void Serve_4_Set(double angle);

#endif
