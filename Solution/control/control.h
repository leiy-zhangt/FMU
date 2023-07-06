#ifndef __CONTROL_H
#define __CONTROL_H

#include "computation.h"
#include "serve.h"

extern uint8_t ControlMode;
extern double control_pitch,control_roll,control_yaw,yaw_init,roll_e,pitch_e;//¿ØÖÆÊä³öÁ¿

void Parafoil_Control(void);
void FixdWing_Control(void);

float AngleDifference(float angle_1,float angle_2);
void Serve_1_Set(uint16_t angle);
void Serve_2_Set(uint16_t angle);
void Serve_3_Set(uint16_t angle);
void Serve_4_Set(uint16_t angle);

#endif

