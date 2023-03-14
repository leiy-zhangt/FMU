#ifndef __CONTROL_H
#define __CONTROL_H

#include "computation.h"
#include "serve.h"

extern double ze_p,pe_p,re_p;
extern double ze,ze_i,ze_d,pe,pe_i,pe_d,re,re_i,re_d;
extern double U1,U2,U3,f1,f2,f3,f4;//旋翼螺旋桨参数，代表升力、力矩
extern const double Kpz,Kiz,Kdz,Kpp,Kip,Kdp,Kpr,Kir,Kdr;//控制器系数

void Control(void);
void Serve_1_Set(double f);
void Serve_2_Set(double f);
void Serve_3_Set(double f);
void Serve_4_Set(double f);

#endif
