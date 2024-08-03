#ifndef __GUIDE_H
#define __GUIDE_H

#include "tmwtypes.h"
#include "point_line_gap.h"
#include "path_follow.h"
#include "guidence_plane.h"
#include "control_roll.h"
#include "judge_curve_change.h"
#include "latlon_to_meter.h"
#include "stdio.h"

extern double X[6];//制导中间变量
extern int curve_size[];//轨迹数量
extern double I_roll;//制导律滚转积分
extern double num_curve;//轨迹数
extern double mode_return;//表示是否进入返航模式，指针double型，全局初始化置0
extern double num_return;//表示跟踪返航航线组中第几条航线，指针double型，全局初始化置0
extern double curve_return_data;//用于储存返航轨迹的数组，长度16，double型，只需全局初始化，无需赋值
extern int curve_return_size[2];//返航轨迹数组的纬度，长度2,int型，只需全局初始化，无需赋值
extern double judge_arrive;//判断航迹组是否完成，指针double型，全局初始化置0
extern double guideswitch;//模式切换标志位，切换时为0


#endif
