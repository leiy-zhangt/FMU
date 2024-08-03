#include "guide.h"
#include "navigation.h"
#include "gnss.h"

double X[6];//制导中间变量
int curve_size[2];//轨迹数量
double I_roll;//制导律滚转积分
double num_curve = 2;//轨迹数
double mode_return = 0;//表示是否进入返航模式，指针double型，全局初始化置0
double num_return = 0;//表示跟踪返航航线组中第几条航线，指针double型，全局初始化置0
double curve_return_data;//用于储存返航轨迹的数组，长度16，double型，只需全局初始化，无需赋值
int curve_return_size[2];//返航轨迹数组的纬度，长度2,int型，只需全局初始化，无需赋值
double judge_arrive;//判断航迹组是否完成，指针double型，全局初始化置0
double guideswitch = 0;//模式切换标志位，切换时为0
double Pe,Pn;//东北距离


