#ifndef __GUIDE_H
#define __GUIDE_H

#include "tmwtypes.h"
#include "point_line_gap.h"
#include "path_follow.h"
#include "guidence_roll.h"
#include "control_roll.h"
#include "judge_curve_change.h"

typedef struct
{
	double posx,posy,height;
}GuidePositionStruct;

extern GuidePositionStruct GuideInitPos;//路径起点
extern int PathChangeJudge;//路径切换判断
extern double PathInte;//路径控制积分
extern double GuidePe,GuidePn,GuideAngle;

#endif
