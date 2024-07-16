#include "guide.h"
#include "navigation.h"

GuidePositionStruct GuideInitPos;//路径起点
double *PathChangeJudge;//路径切换判断
double *PathInte;//路径控制积分

double GuidePe,GuidePn,GuideAngle;