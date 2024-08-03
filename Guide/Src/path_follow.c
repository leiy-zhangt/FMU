/*
 * File: path_follow.c
 *
 * MATLAB Coder version            : 4.1
 * C/C++ source code generated on  : 29-Jul-2024 11:36:07
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "control_roll.h"
#include "guidence_plane.h"
#include "judge_curve_change.h"
#include "path_follow.h"
#include "point_curve_gap.h"
#include "point_line_gap.h"

/* Function Definitions */

/*
 * X(1)北向速度，X(2)向上速度，X(3)东向速度
 * X(1)北向位置，X(2)向上位置，X(3)东向位置
 * Arguments    : const double X[6]
 *                const double curve[8]
 *                double *I_roll
 * Return Type  : double
 */
double b_path_follow(const double X[6], const double curve[8], double *I_roll)
{
  double roll;
  double dphi;
  dphi = 0.0;
  if (curve[0] == 0.0) {
    /* 判断轨迹类型是否为直线 */
    dphi = point_line_gap(X, curve);
  }

  if ((curve[0] == 1.0) || (curve[0] == -1.0)) {
    /* 判断轨迹类型是否为圆弧 */
    dphi = point_curve_gap(X, curve);
  }

  /* 防止掉头多转圈 */
  while (dphi < -3.1415926535897931) {
    dphi += 6.2831853071795862;
  }

  while (dphi > 3.1415926535897931) {
    dphi -= 6.2831853071795862;
  }

  roll = control_roll(X, dphi, I_roll);
  *I_roll = 0.0;
  return roll;
}

/*
 * X(1)北向速度，X(2)向上速度，X(3)东向速度
 * X(1)北向位置，X(2)向上位置，X(3)东向位置
 * Arguments    : const double X[6]
 *                const double curve[8]
 *                double *I_roll
 * Return Type  : double
 */
double path_follow(const double X[6], const double curve[8], double *I_roll)
{
  double roll;
  double dphi;
  dphi = 0.0;
  if (curve[0] == 0.0) {
    /* 判断轨迹类型是否为直线 */
    dphi = point_line_gap(X, curve);
  }

  if ((curve[0] == 1.0) || (curve[0] == -1.0)) {
    /* 判断轨迹类型是否为圆弧 */
    dphi = point_curve_gap(X, curve);
  }

  /* 防止掉头多转圈 */
  while (dphi < -3.1415926535897931) {
    dphi += 6.2831853071795862;
  }

  while (dphi > 3.1415926535897931) {
    dphi -= 6.2831853071795862;
  }

  roll = control_roll(X, dphi, I_roll);
  *I_roll = 0.0;
  return roll;
}

/*
 * File trailer for path_follow.c
 *
 * [EOF]
 */
