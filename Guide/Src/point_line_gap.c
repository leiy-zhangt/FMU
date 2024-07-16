/*
 * File: point_line_gap.c
 *
 * MATLAB Coder version            : 4.1
 * C/C++ source code generated on  : 10-Jul-2024 11:04:54
 */

/* Include Files */
#include <math.h>
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include "control_roll.h"
#include "guidence_roll.h"
#include "judge_curve_change.h"
#include "path_follow.h"
#include "point_line_gap.h"

/* Function Declarations */
static double rt_atan2d_snf(double u0, double u1);

/* Function Definitions */

/*
 * Arguments    : double u0
 *                double u1
 * Return Type  : double
 */
static double rt_atan2d_snf(double u0, double u1)
{
  double y;
  int b_u0;
  int b_u1;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u0 > 0.0) {
      b_u0 = 1;
    } else {
      b_u0 = -1;
    }

    if (u1 > 0.0) {
      b_u1 = 1;
    } else {
      b_u1 = -1;
    }

    y = atan2(b_u0, b_u1);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

/*
 * Arguments    : double p_e
 *                double p_n
 *                double phiv
 *                const double curve[8]
 * Return Type  : double
 */
double point_line_gap(double p_e, double p_n, double phiv, const double curve[8])
{
  double scale;
  double absxk;
  double t;
  double y;
  scale = 3.3121686421112381E-170;
  absxk = fabs(curve[3]);
  if (absxk > 3.3121686421112381E-170) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    y = t * t;
  }

  absxk = fabs(curve[4]);
  if (absxk > scale) {
    t = scale / absxk;
    y = 1.0 + y * t * t;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  y = scale * sqrt(y);
  t = curve[3] / y;
  scale = curve[4] / y;

  /*      phiv=-atan2(v_e,v_n); */
  /* 防止出现“掉头”情况 */
  for (absxk = -rt_atan2d_snf(curve[3], curve[4]); absxk - phiv >
       3.1415926535897931; absxk -= 6.2831853071795862) {
  }

  while (absxk - phiv < -3.1415926535897931) {
    absxk += 6.2831853071795862;
  }

  /* 当前位置期望偏航角 */
  /* 期望-实际 */
  return (absxk + atan(0.05 * ((p_e - curve[1]) * (0.0 * t + scale) + (p_n -
             curve[2]) * (-t + 0.0 * scale)))) - phiv;
}

/*
 * File trailer for point_line_gap.c
 *
 * [EOF]
 */
