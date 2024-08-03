/*
 * File: point_curve_gap.c
 *
 * MATLAB Coder version            : 4.1
 * C/C++ source code generated on  : 29-Jul-2024 11:36:07
 */

/* Include Files */
#include <math.h>
#include "rt_nonfinite.h"
#include "control_roll.h"
#include "guidence_plane.h"
#include "judge_curve_change.h"
#include "path_follow.h"
#include "point_curve_gap.h"
#include "point_line_gap.h"
#include "norm.h"
#include "guidence_plane_rtwutil.h"

/* Function Definitions */

/*
 * Arguments    : const double X[6]
 *                const double curve[8]
 * Return Type  : double
 */
double point_curve_gap(const double X[6], const double curve[8])
{
  double phi_q;
  double r;
  double b_curve[2];

  /* 期望-实际 */
  /*      ddh=norm([v_e,v_n])*tan(curve(8))-v_h; */
  phi_q = -rt_atan2d_snf(X[5] - curve[1], X[3] - curve[2]);
  r = rt_atan2d_snf(X[2], X[0]);

  /* 防止出现“掉头”情况 */
  while (phi_q - (-r) > 3.1415926535897931) {
    phi_q -= 6.2831853071795862;
  }

  while (phi_q - (-r) < -3.1415926535897931) {
    phi_q += 6.2831853071795862;
  }

  /* 期望-实际 */
  b_curve[0] = curve[1] - X[5];
  b_curve[1] = curve[2] - X[3];
  return ((phi_q + curve[0] * 3.1415926535897931 / 2.0) + curve[0] * atan(2.0 *
           -(curve[5] - c_norm(b_curve)) / curve[5])) - (-r);
}

/*
 * File trailer for point_curve_gap.c
 *
 * [EOF]
 */
