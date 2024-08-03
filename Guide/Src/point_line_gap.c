/*
 * File: point_line_gap.c
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
double point_line_gap(const double X[6], const double curve[8])
{
  double b_curve[2];
  double q[3];
  double phi_q;
  double r;
  b_curve[0] = curve[3];
  b_curve[1] = curve[4];
  q[0] = curve[3];
  q[1] = c_norm(b_curve) * tan(curve[7]);
  q[2] = curve[4];
  phi_q = b_norm(q);
  q[0] = curve[3] / phi_q;
  q[1] /= phi_q;
  q[2] = curve[4] / phi_q;

  /* 期望-实际 */
  /*      ddh=norm([v_e,v_n])*tan(curve(8))-v_h; */
  /*      dL=dot(ep,n); */
  phi_q = -rt_atan2d_snf(curve[3], curve[4]);
  r = rt_atan2d_snf(X[2], X[0]);

  /* 防止出现“掉头”情况 */
  while (phi_q - (-r) > 3.1415926535897931) {
    phi_q -= 6.2831853071795862;
  }

  while (phi_q - (-r) < -3.1415926535897931) {
    phi_q += 6.2831853071795862;
  }

  /* 当前位置期望偏航角 */
  /* 期望-实际 */
  return (phi_q + atan(0.05 * (((X[5] - curve[1]) * (q[1] * 0.0 - (-q[2])) + (X
              [4] - X[4]) * (q[2] * 0.0 - q[0] * 0.0)) + (X[3] - curve[2]) *
            (-q[0] - q[1] * 0.0)))) - (-r);
}

/*
 * File trailer for point_line_gap.c
 *
 * [EOF]
 */
