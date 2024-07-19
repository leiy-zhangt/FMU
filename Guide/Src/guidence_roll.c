/*
 * File: guidence_roll.c
 *
 * MATLAB Coder version            : 4.1
 * C/C++ source code generated on  : 10-Jul-2024 11:04:54
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "control_roll.h"
#include "guidence_roll.h"
#include "judge_curve_change.h"
#include "path_follow.h"
#include "point_line_gap.h"

/* Function Definitions */

/*
 * Arguments    : double p_e
 *                double p_n
 *                double phiv
 *                double *I_roll
 *                double *judge
 * Return Type  : double
 */
double guidence_roll(double p_e, double p_n, double phiv, double *I_roll, int *judge)
{
  int judge_tmp;
  int i0;
  double dphi;
  double curve[8];
  static const short b_curve[32] = { 0, 0, 0, 0, 0, 30, 140, 110, 0, -110, -80,
    30, 0, 1, 0, -1, -1, 0, 1, 0, 80, 80, 80, 80, 70, 70, 70, 70, 0, 0, 0, 0 };

  judge_tmp = (int)*judge;
  for (i0 = 0; i0 < 8; i0++) {
    curve[i0] = b_curve[(judge_tmp + (i0 << 2)) - 1];
  }

  dphi = judge_curve_change(p_e, p_n, curve);
  if (!(dphi != 0.0)) {
    (*judge)++;
    if (*judge == 5) {
      *judge = 1;
    }
  }

  /* X(1)北向速度，X(2)向上速度，X(3)东向速度 */
  /* X(1)北向位置，X(2)向上位置，X(3)东向位置 */
  judge_tmp = (int)*judge;
  for (i0 = 0; i0 < 8; i0++) {
    curve[i0] = b_curve[(judge_tmp + (i0 << 2)) - 1];
  }

  /* 防止掉头多转圈 */
  for (dphi = point_line_gap(p_e, p_n, phiv, curve); dphi < -3.1415926535897931;
       dphi += 6.2831853071795862) {
  }

  while (dphi > 3.1415926535897931) {
    dphi -= 6.2831853071795862;
  }

  return control_roll(dphi, I_roll);
}

/*
 * File trailer for guidence_roll.c
 *
 * [EOF]
 */
