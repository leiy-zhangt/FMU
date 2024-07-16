/*
 * File: path_follow.c
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
 * X(1)�����ٶȣ�X(2)�����ٶȣ�X(3)�����ٶ�
 * X(1)����λ�ã�X(2)����λ�ã�X(3)����λ��
 * Arguments    : double p_e
 *                double p_n
 *                double phiv
 *                const double curve[8]
 *                double *I_roll
 * Return Type  : double
 */
double path_follow(double p_e, double p_n, double phiv, const double curve[8],
                   double *I_roll)
{
  double dphi;

  /* ��ֹ��ͷ��תȦ */
  for (dphi = point_line_gap(p_e, p_n, phiv, curve); dphi < -3.1415926535897931;
       dphi += 6.2831853071795862) {
  }

  while (dphi > 3.1415926535897931) {
    dphi -= 6.2831853071795862;
  }

  return control_roll(dphi, I_roll);
}

/*
 * File trailer for path_follow.c
 *
 * [EOF]
 */
