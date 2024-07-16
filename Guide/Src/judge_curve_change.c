/*
 * File: judge_curve_change.c
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
 *                const double curve[8]
 * Return Type  : double
 */
double judge_curve_change(double p_e, double p_n, const double curve[8])
{
  return !((p_e - (curve[1] + curve[5] * curve[3])) * curve[3] + (p_n - (curve[2]
             + curve[5] * curve[4])) * curve[4] >= 0.0);
}

/*
 * File trailer for judge_curve_change.c
 *
 * [EOF]
 */
