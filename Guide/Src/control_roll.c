/*
 * File: control_roll.c
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
 * Arguments    : double dphi
 *                double *b_I
 * Return Type  : double
 */
double control_roll(double dphi, double *b_I)
{
  double roll_desire;
  *b_I -= -0.000489795918367347 * dphi;
  if (-0.0489795918367347 * *b_I < (-1.0471975511965976/4)) {
    *b_I = 21.380283336930532/4;
  }

  if (-0.0489795918367347 * *b_I > (1.0471975511965976/4)) {
    *b_I = -21.380283336930532/4;
  }

  roll_desire = -1.4693877551020409*0.66 * dphi + -0.000489795918367347 * *b_I;
  if (roll_desire > (1.0471975511965976/2)) {
    roll_desire = 1.0471975511965976/2;
  }

  if (roll_desire < (-1.0471975511965976/2)) {
    roll_desire = -1.0471975511965976/2;
  }

  return roll_desire;
}

/*
 * File trailer for control_roll.c
 *
 * [EOF]
 */
