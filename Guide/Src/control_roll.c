/*
 * File: control_roll.c
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
 * Arguments    : const double X[6]
 *                double dphi
 *                double *b_I
 * Return Type  : double
 */
double control_roll(const double X[6], double dphi, double *b_I)
{
  double roll_desire;
  (void)X;

  /*      dt=0.2; */
  /*      kp=-2*epsilon*wn*Vg/g; */
  /*      if abs(dphi)<0.01 */
  /*          dphi=0; */
  /*      end */
  *b_I = 0.0;
  roll_desire = -0.9698 * dphi;
  if (roll_desire > 0.52359877559829882) {
    roll_desire = 0.52359877559829882;
  }

  if (roll_desire < -0.52359877559829882) {
    roll_desire = -0.52359877559829882;
  }

  return roll_desire;
}

/*
 * File trailer for control_roll.c
 *
 * [EOF]
 */
