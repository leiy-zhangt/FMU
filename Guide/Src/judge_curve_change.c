/*
 * File: judge_curve_change.c
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

/* Function Definitions */

/*
 * if curve(1)==0
 *          n=curve(4:5)';
 *      else
 *          n=curve(1)*[cos(curve(5)+pi/2);sin(curve(5)+pi/2)];
 *      end
 *
 *      if curve(1)==0
 *          r=curve(2:3)'+curve(6)*curve(4:5)';
 *      else
 *          r=curve(2:3)'+curve(6)*[cos(curve(5));sin(curve(5))];
 *      end
 * Arguments    : const double X[6]
 *                const double curve[8]
 * Return Type  : double
 */
double judge_curve_change(const double X[6], const double curve[8])
{
  double judge;
  double n_idx_0_tmp;
  double n_idx_1_tmp;
  if (curve[0] == 0.0) {
    judge = !((X[5] - (curve[1] + curve[5] * curve[3])) * curve[3] + (X[3] -
               (curve[2] + curve[5] * curve[4])) * curve[4] >= 0.0);
  } else {
    /*          n=curve(1)*[cos(curve(5)+pi/2);sin(curve(5)+pi/2)]; */
    n_idx_0_tmp = sin(curve[4]);
    n_idx_1_tmp = cos(curve[4]);
    if (((X[5] - (curve[1] + curve[5] * n_idx_1_tmp)) * -n_idx_0_tmp + (X[3] -
          (curve[2] + curve[5] * n_idx_0_tmp)) * n_idx_1_tmp >= 0.0) && (X[2] *
         -n_idx_0_tmp + X[0] * n_idx_1_tmp > 0.0)) {
      judge = 0.0;
    } else {
      judge = 1.0;
    }
  }

  return judge;
}

/*
 * File trailer for judge_curve_change.c
 *
 * [EOF]
 */
