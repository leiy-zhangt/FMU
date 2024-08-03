/*
 * File: guidence_plane.h
 *
 * MATLAB Coder version            : 4.1
 * C/C++ source code generated on  : 29-Jul-2024 11:36:07
 */

#ifndef GUIDENCE_PLANE_H
#define GUIDENCE_PLANE_H

/* Include Files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "guidence_plane_types.h"

/* Function Declarations */
extern void guidence_plane(const double X[6], const double curve_data[], const
  int curve_size[2], double *I_roll, double *num_curve, double *mode_return,
  double *num_return, double curve_return_data[], int curve_return_size[2],
  double *roll, double *judge_arrive);

#endif

/*
 * File trailer for guidence_plane.h
 *
 * [EOF]
 */
