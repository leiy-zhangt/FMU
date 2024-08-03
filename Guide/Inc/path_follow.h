/*
 * File: path_follow.h
 *
 * MATLAB Coder version            : 4.1
 * C/C++ source code generated on  : 29-Jul-2024 11:36:07
 */

#ifndef PATH_FOLLOW_H
#define PATH_FOLLOW_H

/* Include Files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "guidence_plane_types.h"

/* Function Declarations */
extern double b_path_follow(const double X[6], const double curve[8], double
  *I_roll);
extern double path_follow(const double X[6], const double curve[8], double
  *I_roll);

#endif

/*
 * File trailer for path_follow.h
 *
 * [EOF]
 */
