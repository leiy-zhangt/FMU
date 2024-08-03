/*
 * File: latlon_to_meter.c
 *
 * MATLAB Coder version            : 4.1
 * C/C++ source code generated on  : 29-Jul-2024 15:51:43
 */

/* Include Files */
#include "latlon_to_meter.h"

/* Function Definitions */

/*
 * Arguments    : double lat
 *                double lon
 *                double lat0
 *                double lon0
 *                double *p_n
 *                double *p_e
 * Return Type  : void
 */
void latlon_to_meter(double lat, double lon, double lat0, double lon0, double
                     *p_n, double *p_e)
{
  *p_n = (lat - lat0) * 111195.08;
  *p_e = (lon - lon0) * 83139.58;
}

/*
 * File trailer for latlon_to_meter.c
 *
 * [EOF]
 */
