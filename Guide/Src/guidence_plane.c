/*
 * File: guidence_plane.c
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
 *                const double curve_data[]
 *                const int curve_size[2]
 *                double *I_roll
 *                double *num_curve
 *                double *mode_return
 *                double *num_return
 *                double curve_return_data[]
 *                int curve_return_size[2]
 *                double *roll
 *                double *judge_arrive
 * Return Type  : void
 */
void guidence_plane(const double X[6], const double curve_data[], const int
                    curve_size[2], double *I_roll, double *num_curve, double
                    *mode_return, double *num_return, double curve_return_data[],
                    int curve_return_size[2], double *roll, double *judge_arrive)
{
  double b_X[2];
  double dL;
  int loop_ub;
  int i0;
  double curve[8];
  double n_idx_0_tmp;
  double dv9[16];
  int num_curve_tmp;
  double b_curve_data[8];
  double phi_q;
  double q[3];
  *judge_arrive = 0.0;
  *roll = 0.0;
  if ((*mode_return < 0.5) && ((X[3] < -200.0) || (X[3] > 200.0) || (X[5] <
        -200.0) || (X[5] > 200.0))) {
    /* 超出划定飞行区域 */
    *mode_return = 1.0;

    /* 返航 */
    *num_curve = 1.0;
  }

  if (*mode_return < 0.5) {
    /* 非返航模式 */
    if (*num_curve < curve_size[0]) {
      /* 判断路径是否需要切换 */
      loop_ub = (int)*num_curve;
      for (i0 = 0; i0 < 8; i0++) {
        curve[i0] = curve_data[(loop_ub + curve_size[0] * i0) - 1];
      }

      n_idx_0_tmp = judge_curve_change(X, curve);
      if (!(n_idx_0_tmp != 0.0)) {
        (*num_curve)++;
      }

      loop_ub = (int)*num_curve;
      for (i0 = 0; i0 < 8; i0++) {
        curve[i0] = curve_data[(loop_ub + curve_size[0] * i0) - 1];
      }

      *roll = b_path_follow(X, curve, I_roll);
      *I_roll = 0.0;
    } else if (*num_curve == curve_size[0]) {
      /* 判断路径是否需要切换 */
      loop_ub = (int)*num_curve;
      for (i0 = 0; i0 < 8; i0++) {
        curve[i0] = curve_data[(loop_ub + curve_size[0] * i0) - 1];
      }

      n_idx_0_tmp = judge_curve_change(X, curve);
      if (!(n_idx_0_tmp != 0.0)) {
        (*num_curve)++;
        *judge_arrive = 1.0;

        /* 路径已完成 */
      } else {
        loop_ub = (int)*num_curve;
        for (i0 = 0; i0 < 8; i0++) {
          curve[i0] = curve_data[(loop_ub + curve_size[0] * i0) - 1];
        }

        *roll = b_path_follow(X, curve, I_roll);
        *I_roll = 0.0;
      }
    } else {
      *judge_arrive = 1.0;

      /* 路径已完成 */
    }
  } else {
    /* 返航模式 */
    b_X[0] = X[3];
    b_X[1] = X[5];
    dL = c_norm(b_X);
    if ((dL > 70.0) && (*num_return == 0.0)) {
      curve[0] = 0.0;
      curve[1] = X[5];
      curve[2] = X[3];
      curve[3] = -X[5] / dL;
      curve[4] = -X[3] / dL;
      curve[5] = dL;
      curve[6] = X[4];
      curve[7] = 0.0;
      curve_return_size[0] = 1;
      curve_return_size[1] = 8;
      for (i0 = 0; i0 < 8; i0++) {
        curve_return_data[curve_return_size[0] * i0] = curve[i0];
      }

      *num_return = 1.0;
    } else {
      if ((*num_return == 1.0) && (curve_return_size[0] == 1) && (dL < 60.0)) {
        /*              c_e=X(6)+40*[0,-1]*[X(3);X(1)]/norm([X(3);X(1)]); */
        /*              c_n=X(4)+40*[1,0]*[X(3);X(1)]/norm([X(3);X(1)]); */
        /*              curve_return=[1,c_e,c_n,0,pi/2,40,X(5),0;1,c_e,c_n,pi/2,pi,40,X(5),0;1,c_e,c_n,pi,pi*3/2,40,X(5),0;1,c_e,c_n,pi*3/2,pi*2,40,X(5),0]; */
        dv9[0] = 1.0;
        dv9[2] = 0.0;
        dv9[4] = 0.0;
        dv9[6] = 0.0;
        dv9[8] = 3.1415926535897931;
        dv9[10] = 40.0;
        dv9[12] = X[4];
        dv9[14] = 0.0;
        dv9[1] = 1.0;
        dv9[3] = 0.0;
        dv9[5] = 0.0;
        dv9[7] = 3.1415926535897931;
        dv9[9] = 6.2831853071795862;
        dv9[11] = 40.0;
        dv9[13] = X[4];
        dv9[15] = 0.0;
        curve_return_size[0] = 2;
        curve_return_size[1] = 8;
        for (i0 = 0; i0 < 8; i0++) {
          loop_ub = i0 << 1;
          num_curve_tmp = curve_return_size[0] * i0;
          curve_return_data[num_curve_tmp] = dv9[loop_ub];
          curve_return_data[1 + num_curve_tmp] = dv9[1 + loop_ub];
        }

        *num_return = 2.0;
      }
    }

    /* 判断路径是否需要切换 */
    loop_ub = curve_return_size[1];
    num_curve_tmp = (int)*num_curve;
    for (i0 = 0; i0 < loop_ub; i0++) {
      b_curve_data[i0] = curve_return_data[(num_curve_tmp + curve_return_size[0]
        * i0) - 1];
    }

    /*      if curve(1)==0 */
    /*          n=curve(4:5)'; */
    /*      else */
    /*          n=curve(1)*[cos(curve(5)+pi/2);sin(curve(5)+pi/2)]; */
    /*      end */
    /*       */
    /*      if curve(1)==0 */
    /*          r=curve(2:3)'+curve(6)*curve(4:5)'; */
    /*      else */
    /*          r=curve(2:3)'+curve(6)*[cos(curve(5));sin(curve(5))]; */
    /*      end */
    if (curve_return_data[num_curve_tmp - 1] == 0.0) {
      dL = curve_return_data[((int)*num_curve + curve_return_size[0] * 5) - 1];
      i0 = !((X[5] - (b_curve_data[1] + dL * b_curve_data[3])) * b_curve_data[3]
             + (X[3] - (b_curve_data[2] + dL * b_curve_data[4])) * b_curve_data
             [4] >= 0.0);
    } else {
      /*          n=curve(1)*[cos(curve(5)+pi/2);sin(curve(5)+pi/2)]; */
      dL = curve_return_data[(num_curve_tmp + (curve_return_size[0] << 2)) - 1];
      n_idx_0_tmp = sin(dL);
      dL = cos(dL);
      phi_q = curve_return_data[(num_curve_tmp + curve_return_size[0] * 5) - 1];
      if (((X[5] - (b_curve_data[1] + phi_q * dL)) * -n_idx_0_tmp + (X[3] -
            (b_curve_data[2] + phi_q * n_idx_0_tmp)) * dL >= 0.0) && (X[2] *
           -n_idx_0_tmp + X[0] * dL > 0.0)) {
        i0 = 0;
      } else {
        i0 = 1;
      }
    }

    if (i0 == 0) {
      (*num_curve)++;
      if (*num_curve > curve_return_size[0]) {
        *num_curve = 1.0;
      }
    }

    if (*num_curve <= curve_return_size[0]) {
      /* X(1)北向速度，X(2)向上速度，X(3)东向速度 */
      /* X(1)北向位置，X(2)向上位置，X(3)东向位置 */
      dL = 0.0;
      if (curve_return_data[(int)*num_curve - 1] == 0.0) {
        /* 判断轨迹类型是否为直线 */
        dL = curve_return_data[((int)*num_curve + curve_return_size[0] * 3) - 1];
        n_idx_0_tmp = curve_return_data[((int)*num_curve + (curve_return_size[0]
          << 2)) - 1];
        b_X[0] = dL;
        b_X[1] = n_idx_0_tmp;
        q[0] = dL;
        q[1] = c_norm(b_X) * tan(curve_return_data[((int)*num_curve +
          curve_return_size[0] * 7) - 1]);
        q[2] = curve_return_data[((int)*num_curve + (curve_return_size[0] << 2))
          - 1];
        phi_q = b_norm(q);
        q[0] = dL / phi_q;
        q[1] /= phi_q;
        q[2] = curve_return_data[((int)*num_curve + (curve_return_size[0] << 2))
          - 1] / phi_q;

        /* 期望-实际 */
        /*      ddh=norm([v_e,v_n])*tan(curve(8))-v_h; */
        /*      dL=dot(ep,n); */
        phi_q = -rt_atan2d_snf(dL, n_idx_0_tmp);
        dL = rt_atan2d_snf(X[2], X[0]);

        /* 防止出现“掉头”情况 */
        while (phi_q - (-dL) > 3.1415926535897931) {
          phi_q -= 6.2831853071795862;
        }

        while (phi_q - (-dL) < -3.1415926535897931) {
          phi_q += 6.2831853071795862;
        }

        /* 当前位置期望偏航角 */
        /* 期望-实际 */
        dL = (phi_q + atan(0.05 * (((X[5] - curve_return_data[((int)*num_curve +
                   curve_return_size[0]) - 1]) * (q[1] * 0.0 - (-q[2])) + (X[4]
                  - X[4]) * (q[2] * 0.0 - q[0] * 0.0)) + (X[3] -
                 curve_return_data[((int)*num_curve + (curve_return_size[0] << 1))
                 - 1]) * (-q[0] - q[1] * 0.0)))) - (-dL);
      }

      if ((curve_return_data[(int)*num_curve - 1] == 1.0) || (curve_return_data
           [(int)*num_curve - 1] == -1.0)) {
        /* 判断轨迹类型是否为圆弧 */
        n_idx_0_tmp = curve_return_data[(int)*num_curve - 1];

        /* 期望-实际 */
        /*      ddh=norm([v_e,v_n])*tan(curve(8))-v_h; */
        phi_q = -rt_atan2d_snf(X[5] - curve_return_data[((int)*num_curve +
          curve_return_size[0]) - 1], X[3] - curve_return_data[((int)*num_curve
          + (curve_return_size[0] << 1)) - 1]);
        dL = rt_atan2d_snf(X[2], X[0]);

        /* 防止出现“掉头”情况 */
        while (phi_q - (-dL) > 3.1415926535897931) {
          phi_q -= 6.2831853071795862;
        }

        while (phi_q - (-dL) < -3.1415926535897931) {
          phi_q += 6.2831853071795862;
        }

        /* 期望-实际 */
        b_X[0] = curve_return_data[((int)*num_curve + curve_return_size[0]) - 1]
          - X[5];
        b_X[1] = curve_return_data[((int)*num_curve + (curve_return_size[0] << 1))
          - 1] - X[3];
        dL = ((phi_q + n_idx_0_tmp * 3.1415926535897931 / 2.0) + n_idx_0_tmp *
              atan(2.0 * -(curve_return_data[((int)*num_curve +
                 curve_return_size[0] * 5) - 1] - c_norm(b_X)) /
                   curve_return_data[((int)*num_curve + curve_return_size[0] * 5)
                   - 1])) - (-dL);
      }

      /* 防止掉头多转圈 */
      while (dL < -3.1415926535897931) {
        dL += 6.2831853071795862;
      }

      while (dL > 3.1415926535897931) {
        dL -= 6.2831853071795862;
      }

      *roll = control_roll(X, dL, I_roll);
      *I_roll = 0.0;
    }
  }
}

/*
 * File trailer for guidence_plane.c
 *
 * [EOF]
 */
