#include "tmwtypes.h"
#include "point_line_gap.h"
#include "path_follow.h"
#include "guidence_plane.h"
#include "control_roll.h"
#include "judge_curve_change.h"
#include "latlon_to_meter.h"
#include "stdio.h"

int main(void) {
	double vn = -5.7025;
	double vh = 0;
	double ve = 10.5585;
	
	double h = 0;
	double lat0 = 34;
	double lon0 = 109;
	double lat = 34.0003706;
	double lon = 108.9988737;
	

	
	double X[6];

	double curve_data[] = {0,-93.6334,41.2092,0.9153,-0.4028,81.8405,0,0};
	int curve_size[2] = {1,8};
	double tmp1 = 0;
	double* I_roll = &tmp1;
	double tmp2 = 1;
	double* num_curve = &tmp2;
	double tmp3 = 0;
	double* mode_return = &tmp3;
	double tmp4 = 0;
	double* num_return = &tmp4;
	double curve_return_data[16];
	int curve_return_size[2];
	double tmp5 = 0;
	double* roll = &tmp5;
	double tmp6 = 0;
	double* judge_arrive = &tmp6;



	latlon_to_meter(lat, lon, lat0, lon0,X+3,X+5);
	X[0] = vn;
	X[1] = vh;
	X[2] = ve;
	X[4] = h;

		guidence_plane(X, curve_data,
		curve_size, I_roll, num_curve, mode_return, num_return, curve_return_data,
		curve_return_size, roll, judge_arrive);

	printf("%f\n", *roll);

	

	return 0;
}