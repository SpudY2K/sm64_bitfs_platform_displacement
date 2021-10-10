#include <fstream>
#include <string>

using namespace std;

#ifndef PARAMS_H
#define PARAMS_H

struct SearchParams {
		double max_speed;
		double *upper_y;
		double *lower_y;
		int n_y_ranges;
		int platform_idx;

		double min_nx;
		double max_nx;
		double min_nz;
		double max_nz;
		double min_ny;
		double max_ny;

		int n_samples_x;
		int n_samples_y;
		int n_samples_z;

		bool filter_floor_ranges;
		bool check_fall_through_pus;

		ofstream out_stream;
};
#endif