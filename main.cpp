#include "Search.hpp"
#include <string>

int main() {
	SearchParams params;

	params.platform_idx = 0;

	params.max_speed = 1800000000.0;

	const int n_y_ranges = 1;
	double lower_y[n_y_ranges] = { -1357.0 };
	double upper_y[n_y_ranges] = { 32767.0 };

	params.n_y_ranges = n_y_ranges;
	params.lower_y = &(lower_y[0]);
	params.upper_y = &(upper_y[0]);

	params.min_nx = -0.156;
	params.max_nx = -0.118;
	params.min_nz = 0.46;
	params.max_nz = 0.498;
	params.min_ny = 0.75428307056427;
	params.max_ny = 0.75428307056427;

	params.n_samples_x = 200;
	params.n_samples_y = 1;
	params.n_samples_z = 200;

	params.filter_floor_ranges = true;
	params.check_fall_through_pus = true;

	const string out_file = "Solutions.bin";
	params.out_stream.open(out_file, std::ios_base::binary);

	search_normals(&params);

	params.out_stream.close();
}

