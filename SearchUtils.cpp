#include "Constants.hpp"
#include "SearchUtils.hpp"
#include <cmath>
#include <fstream>

bool in_triangle(Surface surf, float x, float z) {
	// Check if Mario is currently stood inside the (x, z) bounds
	// of the triangle within the original universe.
	int32_t x1 = surf.vectors[0][0];
	int32_t z1 = surf.vectors[0][2];
	int32_t x2 = surf.vectors[1][0];
	int32_t z2 = surf.vectors[1][2];

	// Check that the point is within the triangle bounds.
	if ((z1 - z) * (x2 - x1) - (x1 - x) * (z2 - z1) < 0) {
		return false;
	}

	// To slightly save on computation time, set this later.
	int32_t x3 = surf.vectors[2][0];
	int32_t z3 = surf.vectors[2][2];

	if ((z2 - z) * (x3 - x2) - (x2 - x) * (z3 - z2) < 0) {
		return false;
	}
	if ((z3 - z) * (x1 - x3) - (x3 - x) * (z1 - z3) < 0) {
		return false;
	}

	return true;
}

bool on_triangle(Surface surf, float x, float y, float z) {
	// Standard floor check for being on the triangle
	float x_mod = static_cast<int16_t>(static_cast<int>(x));
	float y_mod = static_cast<int16_t>(static_cast<int>(y));
	float z_mod = static_cast<int16_t>(static_cast<int>(z));

	int32_t x1 = surf.vectors[0][0];
	int32_t z1 = surf.vectors[0][2];
	int32_t x2 = surf.vectors[1][0];
	int32_t z2 = surf.vectors[1][2];

	// Check that the point is within the triangle bounds.
	if ((z1 - z_mod) * (x2 - x1) - (x1 - x_mod) * (z2 - z1) < 0) {
		return false;
	}

	// To slightly save on computation time, set this later.
	int32_t x3 = surf.vectors[2][0];
	int32_t z3 = surf.vectors[2][2];

	if ((z2 - z_mod) * (x3 - x2) - (x2 - x_mod) * (z3 - z2) < 0) {
		return false;
	}
	if ((z3 - z_mod) * (x1 - x3) - (x3 - x_mod) * (z1 - z3) < 0) {
		return false;
	}

	// Find the height of the floor at a given location.
	float height = -(x_mod * surf.normal[0] + surf.normal[2] * z_mod + surf.originOffset) / surf.normal[1];
	// Checks for floor interaction with a 78 unit buffer.
	if (y_mod - (height + -78.0f) < 0.0f) {
		return false;
	}

	return true;
}

void write_to_stream(ofstream* out_stream, Solution* s, int tri_idx, float tri_y_norm, Vec3f& final_pos) {
	char p_idx_b = (char)s->platform_idx;
	char t_idx_b = (char)tri_idx;
	uint16_t yaw16 = (uint16_t)s->yaw;

	#pragma omp critical 
	{
		out_stream->write((char *)&p_idx_b, sizeof(char));
		out_stream->write((char *)&t_idx_b, sizeof(char));
		out_stream->write((char *)&yaw16, sizeof(uint16_t));
		out_stream->write((char *)&s->speed, sizeof(float));
		out_stream->write((char *)&s->platform_normal[0], sizeof(float));
		out_stream->write((char *)&s->platform_normal[1], sizeof(float));
		out_stream->write((char *)&s->platform_normal[2], sizeof(float));
		out_stream->write((char *)&s->position[0], sizeof(float));
		out_stream->write((char *)&s->position[1], sizeof(float));
		out_stream->write((char *)&s->position[2], sizeof(float));
		out_stream->write((char *)&final_pos[0], sizeof(float));
		out_stream->write((char *)&final_pos[1], sizeof(float));
		out_stream->write((char *)&final_pos[2], sizeof(float));
		out_stream->write((char *)&tri_y_norm, sizeof(float));
	}

}

void get_start_poly(std::vector<Vec3f> &start_poly, Vec3f &normal, const float (&tilt)[3]) {
	std::vector<Vec3f> new_start_poly;
	bool first_side;
	bool current_side;

	first_side = (start_poly[0][0] / sqrt(start_poly[0][0] * start_poly[0][0] + start_poly[0][2] * start_poly[0][2] + 250000.0) < normal[0]) ^ (tilt[0] > 0.0);
	current_side = first_side;

	for (int j = 0; j < start_poly.size(); j++) {
		if (current_side) {
			new_start_poly.push_back(start_poly[j]);
		}

		bool next_side;

		if (j == start_poly.size() - 1) {
			next_side = first_side;
		}
		else {
			next_side = (start_poly[j + 1][0] / sqrt(start_poly[j + 1][0] * start_poly[j + 1][0] + start_poly[j + 1][2] * start_poly[j + 1][2] + 250000.0) < normal[0]) ^ (tilt[0] > 0.0);
		}

		if (current_side ^ next_side) {
			double x_diff = start_poly[(j + 1) % start_poly.size()][0] - start_poly[j][0];
			double y_diff = start_poly[(j + 1) % start_poly.size()][1] - start_poly[j][1];
			double z_diff = start_poly[(j + 1) % start_poly.size()][2] - start_poly[j][2];

			double a = ((1.0 - normal[0] * normal[0]) * x_diff * x_diff - normal[0] * normal[0] * z_diff * z_diff);
			double b = (2.0 * start_poly[j][0] * (1.0 - normal[0] * normal[0])*x_diff - 2.0 * start_poly[j][2] * normal[0] * normal[0] * z_diff);
			double c = ((1.0 - normal[0] * normal[0]) * start_poly[j][0] * start_poly[j][0] - normal[0] * normal[0] * start_poly[j][2] * start_poly[j][2] - 250000.0 * normal[0] * normal[0]);

			double p1 = -b / (2.0 * a);
			double p2 = sqrt(b * b - 4.0 * a * c) / (2.0 * a);

			double t1 = p1 - p2;
			double t2 = p1 + p2;

			if (t1 >= 0 && t1 <= 1) {
				Vec3f new_point;
				new_point[0] = start_poly[j][0] + x_diff * t1;
				new_point[1] = start_poly[j][1] + y_diff * t1;
				new_point[2] = start_poly[j][2] + z_diff * t1;

				if (!((new_point[0] < 0) ^ (normal[0] < 0))) {
					new_start_poly.push_back(new_point);
				}
			}

			if (t2 >= 0 && t2 <= 1) {
				Vec3f new_point;
				new_point[0] = start_poly[j][0] + x_diff * t2;
				new_point[1] = start_poly[j][1] + y_diff * t2;
				new_point[2] = start_poly[j][2] + z_diff * t2;

				if (!((new_point[0] < 0) ^ (normal[0] < 0))) {
					new_start_poly.push_back(new_point);
				}
			}
		}

		current_side = next_side;
	}

	start_poly = std::move(new_start_poly);

	if (start_poly.empty()) return;

	first_side = (start_poly[0][2] / sqrt(start_poly[0][0] * start_poly[0][0] + start_poly[0][2] * start_poly[0][2] + 250000.0) < normal[2]) ^ (tilt[2] > 0.0);
	current_side = first_side;

	for (int j = 0; j < start_poly.size(); j++) {
		if (current_side) {
			new_start_poly.push_back(start_poly[j]);
		}

		bool next_side;

		if (j == start_poly.size() - 1) {
			next_side = first_side;
		}
		else {
			next_side = (start_poly[j + 1][2] / sqrt(start_poly[j + 1][0] * start_poly[j + 1][0] + start_poly[j + 1][2] * start_poly[j + 1][2] + 250000.0) < normal[2]) ^ (tilt[2] > 0.0);
		}

		if (current_side ^ next_side) {
			double x_diff = start_poly[(j + 1) % start_poly.size()][0] - start_poly[j][0];
			double y_diff = start_poly[(j + 1) % start_poly.size()][1] - start_poly[j][1];
			double z_diff = start_poly[(j + 1) % start_poly.size()][2] - start_poly[j][2];

			double a = ((1.0 - normal[2] * normal[2]) * z_diff * z_diff - normal[2] * normal[2] * x_diff * x_diff);
			double b = (2.0 * start_poly[j][2] * (1.0 - normal[2] * normal[2]) * z_diff - 2.0 * normal[2] * normal[2] * start_poly[j][0] * x_diff);
			double c = ((1.0 - normal[2] * normal[2]) * start_poly[j][2] * start_poly[j][2] - normal[2] * normal[2] * start_poly[j][0] * start_poly[j][0] - 250000.0 * normal[2] * normal[2]);

			double p1 = -b / (2.0 * a);
			double p2 = sqrt(b * b - 4.0 * a * c) / (2.0 * a);

			double t1 = p1 - p2;
			double t2 = p1 + p2;

			if (t1 >= 0 && t1 <= 1) {
				Vec3f new_point;
				new_point[0] = start_poly[j][0] + x_diff * t1;
				new_point[1] = start_poly[j][1] + y_diff * t1;
				new_point[2] = start_poly[j][2] + z_diff * t1;

				if (!((new_point[2] < 0) ^ (normal[2] < 0))) {
					new_start_poly.push_back(new_point);
				}
			}

			if (t2 >= 0 && t2 <= 1) {
				Vec3f new_point;
				new_point[0] = start_poly[j][0] + x_diff * t2;
				new_point[1] = start_poly[j][1] + y_diff * t2;
				new_point[2] = start_poly[j][2] + z_diff * t2;

				if (!((new_point[2] < 0) ^ (normal[2] < 0))) {
					new_start_poly.push_back(new_point);
				}
			}
		}

		current_side = next_side;
	}

	start_poly = std::move(new_start_poly);

	if (start_poly.empty()) return;

	double r2 = 250000.0*((1.0 / (normal[1] * normal[1])) - 1.0);

	first_side = ((start_poly[0][0] * start_poly[0][0] + start_poly[0][2] * start_poly[0][2]) < r2) ^ (tilt[1] < 0.0);
	current_side = first_side;

	for (int j = 0; j < start_poly.size(); j++) {
		if (current_side) {
			new_start_poly.push_back(start_poly[j]);
		}

		bool next_side;

		if (j == start_poly.size() - 1) {
			next_side = first_side;
		}
		else {
			next_side = ((start_poly[j + 1][0] * start_poly[j + 1][0] + start_poly[j + 1][2] * start_poly[j + 1][2]) < r2) ^ (tilt[1] < 0.0);
		}

		if (current_side ^ next_side) {
			double x_diff = start_poly[(j + 1) % start_poly.size()][0] - start_poly[j][0];
			double y_diff = start_poly[(j + 1) % start_poly.size()][1] - start_poly[j][1];
			double z_diff = start_poly[(j + 1) % start_poly.size()][2] - start_poly[j][2];

			double a = (z_diff * z_diff) + (x_diff * x_diff);
			double b = (2.0 * start_poly[j][2] * z_diff) + (2.0 * start_poly[j][0] * x_diff);
			double c = start_poly[j][2] * start_poly[j][2] + start_poly[j][0] * start_poly[j][0] - r2;

			double p1 = -b / (2.0 * a);
			double p2 = sqrt(b * b - 4.0 * a * c) / (2.0 * a);

			double t1 = p1 - p2;
			double t2 = p1 + p2;

			if (t1 >= 0 && t1 <= 1) {
				Vec3f new_point;
				new_point[0] = start_poly[j][0] + x_diff * t1;
				new_point[1] = start_poly[j][1] + y_diff * t1;
				new_point[2] = start_poly[j][2] + z_diff * t1;

				new_start_poly.push_back(new_point);
			}

			if (t2 >= 0 && t2 <= 1) {
				Vec3f new_point;
				new_point[0] = start_poly[j][0] + x_diff * t2;
				new_point[1] = start_poly[j][1] + y_diff * t2;
				new_point[2] = start_poly[j][2] + z_diff * t2;

				new_start_poly.push_back(new_point);
			}
		}

		current_side = next_side;
	}

	start_poly = std::move(new_start_poly);
}

void gaussian_elimination(double (&coeff1a)[4], double (&coeff1b)[4], double (&coeff1c)[4]) {
	if (abs(coeff1a[1]) < abs(coeff1b[1])) {
		double temp = *coeff1a;
		*coeff1a = *coeff1b;
		*coeff1b = temp;
	}

	if (abs(coeff1b[1]) < abs(coeff1c[1])) {
		double temp = *coeff1b;
		*coeff1b = *coeff1c;
		*coeff1c = temp;

		if (abs(coeff1a[1]) < abs(coeff1b[1])) {
			double temp = *coeff1a;
			*coeff1a = *coeff1b;
			*coeff1b = temp;
		}
	}

	if (coeff1a[1] != 0.0) {
		coeff1a[0] = coeff1a[0] / coeff1a[1];
		coeff1a[2] = coeff1a[2] / coeff1a[1];
		coeff1a[3] = coeff1a[3] / coeff1a[1];
		coeff1a[1] = 1.0;

		coeff1b[0] -= coeff1b[1] * coeff1a[0];
		coeff1b[2] -= coeff1b[1] * coeff1a[2];
		coeff1b[3] -= coeff1b[1] * coeff1a[3];
		coeff1b[1] = 0.0;

		coeff1c[0] -= coeff1c[1] * coeff1a[0];
		coeff1c[2] -= coeff1c[1] * coeff1a[2];
		coeff1c[3] -= coeff1c[1] * coeff1a[3];
		coeff1c[1] = 0.0;
	}

	if (abs(coeff1b[2]) < abs(coeff1c[2])) {
		double temp = *coeff1b;
		*coeff1b = *coeff1c;
		*coeff1c = temp;
	}

	if (coeff1b[2] != 0.0) {
		coeff1b[0] = coeff1b[0] / coeff1b[2];
		coeff1b[1] = coeff1b[1] / coeff1b[2];
		coeff1b[3] = coeff1b[3] / coeff1b[2];
		coeff1b[2] = 1.0;

		coeff1a[0] -= coeff1a[2] * coeff1b[0];
		coeff1a[1] -= coeff1a[2] * coeff1b[1];
		coeff1a[3] -= coeff1a[2] * coeff1b[3];
		coeff1a[2] = 0.0;

		coeff1c[0] -= coeff1c[2] * coeff1b[0];
		coeff1c[1] -= coeff1c[2] * coeff1b[1];
		coeff1c[3] -= coeff1c[2] * coeff1b[3];
		coeff1c[2] = 0.0;
	}

	if (coeff1c[3] != 0.0) {
		coeff1c[0] = coeff1c[0] / coeff1c[3];
		coeff1c[1] = coeff1c[1] / coeff1c[3];
		coeff1c[2] = coeff1c[2] / coeff1c[3];
		coeff1c[3] = 1.0;

		coeff1a[0] -= coeff1a[3] * coeff1c[0];
		coeff1a[1] -= coeff1a[3] * coeff1c[1];
		coeff1a[2] -= coeff1a[3] * coeff1c[2];
		coeff1a[3] = 0.0;

		coeff1b[0] -= coeff1b[3] * coeff1c[0];
		coeff1b[1] -= coeff1b[3] * coeff1c[1];
		coeff1b[2] -= coeff1b[3] * coeff1c[2];
		coeff1b[3] = 0.0;
	}
}

bool check_fall_through_pus(Solution* s, Vec3f &final_pos) {
	float floor_dist = 65536.0;

	for (int f = 0; f < n_floor_ranges; f++) {
		float f_dist = final_pos[1] - lower_floor[f];

		if (f_dist > 0) {
			floor_dist = f_dist;
		}
		else {
			break;
		}
	}

	int falling_frames = (int)ceil((sqrt(2.0*floor_dist + 1.0) + 1.0) / 2.0);

	int closest_pu_dist = min(min(final_pos[0] + pow(2, 31), pow(2, 31) - 1.0 - final_pos[0]), min(final_pos[2] + pow(2, 31), pow(2, 31) - 1.0 - final_pos[2]));

	if (closest_pu_dist >= s->speed / 4.0) {
		int total_falling_frames = (int)floor((pow(2, 32) - closest_pu_dist - 3.0 * s->speed / 2.0) / s->speed);

		if (falling_frames <= total_falling_frames) {
			return true;
		}
	}

	return false;
}