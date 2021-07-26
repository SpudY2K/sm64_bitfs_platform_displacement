#include "Mario.hpp"
#include "Magic.hpp"
#include "Platform.hpp"
#include "vmath.hpp"
#include <cmath>

# define M_PI           3.14159265358979323846  /* pi */

using namespace std;

const double lower_y = 3521.0;
const double upper_y = 3841.0;
const double max_speed = 1000000000.0;
const double lava_y = -3071.0;
const double normal_offsets[4][3] = { {0.01f, -0.01f, 0.01f}, {-0.01f, -0.01f, 0.01f}, {-0.01f, -0.01f, -0.01f}, {0.01f, -0.01f, -0.01f} };

bool validate_solution(const Vec3f& m_pos, const Vec3f& normals, const float speed, int hau) {
	Platform plat;

	plat.normal[0] = 0;
	plat.normal[1] = 1;
	plat.normal[2] = 0;
	plat.create_transform_from_normals();

	Mat4 old_mat = plat.transform;

	plat.normal = normals;

	plat.create_transform_from_normals();
	plat.triangles[0].rotate(plat.pos, old_mat, plat.transform);
	plat.triangles[1].rotate(plat.pos, old_mat, plat.transform);

	Mario mario(m_pos, speed);

	if (mario.ground_step(hau, normals[1]) == 0) { 
		return false;
	}

	if (!plat.find_floor(&mario)) { 
		return false;
	}

	plat.platform_logic(&mario);

	if (!check_inbounds(mario)) { 
		return false;
	}

	if (mario.pos[1] >= lower_y && mario.pos[1] < upper_y) {
		return true;
	}
	else {
		return false;
	}
}
 
/*
	The original code I used for validating solutions.
	I replaced this with the current function that
	more closely emulates SM64's behaviour.

	I kept it here because it does a nice job of 
	documenting the situations where the code can
	produce invalid solutions, and how they could be
	resolved with some more work.
bool validate_solution(Platform* plat, double hau, double pu_x, double pu_z, double* platform_x, double* platform_z, double nx, double ny, double nz, float start_x, float start_y, float start_z, float end_x, float end_z, float speed, int yaw, int tilt_idx) {
	Vec3f dist;

	dist[0] = end_x - plat->pos[0];
	dist[1] = start_y - plat->pos[1];
	dist[2] = end_z - plat->pos[2];;

	float dx = dist[0];
	float dy = 500.0f;
	float dz = dist[2];
	float d = sqrtf(dx * dx + dy * dy + dz * dz);

	if (d != 0.0f) {
		// Normalizing
		d = 1.0 / d;
		dx *= d;
		dy *= d;
		dz *= d;
	}
	else {
		dx = 0.0f;
		dy = 1.0f;
		dz = 0.0f;
	}

	float new_nx = approach_by_increment(dx, (float)nx, 0.01f);
	float new_ny = approach_by_increment(dy, (float)ny, 0.01f);
	float new_nz = approach_by_increment(dz, (float)nz, 0.01f);

	// Check the platform tilt matches what we expect
	if ((new_nx - nx)*normal_offsets[tilt_idx][0] > 0 && (new_ny - ny)*normal_offsets[tilt_idx][1] > 0 && (new_nz - nz)*normal_offsets[tilt_idx][2] > 0) {
		// Check we're still on the platform after tilts
		if (on_triangle(plat->triangles[0], end_x, start_y, end_z, new_nx, new_ny, new_nz) || on_triangle(plat->triangles[1], end_x, start_y, end_z, new_nx, new_ny, new_nz)) {
			plat->normal[0] = nx;
			plat->normal[1] = ny;
			plat->normal[2] = nz;
			plat->create_transform_from_normals();

			Vec3f posBeforeRotation;
			linear_mtxf_mul_vec3f(posBeforeRotation, plat->transform, dist);

			plat->normal[0] = new_nx;
			plat->normal[1] = new_ny;
			plat->normal[2] = new_nz;
			plat->create_transform_from_normals();

			Vec3f posAfterRotation;
			linear_mtxf_mul_vec3f(posAfterRotation, plat->transform, dist);

			float final_x = end_x + (posAfterRotation[0] - posBeforeRotation[0]);
			float final_y = start_y + (posAfterRotation[1] - posBeforeRotation[1]);
			float final_z = end_z + (posAfterRotation[2] - posBeforeRotation[2]);

			float final_x_mod = static_cast<int16_t>(static_cast<int>(final_x));
			float final_z_mod = static_cast<int16_t>(static_cast<int>(final_z));

			if (abs(final_x_mod) < 8192 && abs(final_z_mod) < 8192) {
				// Check our final y position is within the acceptable range.
				if (final_y >= lower_y && final_y <= upper_y) {
					if (validate_solution({ start_x, start_y, start_z }, { (float)nx, (float)ny, (float)nz }, speed, 16 * hau)) {
						printf("Solution found:\nSpeed: %f\nHau: %d\nPlatform normals: (%.9f, %.9f, %.9f)\nMario pos: (%.9f, %.9f, %.9f)\nMario start: (%.9f, %.9f, %.9f)\n",
							speed, (int)hau, (float)nx, (float)ny, (float)nz, end_x, start_y, end_z, start_x, start_y, start_z);
						return true;
					}
					else {
						// Erm, it would be bad if this happens!
						// Both validators should be identical.
						return false;
					}
				}
				else {
					// Sometimes we end up slightly out of our desired height range, 
					// due being at a slightly different starting height to our initial estimate.
					// Not sure if this can be avoided easily, may be easier just to ignore these cases.

					//printf("Something has gone wrong here:\nSpeed: %f\nHau: %d\nPlatform normals: (%.9f, %.9f, %.9f)\nMario pos: (%.9f, %.9f, %.9f)\nMario start: (%.9f, %.9f, %.9f)\n",
					//	speed, (int)hau, (float)nx, (float)ny, (float)nz, end_x, start_y, end_z, start_x, start_y, start_z);
					return false;
				}
			}
			else {
				// Sometimes the platform displacement puts us slightly out of bounds
				// May be able to eliminate some of these cases by more careful selection
				// of yaw and platform position. Or less leeway in picking PUs.
				return false;
			}
		}
		else {
			// Rarely, Mario can be "lifted" off of the platform when it tilts
			// This means he doesn't get the platform displacement.
			//
			// Chances are that if we try again with a different speed or
			// starting location this won't happen.
			// Maybe consider adding that in.
			return false;
		}
	}
	else {
		// Sometimes the PU we test is slightly outside the polygon, and so we end up here.

		//printf("Something has gone wrong here:\nSpeed: %f\nHau: %d\nPlatform normals: (%.9f, %.9f, %.9f)\nMario pos: (%.9f, %.9f, %.9f)\nMario start: (%.9f, %.9f, %.9f)\n",
		//	speed, (int)hau, (float)nx, (float)ny, (float)nz, end_x, start_y, end_z, start_x, start_y, start_z);
		return false;
	}
}
*/

bool in_triangle(Surface surf, float x, float z) {
	// Check if Mario is currently stood inside the (x, z) bounds
	// of the triangle within the original universe.
	int32_t x1 = surf.vector1[0];
	int32_t z1 = surf.vector1[2];
	int32_t x2 = surf.vector2[0];
	int32_t z2 = surf.vector2[2];

	// Check that the point is within the triangle bounds.
	if ((z1 - z) * (x2 - x1) - (x1 - x) * (z2 - z1) < 0) {
		return false;
	}

	// To slightly save on computation time, set this later.
	int32_t x3 = surf.vector3[0];
	int32_t z3 = surf.vector3[2];

	if ((z2 - z) * (x3 - x2) - (x2 - x) * (z3 - z2) < 0) {
		return false;
	}
	if ((z3 - z) * (x1 - x3) - (x3 - x) * (z1 - z3) < 0) {
		return false;
	}

	return true;
}

bool on_triangle(Surface surf, float x, float y, float z, float nx, float ny, float nz) {
	// Standard floor check for being on the triangle
	float x_mod = static_cast<int16_t>(static_cast<int>(x));
	float y_mod = static_cast<int16_t>(static_cast<int>(y));
	float z_mod = static_cast<int16_t>(static_cast<int>(z));

	int32_t x1 = surf.vector1[0];
	int32_t z1 = surf.vector1[2];
	int32_t x2 = surf.vector2[0];
	int32_t z2 = surf.vector2[2];

	// Check that the point is within the triangle bounds.
	if ((z1 - z_mod) * (x2 - x1) - (x1 - x_mod) * (z2 - z1) < 0) {
		return false;
	}

	// To slightly save on computation time, set this later.
	int32_t x3 = surf.vector3[0];
	int32_t z3 = surf.vector3[2];

	if ((z2 - z_mod) * (x3 - x2) - (x2 - x_mod) * (z3 - z2) < 0) {
		return false;
	}
	if ((z3 - z_mod) * (x1 - x3) - (x3 - x_mod) * (z1 - z3) < 0) {
		return false;
	}

	float oo = -(nx * x1 + ny * surf.vector1[1] + nz * z1);

	// Find the height of the floor at a given location.
	float height = -(x_mod * nx + nz * z_mod + oo) / ny;
	// Checks for floor interaction with a 78 unit buffer.
	if (y_mod - (height + -78.0f) < 0.0f) {
		return false;
	}

	return true;
}

void try_hau(Platform* plat, double hau, double pu_x, double pu_z, double* platform_x, double* platform_z, double nx, double ny, double nz, int tilt_idx) {
	double angle = 2.0 * M_PI * (hau / 4096.0);
	double max_speed = 0;
	double min_speed = INFINITY;

	double min_speed_start_x = NAN;
	double min_speed_start_z = NAN;
	double min_speed_end_x = NAN;
	double min_speed_end_z = NAN;
	double max_speed_start_x = NAN;
	double max_speed_start_z = NAN;
	double max_speed_end_x = NAN;
	double max_speed_end_z = NAN;

	// Find the maximum and minimum possible speed that allow you 
	// to land on the PU platform with the current yaw.
	// 
	// Will be to/from one of the platform corners, so test all of them.
	//
	// For each corner, find the closest and furthest points on other
	// platform that can be reached with current yaw.
	//
	// Then calculate required speed from distance.

	// Check corners of real platform.
	for (int a = 0; a < 4; a++) {
		for (int b = 0; b < 4; b++) {
			double t_n = (platform_z[b] + pu_z - platform_z[a] - (1 / tan(angle))*(platform_x[b] + pu_x) + (1 / tan(angle))*platform_x[a]);
			double t_d = ((1 / tan(angle))*platform_x[(b + 1) % 4] - (1 / tan(angle))*platform_x[b] - platform_z[(b + 1) % 4] + platform_z[b]);

			if (t_d < 0) {
				t_n = -t_n;
				t_d = -t_d;
			}

			if (t_n >= 0 && t_n <= t_d) {
				double start_x = platform_x[a];
				double end_x = pu_x + platform_x[b] + (platform_x[(b + 1) % 4] - platform_x[b])*(t_n / t_d);

				double start_z = platform_z[a];
				double end_z = pu_z + platform_z[b] + (platform_z[(b + 1) % 4] - platform_z[b])*(t_n / t_d);

				double x_dist = end_x - start_x;
				double z_dist = end_z - start_z;

				double speed = sqrt(x_dist * x_dist + z_dist * z_dist) / ny;

				if (speed < min_speed) {
					min_speed = speed;
					min_speed_start_x = start_x;
					min_speed_start_z = start_z;
					min_speed_end_x = end_x;
					min_speed_end_z = end_z;
				}

				if (speed > max_speed) {
					max_speed = speed;
					max_speed_start_x = start_x;
					max_speed_start_z = start_z;
					max_speed_end_x = end_x;
					max_speed_end_z = end_z;
				}
			}
		}
	}

	// Check corners of PU platform.
	for (int b = 0; b < 4; b++) {
		for (int a = 0; a < 4; a++) {
			double t_n = (platform_z[a] - platform_z[b] - pu_z - (1 / tan(angle))*platform_x[a] + (1 / tan(angle))*(platform_x[b] + pu_x));
			double t_d = ((1 / tan(angle))*platform_x[(b + 1) % 4] - (1 / tan(angle))*platform_x[b] - platform_z[(b + 1) % 4] + platform_z[b]);

			if (t_d < 0) {
				t_n = -t_n;
				t_d = -t_d;
			}

			if (t_n >= 0 && t_n <= t_d) {
				double start_x = platform_x[a];
				double end_x = pu_x + platform_x[b] + (platform_x[(b + 1) % 4] - platform_x[b])*(t_n / t_d);

				double start_z = platform_z[a];
				double end_z = pu_z + platform_z[b] + (platform_z[(b + 1) % 4] - platform_z[b])*(t_n / t_d);

				double x_dist = end_x - start_x;
				double z_dist = end_z - start_z;

				double speed = sqrt(x_dist * x_dist + z_dist * z_dist) / ny;

				if (speed < min_speed) {
					min_speed = speed;
					min_speed_start_x = start_x;
					min_speed_start_z = start_z;
					min_speed_end_x = end_x;
					min_speed_end_z = end_z;
				}

				if (speed > max_speed) {
					max_speed = speed;
					max_speed_start_x = start_x;
					max_speed_start_z = start_z;
					max_speed_end_x = end_x;
					max_speed_end_z = end_z;
				}
			}
		}
	}

	// So far we've been working with doubles for precision,
	// but our final parameters need to be floats.
	//
	// Now we have most of our parameters calculuated, 
	// convert back to floats.
	float min_speed_f = (float)min_speed;
	float max_speed_f = (float)max_speed;

	// We want our float speed range to be a subset of our double
	// speed range. So if conversion has pushed our range outwards
	// shift it back inwards.
	if ((double)min_speed_f < min_speed) {
		min_speed_f = nextafterf(min_speed_f, INFINITY);
	}

	if ((double)max_speed_f > max_speed) {
		max_speed_f = nextafterf(max_speed_f, -INFINITY);
	}

	// Check that there even is a float value within the valid speed range.
	if (min_speed_f < max_speed_f) {
		// We need to find a place on the real platform where Mario 
		// can stand that allows him to reach the PU platform.
		//
		// Start by picking the location halfway between the two speed extremes.
		float start_x = (float)((min_speed_start_x + max_speed_start_x) / 2.0f);
		float start_z = (float)((min_speed_start_z + max_speed_start_z) / 2.0f);

		float oo;

		if (in_triangle(plat->triangles[0], start_x, start_z)) {
			oo = -(nx * plat->triangles[0].vector1[0] + ny * plat->triangles[0].vector1[1] + nz * plat->triangles[0].vector1[2]);
		}
		else if (in_triangle(plat->triangles[1], start_x, start_z)) {
			oo = -(nx * plat->triangles[1].vector1[0] + ny * plat->triangles[1].vector1[1] + nz * plat->triangles[1].vector1[2]);
		}
		else {
			// Somehow, our start position has missed the platform.
			// Shouldn't be possible unless something is coded wrong.
			//
			//printf("Something has gone wrong here: %f\nHau: %d\nPlatform normals: (%.9f, %.9f, %.9f)\nMario start: (%.9f, %.9f)\n",
			//	hau, nx, ny, nz, start_x, start_z);
			return;
		}

		// Find the height of the floor at a given location.
		float start_y = -(start_x * nx + nz * start_z + oo) / ny;

		bool searching = true;

		// Search for a position on the real platform and a speed that put us on the PU platform
		float upper_speed = max_speed_f;
		float lower_speed = min_speed_f;

		while (searching) {
			float speed = (upper_speed + lower_speed) / 2.0f;
			float end_x = start_x;
			float end_z = start_z;

			// Calculate our final position with quarter steps
			for (int j = 0; j < 4; j++) {
				end_x = end_x + gSineTable[hau] * (float)ny * (speed / 4.0f);
				end_z = end_z + gCosineTable[hau] * (float)ny * (speed / 4.0f);
			}

			// Check if Mario is on the PU platform. 
			if (on_triangle(plat->triangles[0], end_x, start_y, end_z, nx, ny, nz) || on_triangle(plat->triangles[1], end_x, start_y, end_z, nx, ny, nz)) {
				// If so, validate the final parameter set.
				// Can fail for a number of reasons, see original
				// validate_solution for more details
				if (validate_solution({ start_x, start_y, start_z }, { (float)nx, (float)ny, (float)nz }, speed, 16 * hau)) {
					printf("Solution found:\nSpeed: %f\nHau: %d\nPlatform normals: (%.9f, %.9f, %.9f)\nMario pos: (%.9f, %.9f, %.9f)\nMario start: (%.9f, %.9f, %.9f)\n",
						speed, (int)hau, (float)nx, (float)ny, (float)nz, end_x, start_y, end_z, start_x, start_y, start_z);
				}
				searching = false;
			}
			else {
				// If we missed the PU platform, adjust our speed and try again.
				// Work out if we over or undershot the platform, and adjust speed accordingly.
				bool overshot = false;

				for (int b = 0; b < 4; b++) {
					double pu_platform_start_x = pu_x + platform_x[b];
					double pu_platform_start_z = pu_z + platform_z[b];
					double pu_platform_end_x = pu_x + platform_x[(b + 1) % 4];
					double pu_platform_end_z = pu_z + platform_z[(b + 1) % 4];

					double t_n = (start_x - pu_platform_start_x)*(pu_platform_start_z - pu_platform_end_z) - (start_z - pu_platform_start_z)*(pu_platform_start_x - pu_platform_end_x);
					double t_d = (start_x - end_x)*(pu_platform_start_z - pu_platform_end_z) - (start_z - end_z)*(pu_platform_start_x - pu_platform_end_x);
					double u_n = (end_x - start_x)*(start_z - pu_platform_start_z) - (end_z - start_z)*(start_x - pu_platform_start_x);

					if (t_d < 0) {
						t_n = -t_n;
						t_d = -t_d;
						u_n = -u_n;
					}

					if (t_n >= 0 && t_n <= t_d && u_n >= 0 && u_n <= t_d) {
						overshot = true;
						break;
					}
				}

				if (overshot) {
					upper_speed = nextafterf(speed, -INFINITY);
				}
				else {
					lower_speed = nextafterf(speed, INFINITY);
				}

				if (upper_speed < lower_speed) {
					if (start_x == max_speed_start_x && start_z == max_speed_start_z) {
						// Search couldn't find a suitable place on the platform to do the desired displacement.
						// Maybe no solutions exist here.
						// Though, perhaps this can be resolved with a more comprehensive search of the platform.

						//printf("Something has gone wrong here:\nSpeed: %f\nHau: %d\nPlatform normals: (%.9f, %.9f, %.9f)\nMario pos: (%.9f, %.9f, %.9f)\nMario start: (%.9f, %.9f, %.9f)\n",
						//	speed, (int)hau, (float)nx, (float)ny, (float)nz, end_x, start_y, end_z, start_x, start_y, start_z);
						searching = false;
					}
					else {
						// If all speeds fail to land us on the platform, try a different starting position.
						if (start_x != max_speed_start_x) {
							float dir_x = (max_speed_start_x - start_x)*INFINITY;
							start_x = (float)((nextafterf(start_x, dir_x) + max_speed_start_x) / 2.0f);
						}

						if (start_z != max_speed_start_z) {
							float dir_z = (max_speed_start_z - start_z)*INFINITY;
							start_z = (float)((nextafterf(start_z, dir_z) + max_speed_start_z) / 2.0f);
						}

						start_y = -(start_x * nx + nz * start_z + oo) / ny;

						float upper_speed = max_speed_f;
						float lower_speed = min_speed_f;
					}
				}
			}
		}
	}
}

bool try_pu_xz(Platform* plat, double x, double z, double nx, double ny, double nz, double* platform_x, double* platform_z, int tilt_idx) {
	// For current (x, z) PU position, find range of yaws that
	// allow you to reach the PU platform from the original universe.
	double min_hau = 4096;
	double max_hau = -1;

	double min_dist = INFINITY;

	for (int a = 0; a < 4; a++) {
		for (int b = 0; b < 4; b++) {
			double x_dist = x + platform_x[b] - platform_x[a];
			double z_dist = z + platform_z[b] - platform_z[a];

			double hau;

			if (x == 0 && z > 0) {
				//Special case to handle the discontinuity at 0 degrees
				hau = 4096 * (atan2(x_dist, z_dist) / (2.0 * M_PI));
			}
			else {
				hau = 4096 * (fmod(2.0 * M_PI + atan2(x_dist, z_dist), 2.0 * M_PI) / (2.0 * M_PI));
			}

			min_hau = fmin(min_hau, hau);
			max_hau = fmax(max_hau, hau);

			min_dist = fmin(min_dist, sqrt(x_dist * x_dist + z_dist * z_dist));
		}
	}

	// Check that the minimum speed needed to reach the PU platform does 
	// not exceed our desired maximum 
	double min_needed_speed = min_dist / ny;

	if (min_needed_speed > max_speed) {
		return false;
	}

	// Round yaws to nearest hau index 
	min_hau = ceil(min_hau);
	max_hau = floor(max_hau);

	// Try each possible hau index
	for (double hau = min_hau; hau <= max_hau; hau++) {
		try_hau(plat, fmod(4096.0 + hau, 4096.0), x, z, platform_x, platform_z, nx, ny, nz, tilt_idx);
	}

	return true;
}

bool try_pu_x(Platform* plat, Mat4 T_start, Mat4 T_tilt, double x, double x1_min, double x1_max, double x2_min, double x2_max, double platform_min_x, double platform_max_x, double platform_min_z, double platform_max_z, double m, double c_min, double c_max, double nx, double ny, double nz, double* platform_x, double* platform_z, int tilt_idx) {
	double pu_platform_min_x = x + platform_min_x;
	double pu_platform_max_x = x + platform_max_x;

	// Find maximal range of PUs along z axis from current x PU position
	double min_z_pu_idx = (m * pu_platform_min_x + c_min) / 262144.0;
	double max_z_pu_idx = (m * pu_platform_max_x + c_max) / 262144.0;

	if (min_z_pu_idx > max_z_pu_idx) {
		double temp = min_z_pu_idx;
		min_z_pu_idx = max_z_pu_idx;
		max_z_pu_idx = temp;
	}

	// Check max_x_pu_idx and min_x_pu_idx are in range for valid platform tilt.
	// Correct them if they're not.
	//
	// Possible for only part of the platform to be in range.
	// In this case just skip whole PU to avoid headaches later on.

	if (pu_platform_max_x > fmin(x1_min, x1_max) && pu_platform_min_x < fmax(x1_min, x1_max)) {
		double z1_min = m * x1_min + c_min;
		double z1_max = m * x1_max + c_max;
		double tilt_cutoff_z = (z1_max - z1_min)*(x - x1_min) / (x1_max - x1_min) + z1_min;

		if (x1_min > 0) {
			// Find new lower bound for z_pu
			double tilt_cutoff_pu_idx = (tilt_cutoff_z - platform_max_z) / 262144.0;
			min_z_pu_idx = fmax(min_z_pu_idx, tilt_cutoff_pu_idx);
		}
		else {
			// Find new upper bound for z_pu
			double tilt_cutoff_pu_idx = (tilt_cutoff_z - platform_min_z) / 262144.0;
			max_z_pu_idx = fmin(max_z_pu_idx, tilt_cutoff_pu_idx);
		}
	}

	if (pu_platform_max_x > fmin(x2_min, x2_max) && pu_platform_min_x < fmax(x2_min, x2_max)) {
		double z2_min = m * x2_min + c_min;
		double z2_max = m * x2_max + c_max;
		double tilt_cutoff_z = (z2_max - z2_min)*(x - x2_min) / (x2_max - x2_min) + z2_min;

		if (x2_min > 0) {
			// Find new upper bound for z_pu
			double tilt_cutoff_pu_idx = (tilt_cutoff_z - platform_min_z) / 262144.0;
			max_z_pu_idx = fmin(max_z_pu_idx, tilt_cutoff_pu_idx);
		}
		else {
			// Find new lower bound for z_pu
			double tilt_cutoff_pu_idx = (tilt_cutoff_z - platform_max_z) / 262144.0;
			min_z_pu_idx = fmax(min_z_pu_idx, tilt_cutoff_pu_idx);
		}
	}

	min_z_pu_idx = 4.0*ceil(min_z_pu_idx);
	max_z_pu_idx = 4.0*floor(max_z_pu_idx);

	double min_z_pu = 65536.0 * min_z_pu_idx;
	double max_z_pu = 65536.0 * max_z_pu_idx;

	double closest_z_pu_platform;

	if (min_z_pu < 0) {
		if (max_z_pu < 0) {
			closest_z_pu_platform = max_z_pu + platform_max_z;
		}
		else {
			if (abs(min_z_pu) < abs(max_z_pu)) {
				closest_z_pu_platform = min_z_pu + platform_max_z;
			}
			else {
				closest_z_pu_platform = max_z_pu + platform_min_z;
			}
		}
	}
	else {
		closest_z_pu_platform = min_z_pu + platform_min_z;
	}

	// Find the minimum speed to reach a valid PU from current x position.
	// If this exceeds our maximum allowed speed, then we can stop searching polygon
	// in this direction.
	double min_needed_speed = sqrt((pu_platform_max_x*pu_platform_max_x) + (closest_z_pu_platform*closest_z_pu_platform)) / ny;

	if (min_needed_speed > max_speed) {
		return false;
	}

	double T_diff00 = T_tilt[0][0] - T_start[0][0];
	double T_diff20 = T_tilt[2][0] - T_start[2][0];
	double T_diff02 = T_tilt[0][2] - T_start[0][2];
	double T_diff22 = T_tilt[2][2] - T_start[2][2];

	// Tolerance for picking PUs that may result 
	// in out of bounds displacements.
	//
	// If we're more than the dimensions of the platform 
	// away from being in-bounds then we probably can't
	// get an in-bounds displacement anyway.
	double disp_leeway = abs(platform_min_x - platform_max_x) + abs(platform_min_z - platform_max_z);

	// Search backwards from z=0
	for (double z = fmin(0, max_z_pu); z + 8192 > min_z_pu; z -= 262144.0) {
		double base_platform_displacement_x = x * T_diff00 + z * T_diff20;
		double base_platform_displacement_z = x * T_diff02 + z * T_diff22;

		double bpd_x_mod = static_cast<int16_t>(static_cast<int>(base_platform_displacement_x));
		double bpd_z_mod = static_cast<int16_t>(static_cast<int>(base_platform_displacement_z));

		// Check if our likely horizontal platform displacement puts us out of bounds.
		// If so, skip checking this PU.
		if (abs(bpd_x_mod) < 8192 + disp_leeway && abs(bpd_z_mod) < 8192 + disp_leeway) {
			if (!try_pu_xz(plat, x, z, nx, ny, nz, platform_x, platform_z, tilt_idx)) {
				break;
			}
		}
	}

	// Search forwards from z>0
	for (double z = fmax(262144.0, min_z_pu); z - 8192 < max_z_pu; z += 262144.0) {
		double base_platform_displacement_x = x + x * T_diff00 + z * T_diff20;
		double base_platform_displacement_z = z + x * T_diff02 + z * T_diff22;

		double bpd_x_mod = static_cast<int16_t>(static_cast<int>(base_platform_displacement_x));
		double bpd_z_mod = static_cast<int16_t>(static_cast<int>(base_platform_displacement_z));

		// Check if our likely horizontal platform displacement puts us out of bounds.
		// If so, skip checking this PU.
		if (abs(bpd_x_mod) < 8192 + disp_leeway && abs(bpd_z_mod) < 8192 + disp_leeway) {
			if (!try_pu_xz(plat, x, z, nx, ny, nz, platform_x, platform_z, tilt_idx)) {
				break;
			}
		}
	}

	return true;
}

bool try_pu_z(Platform* plat, Mat4 T_start, Mat4 T_tilt, double z, double z1_min, double z1_max, double z2_min, double z2_max, double platform_min_x, double platform_max_x, double platform_min_z, double platform_max_z, double m, double c_min, double c_max, double nx, double ny, double nz, double* platform_x, double* platform_z, int tilt_idx) {
	double pu_platform_min_z = z + platform_min_z;
	double pu_platform_max_z = z + platform_max_z;

	// Find maximal range of PUs along x axis from current z PU position
	double min_x_pu_idx = ((pu_platform_min_z - c_min) / m) / 262144.0;
	double max_x_pu_idx = ((pu_platform_max_z - c_max) / m) / 262144.0;

	if (min_x_pu_idx > max_x_pu_idx) {
		double temp = min_x_pu_idx;
		min_x_pu_idx = max_x_pu_idx;
		max_x_pu_idx = temp;
	}

	// Check max_x_pu and min_x_pu are in range for valid platform tilt.
	// Correct them if they're not.
	//
	// Possible for only part of the platform to be in range.
	// In this case just skip it to avoid headaches later on.

	if (pu_platform_max_z > fmin(z1_min, z1_max) && pu_platform_min_z < fmax(z1_min, z1_max)) {
		double x1_min = (z1_min - c_min) / m;
		double x1_max = (z1_max - c_max) / m;
		double tilt_cutoff_x = (x1_max - x1_min) * (z - z1_min) / (z1_max - z1_min) + x1_min;

		if (z1_min > 0) {
			// Find new upper bound for z_pu
			double tilt_cutoff_pu_idx = (tilt_cutoff_x - platform_min_x) / 262144.0;
			max_x_pu_idx = fmin(max_x_pu_idx, tilt_cutoff_pu_idx);
		}
		else {
			// Find new lower bound for z_pu
			double tilt_cutoff_pu_idx = (tilt_cutoff_x - platform_max_x) / 262144.0;
			min_x_pu_idx = fmax(min_x_pu_idx, tilt_cutoff_pu_idx);
		}
	}

	if (pu_platform_max_z > fmin(z2_min, z2_max) && pu_platform_min_z < fmax(z2_min, z2_max)) {
		double x2_min = (z2_min - c_min) / m;
		double x2_max = (z2_max - c_max) / m;
		double tilt_cutoff_x = (x2_max - x2_min) * (z - z2_min) / (z2_max - z2_min) + x2_min;

		if (z2_min > 0) {
			// Find new lower bound for z_pu
			double tilt_cutoff_pu_idx = (tilt_cutoff_x - platform_max_x) / 262144.0;
			min_x_pu_idx = fmax(min_x_pu_idx, tilt_cutoff_pu_idx);
		}
		else {
			// Find new upper bound for z_pu
			double tilt_cutoff_pu_idx = (tilt_cutoff_x - platform_min_x) / 262144.0;
			max_x_pu_idx = fmin(max_x_pu_idx, tilt_cutoff_pu_idx);
		}
	}

	min_x_pu_idx = 4.0*ceil(min_x_pu_idx);
	max_x_pu_idx = 4.0*floor(max_x_pu_idx);

	double min_x_pu = 65536.0 * min_x_pu_idx;
	double max_x_pu = 65536.0 * max_x_pu_idx;

	double closest_x_pu_platform;

	if (min_x_pu < 0) {
		if (max_x_pu < 0) {
			closest_x_pu_platform = max_x_pu + platform_max_x;
		}
		else {
			if (abs(min_x_pu) < abs(max_x_pu)) {
				closest_x_pu_platform = min_x_pu + platform_max_x;
			}
			else {
				closest_x_pu_platform = max_x_pu + platform_min_x;
			}
		}
	}
	else {
		closest_x_pu_platform = min_x_pu + platform_min_x;
	}

	// Find the minimum speed to reach a valid PU from current z position.
	// If this exceeds our maximum allowed speed, then we can stop searching
	// the polygon in this direction.
	double min_needed_speed = sqrt((pu_platform_max_z*pu_platform_max_z) + (closest_x_pu_platform*closest_x_pu_platform)) / ny;

	if (min_needed_speed > max_speed) {
		return false;
	}

	double T_diff00 = T_tilt[0][0] - T_start[0][0];
	double T_diff20 = T_tilt[2][0] - T_start[2][0];
	double T_diff02 = T_tilt[0][2] - T_start[0][2];
	double T_diff22 = T_tilt[2][2] - T_start[2][2];

	// Tolerance for picking PUs that may result 
	// in out of bounds displacements.
	//
	// If we're more than the dimensions of the platform 
	// away from being in-bounds then we probably can't
	// get an in-bounds displacement anyway.
	double disp_leeway = abs(platform_min_x - platform_max_x) + abs(platform_min_z - platform_max_z);

	// Search backwards from x=0
	for (double x = fmin(0, max_x_pu); x + 8192 > min_x_pu; x -= 262144.0) {
		double base_platform_displacement_x = x * T_diff00 + z * T_diff20;
		double base_platform_displacement_z = x * T_diff02 + z * T_diff22;

		double bpd_x_mod = static_cast<int16_t>(static_cast<int>(base_platform_displacement_x));
		double bpd_z_mod = static_cast<int16_t>(static_cast<int>(base_platform_displacement_z));

		// Check if our likely horizontal platform displacement puts us out of bounds.
		// If so, skip checking this PU.
		if (abs(bpd_x_mod) < 8192 + disp_leeway && abs(bpd_z_mod) < 8192 + disp_leeway) {
			if (!try_pu_xz(plat, x, z, nx, ny, nz, platform_x, platform_z, tilt_idx)) {
				break;
			}
		}
	}

	// Search forwards from x>0
	for (double x = fmax(262144.0, min_x_pu); x - 8192 < max_x_pu; x += 262144.0) {
		double base_platform_displacement_x = x * T_diff00 + z * T_diff20;
		double base_platform_displacement_z = x * T_diff02 + z * T_diff22;

		double bpd_x_mod = static_cast<int16_t>(static_cast<int>(base_platform_displacement_x));
		double bpd_z_mod = static_cast<int16_t>(static_cast<int>(base_platform_displacement_z));

		// Check if our likely horizontal platform displacement puts us out of bounds.
		// If so, skip checking this PU.
		if (abs(bpd_x_mod) < 8192 + disp_leeway && abs(bpd_z_mod) < 8192 + disp_leeway) {
			if (!try_pu_xz(plat, x, z, nx, ny, nz, platform_x, platform_z, tilt_idx)) {
				break;
			}
		}
	}

	return true;
}

void search_normals() {
	Mario mario;
	Platform plat;
	Vec2S tri = plat.triangles;

	double nx; double nz;
	while (true) {
			nx = 2.0f * (static_cast <float> (rand()) / static_cast <float> (RAND_MAX)) - 1.0f;
			nz = (powf(nx, 2) - 1.0f)*(static_cast <float> (rand()) / static_cast <float> (RAND_MAX));

			double ny = sqrtf(1 - powf((float)nx, 2) - powf((float)nz, 2));

			// Tilt angle cut-offs
			// These are the yaw boundaries where the platform tilt 
			// switches direction. Directions match normal_offsets:
			// Between a[0] and a[1]: +x +z
			// Between a[1] and a[2]: -x +z
			// Between a[2] and a[3]: -x -z
			// Between a[3] and a[0]: +x -z
			double a[4];
			a[0] = atan2(nz, sqrt(1 - nz * nz));
			a[1] = atan2(sqrt(1 - nx * nx), nx);
			a[2] = M_PI - a[0];
			a[3] = 2 * M_PI - a[1];

			plat.normal[0] = 0;
			plat.normal[1] = 1;
			plat.normal[2] = 0;

			plat.create_transform_from_normals();

			Mat4 old_mat = plat.transform;

			plat.normal[0] = (float)nx;
			plat.normal[1] = (float)ny; 
			plat.normal[2] = (float)nz;

			plat.create_transform_from_normals();

			plat.triangles[0].rotate(plat.pos, old_mat, plat.transform);
			plat.triangles[1].rotate(plat.pos, old_mat, plat.transform);

			double platform_x[4] = { plat.triangles[0].vector1[0], plat.triangles[0].vector2[0], plat.triangles[0].vector3[0], plat.triangles[1].vector3[0] };
			double platform_y[4] = { plat.triangles[0].vector1[1], plat.triangles[0].vector2[1], plat.triangles[0].vector3[1], plat.triangles[1].vector3[1] };
			double platform_z[4] = { plat.triangles[0].vector1[2], plat.triangles[0].vector2[2], plat.triangles[0].vector3[2], plat.triangles[1].vector3[2] };

			double platform_min_x = fmin(fmin(platform_x[0], platform_x[1]), fmin(platform_x[2], platform_x[3]));
			double platform_max_x = fmax(fmax(platform_x[0], platform_x[1]), fmax(platform_x[2], platform_x[3]));
			double platform_min_z = fmin(fmin(platform_z[0], platform_z[1]), fmin(platform_z[2], platform_z[3]));
			double platform_max_z = fmax(fmax(platform_z[0], platform_z[1]), fmax(platform_z[2], platform_z[3]));

			double min_y = fmin(lava_y, fmin(fmin(platform_y[0], platform_y[1]), fmin(platform_y[2], platform_y[3]))) - plat.pos[1];
			double max_y = fmax(fmax(platform_y[0], platform_y[1]), fmax(platform_y[2], platform_y[3])) - plat.pos[1];

			Mat4 T_start = plat.transform;
			
			// Try to find solutions for each possible platform tilt direction
			for (int i = 0; i < 4; i++) {
				plat.normal[0] += normal_offsets[i][0];
				plat.normal[1] += normal_offsets[i][1];
				plat.normal[2] += normal_offsets[i][2];
				plat.create_transform_from_normals();

				Mat4 T_tilt = plat.transform;
				plat.normal = { (float)nx, (float)ny, (float)nz };

				double T_diff01 = T_tilt[0][1] - T_start[0][1];
				double T_diff21 = T_tilt[2][1] - T_start[2][1];
				double r_min = lower_y - 0.99*max_y - plat.pos[1];
				double r_max = upper_y - 0.99*min_y - plat.pos[1];

				// z = mx + c_min
				// z = mx + c_max
				//
				// PU platforms occurring between these lines will (usually) 
				// give a y displacement within our desired range.
				double m = -T_diff01 / T_diff21;
				double c_min; double c_max;

				if (T_diff21 < 0) {
					c_min = r_max / T_diff21;
					c_max = r_min / T_diff21;
				}
				else {
					c_min = r_min / T_diff21;
					c_max = r_max / T_diff21;
				}

				// Find intersection between y displacement lines and 
				// good platform tilt angle ranges.
				//
				// Intersection forms a polygon that may (or may not)
				// stretch to infinity in one direction.
				// 
				// Find the x coordinates where displacement lines and 
				// platform tilt lines intersect.
				//
				// Non-intersecting lines have x coordinate set to NaN. 
				double a1_cos = cos(a[i]);
				double a2_cos = cos(a[(i + 1) % 4]);

				double x1_min; double x1_max; double x2_min; double x2_max;

				if (nx == 0) {
					if (i % 2 == 0) {
						x1_min = (c_min + tan(a[i])*plat.pos[0] - plat.pos[2]) / (tan(a[i]) - m);
						x1_max = (c_max + tan(a[i])*plat.pos[0] - plat.pos[2]) / (tan(a[i]) - m);
						x2_min = 0;
						x2_max = 0;

						if (a1_cos > 0 && x1_min < plat.pos[0] || a1_cos < 0 && x1_min > plat.pos[0]) {
							x1_min = NAN;
						}

						if (a1_cos > 0 && x1_max < plat.pos[0] || a1_cos < 0 && x1_max > plat.pos[0]) {
							x1_max = NAN;
						}

						if (nz > 0 && c_min < plat.pos[0] || nz < 0 && c_min > plat.pos[0]) {
							x2_min = NAN;
						}

						if (nz > 0 && c_max < plat.pos[0] || nz < 0 && c_max > plat.pos[0]) {
							x2_max = NAN;
						}
					}
					else {
						x1_min = 0;
						x1_max = 0;
						x2_min = (c_min + tan(a[(i + 1) % 4])*plat.pos[0] - plat.pos[2]) / (tan(a[(i + 1) % 4]) - m);
						x2_max = (c_max + tan(a[(i + 1) % 4])*plat.pos[0] - plat.pos[2]) / (tan(a[(i + 1) % 4]) - m);

						if (nz > 0 && c_min < plat.pos[0] || nz < 0 && c_min > plat.pos[0]) {
							x1_min = NAN;
						}

						if (nz > 0 && c_max < plat.pos[0] || nz < 0 && c_max > plat.pos[0]) {
							x1_max = NAN;
						}

						if (a2_cos > 0 && x2_min < plat.pos[0] || a2_cos < 0 && x2_min > plat.pos[0]) {
							x2_min = NAN;
						}

						if (a2_cos > 0 && x2_max < plat.pos[0] || a2_cos < 0 && x2_max > plat.pos[0]) {
							x2_max = NAN;
						}
					}
				}
				else {
					x1_min = (c_min + tan(a[i])*plat.pos[0] - plat.pos[2]) / (tan(a[i]) - m);
					x1_max = (c_max + tan(a[i])*plat.pos[0] - plat.pos[2]) / (tan(a[i]) - m);
					x2_min = (c_min + tan(a[(i + 1) % 4])*plat.pos[0] - plat.pos[2]) / (tan(a[(i + 1) % 4]) - m);
					x2_max = (c_max + tan(a[(i + 1) % 4])*plat.pos[0] - plat.pos[2]) / (tan(a[(i + 1) % 4]) - m);

					if (a1_cos > 0 && x1_min < plat.pos[0] || a1_cos < 0 && x1_min > plat.pos[0]) {
						x1_min = NAN;
					}

					if (a1_cos > 0 && x1_max < plat.pos[0] || a1_cos < 0 && x1_max > plat.pos[0]) {
						x1_max = NAN;
					}

					if (a2_cos > 0 && x2_min < plat.pos[0] || a2_cos < 0 && x2_min > plat.pos[0]) {
						x2_min = NAN;
					}

					if (a2_cos > 0 && x2_max < plat.pos[0] || a2_cos < 0 && x2_max > plat.pos[0]) {
						x2_max = NAN;
					}
				}

				// Start searching for PUs in the polygon.
				//
				// We want to minimise speed, so we search outwards
				// from the point closest to the real platform.
				//
				// This will be at the x = 0 (if abs(m) < 1)
				// or z = 0 (if abs(m) > 1)
				if (abs(m) < 1) {
					// Find x limits of polygon
					double poly_x_start; double poly_x_end;

					if (!isnan(x1_min) && !isnan(x1_max)) {
						if (!isnan(x2_min) && !isnan(x2_max)) {
							poly_x_start = fmin(fmin(x1_min, x1_max), fmin(x2_min, x2_max));
							poly_x_end = fmax(fmax(x1_min, x1_max), fmax(x2_min, x2_max));
						}
						else {
							if (c_min > 0) {
								poly_x_start = -INFINITY;
								poly_x_end = fmax(x1_min, x1_max);
							}
							else {
								poly_x_start = fmin(x1_min, x1_max);
								poly_x_end = INFINITY;
							}
						}
					}
					else if (!isnan(x2_min) && !isnan(x2_max)) {
						if (c_min > 0) {
							poly_x_start = fmin(x2_min, x2_max);
							poly_x_end = INFINITY;
						}
						else {
							poly_x_start = -INFINITY;
							poly_x_end = fmax(x2_min, x2_max);
						}
					}
					else {
						continue;
					}

					double first_x_pu = ceil((poly_x_start - platform_max_x) / 262144.0)*262144.0;
					double last_x_pu = floor((poly_x_end - platform_min_x) / 262144.0)*262144.0;

					// Search backwards from x=0
					for (double x = fmin(0, last_x_pu); x + platform_min_x > poly_x_start; x -= 262144.0) {
						if (!try_pu_x(&plat, T_start, T_tilt, x, x1_min, x1_max, x2_min, x2_max, platform_min_x, platform_max_x, platform_min_z, platform_max_z, m, c_min, c_max, nx, ny, nz, platform_x, platform_z, i)) {
							break;
						}
					}

					// Search forwards from x>0
					for (double x = fmax(262144.0, first_x_pu); x - platform_max_x < poly_x_end; x += 262144.0) {
						if (!try_pu_x(&plat, T_start, T_tilt, x, x1_min, x1_max, x2_min, x2_max, platform_min_x, platform_max_x, platform_min_z, platform_max_z, m, c_min, c_max, nx, ny, nz, platform_x, platform_z, i)) {
							break;
						}
					}
				}
				else {
					// Calculate z coordinates of intersection points
					double z1_min = tan(a[i])*x1_min + plat.pos[2] - tan(a[i])*plat.pos[0];
					double z1_max = tan(a[i])*x1_max + plat.pos[2] - tan(a[i])*plat.pos[0];
					double z2_min = tan(a[(i + 1) % 4])*x2_min + plat.pos[2] - tan(a[(i + 1) % 4])*plat.pos[0];
					double z2_max = tan(a[(i + 1) % 4])*x2_max + plat.pos[2] - tan(a[(i + 1) % 4])*plat.pos[0];

					// Find z limits of polygon
					double poly_z_start; double poly_z_end;

					if (!isnan(z1_min) && !isnan(z1_max)) {
						if (!isnan(z2_min) && !isnan(z2_max)) {
							poly_z_start = fmin(fmin(z1_min, z1_max), fmin(z2_min, z2_max));
							poly_z_end = fmax(fmax(z1_min, z1_max), fmax(z2_min, z2_max));
						}
						else {
							if (c_min / m > 0) {
								poly_z_start = -INFINITY;
								poly_z_end = fmax(z1_min, z1_max);
							}
							else {
								poly_z_start = fmin(z1_min, z1_max);
								poly_z_end = INFINITY;
							}
						}
					}
					else if (!isnan(z2_min) && !isnan(z2_max)) {
						if (c_min / m > 0) {
							poly_z_start = fmin(z2_min, z2_max);
							poly_z_end = INFINITY;
						}
						else {
							poly_z_start = -INFINITY;
							poly_z_end = fmax(z2_min, z2_max);
						}
					}
					else {
						continue;
					}

					double first_z_pu = ceil((poly_z_start - platform_max_z) / 262144.0)*262144.0;
					double last_z_pu = floor((poly_z_end - platform_min_z) / 262144.0)*262144.0;

					// Search backwards from z=0
					for (double z = fmin(0, last_z_pu); z + platform_min_z > poly_z_start; z -= 262144.0) {
						if (!try_pu_z(&plat, T_start, T_tilt, z, z1_min, z1_max, z2_min, z2_max, platform_min_x, platform_max_x, platform_min_z, platform_max_z, m, c_min, c_max, nx, ny, nz, platform_x, platform_z, i)) {
							break;
						}
					}

					// Search forwards from z>0
					for (double z = fmax(262144.0, first_z_pu); z - platform_max_z < poly_z_end; z += 262144.0) {
						if (!try_pu_z(&plat, T_start, T_tilt, z, z1_min, z1_max, z2_min, z2_max, platform_min_x, platform_max_x, platform_min_z, platform_max_z, m, c_min, c_max, nx, ny, nz, platform_x, platform_z, i)) {
							break;
						}
					}
				}
			}

			plat.triangles = tri;
	}
}

int main() {
	search_normals();
}
