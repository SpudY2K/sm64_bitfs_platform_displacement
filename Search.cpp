#include "Constants.hpp"
#include "Mario.hpp"
#include "Magic.hpp"
#include "Search.hpp"
#include "SearchUtils.hpp"
#include "Validate.hpp"
#include <cmath>
#include <fstream>

void try_hau(SearchParams* params, Platform* plat, double hau, double pu_x, double pu_z, double nx, double ny, double nz, int tilt_idx, int tri_idx, int q_steps) {
	double angle = 2.0 * M_PI * (hau / 4096.0);
	double sinA = sin(angle);
	double cosA = cos(angle);

	Platform pre_plat(platform_positions[params->platform_idx][0], platform_positions[params->platform_idx][1], platform_positions[params->platform_idx][2]);

	// Work out the region where mario can finish his movement
	// This is easy, it's either the opposte triangle if we're terminating
	// Mario's movement early, or the whole platform area if we're doing
	// all 4 q-steps.
	std::vector<Vec3s> end_poly;

	if (q_steps == 4) {
		end_poly = { plat->triangles[0].vectors[0], plat->triangles[0].vectors[1], plat->triangles[0].vectors[2], plat->triangles[1].vectors[2] };
	}
	else {
		end_poly = { plat->triangles[1 - tri_idx].vectors[0], plat->triangles[1 - tri_idx].vectors[1], plat->triangles[1 - tri_idx].vectors[2] };
	}

	Mat4 t_diff;

	// Movement takes place over two frames, so platform tilts twice
	// Consider every possible maximum first frame tilt
	for (int i = 0; i < n_pre_tilt_offsets; i++) {
		float initial_nx = plat->normal[0] - pre_tilt_offsets[i][0];
		float initial_ny = plat->normal[1] - pre_tilt_offsets[i][1];
		float initial_nz = plat->normal[2] - pre_tilt_offsets[i][2];

		pre_plat.normal[0] = initial_nx;
		pre_plat.normal[1] = initial_ny;
		pre_plat.normal[2] = initial_nz;
		pre_plat.create_transform_from_normals();

		pre_plat.triangles[0].rotate(pre_plat.transform);
		pre_plat.triangles[1].rotate(pre_plat.transform);

		t_diff[0][0] = plat->transform[0][0] - pre_plat.transform[0][0];
		t_diff[0][1] = plat->transform[0][1] - pre_plat.transform[0][1];
		t_diff[0][2] = plat->transform[0][2] - pre_plat.transform[0][2];
		t_diff[1][0] = plat->transform[1][0] - pre_plat.transform[1][0];
		t_diff[1][1] = plat->transform[1][1] - pre_plat.transform[1][1];
		t_diff[1][2] = plat->transform[1][2] - pre_plat.transform[1][2];
		t_diff[2][0] = plat->transform[2][0] - pre_plat.transform[2][0];
		t_diff[2][1] = plat->transform[2][1] - pre_plat.transform[2][1];
		t_diff[2][2] = plat->transform[2][2] - pre_plat.transform[2][2];

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

		std::vector<Vec3f> start_poly;
		start_poly.push_back({ (float)pre_plat.triangles[tri_idx].vectors[0][0] - pre_plat.pos[0], (float)pre_plat.triangles[tri_idx].vectors[0][1] - pre_plat.pos[1], (float)pre_plat.triangles[tri_idx].vectors[0][2] - pre_plat.pos[2] });
		start_poly.push_back({ (float)pre_plat.triangles[tri_idx].vectors[1][0] - pre_plat.pos[0], (float)pre_plat.triangles[tri_idx].vectors[1][1] - pre_plat.pos[1], (float)pre_plat.triangles[tri_idx].vectors[1][2] - pre_plat.pos[2] });
		start_poly.push_back({ (float)pre_plat.triangles[tri_idx].vectors[2][0] - pre_plat.pos[0], (float)pre_plat.triangles[tri_idx].vectors[2][1] - pre_plat.pos[1], (float)pre_plat.triangles[tri_idx].vectors[2][2] - pre_plat.pos[2] });

		get_start_poly(start_poly, pre_plat.normal, pre_tilt_offsets[i]);

		for (int j = 0; j < start_poly.size(); j++) {
			Vec3f tform_point;
			linear_mtxf_mul_vec3f(tform_point, t_diff, start_poly[j]);
			start_poly[j][0] += (tform_point[0] + pre_plat.pos[0]);
			start_poly[j][1] += (tform_point[1] + pre_plat.pos[1]);
			start_poly[j][2] += (tform_point[2] + pre_plat.pos[2]);
		}

		// Find the maximum and minimum possible speed that allow you 
		// to land on the PU platform with the current yaw.
		// 
		// Will be to/from one of the platform corners, so test all of them.
		//
		// For each corner, find the closest and furthest points on other
		// platform that can be reached with current yaw.
		//
		// Then calculate required speed from distance.
		for (int j = 0; j < start_poly.size(); j++) {
			for (int k = 0; k < end_poly.size(); k++) {
				double start_x = start_poly[j][0];
				double start_z = start_poly[j][2];

				double t_n = sinA * ((end_poly[k][2] + pu_z) - start_poly[j][2]) - cosA * ((end_poly[k][0] + pu_x) - start_poly[j][0]);
				double t_d = cosA * (end_poly[(k + 1) % end_poly.size()][0] - end_poly[k][0]) - sinA * (end_poly[(k + 1) % end_poly.size()][2] - end_poly[k][2]);

				if (t_d < 0) {
					t_n = -t_n;
					t_d = -t_d;
				}

				if (t_n >= 0 && t_n <= t_d) {
					double end_x = end_poly[k][0] + pu_x + (end_poly[(k + 1) % end_poly.size()][0] - end_poly[k][0])*(t_n / t_d);
					double end_z = end_poly[k][2] + pu_z + (end_poly[(k + 1) % end_poly.size()][2] - end_poly[k][2])*(t_n / t_d);

					double x_dist = (end_x - start_x) * (4.0 / (double)q_steps);
					double z_dist = (end_z - start_z) * (4.0 / (double)q_steps);

					double speed = sqrt(x_dist * x_dist + z_dist * z_dist) / plat->triangles[tri_idx].normal[1];

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

		for (int j = 0; j < end_poly.size(); j++) {
			for (int k = 0; k < start_poly.size(); k++) {
				double end_x = end_poly[j][0] + pu_x;
				double end_z = end_poly[j][2] + pu_z;

				double t_n = sinA * (start_poly[k][2] - (end_poly[j][2] + pu_z)) - cosA * (start_poly[k][0] - (end_poly[j][0] + pu_x));
				double t_d = cosA * (start_poly[(k + 1) % start_poly.size()][0] - start_poly[k][0]) - sinA * (start_poly[(k + 1) % start_poly.size()][2] - start_poly[k][2]);

				if (t_d < 0) {
					t_n = -t_n;
					t_d = -t_d;
				}

				if (t_n >= 0 && t_n <= t_d) {
					double start_x = start_poly[k][0] + (start_poly[(k + 1) % start_poly.size()][0] - start_poly[k][0])*(t_n / t_d);
					double start_z = start_poly[k][2] + (start_poly[(k + 1) % start_poly.size()][2] - start_poly[k][2])*(t_n / t_d);

					double x_dist = (end_x - start_x) * (4.0 / (double)q_steps);
					double z_dist = (end_z - start_z) * (4.0 / (double)q_steps);

					double speed = sqrt(x_dist * x_dist + z_dist * z_dist) / plat->triangles[tri_idx].normal[1];

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

			// We don't want this to be on the edge of the triangle. Move five units towards triangle's centre
			float centre_x = (float)(plat->triangles[tri_idx].vectors[0][0] + plat->triangles[tri_idx].vectors[1][0] + plat->triangles[tri_idx].vectors[2][0]) / 3.0f;
			float centre_z = (float)(plat->triangles[tri_idx].vectors[0][2] + plat->triangles[tri_idx].vectors[1][2] + plat->triangles[tri_idx].vectors[2][2]) / 3.0f;

			float start_centre_dist_x = centre_x - start_x;
			float start_centre_dist_z = centre_z - start_z;
			float start_centre_dist = sqrt(start_centre_dist_x * start_centre_dist_x + start_centre_dist_z * start_centre_dist_z);

			start_x = start_x + 5.0f*(start_centre_dist_x / start_centre_dist);
			start_z = start_z + 5.0f*(start_centre_dist_z / start_centre_dist);

			start_x = start_centre_dist_x > 0 ? ceil(start_x) : floor(start_x);
			start_z = start_centre_dist_z > 0 ? ceil(start_z) : floor(start_z);

			// Find the height of the floor at a given location.
			float oo = plat->triangles[tri_idx].originOffset;
			float start_y = -(start_x * plat->triangles[tri_idx].normal[0] + plat->triangles[tri_idx].normal[2] * start_z + oo) / plat->triangles[tri_idx].normal[1];

			bool searching = true;

			// Search for a position on the real platform and a speed that puts us on the PU platform
			float upper_speed = max_speed_f;
			float lower_speed = min_speed_f;

			while (searching) {
				float speed = (upper_speed + lower_speed) / 2.0f;
				float end_x = start_x;
				float end_z = start_z;

				// Calculate our final position with quarter steps
				float x_vel = speed * gSineTable[(int)hau];
				float z_vel = speed * gCosineTable[(int)hau];

				for (int j = 0; j < q_steps - 1; j++) {
					end_x = end_x + plat->triangles[tri_idx].normal[1] * (x_vel / 4.0f);
					end_z = end_z + plat->triangles[tri_idx].normal[1] * (z_vel / 4.0f);
				}

				bool tri_test = on_triangle(plat->triangles[tri_idx], end_x, start_y, end_z);

				if (tri_test) {
					end_x = end_x + plat->triangles[tri_idx].normal[1] * (x_vel / 4.0f);
					end_z = end_z + plat->triangles[tri_idx].normal[1] * (z_vel / 4.0f);

					bool tri_test1 = on_triangle(plat->triangles[tri_idx], end_x, start_y, end_z);
					bool tri_test2 = on_triangle(plat->triangles[1 - tri_idx], end_x, start_y, end_z);

					tri_test = (q_steps == 4 && tri_test1) || tri_test2;
				}

				// Check if Mario is on the PU platform. 
				if (tri_test) {
					double coeff1a[4] = { start_x + ((double)plat->pos[0])*t_diff[0][0] + ((double)plat->pos[1])*t_diff[1][0] + ((double)plat->pos[2])*t_diff[2][0], 1 + t_diff[0][0], t_diff[1][0], t_diff[2][0] };
					double coeff1b[4] = { start_y + ((double)plat->pos[0])*t_diff[0][1] + ((double)plat->pos[1])*t_diff[1][1] + ((double)plat->pos[2])*t_diff[2][1], t_diff[0][1], 1 + t_diff[1][1], t_diff[2][1] };
					double coeff1c[4] = { start_z + ((double)plat->pos[0])*t_diff[0][2] + ((double)plat->pos[1])*t_diff[1][2] + ((double)plat->pos[2])*t_diff[2][2], t_diff[0][2], t_diff[1][2], 1 + t_diff[2][2] };

					gaussian_elimination(coeff1a, coeff1b, coeff1c);

					float original_x = (float)coeff1a[0];
					float original_y = (float)coeff1b[0];
					float original_z = (float)coeff1c[0];

					Vec3f dist;

					dist[0] = original_x - (float)plat->pos[0];
					dist[1] = original_y - (float)plat->pos[1];
					dist[2] = original_z - (float)plat->pos[2];

					float dx = original_x - (float)plat->pos[0];
					float dy = 500.0f;
					float dz = original_z - (float)plat->pos[2];
					float d = sqrtf(dx * dx + dy * dy + dz * dz);

					d = 1.0f / d;
					dx *= d;
					dy *= d;
					dz *= d;

					float next_nx = approach_by_increment(dx, initial_nx, 0.01f);
					float next_ny = approach_by_increment(dy, initial_ny, 0.01f);
					float next_nz = approach_by_increment(dz, initial_nz, 0.01f);

					// Check the new starting position is actually on the original platform
					// It's technically possible for this to be on a PU version of the platform
					// which would fool the validator, so check for this here
					bool tilt_test = fabs(next_nx - nx) < 0.001f && fabs(next_ny - ny) < 0.001f && fabs(next_nz - nz) < 0.001f;
					bool universe_test = fabs(original_x) <= 32768.0f && fabs(original_y) <= 32768.0f && fabs(original_z) <= 32768.0f;
					bool start_tri_test = on_triangle(pre_plat.triangles[tri_idx], original_x, original_y, original_z);

					if (tilt_test && universe_test && start_tri_test) {
						// Rotate platform to second frame position
						// We need this to find the y normal for the adjusted speed calculation
						pre_plat.triangles[0].rotate(pre_plat.transform);
						pre_plat.triangles[1].rotate(pre_plat.transform);

						pre_plat.normal = { next_nx, next_ny, next_nz };
						pre_plat.create_transform_from_normals();
						pre_plat.triangles[0].rotate(pre_plat.transform);
						pre_plat.triangles[1].rotate(pre_plat.transform);

						// Adjust the estimated speed. 
						// Triangle normal will have changed, so we need to adjust our speed to account for this.
						// This triangle normal will probably be different if this is used in game,
						// in which case this would need to be adjusted again.
						float adj_speed = (float)((double)speed * ((double)plat->triangles[tri_idx].normal[1] / (double)pre_plat.triangles[tri_idx].normal[1]));

						// Validate the final parameter set.
						// Can fail for a number of reasons, see original
						// validate_solution for more details
						Solution s(params, adj_speed, (int)(16.0 * hau), original_x, original_y, original_z, (float)initial_nx, (float)initial_ny, (float)initial_nz);
						Vec3f final_pos;

						if (validate_solution(&s, params, final_pos, false)) {
							bool good_solution = false;

							if (params->filter_floor_ranges) {
								for (int f = 0; f < n_floor_ranges; f++) {
									if (final_pos[1] >= lower_floor[f] && final_pos[1] <= upper_floor[f]) {
										good_solution = true;
										break;
									}
								}

								if (!good_solution && params->check_fall_through_pus) {
									good_solution = check_fall_through_pus(&s, final_pos);
								}
							}
							else {
								good_solution = true;
							}
							
							if (good_solution) {
								write_to_stream(&(params->out_stream), &s, tri_idx, pre_plat.triangles[tri_idx].normal[1], final_pos);

								//printf("Solution found:\nSpeed: %f\nDe Facto Speed: %f\nYaw: %d\nPlatform normals: (%.9f, %.9f, %.9f)\nMario start: (%.9f, %.9f, %.9f)\nMario end: (%.9f, %.9f, %.9f)\n",
								//	adj_speed, speed*plat->triangles[tri_idx].normal[1], (int)(16.0*hau), v_normal[0], v_normal[1], v_normal[2], original_x, original_y, original_z, v_mario.pos[0], v_mario.pos[1], v_mario.pos[2]);
							}
						}
					}

					searching = false;
				}
				else {
					// If we missed the PU platform, adjust our speed and try again.
					// Work out if we over or undershot the platform, and adjust speed accordingly.
					bool overshot = false;

					for (int b = 0; b < 3; b++) {
						double pu_platform_start_x = pu_x + plat->triangles[1 - tri_idx].vectors[b][0];
						double pu_platform_start_z = pu_z + plat->triangles[1 - tri_idx].vectors[b][2];
						double pu_platform_end_x = pu_x + plat->triangles[1 - tri_idx].vectors[(b + 1) % 3][0];
						double pu_platform_end_z = pu_z + plat->triangles[1 - tri_idx].vectors[(b + 1) % 3][2];

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
						float min_start_dist = 0.01;

						if (abs(start_x - max_speed_start_x) < min_start_dist && abs(start_z - max_speed_start_z) < min_start_dist) {
							// Search couldn't find a suitable place on the platform to do the desired displacement.
							// Maybe no solutions exist here.
							// Though, perhaps this can be resolved with a more comprehensive search of the platform.

							//printf("Something has gone wrong here:\nSpeed: %f\nHau: %d\nPlatform normals: (%.9f, %.9f, %.9f)\nMario pos: (%.9f, %.9f, %.9f)\nMario start: (%.9f, %.9f, %.9f)\n",
							//	speed, (int)hau, (float)nx, (float)ny, (float)nz, end_x, start_y, end_z, start_x, start_y, start_z);
							searching = false;
						}
						else {
							// If all speeds fail to land us on the platform, try a different starting position.
							if (abs(start_x - max_speed_start_x) >= min_start_dist) {
								float dir_x = (float)(max_speed_start_x - start_x)*INFINITY;
								start_x = (float)((nextafterf(start_x, dir_x) + max_speed_start_x) / 2.0f);
							}

							if (abs(start_z - max_speed_start_z) >= min_start_dist) {
								float dir_z = (float)(max_speed_start_z - start_z)*INFINITY;
								start_z = (float)((nextafterf(start_z, dir_z) + max_speed_start_z) / 2.0f);
							}

							float oo = plat->triangles[tri_idx].originOffset;
							start_y = -(start_x * plat->triangles[tri_idx].normal[0] + plat->triangles[tri_idx].normal[2] * start_z + oo) / plat->triangles[tri_idx].normal[1];

							upper_speed = max_speed_f;
							lower_speed = min_speed_f;
						}
					}
				}
			}
		}
	}
}

bool try_pu_xz(SearchParams* params, Platform* plat, double x, double z, double nx, double ny, double nz, int tilt_idx, int q_steps) {
	// For current (x, z) PU position, find range of yaws that
	// allow you to reach the PU platform from the original universe.

	double x_dist1 = plat->triangles[0].vectors[1][0] - plat->triangles[1].vectors[2][0];
	double z_dist1 = plat->triangles[0].vectors[1][2] - plat->triangles[1].vectors[2][2];
	double dist1 = sqrt(x_dist1 * x_dist1 + z_dist1 * z_dist1);

	double x_dist2 = plat->triangles[0].vectors[0][0] - plat->triangles[0].vectors[2][0];
	double z_dist2 = plat->triangles[0].vectors[0][2] - plat->triangles[0].vectors[2][2];
	double dist2 = sqrt(x_dist2 * x_dist2 + z_dist2 * z_dist2);

	double min_dist = sqrt(x * x + z * z) - fmax(dist1, dist2);

	// Check that the minimum speed needed to reach the PU platform does 
	// not exceed our desired maximum 
	double min_needed_speed = (4.0 / (double)q_steps) * min_dist / fmax(plat->triangles[0].normal[1], plat->triangles[1].normal[1]);

	if (min_needed_speed > params->max_speed) {
		return false;
	}

	// Search for solutions starting on each of the platform's triangles
	for (int i = 0; i < 2; i++) {
		if (min_dist / plat->triangles[i].normal[1] <= params->max_speed) {
			double min_hau = 4096;
			double max_hau = -1;
			
			if (q_steps == 4) {
				for (int a = 0; a < 3; a++) {
					for (int b = 0; b < 3; b++) {
						double x_dist = x + plat->triangles[1 - i].vectors[b][0] - plat->triangles[i].vectors[a][0];
						double z_dist = z + plat->triangles[1 - i].vectors[b][2] - plat->triangles[i].vectors[a][2];

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
					}

					for (int b = 0; b < 3; b++) {
						double x_dist = x + plat->triangles[i].vectors[b][0] - plat->triangles[i].vectors[a][0];
						double z_dist = z + plat->triangles[i].vectors[b][2] - plat->triangles[i].vectors[a][2];

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
					}
				}
			}
			else {
				Vec3s* firstVector = &(plat->triangles[i].vectors[0]);
				Vec3s* secondVector = i == 0 ? &(plat->triangles[1 - i].vectors[1]) : &(plat->triangles[1 - i].vectors[2]);

				for (int a = 0; a < 3; a++) {
					double hau; double x_dist; double z_dist;

					x_dist = x + (*firstVector)[0] - plat->triangles[i].vectors[a][0];
					z_dist = z + (*firstVector)[2] - plat->triangles[i].vectors[a][2];

					if (x == 0 && z > 0) {
						//Special case to handle the discontinuity at 0 degrees
						hau = 4096 * (atan2(x_dist, z_dist) / (2.0 * M_PI));
					}
					else {
						hau = 4096 * (fmod(2.0 * M_PI + atan2(x_dist, z_dist), 2.0 * M_PI) / (2.0 * M_PI));
					}

					min_hau = fmin(min_hau, hau);
					max_hau = fmax(max_hau, hau);

					x_dist = x + (*secondVector)[0] - plat->triangles[i].vectors[a][0];
					z_dist = z + (*secondVector)[2] - plat->triangles[i].vectors[a][2];

					if (x == 0 && z > 0) {
						//Special case to handle the discontinuity at 0 degrees
						hau = 4096 * (atan2(x_dist, z_dist) / (2.0 * M_PI));
					}
					else {
						hau = 4096 * (fmod(2.0 * M_PI + atan2(x_dist, z_dist), 2.0 * M_PI) / (2.0 * M_PI));
					}

					min_hau = fmin(min_hau, hau);
					max_hau = fmax(max_hau, hau);
				}
			}

			// Round yaws to nearest hau index 
			min_hau = ceil(min_hau);
			max_hau = floor(max_hau);

			// Try each possible hau index
			for (double hau = min_hau; hau <= max_hau; hau++) {
				try_hau(params, plat, fmod(4096.0 + hau, 4096.0), x, z, nx, ny, nz, tilt_idx, i, q_steps);
			}
		}
	}
	return true;
}

bool try_pu_x(SearchParams* params, Platform* plat, Mat4 T_start, Mat4 T_tilt, double x, double x1_min, double x1_max, double x2_min, double x2_max, double platform_min_x, double platform_max_x, double platform_min_z, double platform_max_z, double m, double c_min, double c_max, double nx, double ny, double nz, int tilt_idx, int q_steps) {
	double pu_platform_min_x = x + platform_min_x;
	double pu_platform_max_x = x + platform_max_x;

	double pu_gap = 65536.0*q_steps;

	// Find maximal range of PUs along z axis from current x PU position
	double min_z_pu_idx = (m * pu_platform_min_x + c_min) / pu_gap;
	double max_z_pu_idx = (m * pu_platform_max_x + c_max) / pu_gap;

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
			double tilt_cutoff_pu_idx = (tilt_cutoff_z - platform_max_z) / pu_gap;
			min_z_pu_idx = fmax(min_z_pu_idx, tilt_cutoff_pu_idx);
		}
		else {
			// Find new upper bound for z_pu
			double tilt_cutoff_pu_idx = (tilt_cutoff_z - platform_min_z) / pu_gap;
			max_z_pu_idx = fmin(max_z_pu_idx, tilt_cutoff_pu_idx);
		}
	}

	if (pu_platform_max_x > fmin(x2_min, x2_max) && pu_platform_min_x < fmax(x2_min, x2_max)) {
		double z2_min = m * x2_min + c_min;
		double z2_max = m * x2_max + c_max;
		double tilt_cutoff_z = (z2_max - z2_min)*(x - x2_min) / (x2_max - x2_min) + z2_min;

		if (x2_min > 0) {
			// Find new upper bound for z_pu
			double tilt_cutoff_pu_idx = (tilt_cutoff_z - platform_min_z) / pu_gap;
			max_z_pu_idx = fmin(max_z_pu_idx, tilt_cutoff_pu_idx);
		}
		else {
			// Find new lower bound for z_pu
			double tilt_cutoff_pu_idx = (tilt_cutoff_z - platform_max_z) / pu_gap;
			min_z_pu_idx = fmax(min_z_pu_idx, tilt_cutoff_pu_idx);
		}
	}

	min_z_pu_idx = q_steps * ceil(min_z_pu_idx);
	max_z_pu_idx = q_steps * floor(max_z_pu_idx);

	double min_z_pu = 65536.0 * min_z_pu_idx;
	double max_z_pu = 65536.0 * max_z_pu_idx;

	double closest_z_pu_platform;

	if (min_z_pu < 0) {
		if (max_z_pu < 0) {
			closest_z_pu_platform = max_z_pu + platform_max_z - platform_min_z;
		}
		else {
			if (abs(min_z_pu) < abs(max_z_pu)) {
				closest_z_pu_platform = min_z_pu + platform_max_z - platform_min_z;
			}
			else {
				closest_z_pu_platform = max_z_pu + platform_min_z - platform_max_z;
			}
		}
	}
	else {
		closest_z_pu_platform = min_z_pu + platform_min_z - platform_max_z;
	}

	// Find the minimum speed to reach a valid PU from current x position.
	// If this exceeds our maximum allowed speed, then we can stop searching polygon
	// in this direction.
	double min_needed_speed = (4.0 / (double)q_steps) * sqrt((x + platform_max_z - platform_min_z)*(x + platform_max_z - platform_min_z) + (closest_z_pu_platform*closest_z_pu_platform)) / fmin(plat->triangles[0].normal[1], plat->triangles[1].normal[1]);

	if (min_needed_speed > params->max_speed) {
		return false;
	}

	double min_pu_oob_z;

	if (q_steps < 4) {
		// If we're terminating Mario's movement early, then we need to be sure that 
		// there's enough of a difference between the y normals of the platform's two 
		// triangles to force Mario into out of bounds

		double closest_oob = 9743.23; // An estimate, based on the platforms pivot

		double min_dist_oob = closest_oob / (fmax(plat->triangles[0].normal[1], plat->triangles[1].normal[1]) / fmin(plat->triangles[0].normal[1], plat->triangles[1].normal[1]) - 1.0);
		double min_dist_oob_z = sqrt(min_dist_oob * min_dist_oob - x * x);

		min_pu_oob_z = ceil(min_dist_oob_z / 262144.0)*pu_gap;
	}
	else {
		min_pu_oob_z = 0.0;
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
	for (double z = fmin(fmin(0, max_z_pu), -min_pu_oob_z); z + 8192 > min_z_pu; z -= pu_gap) {
		double base_platform_displacement_x = x * T_diff00 + z * T_diff20;
		double base_platform_displacement_z = x * T_diff02 + z * T_diff22;

		double bpd_x_mod = static_cast<int16_t>(static_cast<int>(x + plat->pos[0] + base_platform_displacement_x));
		double bpd_z_mod = static_cast<int16_t>(static_cast<int>(z + plat->pos[2] + base_platform_displacement_z));

		// Check if our likely horizontal platform displacement puts us out of bounds.
		// If so, skip checking this PU.
		if (abs(bpd_x_mod) < 8192 + disp_leeway && abs(bpd_z_mod) < 8192 + disp_leeway) {
			if (!try_pu_xz(params, plat, x, z, nx, ny, nz, tilt_idx, q_steps)) {
				break;
			}
		}
	}

	// Search forwards from z>0
	for (double z = fmax(fmax(q_steps*pu_gap, min_z_pu), min_pu_oob_z); z - 8192 < max_z_pu; z += pu_gap) {
		double base_platform_displacement_x = x * T_diff00 + z * T_diff20;
		double base_platform_displacement_z = x * T_diff02 + z * T_diff22;

		double bpd_x_mod = static_cast<int16_t>(static_cast<int>(x + plat->pos[0] + base_platform_displacement_x));
		double bpd_z_mod = static_cast<int16_t>(static_cast<int>(z + plat->pos[2] + base_platform_displacement_z));

		// Check if our likely horizontal platform displacement puts us out of bounds.
		// If so, skip checking this PU.
		if (abs(bpd_x_mod) < 8192 + disp_leeway && abs(bpd_z_mod) < 8192 + disp_leeway) {
			if (!try_pu_xz(params, plat, x, z, nx, ny, nz, tilt_idx, q_steps)) {
				break;
			}
		}
	}

	return true;
}

bool try_pu_z(SearchParams* params, Platform* plat, Mat4 T_start, Mat4 T_tilt, double z, double z1_min, double z1_max, double z2_min, double z2_max, double platform_min_x, double platform_max_x, double platform_min_z, double platform_max_z, double m, double c_min, double c_max, double nx, double ny, double nz, int tilt_idx, int q_steps) {
	double pu_platform_min_z = z + platform_min_z;
	double pu_platform_max_z = z + platform_max_z;

	double pu_gap = 65535.0*q_steps;

	// Find maximal range of PUs along x axis from current z PU position
	double min_x_pu_idx = ((pu_platform_min_z - c_min) / m) / pu_gap;
	double max_x_pu_idx = ((pu_platform_max_z - c_max) / m) / pu_gap;

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
			double tilt_cutoff_pu_idx = (tilt_cutoff_x - platform_min_x) / pu_gap;
			max_x_pu_idx = fmin(max_x_pu_idx, tilt_cutoff_pu_idx);
		}
		else {
			// Find new lower bound for z_pu
			double tilt_cutoff_pu_idx = (tilt_cutoff_x - platform_max_x) / pu_gap;
			min_x_pu_idx = fmax(min_x_pu_idx, tilt_cutoff_pu_idx);
		}
	}

	if (pu_platform_max_z > fmin(z2_min, z2_max) && pu_platform_min_z < fmax(z2_min, z2_max)) {
		double x2_min = (z2_min - c_min) / m;
		double x2_max = (z2_max - c_max) / m;
		double tilt_cutoff_x = (x2_max - x2_min) * (z - z2_min) / (z2_max - z2_min) + x2_min;

		if (z2_min > 0) {
			// Find new lower bound for z_pu
			double tilt_cutoff_pu_idx = (tilt_cutoff_x - platform_max_x) / pu_gap;
			min_x_pu_idx = fmax(min_x_pu_idx, tilt_cutoff_pu_idx);
		}
		else {
			// Find new upper bound for z_pu
			double tilt_cutoff_pu_idx = (tilt_cutoff_x - platform_min_x) / pu_gap;
			max_x_pu_idx = fmin(max_x_pu_idx, tilt_cutoff_pu_idx);
		}
	}

	min_x_pu_idx = q_steps * ceil(min_x_pu_idx);
	max_x_pu_idx = q_steps * floor(max_x_pu_idx);

	double min_x_pu = 65536.0 * min_x_pu_idx;
	double max_x_pu = 65536.0 * max_x_pu_idx;

	double closest_x_pu_platform;

	if (min_x_pu < 0) {
		if (max_x_pu < 0) {
			closest_x_pu_platform = max_x_pu + platform_max_x - platform_min_x;
		}
		else {
			if (abs(min_x_pu) < abs(max_x_pu)) {
				closest_x_pu_platform = min_x_pu + platform_max_x - platform_min_x;
			}
			else {
				closest_x_pu_platform = max_x_pu + platform_min_x - platform_max_x;
			}
		}
	}
	else {
		closest_x_pu_platform = min_x_pu + platform_min_x - platform_max_x;
	}

	// Find the minimum speed to reach a valid PU from current z position.
	// If this exceeds our maximum allowed speed, then we can stop searching
	// the polygon in this direction.
	double min_needed_speed = (4.0 / (double)q_steps) * sqrt((z + platform_max_x - platform_min_x)*(z + platform_max_x - platform_min_x) + (closest_x_pu_platform*closest_x_pu_platform)) / fmin(plat->triangles[0].normal[1], plat->triangles[1].normal[1]);

	if (min_needed_speed > params->max_speed) {
		return false;
	}

	double min_pu_oob_x;

	if (q_steps < 4) {
		// If we're terminating Mario's movement early, then we need to be sure that 
		// there's enough of a difference between the y normals of the platform's two 
		// triangles to force Mario into out of bounds

		double closest_oob = 9743.23; // An estimate, based on the platform's pivot

		double min_dist_oob = closest_oob / (fmax(plat->triangles[0].normal[1], plat->triangles[1].normal[1]) / fmin(plat->triangles[0].normal[1], plat->triangles[1].normal[1]) - 1.0);
		double min_dist_oob_x = sqrt(min_dist_oob * min_dist_oob - z * z);

		min_pu_oob_x = ceil(min_dist_oob_x / 262144.0)*pu_gap;
	}
	else {
		min_pu_oob_x = 0.0;
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
	for (double x = fmin(fmin(0, max_x_pu), -min_pu_oob_x); x + 8192 > min_x_pu; x -= pu_gap) {
		double base_platform_displacement_x = x * T_diff00 + z * T_diff20;
		double base_platform_displacement_z = x * T_diff02 + z * T_diff22;

		double bpd_x_mod = static_cast<int16_t>(static_cast<int>(x + plat->pos[0] + base_platform_displacement_x));
		double bpd_z_mod = static_cast<int16_t>(static_cast<int>(z + plat->pos[2] + base_platform_displacement_z));

		// Check if our likely horizontal platform displacement puts us out of bounds.
		// If so, skip checking this PU.
		if (abs(bpd_x_mod) < 8192 + disp_leeway && abs(bpd_z_mod) < 8192 + disp_leeway) {
			if (!try_pu_xz(params, plat, x, z, nx, ny, nz, tilt_idx, q_steps)) {
				break;
			}
		}
	}

	// Search forwards from x>0
	for (double x = fmax(fmax(pu_gap, min_x_pu), min_pu_oob_x); x - 8192 < max_x_pu; x += pu_gap) {
		double base_platform_displacement_x = x * T_diff00 + z * T_diff20;
		double base_platform_displacement_z = x * T_diff02 + z * T_diff22;

		double bpd_x_mod = static_cast<int16_t>(static_cast<int>(x + plat->pos[0] + base_platform_displacement_x));
		double bpd_z_mod = static_cast<int16_t>(static_cast<int>(z + plat->pos[2] + base_platform_displacement_z));

		// Check if our likely horizontal platform displacement puts us out of bounds.
		// If so, skip checking this PU.
		if (abs(bpd_x_mod) < 8192 + disp_leeway && abs(bpd_z_mod) < 8192 + disp_leeway) {
			if (!try_pu_xz(params, plat, x, z, nx, ny, nz, tilt_idx, q_steps)) {
				break;
			}
		}
	}

	return true;
}

void try_normal(SearchParams* params, Platform* plat, double nx, double ny, double nz) {
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

	plat->normal[0] = 0;
	plat->normal[1] = 1;
	plat->normal[2] = 0;

	plat->create_transform_from_normals();

	plat->normal[0] = (float)nx;
	plat->normal[1] = (float)ny;
	plat->normal[2] = (float)nz;

	plat->create_transform_from_normals();

	plat->triangles[0].rotate(plat->transform);
	plat->triangles[1].rotate(plat->transform);

	double platform_min_x = fmin(fmin(plat->triangles[0].vectors[0][0], plat->triangles[0].vectors[1][0]), fmin(plat->triangles[0].vectors[2][0], plat->triangles[1].vectors[2][0]));
	double platform_max_x = fmax(fmax(plat->triangles[0].vectors[0][0], plat->triangles[0].vectors[1][0]), fmax(plat->triangles[0].vectors[2][0], plat->triangles[1].vectors[2][0]));
	double platform_min_z = fmin(fmin(plat->triangles[0].vectors[0][2], plat->triangles[0].vectors[1][2]), fmin(plat->triangles[0].vectors[2][2], plat->triangles[1].vectors[2][2]));
	double platform_max_z = fmax(fmax(plat->triangles[0].vectors[0][2], plat->triangles[0].vectors[1][2]), fmax(plat->triangles[0].vectors[2][2], plat->triangles[1].vectors[2][2]));

	double min_y = fmin(lava_y, fmin(fmin(plat->triangles[0].vectors[0][1], plat->triangles[0].vectors[1][1]), fmin(plat->triangles[0].vectors[2][1], plat->triangles[1].vectors[2][1])));
	double max_y = fmax(fmax(plat->triangles[0].vectors[0][1], plat->triangles[0].vectors[1][1]), fmax(plat->triangles[0].vectors[2][1], plat->triangles[1].vectors[2][1]));

	Mat4 T_start = plat->transform;

	// Try to find solutions for each possible platform tilt direction
	for (int i = 0; i < n_normal_offsets; i++) {
		plat->normal[0] += normal_offsets[i][0];
		plat->normal[1] += normal_offsets[i][1];
		plat->normal[2] += normal_offsets[i][2];
		plat->create_transform_from_normals();

		Mat4 T_tilt = plat->transform;
		plat->normal = { (float)nx, (float)ny, (float)nz };
		plat->create_transform_from_normals();

		double T_diff01 = T_tilt[0][1] - T_start[0][1];
		double T_diff11 = T_tilt[1][1] - T_start[1][1];
		double T_diff21 = T_tilt[2][1] - T_start[2][1];

		for (int j = 0; j < params->n_y_ranges; j++) {
			double r_min = *(params->lower_y+j) - (1 + T_diff11)*max_y + T_diff01 * plat->pos[0] + T_diff11 * plat->pos[1] + T_diff21 * plat->pos[2];
			double r_max = *(params->upper_y+j) - (1 + T_diff11)*min_y + T_diff01 * plat->pos[0] + T_diff11 * plat->pos[1] + T_diff21 * plat->pos[2];

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
					x1_min = (c_min + tan(a[i])*plat->pos[0] - plat->pos[2]) / (tan(a[i]) - m);
					x1_max = (c_max + tan(a[i])*plat->pos[0] - plat->pos[2]) / (tan(a[i]) - m);
					x2_min = 0;
					x2_max = 0;

					if (a1_cos > 0 && x1_min < plat->pos[0] || a1_cos < 0 && x1_min > plat->pos[0]) {
						x1_min = NAN;
					}

					if (a1_cos > 0 && x1_max < plat->pos[0] || a1_cos < 0 && x1_max > plat->pos[0]) {
						x1_max = NAN;
					}

					if (nz > 0 && c_min < plat->pos[0] || nz < 0 && c_min > plat->pos[0]) {
						x2_min = NAN;
					}

					if (nz > 0 && c_max < plat->pos[0] || nz < 0 && c_max > plat->pos[0]) {
						x2_max = NAN;
					}
				}
				else {
					x1_min = 0;
					x1_max = 0;
					x2_min = (c_min + tan(a[(i + 1) % 4])*plat->pos[0] - plat->pos[2]) / (tan(a[(i + 1) % 4]) - m);
					x2_max = (c_max + tan(a[(i + 1) % 4])*plat->pos[0] - plat->pos[2]) / (tan(a[(i + 1) % 4]) - m);

					if (nz > 0 && c_min < plat->pos[0] || nz < 0 && c_min > plat->pos[0]) {
						x1_min = NAN;
					}

					if (nz > 0 && c_max < plat->pos[0] || nz < 0 && c_max > plat->pos[0]) {
						x1_max = NAN;
					}

					if (a2_cos > 0 && x2_min < plat->pos[0] || a2_cos < 0 && x2_min > plat->pos[0]) {
						x2_min = NAN;
					}

					if (a2_cos > 0 && x2_max < plat->pos[0] || a2_cos < 0 && x2_max > plat->pos[0]) {
						x2_max = NAN;
					}
				}
			}
			else {
				x1_min = (c_min + tan(a[i])*plat->pos[0] - plat->pos[2]) / (tan(a[i]) - m);
				x1_max = (c_max + tan(a[i])*plat->pos[0] - plat->pos[2]) / (tan(a[i]) - m);
				x2_min = (c_min + tan(a[(i + 1) % 4])*plat->pos[0] - plat->pos[2]) / (tan(a[(i + 1) % 4]) - m);
				x2_max = (c_max + tan(a[(i + 1) % 4])*plat->pos[0] - plat->pos[2]) / (tan(a[(i + 1) % 4]) - m);

				if (a1_cos > 0 && x1_min < plat->pos[0] || a1_cos < 0 && x1_min > plat->pos[0]) {
					x1_min = NAN;
				}

				if (a1_cos > 0 && x1_max < plat->pos[0] || a1_cos < 0 && x1_max > plat->pos[0]) {
					x1_max = NAN;
				}

				if (a2_cos > 0 && x2_min < plat->pos[0] || a2_cos < 0 && x2_min > plat->pos[0]) {
					x2_min = NAN;
				}

				if (a2_cos > 0 && x2_max < plat->pos[0] || a2_cos < 0 && x2_max > plat->pos[0]) {
					x2_max = NAN;
				}
			}


			// Mario's movement can end on any of his quarter steps, as long as the next move puts him 
			// out of bounds (or is the last step). So we need to consider PU movement for each possible
			// final quarter step

			// If the normals match then you can't force Mario out of bounds after his final q step.
			// Therefore, only 4 q_steps are possible.
			int min_q_steps = (plat->triangles[0].normal[1] == plat->triangles[1].normal[1]) ? 4 : 1;

			for (int q = 4; q >= min_q_steps; q--) {
				double pu_gap = 65536.0 * q;

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

					double first_x_pu = ceil((poly_x_start - platform_max_x) / pu_gap)*pu_gap;
					double last_x_pu = floor((poly_x_end - platform_min_x) / pu_gap)*pu_gap;

					// Search backwards from x=0
					for (double x = fmin(0, last_x_pu); x + platform_min_x > poly_x_start; x -= pu_gap) {
						if (!try_pu_x(params, plat, T_start, T_tilt, x, x1_min, x1_max, x2_min, x2_max, platform_min_x, platform_max_x, platform_min_z, platform_max_z, m, c_min, c_max, nx, ny, nz, i, q)) {
							break;
						}
					}

					// Search forwards from x>0
					for (double x = fmax(pu_gap, first_x_pu); x - platform_max_x < poly_x_end; x += pu_gap) {
						if (!try_pu_x(params, plat, T_start, T_tilt, x, x1_min, x1_max, x2_min, x2_max, platform_min_x, platform_max_x, platform_min_z, platform_max_z, m, c_min, c_max, nx, ny, nz, i, q)) {
							break;
						}
					}
				}
				else {
					// Calculate z coordinates of intersection points
					double z1_min = tan(a[i])*x1_min + plat->pos[2] - tan(a[i])*plat->pos[0];
					double z1_max = tan(a[i])*x1_max + plat->pos[2] - tan(a[i])*plat->pos[0];
					double z2_min = tan(a[(i + 1) % 4])*x2_min + plat->pos[2] - tan(a[(i + 1) % 4])*plat->pos[0];
					double z2_max = tan(a[(i + 1) % 4])*x2_max + plat->pos[2] - tan(a[(i + 1) % 4])*plat->pos[0];

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

					double first_z_pu = ceil((poly_z_start - platform_max_z) / pu_gap)*pu_gap;
					double last_z_pu = floor((poly_z_end - platform_min_z) / pu_gap)*pu_gap;

					// Search backwards from z=0
					for (double z = fmin(0, last_z_pu); z + platform_min_z > poly_z_start; z -= pu_gap) {
						if (!try_pu_z(params, plat, T_start, T_tilt, z, z1_min, z1_max, z2_min, z2_max, platform_min_x, platform_max_x, platform_min_z, platform_max_z, m, c_min, c_max, nx, ny, nz, i, q)) {
							break;
						}
					}

					// Search forwards from z>0
					for (double z = fmax(pu_gap, first_z_pu); z - platform_max_z < poly_z_end; z += pu_gap) {
						if (!try_pu_z(params, plat, T_start, T_tilt, z, z1_min, z1_max, z2_min, z2_max, platform_min_x, platform_max_x, platform_min_z, platform_max_z, m, c_min, c_max, nx, ny, nz, i, q)) {
							break;
						}
					}
				}
			}
		}
	}
}

void search_normals(SearchParams* params) {
	if (params->min_ny == params->max_ny) {
		#pragma omp parallel for schedule(dynamic, 1)
		for (int h = 0; h <= params->n_samples_x; h++) {
			Mario mario;
			Platform plat(platform_positions[params->platform_idx][0], platform_positions[params->platform_idx][1], platform_positions[params->platform_idx][2]);
			Vec2S tri = plat.triangles;

			double nx = ((double)h / (double)params->n_samples_x)*(params->max_nx - params->min_nx) + params->min_nx;
			printf("nx = %f\n", nx);

			for (int i = 0; i <= params->n_samples_z; i++) {
				double nz = ((double)i / (double)params->n_samples_z)*(params->max_nz - params->min_nz) + params->min_nz;
				try_normal(params, &plat, nx, params->min_ny, nz);
				plat.triangles = tri;
			}
		}
	}
	else
	{
		#pragma omp parallel for schedule(dynamic, 1)
		for (int h = 0; h <= params->n_samples_y; h++) {
			Mario mario;
			Platform plat(platform_positions[params->platform_idx][0], platform_positions[params->platform_idx][1], platform_positions[params->platform_idx][2]);
			Vec2S tri = plat.triangles;

			double ny = ((double)h / (double)params->n_samples_y)*(params->max_ny - params->min_ny) + params->min_ny;
			printf("ny = %f\n", ny);

			for (int i = 0; i <= params->n_samples_x; i++) {
				double nx = ((double)i / (double)params->n_samples_x)*(params->max_nx - params->min_nx) + params->min_nx;

				for (int j = 0; j <= params->n_samples_z; j++) {
					double nz = ((double)j / (double)params->n_samples_z)*(params->max_nz - params->min_nz) + params->min_nz;
					try_normal(params, &plat, nx, ny, nz);
					plat.triangles = tri;
				}
			}
		}
	}
}
