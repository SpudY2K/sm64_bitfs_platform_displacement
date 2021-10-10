#include "Constants.hpp"
#include "Magic.hpp"
#include "Platform.hpp"
#include "Validate.hpp"

bool validate_solution(Solution* s, SearchParams* param, Vec3f& final_pos, bool pancake) {
	if (pancake) {
		//Not implemented yet
		return false;
	}
	else
	{
		Mario v_mario(s->position, s->speed, s->yaw);

		int v_idx = validate_solution_sim(s, &v_mario);

		final_pos = v_mario.pos;

		if (v_idx != 0) {
			return false;
		}
	}

	for (int i = 0; i < param->n_y_ranges; i++) {
		if (final_pos[1] >= *(param->lower_y+i) && final_pos[1] < *(param->upper_y+i)) {
			return true;
		}
	}

	return false;
}

int validate_solution_sim(Solution* s, Mario* mario) {
	if (mario->pos[1] <= lava_y) {
		return 5;
	}

	Platform plat(platform_positions[s->platform_idx][0], platform_positions[s->platform_idx][1], platform_positions[s->platform_idx][2]);

	plat.normal = s->platform_normal;

	plat.create_transform_from_normals();
	plat.triangles[0].rotate(plat.transform);
	plat.triangles[1].rotate(plat.transform);

	plat.platform_logic(mario);

	if (mario->pos[1] <= lava_y) {
		return 6;
	}

	if (mario->ground_step(plat.triangles) == 0) {
		return 1;
	}

	if (mario->pos[1] <= lava_y) {
		return 7;
	}

	float floor_height = lava_y;

	const Surface* floor = find_floor(mario->pos, plat.triangles, &floor_height);

	if (!floor) {
		return 2;
	}

	if (fabs(mario->pos[1] - floor_height) >= 4.0f) {
		return 8;
	}

	Vec3f pre_tilt_pos = mario->pos;

	plat.platform_logic(mario);

	if (!check_inbounds(*mario)) {
		mario->set_pos(pre_tilt_pos);
		return 3;
	}
	
	return 0;
}