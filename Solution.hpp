#include "SearchParams.hpp"
#include "vmath.hpp"

#ifndef SOLUTION_H
#define SOLUTION_H
class Solution {
	public:
		int platform_idx;
		float speed;
		int yaw;
		Vec3f position;
		Vec3f platform_normal;

		Solution(SearchParams* params, float spd, int yw, float x, float y, float z, float nx, float ny, float nz) {
			platform_idx = params->platform_idx;
			speed = spd;
			yaw = yw;
			position = { x, y, z };
			platform_normal = { nx, ny, nz };
		}
};
#endif