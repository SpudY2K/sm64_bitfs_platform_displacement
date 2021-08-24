#include "Mario.hpp"

void Mario::set_pos(const Vec3f& position) {
    this->pos = position;
}

int Mario::ground_step(Vec2S& triangles) {
    int steps = 0;

	const Surface* floor = find_floor(pos, triangles);

	if (floor) {
		for (int i = 0; i < 4; i++) {
			float old_x = this->pos[0];
			float old_z = this->pos[2];

			this->pos[0] = this->pos[0] + gSineTable[uint16_t(this->yaw) >> 4] * floor->normal[1] * (this->speed / 4.0f);
			this->pos[2] = this->pos[2] + gCosineTable[uint16_t(this->yaw) >> 4] * floor->normal[1] * (this->speed / 4.0f);

			short x_mod = (short)(int)this->pos[0];
			short y_mod = (short)(int)this->pos[1];
			short z_mod = (short)(int)this->pos[2];

			if (!(abs(x_mod) < 8192 & abs(y_mod) < 8192 & abs(z_mod) < 8192)) {
				this->pos[0] = old_x;
				this->pos[2] = old_z;
				break;
			}

			steps++;

			floor = find_floor(pos, triangles);

			if (!floor) { break; }

			// Find the height of the floor at a given location.
			this->pos[1] = -(x_mod * floor->normal[0] + floor->normal[2] * z_mod + floor->originOffset) / floor->normal[1];
		}
	}

    return steps;
}
