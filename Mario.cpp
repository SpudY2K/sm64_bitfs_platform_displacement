#include "Mario.hpp"

void Mario::set_pos(const Vec3f& position) {
    this->pos = position;
}

int Mario::ground_step(int yaw, Vec2S& triangles) {
    int steps = 0;

    for (int i = 0; i < 4; i++) {
		const Surface* floor = find_floor(pos, triangles);

		if (!floor) { break; }

		float old_x = this->pos[0];
		float old_z = this->pos[2];

        this->pos[0] = this->pos[0] + gSineTable[uint16_t(yaw) >> 4] * floor->normal[1] * (this->speed / 4.0f);
        this->pos[2] = this->pos[2] + gCosineTable[uint16_t(yaw) >> 4] * floor->normal[1] * (this->speed / 4.0f);

        if (!check_inbounds(*this)) { 
			this->pos[0] = old_x;
			this->pos[2] = old_z;
			break;
		}

        steps++;
    }

    return steps;
}
