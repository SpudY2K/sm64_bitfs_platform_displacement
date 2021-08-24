#pragma once

#include <vector>
#include "Magic.hpp"
#include "vmath.hpp"

using namespace std;

#ifndef MARIO_H
#define MARIO_H

class Mario
{
public:
	Vec3f pos = { 0,0,0 };
	float speed;
	int yaw;

	Mario() {}

	Mario(const Vec3f& position, float spd, int yw) {
		set_pos(position);
		speed = spd;
		yaw = yw;
	}

	void set_pos(const Vec3f& position);
	int ground_step(Vec2S& triangles);
};

#endif
