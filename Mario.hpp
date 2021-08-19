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

	Mario() {}

	Mario(const Vec3f& position, float spd) {
		set_pos(position);
		speed = spd;
	}

	void set_pos(const Vec3f& position);
	int ground_step(int yaw, Vec2S& triangles);
};

#endif
