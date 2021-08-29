#pragma once

#include <vector>
#include "Mario.hpp"
#include "Surface.hpp"
#include "vmath.hpp"

using namespace std;

#ifndef PLATFORM_H
#define PLATFORM_H

class Platform
{
public:
	Vec3s pos = { 0, 0, 0 };
	Vec3f normal = { 0, 1, 0 };
	Mat4 transform = { {} };
	Vec2S triangles = { Surface(true), Surface(false) };

	Platform(int16_t x, int16_t y, int16_t z) {
		pos[0] = x;
		pos[1] = y;
		pos[2] = z;

		create_transform_from_normals();

		triangles[0].rotate(transform);
		triangles[1].rotate(transform);
	}

	void create_transform_from_normals();
	void platform_logic(Mario* m);
};

float approach_by_increment(float goal, float src, float inc);

#endif
