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
	Vec3s pos;
	Vec3f normal = { 0, 1, 0 };
	Mat4 transform = { {} };
	Vec2S triangles;

	Platform() : Platform(0) { }

	Platform(const int plat_idx) {
		if (plat_idx == 0) {
			pos[0] = -1945;
			pos[1] = -3225;
			pos[2] = -715;
		}
		else {
			pos[0] = -2866;
			pos[1] = -3225;
			pos[2] = -715;
		}

		triangles = { Surface(plat_idx, true), Surface(plat_idx, false) };

		create_transform_from_normals();
	}

	void create_transform_from_normals();
	void platform_logic(Mario* m);
};

float approach_by_increment(float goal, float src, float inc);

#endif
