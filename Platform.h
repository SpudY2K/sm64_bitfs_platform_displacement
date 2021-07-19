#pragma once

#include <vector>
#include "Mario.h"
#include "Surface.h"
#include "vmath.h"

using namespace std;

#ifndef PLATFORM_H
#define PLATFORM_H

class Platform
{
public:
	Vec3s pos = { -1945, -3225, -715 };
	Vec3f normal = { 0, 1, 0 };
	Mat4 transform = { {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0} };
	Vec2S triangles = { Surface(true), Surface(false) };

	Platform() {
		create_transform_from_normals();
	}

	void create_transform_from_normals();
	Surface const * find_floor(Mario* m) const;
	void platform_logic(Mario* m);
};

float approach_by_increment(float goal, float src, float inc);

#endif