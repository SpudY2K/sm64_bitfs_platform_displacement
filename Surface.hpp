#pragma once

#include <vector>
#include <cstdio>
#include <cstdint>

#include "vmath.hpp"

using namespace std;

#ifndef SURFACE_H
#define SURFACE_H

class Surface
{
public:
	VecVec3s vectors;
	Vec3f normal;
	float originOffset;
	bool top;

	Surface() {}

	Surface(bool top) : Surface(0, top) { }

	Surface(int plat_idx, bool top) {
		if (plat_idx == 0) {
			vectors[0][0] = -1638;
			vectors[0][1] = -2918;
			vectors[0][2] = -1021;

			if (top) {
				vectors[1][0] = -2251;
				vectors[1][1] = -2918;
				vectors[1][2] = -1021;
				vectors[2][0] = -2251;
				vectors[2][1] = -2918;
				vectors[2][2] = -408;
			}
			else {
				vectors[1][0] = -2251;
				vectors[1][1] = -2918;
				vectors[1][2] = -408;
				vectors[2][0] = -1638;
				vectors[2][1] = -2918;
				vectors[2][2] = -408;
			}
		}
		else if (plat_idx == 1) {
			vectors[0][0] = -2559;
			vectors[0][1] = -2918;
			vectors[0][2] = -1021;

			if (top) {
				vectors[1][0] = -3172;
				vectors[1][1] = -2918;
				vectors[1][2] = -1021;
				vectors[2][0] = -3172;
				vectors[2][1] = -2918;
				vectors[2][2] = -408;
			}
			else {
				vectors[1][0] = -3172;
				vectors[1][1] = -2918;
				vectors[1][2] = -408;
				vectors[2][0] = -2559;
				vectors[2][1] = -2918;
				vectors[2][2] = -408;
			}
		}

		normal[0] = 0.0f;
		normal[1] = 1.0f;
		normal[2] = 0.0f;

		originOffset = -vectors[0][1];

		this->top = top;
	}

	void rotate(const Vec3s& pivot, const Mat4& old_mat, const Mat4& new_mat);
	void repr();
};

typedef std::array<Surface, 2> Vec2S;

#endif
