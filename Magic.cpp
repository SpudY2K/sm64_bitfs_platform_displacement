#include "Magic.hpp"
#include "Mario.hpp"
#include <cmath>

pair<int16_t, float> calc_intended_yawmag(int8_t stickX, int8_t stickY) {
	int16_t intYaw;
	float intMag;

	float intStickX = 0;
	float intStickY = 0;

	//printf("%d, %d, %d\n", stickX, stickY, camYaw);

	if (stickX <= -8) {
		intStickX = stickX + 6;
	}
	else if (stickX >= 8) {
		intStickX = stickX - 6;
	}

	if (stickY <= -8) {
		intStickY = stickY + 6;
	}
	else if (stickY >= 8) {
		intStickY = stickY - 6;
	}

	float stickMag = sqrtf(powf(intStickX, 2) + powf(intStickY, 2));

	if (stickMag > 64.0) {
		intStickX = float(intStickX * 64.0 / stickMag);
		intStickY = float(intStickY * 64.0 / stickMag);
		stickMag = 64.0;
	}

	stickMag = float(((stickMag / 64.0) * (stickMag / 64.0)) * 64.0);
	intMag = float(stickMag / 2.0);

	intYaw = atan2s(-intStickY, intStickX);
	
	/*
	if (intMag > 0.0) {
		intYaw = atan2s(-intStickY, intStickX);
	}
	else {
		intYaw = yaw;
		//intYaw = *marioFYaw(game);
	}*/

	return { intYaw, intMag };
}

bool check_inbounds(const Mario& m) {
	// float x_mod = fmodf(m.pos[0] + 32768, 65536) - 32768;
	// float y_mod = fmodf(m.pos[1] + 32768, 65536) - 32768;
	// float z_mod = fmodf(m.pos[2] + 32768, 65536) - 32768;
	short x_mod = (short)(int)m.pos[0];
	short y_mod = (short)(int)m.pos[1];
	short z_mod = (short)(int)m.pos[2];

	return (abs(x_mod) < 8192 & abs(y_mod) < 8192 & abs(z_mod) < 8192);
}

float dist_calc(const Vec3f& x, const Vec3f& y) {
	return sqrtf(powf(y[0] - x[0], 2) + powf(y[1] - x[1], 2) + powf(y[2] - x[2], 2));
}

float line_point(const Vec3s& p1, const Vec3s& p2, float x, bool followY) {
	if (followY) {
		if (p2[0] - p1[0] == 0) {
			return p1[1];
		}
		else {
			return (p2[1] - p1[1]) / (p2[0] - p1[0]) * (x - p1[0]) + p1[1];
		}
	}
	else {
		if (p2[0] - p1[0] == 0) {
			return p1[2];
		}
		else {
			return (p2[2] - p1[2]) / (p2[0] - p1[0]) * (x - p1[0]) + p1[2];
		}
	}
}

Surface const * find_floor(Vec3f& pos, Vec2S& triangles) {
	Surface const * floor = NULL;

	int16_t x = static_cast<int16_t>(static_cast<int>(pos[0]));
	int16_t y = static_cast<int16_t>(static_cast<int>(pos[1]));
	int16_t z = static_cast<int16_t>(static_cast<int>(pos[2]));

	for (int i = 0; i < 2; i++) {
		const Surface& surf = triangles[i];

		int16_t x1 = surf.vectors[0][0];
		int16_t z1 = surf.vectors[0][2];
		int16_t x2 = surf.vectors[1][0];
		int16_t z2 = surf.vectors[1][2];

		// Check that the point is within the triangle bounds.
		if ((z1 - z) * (x2 - x1) - (x1 - x) * (z2 - z1) < 0) {
			continue;
		}

		// To slightly save on computation time, set this later.
		int16_t x3 = surf.vectors[2][0];
		int16_t z3 = surf.vectors[2][2];

		if ((z2 - z) * (x3 - x2) - (x2 - x) * (z3 - z2) < 0) {
			continue;
		}
		if ((z3 - z) * (x1 - x3) - (x3 - x) * (z1 - z3) < 0) {
			continue;
		}

		float nx = surf.normal[0];
		float ny = surf.normal[1];
		float nz = surf.normal[2];
		float oo = -(nx * x1 + ny * surf.vectors[0][1] + nz * z1);

		// Find the height of the floor at a given location.
		float height = -(x * nx + nz * z + oo) / ny;
		// Checks for floor interaction with a 78 unit buffer.
		if (y - (height + -78.0f) < 0.0f) {
			continue;
		}

		floor = &surf;
		break;
	}

	//! (Surface Cucking) Since only the first floor is returned and not the highest,
	//  higher floors can be "cucked" by lower floors.
	return floor;
}
