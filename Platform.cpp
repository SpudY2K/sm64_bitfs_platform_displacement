#include "Platform.hpp"
#include <cmath>

/**
 * Returns a value that is src incremented/decremented by inc towards goal
 * until goal is reached. Does not overshoot.
 */
float approach_by_increment(float goal, float src, float inc) {
    float newVal;

    if (src <= goal) {
        if (goal - src < inc) {
            newVal = goal;
        }
        else {
            newVal = src + inc;
        }
    }
    else if (goal - src > -inc) {
        newVal = goal;
    }
    else {
        newVal = src - inc;
    }

    return newVal;
}

/**
 * Creates a transform matrix on a variable passed in from given normals
 * and the object's position.
 */
void Platform::create_transform_from_normals() {
	mtxf_align_terrain_normal(transform, normal, pos, 0);
}

void Platform::platform_logic(Mario* m) {
	float dx;
	float dy;
	float dz;
	float d;

	Vec3f dist;
	Vec3f posBeforeRotation;
	Vec3f posAfterRotation;

	//create_transform_from_normals();

	// Mario's position
	float mx;
	float my;
	float mz;

	mx = m->pos[0];
	my = m->pos[1];
	mz = m->pos[2];

	dist[0] = mx - (float)pos[0];
	dist[1] = my - (float)pos[1];
	dist[2] = mz - (float)pos[2];
	linear_mtxf_mul_vec3f(posBeforeRotation, transform, dist);

	dx = mx - (float)pos[0];
	dy = 500.0f;
	dz = mz - (float)pos[2];
	d = sqrtf(dx * dx + dy * dy + dz * dz);

	//! Always true since dy = 500, making d >= 500.
	if (d != 0.0f) {
		// Normalizing
		d = 1.0 / d;
		dx *= d;
		dy *= d;
		dz *= d;
	}
	else {
		dx = 0.0f;
		dy = 1.0f;
		dz = 0.0f;
	}

	/*
	if (o->oTiltingPyramidMarioOnPlatform == TRUE)
		marioOnPlatform++;

	o->oTiltingPyramidMarioOnPlatform = TRUE;*/

	Mat4 old_mat = transform;

	// Approach the normals by 0.01f towards the new goal, then create a transform matrix and orient the object. 
	// Outside of the other conditionals since it needs to tilt regardless of whether Mario is on.
	normal[0] = approach_by_increment(dx, normal[0], 0.01f);
	normal[1] = approach_by_increment(dy, normal[1], 0.01f);
	normal[2] = approach_by_increment(dz, normal[2], 0.01f);
	create_transform_from_normals();

	triangles[0].rotate(transform);
	triangles[1].rotate(transform);

	// pretty sure you can always assume if here, then mario is on the floor
	Surface const* floor = find_floor(m->pos, triangles);

	// If Mario is on the platform, adjust his position for the platform tilt.
	if (floor) {
		linear_mtxf_mul_vec3f(posAfterRotation, transform, dist);
		mx += posAfterRotation[0] - posBeforeRotation[0];
		my += posAfterRotation[1] - posBeforeRotation[1];
		mz += posAfterRotation[2] - posBeforeRotation[2];
		m->pos[0] = mx;
		m->pos[1] = my;
		m->pos[2] = mz;
	}
}
