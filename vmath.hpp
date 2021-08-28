#pragma once

#include "Trig.hpp"
#include <vector>
#include <array>

using namespace std;

#ifndef VMATH_H
#define VMATH_H

typedef std::array<float, 2> Vec2f;
typedef std::array<float, 3> Vec3f; // X, Y, Z, where Y is up
typedef std::array<int16_t, 3> Vec3s;
typedef std::array<int32_t, 3> Vec3i;
typedef std::array<float, 4> Vec4f;
typedef std::array<int16_t, 4> Vec4s;
typedef std::array<Vec3s, 3> VecVec3s;

typedef std::array<std::array<float, 4>, 4> Mat4;

void vec3f_normalize(Vec3f &v);
void vec3f_cross(Vec3f &dst, const Vec3f &a, const Vec3f &b);
void vec3f_set(Vec3f &dst, float x, float y, float z);
void mtxf_align_terrain_normal(Mat4 &dest, Vec3f upDir, const Vec3s &pos, int16_t yaw);
void linear_mtxf_mul_vec3f(Vec3f &dst, const Mat4 &m, const Vec3f &v);

#endif
