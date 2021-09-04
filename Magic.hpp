#pragma once

#include "Trig.hpp"
#include "vmath.hpp"
#include <stdexcept>
#include <utility>
#include "vmath.hpp"
#include "Surface.hpp"

using namespace std;

#ifndef MAGIC_H
#define MAGIC_H
class Mario;

pair<int16_t, float> calc_intended_yawmag(int8_t stick_x, int8_t stick_y);
bool check_inbounds(const Mario &m);
float dist_calc(const Vec3f& x, const Vec3f& y);
float line_point(const Vec3s& p1, const Vec3s& p2, float x, bool yOrZ);
Surface const * find_floor(Vec3f& pos, Vec2S& triangles, float* height);

#endif
