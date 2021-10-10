#include "Surface.hpp"
#include "Solution.hpp"
#include "vmath.hpp"

bool in_triangle(Surface surf, float x, float z);
bool on_triangle(Surface surf, float x, float y, float z);
void write_to_stream(ofstream* out_stream, Solution* s, int tri_idx, float tri_y_norm, Vec3f &final_pos);
void get_start_poly(std::vector<Vec3f> &start_poly, Vec3f &normal, const float(&tilt)[3]);
void gaussian_elimination(double(&coeff1a)[4], double(&coeff1b)[4], double(&coeff1c)[4]);
bool check_fall_through_pus(Solution* s, Vec3f &final_pos);