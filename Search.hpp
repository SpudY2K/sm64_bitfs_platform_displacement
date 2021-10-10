#include "SearchParams.hpp"
#include "Platform.hpp"
#include "vmath.hpp"

using namespace std;

# define M_PI           3.14159265358979323846  /* pi */

void try_hau(SearchParams* params, Platform* plat, double hau, double pu_x, double pu_z, double nx, double ny, double nz, int tilt_idx, int tri_idx, int q_steps);
bool try_pu_xz(SearchParams* params, Platform* plat, double x, double z, double nx, double ny, double nz, int tilt_idx, int q_steps);
bool try_pu_x(SearchParams* params, Platform* plat, Mat4 T_start, Mat4 T_tilt, double x, double x1_min, double x1_max, double x2_min, double x2_max, double platform_min_x, double platform_max_x, double platform_min_z, double platform_max_z, double m, double c_min, double c_max, double nx, double ny, double nz, int tilt_idx, int q_steps);
bool try_pu_z(SearchParams* params, Platform* plat, Mat4 T_start, Mat4 T_tilt, double z, double z1_min, double z1_max, double z2_min, double z2_max, double platform_min_x, double platform_max_x, double platform_min_z, double platform_max_z, double m, double c_min, double c_max, double nx, double ny, double nz, int tilt_idx, int q_steps);
void try_normal(SearchParams* params, Platform* plat, double nx, double ny, double nz);
void search_normals(SearchParams* params);