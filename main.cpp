#include "Mario.h"
#include "Magic.h"
#include "Platform.h"

using namespace std;

void brute_angles(Mario* m, Platform* plat) {
	pair<int16_t, float> yawmag;

	for (int16_t x = -128; x < 128; x++) {
		for (int16_t y = -128; y < 128; y++) {
			Mario mario(m->pos, m->speed);
			Platform p;

			p.normal = plat->normal;

			yawmag = calc_intended_yawmag(x, y);
			int16_t intYaw = yawmag.first - (yawmag.first % 16);

			if (mario.ground_step(x, y, plat->normal[1]) == 0) { continue; }

			p.platform_logic(&mario);

			if (!check_inbounds(mario)) { continue; }

			if (mario.pos[1] >= 3521) {
				if (mario.pos[1] <= 3841) {
					printf("-------------------\nIDEAL SOLN\nBully spd: %f\nStick X: %d\nStick Y: %d\nMario height: %.9f\nMario pos: (%.9f, %.9f, %.9f)\n",
						mario.speed, x, y, mario.pos[1], m->pos[0], m->pos[1], m->pos[2]);
				}
				else if (mario.pos[1] < 8192) {
					printf("-------------------\nACCEPTABLE SOLN\nBully spd: %f\nStick X: %d\nStick Y: %d\nMario height: %.9f\nMario pos: (%.9f, %.9f, %.9f)\n",
						mario.speed, x, y, mario.pos[1], m->pos[0], m->pos[1], m->pos[2]);
				}
			}
		}
	}
}

void brute_position(Platform* plat, float spd) {
	//pseudo code
	//iterate over z away from the moving platform, up to the maximum displacement push by the moving platform hitbox (negative z direction)
	//	iterate over x along the triangle for each z position
	//		check if y position is above the lava and below the top of the moving platform hitbox
	//			construct Mario object
	//			continue to brute_angles
	//		else
	//			continue

	vector<Surface> tri = plat->triangles;

	plat->create_transform_from_normals();
	plat->triangles[0].rotate(plat->transform);
	plat->triangles[1].rotate(plat->transform);

	float max_x = plat->triangles[1].vector1[0];

	if (max_x < plat->triangles[1].vector3[0]) { max_x = plat->triangles[1].vector3[0]; }

	for (float x = plat->triangles[1].vector2[0]; x < max_x; x = nextafterf(x, max_x)) {
		float y1 = line_point(plat->triangles[1].vector1, plat->triangles[1].vector2, x, true);
		float z1 = line_point(plat->triangles[1].vector1, plat->triangles[1].vector2, x, false);

		float y2 = line_point(plat->triangles[1].vector1, plat->triangles[1].vector3, x, true);
		float z2 = line_point(plat->triangles[1].vector1, plat->triangles[1].vector3, x, false);

		float min_z = min(z1, z2);
		float max_z = max(z1, z2);

		for (float z = min_z; z < max_z; z = nextafterf(z, max_z)) {
			float y;
			
			if (min_z == z1) {
				y = (y2 - y1) / (z2 - z1) * (z - z1) - y1;
			}
			else {
				y = (y1 - y2) / (z1 - z2) * (z - z2) - y2;
			}

			Mario m({x, y, z}, spd);
			brute_angles(&m, plat);
		}
	}
}

void brute_normals(float spd) {
	for (float nx = 0.0f; nx < 1.0f; nx = nextafterf(nx, 1.0f)) {
		for (float nz = 0.0f; nz < 1.0f; nz = nextafterf(nz, 1.0f)) {
			float ny = sqrtf(1 - powf(nx, 2) - powf(nz, 2));

			Platform plat;
			plat.normal = { nx, ny, nz };

			brute_position(&plat, spd);
		}
	}
}

void brute_speed() {
	float spd = 58000000.0f;

	while (spd < 1000000000.0) {
		brute_normals(spd);

		spd = nextafterf(spd, 2000000000.0f);
		printf("Finished all loops for speed %.9f\n", spd);
	}
}

int main() {
	brute_speed();
}