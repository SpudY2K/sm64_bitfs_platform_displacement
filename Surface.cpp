#include "Surface.hpp"

void Surface::rotate(const Mat4& m) {
	reset_vectors();

	for (int i = 0; i < 3; i++) {
		float vx = vectors[i][0];
		float vy = vectors[i][1];
		float vz = vectors[i][2];

		vectors[i][0] = (short)(int)(vx * m[0][0] + vy * m[1][0] + vz * m[2][0] + m[3][0]);
		vectors[i][1] = (short)(int)(vx * m[0][1] + vy * m[1][1] + vz * m[2][1] + m[3][1]);
		vectors[i][2] = (short)(int)(vx * m[0][2] + vy * m[1][2] + vz * m[2][2] + m[3][2]);
	}

	normal[0] = ((vectors[1][1] - vectors[0][1]) * (vectors[2][2] - vectors[1][2])) - ((vectors[1][2] - vectors[0][2]) * (vectors[2][1] - vectors[1][1]));
	normal[1] = ((vectors[1][2] - vectors[0][2]) * (vectors[2][0] - vectors[1][0])) - ((vectors[1][0] - vectors[0][0]) * (vectors[2][2] - vectors[1][2]));
	normal[2] = ((vectors[1][0] - vectors[0][0]) * (vectors[2][1] - vectors[1][1])) - ((vectors[1][1] - vectors[0][1]) * (vectors[2][0] - vectors[1][0]));

	vec3f_normalize(normal);

	originOffset = -(normal[0] * vectors[0][0] + normal[1] * vectors[0][1] + normal[2] * vectors[0][2]);
}

void Surface::repr() {
	printf("vector1: (");

	for (int i = 0; i < 3; i++) {
		printf("%d", vectors[0][i]);

		if (i != 2) {
			printf(", ");
		}
		else {
			printf(")\n");
		}
	}

	printf("vector2: (");

	for (int i = 0; i < 3; i++) {
		printf("%d", vectors[1][i]);

		if (i != 2) {
			printf(", ");
		}
		else {
			printf(")\n");
		}
	}

	printf("vector3: (");

	for (int i = 0; i < 3; i++) {
		printf("%d", vectors[2][i]);

		if (i != 2) {
			printf(", ");
		}
		else {
			printf(")\n");
		}
	}
}
