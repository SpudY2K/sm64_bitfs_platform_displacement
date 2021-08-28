#include "Surface.hpp"

void Surface::rotate(const Vec3s& pivot, const Mat4& old_mat, const Mat4& new_mat) {
	Vec3f dist1;
	Vec3f dist2;
	Vec3f dist3;
	Vec3f posBeforeRotation1;
	Vec3f posAfterRotation1;
	Vec3f posBeforeRotation2;
	Vec3f posAfterRotation2;
	Vec3f posBeforeRotation3;
	Vec3f posAfterRotation3;

	for(int i = 0; i < 3; i++) {
		dist1[i] = vectors[0][i] - pivot[i];
		dist2[i] = vectors[1][i] - pivot[i];
		dist3[i] = vectors[2][i] - pivot[i];
	}

	linear_mtxf_mul_vec3f(posBeforeRotation1, old_mat, dist1);
	linear_mtxf_mul_vec3f(posAfterRotation1, new_mat, dist1);

	linear_mtxf_mul_vec3f(posBeforeRotation2, old_mat, dist2);
	linear_mtxf_mul_vec3f(posAfterRotation2, new_mat, dist2);

	linear_mtxf_mul_vec3f(posBeforeRotation3, old_mat, dist3);
	linear_mtxf_mul_vec3f(posAfterRotation3, new_mat, dist3);

	for(int i = 0; i < 3; i++) {
		vectors[0][i] += posAfterRotation1[i] - posBeforeRotation1[i];
		vectors[1][i] += posAfterRotation2[i] - posBeforeRotation2[i];
		vectors[2][i] += posAfterRotation3[i] - posBeforeRotation3[i];
	}

	normal[0] = (((vectors[1][1] - vectors[0][1]) * (vectors[2][2] - vectors[1][2])) - ((vectors[1][2] - vectors[0][2]) * (vectors[2][1] - vectors[1][1]))) * 1000.0f;
	normal[1] = (((vectors[1][2] - vectors[0][2]) * (vectors[2][0] - vectors[1][0])) - ((vectors[1][0] - vectors[0][0]) * (vectors[2][2] - vectors[1][2]))) * 1000.0f;
	normal[2] = (((vectors[1][0] - vectors[0][0]) * (vectors[2][1] - vectors[1][1])) - ((vectors[1][1] - vectors[0][1]) * (vectors[2][0] - vectors[1][0]))) * 1000.0f;

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
