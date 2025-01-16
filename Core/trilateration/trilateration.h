#ifndef __TRILATERATION_H__
#define __TRILATERATION_H__

//#define TREK1000_TRILATERATION
#define SR150_3D

typedef enum {
	UWB_OK = 1,
	UWB_ANC_BELOW_THREE = -1,
	UWB_LIN_DEP_FOR_THREE = -2,
	UWB_ANC_ON_ONE_LEVEL = -3,
	UWB_LIN_DEP_FOR_FOUR = -4,
	UWB_RANK_ZERO = -5
}UWB_POS_ERROR_CODES;

typedef struct {
	int x, y, z; //axis in cm
} position_t; // Position of a device or target in 3D space


#define		TRIL_3SPHERES		3
#define		TRIL_4SPHERES		4

typedef struct vec3d	vec3d;
struct vec3d {
	double	x;
	double	y;
	double	z;
};

/* Return the difference of two vectors, (vector1 - vector2). */
vec3d vdiff(const vec3d vector1, const vec3d vector2);

/* Return the sum of two vectors. */
vec3d vsum(const vec3d vector1, const vec3d vector2);

/* Multiply vector by a number. */
vec3d vmul(const vec3d vector, const double n);

/* Divide vector by a number. */
vec3d vdiv(const vec3d vector, const double n);

/* Return the Euclidean norm. */
double vdist(const vec3d v1, const vec3d v2);

/* Return the Euclidean norm. */
double vnorm(const vec3d vector);

/* Return the dot product of two vectors. */
double dot(const vec3d vector1, const vec3d vector2);

/* Replace vector with its cross product with another vector. */
vec3d cross(const vec3d vector1, const vec3d vector2);

int GetLocation(vec3d *best_solution, vec3d* anchorArray, int *distanceArray);

double vdist(const vec3d v1, const vec3d v2);
#endif
