#pragma once
#include <cstdlib>
#include <limits>
#include <cstddef>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <algorithm>
#define PINVOKE extern "C" __declspec(dllexport)
#define doubleMaxValue std::numeric_limits<double>::max()
#define doubleMinValue std::numeric_limits<double>::min()

struct vec3 {
	double x, y, z;
	static const vec3 zero;
	static const vec3 unset;

	vec3(double a, double b, double c);
	vec3();

	vec3 operator+(vec3 v);
	vec3 operator-(vec3 v);
	double operator*(vec3 v);
	vec3 operator^(vec3 v);

	vec3 operator*(double s);
	vec3 operator/(double s);

	vec3 operator-();

	double lenSq();
	double len();
	void copyTo(double* dest, size_t &pos);
	static double solidAngle(vec3 a, vec3 b, vec3 c);
};

class convex_hull;

struct triangle {
	size_t index;
	size_t a, b, c;
	
	triangle();
	triangle(size_t i, size_t v1, size_t v2, size_t v3);
};