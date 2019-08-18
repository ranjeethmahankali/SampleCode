#include "base.h"

const vec3 vec3::zero = vec3(0,0,0);
const vec3 vec3::unset = vec3(doubleMaxValue, doubleMaxValue, doubleMaxValue);

vec3::vec3(double a, double b, double c) {
	x = a;
	y = b;
	z = c;
}

vec3::vec3() : vec3(0, 0, 0) { }

vec3 vec3::operator+(vec3 v) {
	return vec3(x + v.x, y + v.y, z + v.z);
}

vec3 vec3::operator-(vec3 v) {
	return vec3(x - v.x, y - v.y, z - v.z);
}

double vec3::operator*(vec3 v) {
	return (x * v.x + y * v.y + z * v.z);
}

vec3 vec3::operator^(vec3 v) {
	return vec3(y * v.z - z * v.y, z * v.x - x * v.z,
		x * v.y - y * v.x);
}

vec3 vec3::operator*(double s) {
	return vec3(x * s, y * s, z * s);
}

vec3 vec3::operator/(double s) {
	return vec3(x / s, y / s, z / s);
}

vec3 vec3::operator-() {
	return vec3(-x, -y, -z);
}

double vec3::lenSq() {
	return x * x + y * y + z * z;
}

double vec3::len() {
	return sqrt(lenSq());
}

void vec3::copyTo(double* dest, size_t &pos) {
	dest[pos++] = x;
	dest[pos++] = y;
	dest[pos++] = z;
}

double vec3::solidAngle(vec3 a, vec3 b, vec3 c) {
	return abs(((a ^ b) * c) / (a.len() * b.len() * c.len() + (a * b) * c.len() +
		(b * c) * a.len() + (c * a) * c.len()));
}