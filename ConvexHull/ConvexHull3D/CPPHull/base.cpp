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

bool vec3::operator==(vec3 v) {
	return x == v.x && y == v.y && z == v.z;
}

bool vec3::operator!=(vec3 v) {
	return x != v.x || y != v.y || z != v.z;
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

bool vec3::isZero() {
	return x == 0 && y == 0 && z == 0;
}

bool vec3::isValid() {
	return x != vec3::unset.x && y != vec3::unset.y && z != vec3::unset.z;
}

vec3 vec3::unit() {
	return isZero() ? zero : vec3(x, y, z) / len();
}

triangle::triangle() : triangle::triangle(-1, -1, -1, -1) { }

triangle::triangle(size_t i, size_t v1, size_t v2, size_t v3) {
	index = i;
	a = v1;
	b = v2;
	c = v3;
}

bool triangle::isValid() {
	return index > -1 && a > -1 && b > -1 && c > -1;
}

void triangle::flip() {
	size_t temp;
	temp = b;
	b = c;
	c = temp;
}

triangle triangle::flipped() {
	return triangle(index, a, c, b);
}