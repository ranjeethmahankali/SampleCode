#include "base.h"

const vec3 vec3::zero = vec3(0,0,0);
const vec3 vec3::unset = vec3(doubleMaxValue, doubleMaxValue, doubleMaxValue);
const tri_face tri_face::unset = tri_face(-1, -1, -1, -1);

vec3::vec3(double a, double b, double c) {
	x = a;
	y = b;
	z = c;
}

vec3::vec3() : vec3(0, 0, 0) { }

vec3 vec3::operator+(const vec3& v) const {
	return vec3(x + v.x, y + v.y, z + v.z);
}

vec3 vec3::operator-(const vec3& v) const {
	return vec3(x - v.x, y - v.y, z - v.z);
}

double vec3::operator*(const vec3& v) const {
	return (x * v.x + y * v.y + z * v.z);
}

vec3 vec3::operator^(const vec3& v) const {
	return vec3(y * v.z - z * v.y, z * v.x - x * v.z,
		x * v.y - y * v.x);
}

vec3 vec3::operator*(const double& s) const {
	return vec3(x * s, y * s, z * s);
}

vec3 vec3::operator/(const double& s) const {
	return vec3(x / s, y / s, z / s);
}

vec3 vec3::operator-() const {
	return vec3(-x, -y, -z);
}

bool vec3::operator==(const vec3& v) const {
	return x == v.x && y == v.y && z == v.z;
}

bool vec3::operator!=(const vec3& v) const {
	return x != v.x || y != v.y || z != v.z;
}

vec3 vec3::operator+=(const vec3& v) {
	x += v.x;
	y += v.y;
	z += v.z;
	return *this;
}

vec3 vec3::operator-= (const vec3& v) {
	x -= v.y;
	y -= v.y;
	z -= v.z;
	return *this;
}

vec3 vec3::operator *=(const double& s) {
	x *= s;
	y *= s;
	z *= s;
	return *this;
}

vec3 vec3::operator /=(const double& s) {
	x /= s;
	y /= s;
	z /= s;
	return *this;
}

double vec3::len_sq() const {
	return x * x + y * y + z * z;
}

double vec3::len() const  {
	return sqrt(len_sq());
}

void vec3::copy(double* dest, size_t& pos) const  {
	dest[pos++] = x;
	dest[pos++] = y;
	dest[pos++] = z;
}

void vec3::copy(double dest[3]) const {
	dest[0] = x;
	dest[1] = y;
	dest[2] = z;
}

bool vec3::is_zero() const {
	return x == 0 && y == 0 && z == 0;
}

bool vec3::is_valid() const {
	return x != vec3::unset.x && y != vec3::unset.y && z != vec3::unset.z;
}

vec3 vec3::unit() const {
	return is_zero() ? zero : vec3(x, y, z) / len();
}

vec3 vec3::sum(vec3* vecs, const size_t& nVecs) {
	vec3 sum = vec3::zero;
	for (size_t i = 0; i < nVecs; i++)
	{
		sum += vecs[i];
	}

	return sum;
}

vec3 vec3::average(vec3* vecs, const size_t& nVecs) {
	return sum(vecs, nVecs) / nVecs;
}

tri_face::tri_face() : tri_face::tri_face(-1, -1, -1, -1) { }

tri_face::tri_face(size_t i, size_t v1, size_t v2, size_t v3)
	: a(v1), b(v2), c(v3), index(i), normal(vec3::unset) {}

bool tri_face::is_valid() const {
	// Normally we would check if the numbers are > -1. but because size_t is unsigned. -1 causes integer underflow
	// resulting in a huge number whenever we assign -1 to a size_t type. so here we check the opposite i.e. <.
	return index < -1 && a < -1 && b < -1 && c < -1 && a != b && b != c && c != a;
}

void tri_face::flip() {
	size_t temp;
	temp = b;
	b = c;
	c = temp;
	normal = -normal;
}

index_pair tri_face::edge(char edgeIndex) const
{
	switch (edgeIndex)
	{
	case 0:
		return index_pair(a, b);
	case 1:
		return index_pair(b, c);
	case 2:
		return index_pair(c, a);
	default:
		throw "Invalid edge index";
	}
}

PINVOKE void Unsafe_ReleaseIntArray(int* arr) {
	delete[] arr;
}

bool index_pair::operator==(const index_pair other) const
{
	return (p == other.p && q == other.q) || (p == other.q && q == other.p);
}

bool index_pair::operator!=(const index_pair other) const
{
	return (p != other.p && p != other.q) || (q != other.p && q != other.q);
}

size_t index_pair::hash() const
{
	return p + q + p * q;
}

void index_pair::unset(size_t n)
{
	if (p == n) {
		p = -1;
	}
	else if (q == n) {
		q = -1;
	}
}

bool index_pair::add(size_t val)
{
	if (p == -1) {
		p = val;
		return true;
	}
	else if (q == -1) {
		q = val;
		return true;
	}
	return false;
}

bool index_pair::contains(size_t n) const
{
	return n != -1 && (n == p || n == q);
}
