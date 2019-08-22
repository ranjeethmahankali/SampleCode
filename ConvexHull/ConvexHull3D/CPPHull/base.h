#pragma once
#include <cstdlib>
#include <limits>
#include <cstddef>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <set>
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

	vec3 operator+(const vec3& v) const;
	vec3 operator-(const vec3& v) const;
	double operator*(const vec3& v) const;
	vec3 operator^(const vec3& v) const;
	vec3 operator*(const double& s) const;
	vec3 operator/(const double& s) const;
	bool operator==(const vec3& v) const;
	bool operator!=(const vec3& v) const;
	vec3 operator+=(const vec3& v);
	vec3 operator-=(const vec3& v);
	vec3 operator/=(const double& s);
	vec3 operator*=(const double&s);

	vec3 operator-() const;

	double lenSq() const;
	double len() const;
	void copyTo(double* dest, size_t& pos) const;
	void copyTo(double dest[3]) const;
	bool isZero() const;
	bool isValid() const;
	vec3 unit() const;

	static double solidAngle(const vec3& a, const vec3& b, const vec3& c);
	static vec3 sum(vec3* vecs, const size_t& nVecs);
	static vec3 average(vec3* vecs, const size_t& nVecs);
};

struct indexPair {
	size_t p, q;

	bool operator==(const indexPair other) const;
	bool operator!=(const indexPair other) const;

	indexPair(size_t i, size_t j)
		: p(i), q(j) {}

	indexPair()
		: indexPair(-1, -1) {}

	size_t hash() const;
};

struct triangle {
	size_t index;
	size_t a, b, c;
	vec3 normal;

	triangle();
	triangle(size_t i, size_t v1, size_t v2, size_t v3);

	bool isValid() const;
	void flip();
	triangle flipped() const;
	indexPair edge(char edgeIndex) const;
};


namespace std {
	template<> struct hash<indexPair> {
		typedef indexPair argument_type;
		typedef size_t result_type;
		result_type operator()(const argument_type& pair) const noexcept {
			return pair.hash();
		}
	};
}



class util {
public:
	static double tetVolume(vec3 a, vec3 b, vec3 c, vec3 d);
	static size_t factorial(size_t n);
};

PINVOKE void Unsafe_ReleaseIntArray(int* arr);