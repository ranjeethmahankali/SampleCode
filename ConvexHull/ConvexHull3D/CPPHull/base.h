#pragma once
#include <cstdlib>
#include <limits>
#include <cstddef>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <set>
#include <utility>
#include <algorithm>

#define PINVOKE extern "C" __declspec(dllexport)
#define doubleMaxValue std::numeric_limits<double>::max()
#define doubleMinValue std::numeric_limits<double>::min()

struct vec3 {
	double x, y, z;
	static const vec3 zero;
	static const vec3 unset;

	vec3(double, double, double);
	vec3();

	vec3 operator+(const vec3&) const;
	vec3 operator-(const vec3&) const;
	double operator*(const vec3&) const;
	vec3 operator^(const vec3&) const;
	vec3 operator*(const double&) const;
	vec3 operator/(const double&) const;
	bool operator==(const vec3&) const;
	bool operator!=(const vec3&) const;
	vec3 operator+=(const vec3&);
	vec3 operator-=(const vec3&);
	vec3 operator/=(const double&);
	vec3 operator*=(const double&);

	vec3 operator-() const;

	double len_sq() const;
	double len() const;
	void copy(double* dest, size_t& pos) const;
	void copy(double dest[3]) const;
	bool is_zero() const;
	bool is_valid() const;
	vec3 unit() const;

	static vec3 sum(const std::vector<vec3>& vecs);
	static vec3 average(const std::vector<vec3>& vecs);
};

struct index_pair {
	size_t p, q;

	bool operator==(const index_pair) const;
	bool operator!=(const index_pair) const;

	index_pair(size_t i, size_t j);

	index_pair();

	size_t hash() const;
	void unset(size_t);
	bool add(size_t val);
	bool contains(size_t) const;
};

struct tri_face {
	static const tri_face unset;
	size_t id;
	size_t a, b, c;
	vec3 normal;

	tri_face();
	tri_face(size_t i, size_t v1, size_t v2, size_t v3);

	bool is_valid() const;
	void flip();
	index_pair edge(char edgeIndex) const;
};

struct index_pair_hash {
	size_t operator()(const index_pair&) const noexcept;
};

struct custom_size_t_hash {
	size_t operator()(const size_t&) const noexcept;
};

PINVOKE void Unsafe_ReleaseIntArray(int* arr);