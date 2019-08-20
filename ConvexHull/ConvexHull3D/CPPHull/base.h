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
	bool operator==(vec3 v);
	bool operator!=(vec3 v);
	vec3 operator+=(vec3 v);
	vec3 operator-=(vec3 v);
	vec3 operator/=(double s);
	vec3 operator*=(double s);

	vec3 operator-();

	double lenSq() const;
	double len() const;
	void copyTo(double* dest, size_t &pos) const;
	void copyTo(double dest[3]) const;
	bool isZero() const;
	bool isValid() const;
	vec3 unit() const;

	static double solidAngle(vec3 a, vec3 b, vec3 c);
	static vec3 sum(vec3* vecs, size_t nVecs);
	static vec3 average(vec3* vecs, size_t nVecs);
};

struct triangle {
	size_t index;
	size_t a, b, c;
	
	triangle();
	triangle(size_t i, size_t v1, size_t v2, size_t v3);

	bool isValid() const;
	void flip();
	triangle flipped() const;
};

class util {
private:
	// Default implementations of the template functions needs to be
	// in the header file.
	template<typename T>
	static void combinationsInternal(T* arr, size_t n, size_t total, 
		size_t curPos, std::vector<T*>& combs) {

		if (n == 0) {
			combs.push_back(new T[0]);
			return;
		}
		else if (n == total) {
			combs.push_back(arr);
			return;
		}
		else if (n > total) {
			throw "The combinations cannot contain more elements than the total.";
		}

		for (size_t k = curPos; k < total; k++)
		{
			std::vector<T*> com2;
			util::combinationsInternal(arr, n - 1, total, k + 1, com2);
			for (size_t i = 0; i < com2.size(); i++)
			{
				T* com = new T[n];
				com[0] = arr[k];
				for (size_t j = 0; j < n - 1; j++)
				{
					com[j + 1] = com2[i][j];
				}

				delete com2[i];
				combs.push_back(com);
			}
		}
	}
public:
	// Default implementations of the template functions needs to be
	// in the header file.
	template<typename T>
	static void combinations(T* arr, size_t n, size_t total, 
		std::vector<T*>& combs) {
		
		util::combinationsInternal(arr, n, total, 0, combs);
	}

	static double tetVolume(vec3 a, vec3 b, vec3 c, vec3 d);
	static size_t factorial(size_t n);
};

PINVOKE void Unsafe_ReleaseIntArray(int* arr);