#pragma once

#include "base.h"

class convex_hull
{
private:
	std::vector<triangle> _triangles;
	vec3* _pts;
	size_t _nPts;

	void compute();

public:
	convex_hull(double* pts, size_t nPts);
	~convex_hull();

	vec3 getPt(size_t index);
	size_t numTriangles();
	void getAllTriangles(size_t* triangles);
	void getAllTriangles(int* triangles);
};

PINVOKE void Unsafe_ComputeHull(double* pts, size_t numPoints,
	int* &triangles, int& nTriangles);