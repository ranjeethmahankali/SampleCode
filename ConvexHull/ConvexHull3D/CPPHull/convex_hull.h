#pragma once

#include "base.h"
#include <queue>
#define _USE_MATH_DEFINES
#include <math.h>

class convex_hull
{
private:
	std::unordered_map<size_t, triangle> _triangles;
	std::unordered_set<size_t> _insidePts;
	std::unordered_set<size_t> _outsidePts;
	std::unordered_map<size_t, std::unordered_set<size_t>> _edgeFaceMap;

	vec3 *_pts, _center;
	size_t _nPts;

	void compute();
	
	void getEdgeIndices(triangle tri, size_t indices[3]) const;
	void getVertIndicesForEdge(size_t edgeI, size_t& v1, size_t& v2) const;
	void setTriangle(triangle tri);
	triangle popTriangle(size_t index, size_t edgeIndices[3],
		size_t adjTriangles[3]);
	bool isTriangleFacing(size_t iTri, vec3 pt, triangle &tri);
	bool isTriangleFacing(triangle tri, vec3 pt) const;
	vec3 triangleNormal(size_t iTri, triangle &tri);
	vec3 triangleNormal(triangle tri) const;
	double trianglePlaneDist(size_t iTri, vec3 pt, triangle &tri);
	size_t farthestPoint(size_t iTri, triangle &triangle);
	double triangleSolidAngle(triangle tri, vec3 pt) const;
	void updateInteriorPoints();
	bool isInsideHull(vec3 pt, size_t index);
	void createInitialSimplex(size_t &triIndex);

public:
	convex_hull(double* pts, size_t nPts);
	~convex_hull();

	vec3 getPt(size_t index) const;
	size_t numTriangles() const;
	void getAllTriangles(std::vector<int> &indices);
};

PINVOKE void Unsafe_ComputeHull(double* pts, size_t numPoints,
	int* &triangles, int& nTriangles);