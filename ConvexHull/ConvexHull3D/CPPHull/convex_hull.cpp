#include "convex_hull.h"

convex_hull::convex_hull(double* pts, size_t nPts) {
	_pts = new vec3[nPts];
	for (size_t i = 0; i < nPts; i++)
	{
		_pts[i].x = pts[3 * i];
		_pts[i].y = pts[3 * i + 1];
		_pts[i].z = pts[3 * i + 2];
	}
	_nPts = nPts;

	compute();
}

convex_hull::~convex_hull() {
	_triangles.clear();
	delete _pts;
}

vec3 convex_hull::getPt(size_t index) {
	return (index < 0 || index > _nPts) ? vec3::unset : _pts[index];
}

size_t convex_hull::numTriangles() {
	return _triangles.size();
}

void convex_hull::getAllTriangles(size_t* triangles) {
	size_t nTriangles = numTriangles();
	for (size_t i = 0; i < nTriangles; i++)
	{
		triangles[3 * i] = _triangles[i].a;
		triangles[3 * i + 1] = _triangles[i].b;
		triangles[3 * i + 2] = _triangles[i].c;
	}
}

void convex_hull::getAllTriangles(int* triangles) {
	size_t nTriangles = numTriangles();
	for (size_t i = 0; i < nTriangles; i++)
	{
		triangles[3 * i] = _triangles[i].a;
		triangles[3 * i + 1] = _triangles[i].b;
		triangles[3 * i + 2] = _triangles[i].c;
	}
}

PINVOKE void Unsafe_ComputeHull(double* pts, size_t nPoints,
	int* &triangles, int& nTriangles) {

	convex_hull hull(pts, nPoints);
	nTriangles = hull.numTriangles();
	triangles = new int[(size_t)nTriangles * 3];
	hull.getAllTriangles(triangles);
}

void convex_hull::compute() {
	// throw "Not implemented yet.";
	_triangles.push_back(triangle(0, 0, 1, 2));
}