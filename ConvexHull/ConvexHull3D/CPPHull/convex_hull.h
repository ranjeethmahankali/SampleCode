#pragma once

#include "base.h"
#include <queue>
#define _USE_MATH_DEFINES
#include <math.h>

constexpr double PLANE_DIST_TOL = 1e-10;

class convex_hull
{
private:
	std::unordered_map<size_t, tri_face> m_faces;
	std::unordered_set<size_t> m_outsidePts;
	std::unordered_map<index_pair, std::unordered_set<size_t>> m_edgeFaceMap;

	vec3 *m_pts, m_center;
	size_t m_nPts;

	void compute();
	
	void set_face(tri_face& tri);
	tri_face pop_face(size_t index, index_pair edges[3],
		size_t adjTriangles[3]);
	bool is_face_visible(size_t iTri, const vec3& pt, tri_face& tri);
	bool is_face_visible(const tri_face& tri, const vec3& pt) const;
	double face_plane_dist(size_t iTri, const vec3& pt, tri_face& tri);
	size_t farthest_pt(size_t iTri, tri_face& tri_face);
	double face_solid_angle(const tri_face& tri, const vec3& pt) const;
	void update_interior_points(std::vector<size_t>::iterator newTrStart,
		std::vector<size_t>::iterator newTrEnd, std::vector<size_t>::iterator popStart,
		std::vector<size_t>::iterator popEnd);
	void create_initial_simplex(size_t& triIndex);

public:
	convex_hull(double* pts, size_t nPts);
	~convex_hull();

	vec3 get_pt(size_t index) const;
	size_t num_faces() const;
	void get_all_faces(int* triIndices);
};

PINVOKE void Unsafe_ComputeHull(double* pts, size_t numPoints,
	int*& triangles, int& nTriangles);