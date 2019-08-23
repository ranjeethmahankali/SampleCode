#pragma once

#include "base.h"
#include <queue>
#define _USE_MATH_DEFINES
#include <math.h>

constexpr double PLANE_DIST_TOL = 1e-10;

class convex_hull
{
private:
	std::unordered_map<size_t, tri_face, custom_size_t_hash, std::equal_to<size_t>> m_faces;
	std::unordered_set<size_t, custom_size_t_hash, std::equal_to<size_t>> m_outsidePts;
	std::unordered_map<index_pair, index_pair, index_pair_hash, std::equal_to<index_pair>> m_edgeFaceMap;

	vec3 *m_pts, m_center;
	size_t m_nPts;

	void compute();
	
	void set_face(tri_face& tri);
	tri_face pop_face(size_t index, index_pair edges[3],
		tri_face adjTriangles[3]);
	bool is_face_visible(const tri_face& tri, const vec3& pt) const;
	double face_plane_dist(const tri_face& tri, const vec3& pt);
	size_t farthest_pt(const tri_face& tri_face);
	double face_solid_angle(const tri_face& tri, const vec3& pt) const;
	void update_interior_points(const std::vector<size_t>& newFaceIndices, const std::vector<tri_face>& poppedFaces);
	void create_initial_simplex(size_t& triIndex);
	bool get_face(size_t fi, tri_face& face);
	bool get_edge_faces(index_pair edge, index_pair& faces);
	bool get_edge_faces(index_pair edge, tri_face& face1, tri_face& face2);
	vec3 face_center(const tri_face& face);

public:
	convex_hull(double* pts, size_t nPts);
	~convex_hull();

	vec3 get_pt(size_t index) const;
	size_t num_faces() const;
	void get_all_faces(int* triIndices);
};

PINVOKE void Unsafe_ComputeHull(double* pts, size_t numPoints,
	int*& triangles, int& nTriangles);