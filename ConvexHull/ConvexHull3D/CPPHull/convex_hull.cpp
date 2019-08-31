#include "convex_hull.h"

convex_hull::convex_hull(double* pts, size_t nPts) 
{
	m_pts.reserve(nPts);
	for (size_t i = 0; i < nPts; i++)
	{
		m_pts.push_back(vec3(pts[3 * i], pts[3 * i + 1], pts[3 * i + 2]));
		m_outsidePts.insert(i);
	}
	m_nPts = nPts;
	m_center = vec3::average(m_pts);
	compute();
}

vec3 convex_hull::get_pt(size_t id) const 
{
	return (id < 0 || id > m_nPts) ? vec3::unset : m_pts[id];
}

size_t convex_hull::num_faces() const 
{
	return m_faces.size();
}

void convex_hull::copy_faces(int* triIndices) 
{
	int i = 0;
	for (const auto& pair : m_faces)
	{
		triIndices[i++] = pair.second.a;
		triIndices[i++] = pair.second.b;
		triIndices[i++] = pair.second.c;
	}
}

double convex_hull::face_plane_dist(const tri_face& face, const vec3& pt) const
{
	return (pt - m_pts[face.a]) * face.normal;
}

void convex_hull::set_face(tri_face& tri) 
{
	tri.normal = ((m_pts[tri.b] - m_pts[tri.a]) ^ (m_pts[tri.c] - m_pts[tri.a])).unit();
	if (face_visible(tri, m_center)) {
		tri.flip();
	}

	m_faces.insert_or_assign(tri.id, tri);

	for (char ei = 0; ei < 3; ei++)
	{
		if (!m_edgeFaceMap[tri.edge(ei)].add(tri.id)) {
			throw "failed to add the face to the edge.";
		}
	}
}

tri_face convex_hull::pop_face(size_t id, index_pair edges[3], tri_face adjFaces[3])
{
	tri_face face;
	if (get_face(id, face)) {
		m_faces.erase(id);
		index_pair edge;
		index_pair fPair;
		size_t adjFi;
		for (char ei = 0; ei < 3; ei++)
		{
			edge = face.edge(ei);
			edges[ei] = edge;
			if (!get_edge_faces(edge, fPair) || !fPair.contains(id)) {
				continue;
			}
			fPair.unset(id);
			m_edgeFaceMap[edge] = fPair;
			adjFi = fPair.p == -1 ? fPair.q : fPair.p;
			if (!get_face(adjFi, adjFaces[ei])) {
				adjFaces[ei] = tri_face::unset;
			}
		}
	}

	return face;
}

bool convex_hull::face_visible(const tri_face& tri, const vec3& pt) const
{
	return tri.normal.is_valid() ? (face_plane_dist(tri, pt) > PLANE_DIST_TOL) : false;
}

bool convex_hull::get_farthest_pt(const tri_face& face, vec3& pt, size_t& ptIndex)
{
	ptIndex = -1;
	pt = vec3::unset;
	double dMax = PLANE_DIST_TOL, dist;
	for (const size_t& i : m_outsidePts) {
		dist = face_plane_dist(face, m_pts[i]);
		if (i == face.a || i == face.b || i == face.c) {
			continue;
		}
		if (dist > dMax) {
			dMax = dist;
			ptIndex = i;
			pt = m_pts[i];
		}
	}

	return ptIndex != -1;
}

void convex_hull::update_exterior_points(const std::vector<tri_face>& newFaces, const std::vector<tri_face>& poppedFaces)
{
	std::vector<size_t> remove, check;
	vec3 testPt;
	bool outside;
	for (const size_t& opi : m_outsidePts)
	{
		outside = false;
		testPt = m_pts[opi];
		for (const tri_face& popFace : poppedFaces)
		{
			if (popFace.contains_vertex(opi)) {
				remove.push_back(opi);
			}
			if (face_visible(popFace, testPt)) {
				outside = true;
				break;
			}
		}

		if (outside) {
			check.push_back(opi);
		}
	}

	for (const size_t& ci : check)
	{
		outside = false;
		testPt = m_pts[ci];
		for (const tri_face& newFace : newFaces)
		{
			if (face_visible(newFace, testPt)) {
				outside = true;
				break;
			}
		}

		if (!outside) {
			remove.push_back(ci);
		}
	}

	for (const size_t& ri : remove) {
		m_outsidePts.erase(ri);
	}
}

void convex_hull::create_initial_simplex(size_t& ti)
{
	size_t best[4];
	if (m_nPts < 4) {
		throw "Cannot create the initial simplex";
	}
	else if (m_nPts == 4) {
		for (size_t i = 0; i < 4; i++)
		{
			best[i] = i;
		}
	}
	else {
		double extremes[6];
		for (size_t ei = 0; ei < 6; ei++)
		{
			extremes[ei] = ei % 2 == 0 ? doubleMaxValue : doubleMinValue;
		}

		size_t bounds[6] = {-1, -1, -1, -1, -1, -1};
		double coords[3];
		for (size_t pi = 0; pi < m_nPts; pi++)
		{
			m_pts[pi].copy(coords);
			for (size_t ei = 0; ei < 6; ei++)
			{
				if (ei % 2 == 0 && extremes[ei] > coords[ei / 2]) {
					extremes[ei] = coords[ei / 2];
					bounds[ei] = pi;
				}
				else if (ei % 2 == 1 && extremes[ei] < coords[ei / 2]) {
					extremes[ei] = coords[ei / 2];
					bounds[ei] = pi;
				}
			}
		}

		vec3 pt;
		double maxD = doubleMinValue, dist;
		for (size_t i = 0; i < 6; i++)
		{
			pt = m_pts[bounds[i]];
			for (size_t j = i + 1; j < 6; j++)
			{
				dist = (pt - m_pts[bounds[j]]).len_sq();
				if (dist > maxD) {
					best[0] = bounds[i];
					best[1] = bounds[j];
					maxD = dist;
				}
			}
		}

		if (maxD <= 0) {
			throw "Failed to create initial simplex";
		}

		maxD = doubleMinValue;
		vec3 ref = m_pts[best[0]];
		vec3 uDir = (m_pts[best[1]] - ref).unit();
		for (size_t pi = 0; pi < m_nPts; pi++)
		{
			dist = ((m_pts[pi] - ref) - uDir * (uDir * (m_pts[pi] - ref))).len_sq();
			if (dist > maxD) {
				best[2] = pi;
				maxD = dist;
			}
		}

		if (maxD <= 0) {
			throw "Failed to create initial simplex";
		}

		maxD = doubleMinValue;
		uDir = ((m_pts[best[1]] - ref) ^ (m_pts[best[2]] - ref)).unit();
		for (size_t pi = 0; pi < m_nPts; pi++)
		{
			dist = abs(uDir * (m_pts[pi] - ref));
			if (dist > maxD) {
				best[3] = pi;
				maxD = dist;
			}
		}

		if (maxD <= 0) {
			throw "Failed to create initial simplex";
		}
	}

	tri_face simplex[4];
	simplex[0] = tri_face(ti++, best[0], best[1], best[2]);
	simplex[1] = tri_face(ti++, best[0], best[2], best[3]);
	simplex[2] = tri_face(ti++, best[1], best[2], best[3]);
	simplex[3] = tri_face(ti++, best[0], best[1], best[3]);

	m_center = vec3::zero;
	for (size_t i = 0; i < 4; i++)
	{
		m_center += m_pts[best[i]];
	}
	m_center /= 4;

	for (size_t i = 0; i < 4; i++)
	{
		if (!simplex[i].is_valid()) {
			continue;
		}
		set_face(simplex[i]);
	}

	std::vector<size_t> removePts;
	bool outside;
	for (const size_t& opi : m_outsidePts)
	{
		outside = false;
		for (size_t i = 0; i < 4; i++)
		{
			if (simplex[i].contains_vertex(opi)) {
				removePts.push_back(opi);
				break;
			}
			if (face_visible(simplex[i], get_pt(opi))) {
				outside = true;
				break;
			}
		}

		if (!outside) {
			removePts.push_back(opi);
		}
	}

	for (const size_t& rpt : removePts)
	{
		m_outsidePts.erase(rpt);
	}
}

bool convex_hull::get_face(size_t fi, tri_face& face)
{
	auto match = m_faces.find(fi);
	if (match != m_faces.end()) {
		face = match->second;
		return true;
	}
	return false;
}

bool convex_hull::get_edge_faces(index_pair edge, index_pair& faces)
{
	auto match = m_edgeFaceMap.find(edge);
	if (match != m_edgeFaceMap.end()) {
		faces = match->second;
		return true;
	}
	return false;
}

vec3 convex_hull::face_center(const tri_face& face)
{
	return (m_pts[face.a] + m_pts[face.b] + m_pts[face.c]) / 3;
}

void convex_hull::compute() 
{
	size_t curTi = 0;
	create_initial_simplex(curTi);
	std::queue<size_t> faceQ;
	for (const auto& f : m_faces)
	{
		faceQ.push(f.first);
	}

	size_t ti, fpi;
	vec3 fpt, normal;
	tri_face curFace, newFace, popTri;
	tri_face adjTri[3];
	index_pair adjEdges[3];
	std::queue<size_t> popQ;
	std::vector<index_pair> horizonEdges;
	std::vector<tri_face> newFaces, popped;
	vec3 avg;
	while (!faceQ.empty())
	{
		ti = faceQ.front();
		faceQ.pop();
		if (!get_face(ti, curFace) || !get_farthest_pt(curFace, fpt, fpi)) {
			continue;
		}
		normal = curFace.normal;
		popQ.push(ti);

		horizonEdges.clear();
		popped.clear();
		while (!popQ.empty())
		{
			popTri = pop_face(popQ.front(), adjEdges, adjTri);
			popQ.pop();

			if (!popTri.is_valid()) {
				continue;
			}

			popped.push_back(popTri);

			for (size_t ti = 0; ti < 3; ti++)
			{
				if (!adjTri[ti].is_valid()) {
					continue;
				}
				if (face_visible(adjTri[ti], fpt)) {
					popQ.push(adjTri[ti].id);
				}
				else {
					horizonEdges.push_back(adjEdges[ti]);
				}
			}
		}

		avg = face_center(curFace);
		newFaces.clear();
		newFaces.reserve(horizonEdges.size());
		for (const index_pair& hei : horizonEdges)
		{
			newFace = tri_face(curTi++, fpi, hei.p, hei.q);
			set_face(newFace);
			newFaces.push_back(newFace);
			faceQ.push(newFace.id);
		}
		update_exterior_points(newFaces, popped);
	}
}

PINVOKE void Unsafe_ComputeHull(double* pts, size_t nPoints,
	int*& triangles, int& nTriangles) {

	convex_hull hull(pts, nPoints);
	nTriangles = hull.num_faces();
	triangles = new int[hull.num_faces() * 3];
	hull.copy_faces(triangles);
}