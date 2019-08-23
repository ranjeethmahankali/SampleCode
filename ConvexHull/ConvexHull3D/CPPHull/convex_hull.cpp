#include "convex_hull.h"

convex_hull::convex_hull(double* pts, size_t nPts) {
	m_pts = new vec3[nPts];
	m_outsidePts = std::unordered_set<size_t, custom_size_t_hash, std::equal_to<size_t>>();
	for (size_t i = 0; i < nPts; i++)
	{
		m_pts[i].x = pts[3 * i];
		m_pts[i].y = pts[3 * i + 1];
		m_pts[i].z = pts[3 * i + 2];
		m_outsidePts.insert(i);
	}
	m_nPts = nPts;

	m_center = vec3::average(m_pts, m_nPts);

	m_faces = std::unordered_map<size_t, tri_face, custom_size_t_hash, std::equal_to<size_t>>();
	m_edgeFaceMap = std::unordered_map<index_pair, index_pair, index_pair_hash, std::equal_to<index_pair>>();
	compute();
}

convex_hull::~convex_hull() {
	m_faces.clear();
	m_outsidePts.clear();
	m_edgeFaceMap.clear();
	delete[] m_pts;
}

vec3 convex_hull::get_pt(size_t index) const {
	return (index < 0 || index > m_nPts) ? vec3::unset : m_pts[index];
}

size_t convex_hull::num_faces() const {
	return m_faces.size();
}

void convex_hull::get_all_faces(int* triIndices) {
	std::unordered_map<size_t, tri_face>::iterator iter = m_faces.begin();
	int i = 0;
	while(iter != m_faces.end())
	{
		triIndices[i++] = iter->second.a;
		triIndices[i++] = iter->second.b;
		triIndices[i++] = iter->second.c;
		iter++;
	}
}

double convex_hull::face_plane_dist(const tri_face& face, const vec3& pt){
	return (pt - m_pts[face.a]) * face.normal;
}

void convex_hull::set_face(tri_face& tri) {
	tri.normal = ((m_pts[tri.b] - m_pts[tri.a]) ^ (m_pts[tri.c] - m_pts[tri.a])).unit();
	if (is_face_visible(tri, m_center)) {
		tri.flip();
	}

	m_faces.insert_or_assign(tri.index, tri);

	for (char ei = 0; ei < 3; ei++)
	{
		if (!m_edgeFaceMap[tri.edge(ei)].set(tri.index)) {
			throw "failed to set the face to the edge.";
		}
	}
}

tri_face convex_hull::pop_face(size_t index, index_pair edges[3],
	tri_face adjFaces[3]) {

	tri_face tri;
	auto match = m_faces.find(index);
	if (match != m_faces.end()) {
		tri = match->second;
		m_faces.erase(index);
		index_pair edge;
		index_pair fPair;
		size_t adjFi;
		for (char ei = 0; ei < 3; ei++)
		{
			edge = tri.edge(ei);
			edges[ei] = edge;
			if (!get_edge_faces(edge, fPair) || !fPair.contains(index)) {
				continue;
			}
			fPair.unset(index);
			m_edgeFaceMap[edge] = fPair;
			adjFi = fPair.p == -1 ? fPair.q : fPair.p;
			if (!get_face(adjFi, adjFaces[ei])) {
				adjFaces[ei] = tri_face::unset;
			}
		}
	}

	return tri;
}

bool convex_hull::is_face_visible(const tri_face& tri, const vec3& pt) const {
	return tri.normal.is_valid() ? ((tri.normal * (pt - m_pts[tri.a])) > 0) :
		false;
}

double convex_hull::face_solid_angle(const tri_face& tri, const vec3& pt) const {
	return vec3::solid_angle(m_pts[tri.a] - pt, m_pts[tri.b] - pt, m_pts[tri.c] - pt);
}

size_t convex_hull::farthest_pt(const tri_face& face) {
	size_t farthest = -1;
	double dMax = PLANE_DIST_TOL, dist;
	for (const size_t& i : m_outsidePts) {
		dist = face_plane_dist(face, m_pts[i]);
		if (i == face.a || i == face.b || i == face.c) {
			continue;
		}
		if (dist > dMax) {
			dMax = dist;
			farthest = i;
		}
	}
	return farthest;
}

void convex_hull::update_interior_points(const std::vector<size_t>& newFaces, const std::vector<tri_face>& poppedFaces) {
	std::vector<size_t> remove, check;
	auto iter1 = m_outsidePts.begin();
	std::vector<tri_face>::const_iterator iter2;
	std::vector<size_t>::const_iterator iter3;
	tri_face face;
	vec3 testPt;
	bool outside;
	while (iter1 != m_outsidePts.end()) {
		outside = false;
		testPt = m_pts[*iter1];
		iter2 = poppedFaces.begin();
		while (iter2 != poppedFaces.end()) {
			if (*iter1 == iter2->a || *iter1 == iter2->b || *iter1 == iter2->c) {
				remove.push_back(*iter1);
			}
			if ((iter2->normal * (testPt - m_pts[iter2->a])) > PLANE_DIST_TOL) {
				outside = true;
				break;
			}
			iter2++;
		}

		if (outside) {
			check.push_back(*iter1);
		}

		iter1++;
	}

	auto itCheck = check.begin();
	while (itCheck != check.end()) {
		outside = false;
		iter3 = newFaces.begin();
		while (iter3 != newFaces.end()) {
			if (!get_face(*iter3, face)) {
				continue;
			}
			if ((face.normal * (m_pts[*itCheck] - m_pts[face.a])) > PLANE_DIST_TOL) {
				outside = true;
				break;
			}
			iter3++;
		}

		if (!outside) {
			remove.push_back(*itCheck);
		}

		itCheck++;
	}

	auto iter4 = remove.begin();
	while (iter4 != remove.end())
	{
		m_outsidePts.erase(*iter4);
		iter4++;
	}
}

void convex_hull::create_initial_simplex(size_t& ti) {
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

	std::vector<size_t> newTris;
	newTris.reserve(4);
	for (size_t i = 0; i < 4; i++)
	{
		if (!simplex[i].is_valid()) {
			continue;
		}
		set_face(simplex[i]);
		newTris.push_back(simplex[i].index);
	}

	update_interior_points(newTris, std::vector<tri_face>());
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

bool convex_hull::get_edge_faces(index_pair edge, tri_face& face1, tri_face& face2)
{
	index_pair fPair;
	if (get_edge_faces(edge, fPair)) {
		return get_face(fPair.p, face1) && get_face(fPair.q, face2);
	}
	return false;
}

vec3 convex_hull::face_center(const tri_face& face)
{
	return (m_pts[face.a] + m_pts[face.b] + m_pts[face.c]) / 3;
}

void convex_hull::compute() {
	size_t curTi = 0;
	create_initial_simplex(curTi);
	std::queue<size_t> gQue = std::queue<size_t>();
	auto iter = m_faces.begin();
	while (iter != m_faces.end()) {
		gQue.push(iter->first);
		iter++;
	}

	size_t ti, fpi;
	vec3 fpt, normal;
	tri_face cTri, tri2, newFace, popTri;
	tri_face adjTri[3];
	index_pair adjEdges[3];
	std::queue<size_t> popQ = std::queue<size_t>();
	std::vector<index_pair> horizonIndices = std::vector<index_pair>();
	std::vector<index_pair>::iterator edgeIter;
	std::vector<size_t> newFaceIndices;
	std::vector<tri_face> popped;
	vec3 avg;
	while (!gQue.empty())
	{
		ti = gQue.front();
		gQue.pop();
		if (!get_face(ti, cTri)) {
			continue;
		}
		fpi = farthest_pt(cTri);
		normal = cTri.normal;
		if (fpi == -1) {
			continue;
		}
		fpt = m_pts[fpi];
		popQ.push(ti);

		horizonIndices.clear();
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
				if (is_face_visible(adjTri[ti], fpt)) {
					popQ.push(adjTri[ti].index);
				}
				else {
					horizonIndices.push_back(adjEdges[ti]);
				}
			}
		}

		edgeIter = horizonIndices.begin();
		avg = face_center(cTri);

		newFaceIndices.clear();
		newFaceIndices.reserve(horizonIndices.size());
		edgeIter = horizonIndices.begin();
		while (edgeIter != horizonIndices.end())
		{
			newFace = tri_face(curTi++, fpi, edgeIter->p, edgeIter->q);
			set_face(newFace);
			newFaceIndices.push_back(newFace.index);
			gQue.push(newFace.index);
			edgeIter++;
		}
		update_interior_points(newFaceIndices, popped);
	}
}

PINVOKE void Unsafe_ComputeHull(double* pts, size_t nPoints,
	int*& triangles, int& nTriangles) {

	convex_hull hull(pts, nPoints);
	nTriangles = hull.num_faces();
	triangles = new int[hull.num_faces() * 3];
	hull.get_all_faces(triangles);
}