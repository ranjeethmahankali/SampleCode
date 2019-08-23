#include "convex_hull.h"

convex_hull::convex_hull(double* pts, size_t nPts) {
	m_pts = new vec3[nPts];
	m_outsidePts = std::unordered_set<size_t>();
	for (size_t i = 0; i < nPts; i++)
	{
		m_pts[i].x = pts[3 * i];
		m_pts[i].y = pts[3 * i + 1];
		m_pts[i].z = pts[3 * i + 2];
		m_outsidePts.insert(i);
	}
	m_nPts = nPts;

	m_center = vec3::average(m_pts, m_nPts);

	m_faces = std::unordered_map<size_t, tri_face>();
	m_edgeFaceMap = std::unordered_map<index_pair, std::unordered_set<size_t>>();
	compute();
}

convex_hull::~convex_hull() {
	m_faces.clear();
	m_outsidePts.clear();
	/*auto iter = _edgeFaceMap.begin();
	while (iter != _edgeFaceMap.end()) {
		iter->second.clear();
		iter++;
	}*/
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

double convex_hull::face_plane_dist(size_t iTri, const vec3& pt, tri_face& tri){
	auto match = m_faces.find(iTri);
	if (match == m_faces.end()) {
		tri = tri_face(-1, -1, -1, -1);
		return doubleMinValue;
	}
	tri = match->second;

	return (pt - m_pts[tri.a]) * tri.normal;
}

void convex_hull::set_face(tri_face& tri) {
	tri.normal = ((m_pts[tri.b] - m_pts[tri.a]) ^ (m_pts[tri.c] - m_pts[tri.a])).unit();
	if (is_face_visible(tri, m_center)) {
		tri.flip();
	}

	m_faces.insert_or_assign(tri.index, tri);

	for (char ei = 0; ei < 3; ei++)
	{
		m_edgeFaceMap[tri.edge(ei)].insert(tri.index);
	}
}

tri_face convex_hull::pop_face(size_t index, index_pair edges[3],
	size_t adjTriangles[3]) {

	tri_face tri;
	auto match = m_faces.find(index);
	if (match != m_faces.end()) {
		tri = match->second;
		std::unordered_set<size_t> triSet;
		m_faces.erase(index);
		index_pair edge;
		for (char ei = 0; ei < 3; ei++)
		{
			edge = tri.edge(ei);
			m_edgeFaceMap[edge].erase(index);
			triSet = m_edgeFaceMap[edge];
			adjTriangles[ei] = triSet.size() == 1 ? *triSet.begin() : -1;
			edges[ei] = edge;
		}
	}

	return tri;
}

bool convex_hull::is_face_visible(size_t iTri, const vec3& pt, tri_face& tri) {
	auto match = m_faces.find(iTri);
	if (match == m_faces.end()) {
		tri = tri_face(-1, -1, -1, -1);
		return false;
	}
	tri = match->second;
	return ((tri.normal * (pt - m_pts[tri.a])) > 0);
}

bool convex_hull::is_face_visible(const tri_face& tri, const vec3& pt) const {
	return tri.normal.is_valid() ? ((tri.normal * (pt - m_pts[tri.a])) > 0) :
		false;
}

double convex_hull::face_solid_angle(const tri_face& tri, const vec3& pt) const {
	return vec3::solid_angle(m_pts[tri.a] - pt, m_pts[tri.b] - pt, m_pts[tri.c] - pt);
}

size_t convex_hull::farthest_pt(size_t iTri, tri_face& tri) {
	size_t farthest = -1;
	double dMax = PLANE_DIST_TOL, dist;
	for (const size_t& i : m_outsidePts) {
		dist = face_plane_dist(iTri, m_pts[i], tri);
		if (i == tri.a || i == tri.b || i == tri.c) {
			continue;
		}
		if (dist > dMax) {
			dMax = dist;
			farthest = i;
		}
	}
	return farthest;
}

void convex_hull::update_interior_points(std::vector<size_t>::iterator newTrStart,
	std::vector<size_t>::iterator newTrEnd, std::vector<size_t>::iterator popStart,
	std::vector<size_t>::iterator popEnd) {
	
	std::vector<size_t> remove, check;
	auto iter1 = m_outsidePts.begin();
	std::vector<size_t>::iterator iter2;
	std::unordered_map<size_t, tri_face>::iterator match;
	bool outside;
	while (iter1 != m_outsidePts.end()) {
		outside = false;
		iter2 = popStart;
		while (iter2 != popEnd) {
			match = m_faces.find(*iter2);
			if (match == m_faces.end()) {
				iter2++;
				continue;
			}
			if (*iter1 == match->second.a || *iter1 == match->second.b || *iter1 == match->second.c) {
				remove.push_back(*iter1);
			}
			if ((match->second.normal * (m_pts[*iter1] - m_pts[match->second.a])) > PLANE_DIST_TOL) {
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
		iter2 = newTrStart;
		while (iter2 != newTrEnd) {
			match = m_faces.find(*iter2);
			if (match == m_faces.end()) {
				iter2++;
				continue;
			}
			if ((match->second.normal * (m_pts[*itCheck] - m_pts[match->second.a])) > PLANE_DIST_TOL) {
				outside = true;
				break;
			}
			iter2++;
		}

		if (!outside) {
			remove.push_back(*itCheck);
		}

		itCheck++;
	}

	auto iter3 = remove.begin();
	while (iter3 != remove.end())
	{
		m_outsidePts.erase(*iter3);
		iter3++;
	}
}

PINVOKE void Unsafe_ComputeHull(double* pts, size_t nPoints,
	int*& triangles, int& nTriangles) {

	convex_hull hull(pts, nPoints);
	nTriangles = hull.num_faces();
	triangles = new int[hull.num_faces() * 3];
	hull.get_all_faces(triangles);
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

	update_interior_points(newTris.begin(), newTris.end(), newTris.begin(), newTris.end());
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
	tri_face cTri, tri2, newTri, popTri;
	size_t adjTri[3];
	index_pair adjEdges[3];
	std::queue<size_t> popQ = std::queue<size_t>();
	std::vector<index_pair> horizonIndices = std::vector<index_pair>();
	std::vector<index_pair>::iterator edgeIter;
	std::vector<size_t> newTriangles, popped;
	vec3 avg;
	while (!gQue.empty())
	{
		ti = gQue.front();
		gQue.pop();
		fpi = farthest_pt(ti, cTri);
		normal = cTri.normal;
		if (fpi == -1 || !cTri.is_valid()) {
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

			popped.push_back(popTri.index);

			for (size_t ti = 0; ti < 3; ti++)
			{
				if (adjTri[ti] == -1) {
					continue;
				}
				if (is_face_visible(adjTri[ti], fpt, tri2)) {
					popQ.push(adjTri[ti]);
				}
				else {
					horizonIndices.push_back(adjEdges[ti]);
				}
			}
		}

		edgeIter = horizonIndices.begin();
		avg = vec3(0, 0, 0);
		while (edgeIter != horizonIndices.end()) {
			avg += get_pt(edgeIter->p);
			avg += get_pt(edgeIter->q);
			edgeIter++;
		}
		avg /= 2 * horizonIndices.size();

		newTriangles.clear();
		newTriangles.reserve(horizonIndices.size());
		edgeIter = horizonIndices.begin();
		while (edgeIter != horizonIndices.end())
		{
			newTri = tri_face(curTi++, fpi, edgeIter->p, edgeIter->q);
			if (((get_pt(edgeIter->p) - fpt) ^ (get_pt(edgeIter->q) - fpt)) * (avg - fpt) > 0) {
				newTri.flip();
			}
			set_face(newTri);
			newTriangles.push_back(newTri.index);
			gQue.push(newTri.index);
			edgeIter++;
		}
		update_interior_points(newTriangles.begin(), newTriangles.end(), popped.begin(), popped.end());
	}
}