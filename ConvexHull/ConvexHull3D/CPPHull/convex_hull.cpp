#include "convex_hull.h"

convex_hull::convex_hull(double* pts, size_t nPts) {
	_pts = new vec3[nPts];
	_outsidePts = std::unordered_set<size_t>();
	for (size_t i = 0; i < nPts; i++)
	{
		_pts[i].x = pts[3 * i];
		_pts[i].y = pts[3 * i + 1];
		_pts[i].z = pts[3 * i + 2];
		_outsidePts.insert(i);
	}
	_nPts = nPts;

	_center = vec3::average(_pts, _nPts);

	_triangles = std::unordered_map<size_t, triangle>();
	_edgeFaceMap = std::unordered_map<indexPair, std::unordered_set<size_t>>();
	compute();
}

convex_hull::~convex_hull() {
	_triangles.clear();
	_outsidePts.clear();
	/*auto iter = _edgeFaceMap.begin();
	while (iter != _edgeFaceMap.end()) {
		iter->second.clear();
		iter++;
	}*/
	_edgeFaceMap.clear();
	delete[] _pts;
}

vec3 convex_hull::getPt(size_t index) const {
	return (index < 0 || index > _nPts) ? vec3::unset : _pts[index];
}

size_t convex_hull::numTriangles() const {
	return _triangles.size();
}

void convex_hull::getAllTriangles(int* triIndices) {
	std::unordered_map<size_t, triangle>::iterator iter = _triangles.begin();
	int i = 0;
	while(iter != _triangles.end())
	{
		triIndices[i++] = iter->second.a;
		triIndices[i++] = iter->second.b;
		triIndices[i++] = iter->second.c;
		iter++;
	}
}

double convex_hull::trianglePlaneDist(size_t iTri, const vec3& pt, triangle& tri){
	auto match = _triangles.find(iTri);
	if (match == _triangles.end()) {
		tri = triangle(-1, -1, -1, -1);
		return doubleMinValue;
	}
	tri = match->second;

	return (pt - _pts[tri.a]) * tri.normal;
}

void convex_hull::setTriangle(triangle& tri) {
	tri.normal = ((_pts[tri.b] - _pts[tri.a]) ^ (_pts[tri.c] - _pts[tri.a])).unit();
	if (isTriangleFacing(tri, _center)) {
		tri.flip();
	}

	_triangles.insert_or_assign(tri.index, tri);

	for (char ei = 0; ei < 3; ei++)
	{
		_edgeFaceMap[tri.edge(ei)].insert(tri.index);
	}
}

triangle convex_hull::popTriangle(size_t index, indexPair edges[3],
	size_t adjTriangles[3]) {

	triangle tri;
	auto match = _triangles.find(index);
	if (match != _triangles.end()) {
		tri = match->second;
		std::unordered_set<size_t> triSet;
		_triangles.erase(index);
		indexPair edge;
		for (char ei = 0; ei < 3; ei++)
		{
			edge = tri.edge(ei);
			_edgeFaceMap[edge].erase(index);
			triSet = _edgeFaceMap[edge];
			adjTriangles[ei] = triSet.size() == 1 ? *triSet.begin() : -1;
			edges[ei] = edge;
		}
	}

	return tri;
}

bool convex_hull::isTriangleFacing(size_t iTri, const vec3& pt, triangle& tri) {
	auto match = _triangles.find(iTri);
	if (match == _triangles.end()) {
		tri = triangle(-1, -1, -1, -1);
		return false;
	}
	tri = match->second;
	return ((tri.normal * (pt - _pts[tri.a])) > 0);
}

bool convex_hull::isTriangleFacing(const triangle& tri, const vec3& pt) const {
	return tri.normal.isValid() ? ((tri.normal * (pt - _pts[tri.a])) > 0) :
		false;
}

double convex_hull::triangleSolidAngle(const triangle& tri, const vec3& pt) const {
	return vec3::solidAngle(_pts[tri.a] - pt, _pts[tri.b] - pt, _pts[tri.c] - pt);
}

size_t convex_hull::farthestPoint(size_t iTri, triangle& tri) {
	size_t farthest = -1;
	double dMax = 1e-10, dist;
	for (const size_t& i : _outsidePts) {
		dist = trianglePlaneDist(iTri, _pts[i], tri);
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

void convex_hull::updateInteriorPoints(std::vector<size_t>::iterator newTrStart,
	std::vector<size_t>::iterator newTrEnd, std::vector<size_t>::iterator popStart,
	std::vector<size_t>::iterator popEnd) {
	
	std::vector<size_t> remove, check;
	auto iter1 = _outsidePts.begin();
	std::vector<size_t>::iterator iter2;
	std::unordered_map<size_t, triangle>::iterator match;
	bool outside;
	while (iter1 != _outsidePts.end()) {
		outside = false;
		iter2 = popStart;
		while (iter2 != popEnd) {
			match = _triangles.find(*iter2);
			if (match == _triangles.end()) {
				iter2++;
				continue;
			}
			if (*iter1 == match->second.a || *iter1 == match->second.b || *iter1 == match->second.c) {
				remove.push_back(*iter1);
			}
			if ((match->second.normal * (_pts[*iter1] - _pts[match->second.a])) > 1e-10) {
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
			match = _triangles.find(*iter2);
			if (match == _triangles.end()) {
				iter2++;
				continue;
			}
			if ((match->second.normal * (_pts[*itCheck] - _pts[match->second.a])) > 1e-10) {
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
		_outsidePts.erase(*iter3);
		iter3++;
	}
}

PINVOKE void Unsafe_ComputeHull(double* pts, size_t nPoints,
	int*& triangles, int& nTriangles) {

	convex_hull hull(pts, nPoints);
	nTriangles = hull.numTriangles();
	triangles = new int[nTriangles * 3];
	hull.getAllTriangles(triangles);
}

void convex_hull::createInitialSimplex(size_t& triI) {
	size_t best[4];
	if (_nPts < 4) {
		throw "Cannot create the initial simplex";
	}
	else if (_nPts == 4) {
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
		for (size_t pi = 0; pi < _nPts; pi++)
		{
			_pts[pi].copyTo(coords);
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
			pt = _pts[bounds[i]];
			for (size_t j = i + 1; j < 6; j++)
			{
				dist = (pt - _pts[bounds[j]]).lenSq();
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
		vec3 ref = _pts[best[0]];
		vec3 uDir = (_pts[best[1]] - ref).unit();
		for (size_t pi = 0; pi < _nPts; pi++)
		{
			dist = ((_pts[pi] - ref) - uDir * (uDir * (_pts[pi] - ref))).lenSq();
			if (dist > maxD) {
				best[2] = pi;
				maxD = dist;
			}
		}

		if (maxD <= 0) {
			throw "Failed to create initial simplex";
		}

		maxD = doubleMinValue;
		uDir = ((_pts[best[1]] - ref) ^ (_pts[best[2]] - ref)).unit();
		for (size_t pi = 0; pi < _nPts; pi++)
		{
			dist = abs(uDir * (_pts[pi] - ref));
			if (dist > maxD) {
				best[3] = pi;
				maxD = dist;
			}
		}

		if (maxD <= 0) {
			throw "Failed to create initial simplex";
		}
	}

	triangle simplex[4];
	simplex[0] = triangle(triI++, best[0], best[1], best[2]);
	simplex[1] = triangle(triI++, best[0], best[2], best[3]);
	simplex[2] = triangle(triI++, best[1], best[2], best[3]);
	simplex[3] = triangle(triI++, best[0], best[1], best[3]);

	_center = vec3::zero;
	for (size_t i = 0; i < 4; i++)
	{
		_center += _pts[best[i]];
	}
	_center /= 4;

	std::vector<size_t> newTris;
	newTris.reserve(4);
	for (size_t i = 0; i < 4; i++)
	{
		if (!simplex[i].isValid()) {
			continue;
		}
		setTriangle(simplex[i]);
		newTris.push_back(simplex[i].index);
	}

	updateInteriorPoints(newTris.begin(), newTris.end(), newTris.begin(), newTris.end());
}

void convex_hull::compute() {
	size_t curTriIndex = 0;
	createInitialSimplex(curTriIndex);
	std::queue<size_t> gQue = std::queue<size_t>();
	auto iter = _triangles.begin();
	while (iter != _triangles.end()) {
		gQue.push(iter->first);
		iter++;
	}

	size_t iTri, fpi;
	vec3 fpt, normal;
	triangle cTri, tri2, newTri, popTri;
	size_t adjTri[3];
	indexPair adjEdges[3];
	std::queue<size_t> popQ = std::queue<size_t>();
	std::vector<indexPair> horizonIndices = std::vector<indexPair>();
	std::vector<indexPair>::iterator edgeIter;
	std::vector<size_t> newTriangles, popped;
	vec3 avg;
	while (!gQue.empty())
	{
		iTri = gQue.front();
		gQue.pop();
		fpi = farthestPoint(iTri, cTri);
		normal = cTri.normal;
		if (fpi == -1 || !cTri.isValid()) {
			continue;
		}
		fpt = _pts[fpi];
		popQ.push(iTri);

		horizonIndices.clear();
		popped.clear();
		while (!popQ.empty())
		{
			popTri = popTriangle(popQ.front(), adjEdges, adjTri);
			popQ.pop();

			if (!popTri.isValid()) {
				continue;
			}

			popped.push_back(popTri.index);

			for (size_t ti = 0; ti < 3; ti++)
			{
				if (adjTri[ti] == -1) {
					continue;
				}
				if (isTriangleFacing(adjTri[ti], fpt, tri2)) {
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
			avg += getPt(edgeIter->p);
			avg += getPt(edgeIter->q);
			edgeIter++;
		}
		avg /= 2 * horizonIndices.size();

		newTriangles.clear();
		newTriangles.reserve(horizonIndices.size());
		edgeIter = horizonIndices.begin();
		while (edgeIter != horizonIndices.end())
		{
			newTri = triangle(curTriIndex++, fpi, edgeIter->p, edgeIter->q);
			if (((getPt(edgeIter->p) - fpt) ^ (getPt(edgeIter->q) - fpt)) * (avg - fpt) > 0) {
				newTri.flip();
			}
			setTriangle(newTri);
			newTriangles.push_back(newTri.index);
			gQue.push(newTri.index);
			edgeIter++;
		}
		updateInteriorPoints(newTriangles.begin(), newTriangles.end(), popped.begin(), popped.end());
	}
}