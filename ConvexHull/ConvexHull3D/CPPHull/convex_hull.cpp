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

	_triangles = std::unordered_map<size_t, triangle>();
	_insidePts = std::unordered_set<size_t>();
	_edgeFaceMap = std::unordered_map<size_t, std::unordered_set<size_t>>();
	compute();
}

convex_hull::~convex_hull() {
	_triangles.clear();
	_insidePts.clear();
	_outsidePts.clear();
	/*auto iter = _edgeFaceMap.begin();
	while (iter != _edgeFaceMap.end()) {
		iter->second.clear();
		iter++;
	}*/
	_edgeFaceMap.clear();
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
	std::unordered_map<size_t, triangle>::iterator iter = _triangles.begin();
	int i = 0;
	while(iter != _triangles.end())
	{
		triangles[i++] = iter->second.a;
		triangles[i++] = iter->second.b;
		triangles[i++] = iter->second.c;
		iter++;
	}
}

double convex_hull::trianglePlaneDist(size_t iTri, vec3 pt, triangle &tri) {
	vec3 normal = triangleNormal(iTri, tri);
	if (!tri.isValid()) {
		return doubleMinValue;
	}

	return (pt - _pts[tri.a]) * normal;
}

void convex_hull::getEdgeIndices(triangle tri, size_t indices[3]) {
	indices[0] = std::min(tri.a, tri.b) * _nPts + std::max(tri.a, tri.b);
	indices[1] = std::min(tri.b, tri.c) * _nPts + std::max(tri.b, tri.c);
	indices[2] = std::min(tri.c, tri.a) * _nPts + std::max(tri.c, tri.a);
}

void convex_hull::setTriangle(triangle tri) {
	_triangles.insert_or_assign(tri.index, tri);
	size_t eis[3];
	getEdgeIndices(tri, eis);

	for (size_t ei = 0; ei < 3; ei++)
	{
		_edgeFaceMap[ei].insert(tri.index);
	}
}

triangle convex_hull::popTriangle(size_t index, size_t edgeIndices[3],
	size_t adjTriangles[3]) {

	triangle tri = _triangles[index];
	if (tri.isValid()) {
		std::unordered_set<size_t> triSet;
		_triangles.erase(index);
		getEdgeIndices(tri, edgeIndices);
		for (size_t ei = 0; ei < 3; ei++)
		{
			_edgeFaceMap[edgeIndices[ei]].erase(index);
			triSet = _edgeFaceMap[edgeIndices[ei]];
			adjTriangles[ei] = triSet.size() == 1 ? *triSet.begin() : -1;
		}
	}

	return tri;
}

vec3 convex_hull::triangleNormal(size_t iTri, triangle &tri) {
	tri = _triangles[iTri];
	return tri.isValid() ? triangleNormal(tri) : vec3::unset;
}

vec3 convex_hull::triangleNormal(triangle tri) {
	return ((_pts[tri.b] - _pts[tri.a]) ^ (_pts[tri.c] - _pts[tri.a])).unit();
}

bool convex_hull::isTriangleFacing(size_t iTri, vec3 pt, triangle &tri) {
	vec3 normal = triangleNormal(iTri, tri);
	return normal.isValid() ? ((normal * (pt - _pts[tri.a])) > 0) :
		false;
}

double convex_hull::triangleSolidAngle(triangle tri, vec3 pt) {
	return vec3::solidAngle(_pts[tri.a] - pt, _pts[tri.b] - pt, _pts[tri.c] - pt);
}

size_t convex_hull::farthestPoint(size_t iTri, triangle &tri) {
	size_t farthest = -1;
	double dMax = 0, dist;
	for (const size_t& i : _outsidePts) {
		dist = trianglePlaneDist(iTri, _pts[i], tri);
		if (dist > dMax) {
			dMax = dist;
			farthest = i;
		}
	}
	return farthest;
}

bool convex_hull::isInsideHull(vec3 pt) {
	triangle tri;
	double sign, angSum = 0;
	auto iter = _triangles.begin();
	while (iter != _triangles.end())
	{
		sign = isTriangleFacing(iter->first, pt, tri) ? 1 : -1;
		angSum += sign * triangleSolidAngle(tri, pt);
		iter++;
	}

	return std::abs(angSum - (4 * M_PI)) < 1e-6;
}

void convex_hull::updateInteriorPoints() {
	std::vector<size_t> remove;
	auto iter1 = _outsidePts.begin();
	while (iter1 != _outsidePts.end()) {
		if (isInsideHull(_pts[*iter1])) {
			remove.push_back(*iter1);
		}
		iter1++;
	}

	auto iter2 = remove.begin();
	while (iter2 != remove.end())
	{
		_outsidePts.erase(*iter2);
	}
}

void convex_hull::getVertIndicesForEdge(size_t edgeI, size_t& v1, size_t& v2) {
	v2 = edgeI % _nPts;
	v1 = (edgeI - v2) / _nPts;
}

PINVOKE void Unsafe_ComputeHull(double* pts, size_t nPoints,
	int* &triangles, int& nTriangles) {

	convex_hull hull(pts, nPoints);
	nTriangles = hull.numTriangles();
	triangles = new int[(size_t)nTriangles * 3];
	hull.getAllTriangles(triangles);
}

void convex_hull::createInitialSimplex() {
	throw "This is not implemented yet";
}

void convex_hull::compute() {
	size_t curTriIndex = 0;
	createInitialSimplex();
	std::queue<size_t> gQue = std::queue<size_t>();
	auto iter = _triangles.begin();
	while (iter != _triangles.end()) {
		gQue.push(iter->first);
		iter++;
	}

	size_t iTri, fpi;
	vec3 fpt, normal;
	triangle cTri, tri2, newTri;
	size_t adjTri[3], adjEdges[3];
	std::queue<size_t> popQ = std::queue<size_t>();
	std::vector<size_t> horizonIndices = std::vector<size_t>();
	std::vector<size_t>::iterator hIter;
	size_t ev1, ev2;
	while (!gQue.empty())
	{
		iTri = gQue.front();
		gQue.pop();
		fpi = farthestPoint(iTri, cTri);
		normal = triangleNormal(iTri, cTri);
		if (fpi == -1 || !cTri.isValid()) {
			continue;
		}
		fpt = _pts[fpi];
		popQ.push(iTri);

		horizonIndices.clear();
		while (!popQ.empty())
		{
			cTri = popTriangle(popQ.front(), adjEdges, adjTri);
			popQ.pop();

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

		hIter = horizonIndices.begin();
		while (hIter != horizonIndices.end())
		{
			getVertIndicesForEdge(*hIter, ev1, ev2);
			newTri = triangle(curTriIndex++, fpi, ev1, ev2);
			if (triangleNormal(newTri) * normal < 0) {
				newTri.flip();
			}
			setTriangle(newTri);
			hIter++;
		}
	}
}