import rhinoscriptsyntax as rs
import time

def IsValidTurn(pts):
	assert len(pts) == 3
	v1 = rs.VectorSubtract(pts[1], pts[0])
	v2 = rs.VectorSubtract(pts[2], pts[1])
	cross = rs.VectorCrossProduct(v2, v1)
	return cross[2] > 0

def SafeExtend(polygon, newPt):
	polygon.append(newPt)
	while len(polygon) > 2 and (not IsValidTurn(polygon[-3:])):
		del polygon[-2]
	return polygon

def ConvexHull(pointIds):
	points = rs.coerce3dpointlist(pointIds)
	points = sorted(points, key=lambda p: p[0])
	
	hull = []
	ptCount = len(points)
	indices = list(range(0, ptCount)) + list(range(ptCount - 1, -1, -1))
	for i in indices:
		curPt = points[i]
		if curPt in hull:
			continue
		
		hull = SafeExtend(hull, curPt)
	
	hull = SafeExtend(hull, hull[0])
	return hull
		
if __name__ == "__main__":
	pts = rs.GetObjects("Select points", filter=rs.filter.point)
	hull = ConvexHull(pts)
	rs.AddPolyline(hull)