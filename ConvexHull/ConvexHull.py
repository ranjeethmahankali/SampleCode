"""
Implementation of the convex hull algorithm -
calculates the convex hull (polygon) for a given collection
of points in a plane.
Related Video: https://youtu.be/4y6LORX2Ysg
"""
import rhinoscriptsyntax as rs
import time

# returns true if the turn represented by the list of points is a right turn
def IsValidTurn(pts):
	assert len(pts) == 3
	v1 = rs.VectorSubtract(pts[1], pts[0])
	v2 = rs.VectorSubtract(pts[2], pts[1])
	cross = rs.VectorCrossProduct(v2, v1)
	return cross[2] > 0

# adds the point to the polygon (vertex list) while making sure all the turns
# are right turns
def SafeExtend(polygon, newPt):
	polygon.append(newPt)
	while len(polygon) > 2 and (not IsValidTurn(polygon[-3:])):
		del polygon[-2]
	return polygon

# the main algorithm
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
        # get the points from the user
	pts = rs.GetObjects("Select points", filter=rs.filter.point)
	# compute the convex hull
	hull = ConvexHull(pts)
	# visualize the hull as a polyline
	rs.AddPolyline(hull)
