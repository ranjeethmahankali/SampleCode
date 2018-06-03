"""
Implementation of 'Point in Polyhedron' algorithm for meshes.
Related Video: https://www.youtube.com/watch?v=qBo5oVFqnPc
Source paper: https://www.sciencedirect.com/science/article/pii/0734189X84901336
"""
import rhinoscriptsyntax as rs
import math

# calculates the solid angle between three vectors
# reference: https://math.stackexchange.com/questions/202261/solid-angle-between-vectors-in-n-dimensional-space
def GetSolidAngle(a,b,c):
	a = rs.VectorUnitize(a)
	b = rs.VectorUnitize(b)
	c = rs.VectorUnitize(c)
	numer = rs.VectorDotProduct(rs.VectorCrossProduct(a,b),c)
	denom = 1 + rs.VectorDotProduct(a,b) + rs.VectorDotProduct(b,c) + rs.VectorDotProduct(c,a)
	
	angle = 2*math.atan2(numer, denom)
	return abs(angle)

# calculating the normal for a triangular face
def Normal(p,q,r):
	return rs.VectorCrossProduct(rs.VectorSubtract(q,p), rs.VectorSubtract(r,q))

# calculating the centroid of the triangular face
def Centroid(p,q,r):
	sum = rs.VectorAdd(rs.VectorAdd(p,q),r)
	return rs.VectorScale(sum, 1/3.0)

# the logic of the point in mesh algorithm
def IsPointInside(meshId, pt, tolerance = 1e-6):
	faceVerts = rs.MeshFaces(meshId, face_type=False)
	totalAngle = 0
	i = 0
	while i < len(faceVerts):
		ptA = faceVerts[i]
		ptB = faceVerts[i + 1]
		ptC = faceVerts[i + 2]
		
		a = rs.VectorSubtract(ptA, pt)
		b = rs.VectorSubtract(ptB, pt)
		c = rs.VectorSubtract(ptC, pt)
		
		angle = GetSolidAngle(a,b,c)
		normal = Normal(ptA, ptB, ptC)
		center = Centroid(ptA, ptB, ptC)
		
		faceVec = rs.VectorSubtract(pt, center)
		dot = rs.VectorDotProduct(normal, faceVec)
		
		factor = 1 if dot > 0 else -1
		totalAngle += angle * factor
		
		i += 3
	
	absTotal = abs(totalAngle)
	
	inside = abs(absTotal - (4*math.pi)) < tolerance
	print("The total solid angle is %.02fPI"%(absTotal/math.pi))
	return inside

if __name__ == "__main__":
	# get a mesh from the user
	meshId = rs.GetObject("Select Mesh", rs.filter.mesh)
	inside = IsPointInside(meshId, [0,0,0])
	print("Is the point inside ? %s"%("YES!" if inside else "NO!"))
	# report whether the point is inside or outside