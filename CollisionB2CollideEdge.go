package box2d

import (
	"math"
)

// Compute contact points for edge versus circle.
// This accounts for edge connectivity.
func B2CollideEdgeAndCircle(manifold *B2Manifold, edgeA *B2EdgeShape, xfA B2Transform, circleB *B2CircleShape, xfB B2Transform) {
	manifold.PointCount = 0

	// Compute circle in frame of edge
	Q := B2TransformVec2MulT(xfA, B2TransformVec2Mul(xfB, circleB.M_p))

	A := edgeA.M_vertex1
	B := edgeA.M_vertex2
	e := B2Vec2Sub(B, A)

	// Barycentric coordinates
	u := B2Vec2Dot(e, B2Vec2Sub(B, Q))
	v := B2Vec2Dot(e, B2Vec2Sub(Q, A))

	radius := edgeA.M_radius + circleB.M_radius

	cf := MakeB2ContactFeature()
	cf.IndexB = 0
	cf.TypeB = B2ContactFeature_Type.E_vertex

	// Region A
	if v <= 0.0 {
		P := A
		d := B2Vec2Sub(Q, P)
		dd := B2Vec2Dot(d, d)
		if dd > radius*radius {
			return
		}

		// Is there an edge connected to A?
		if edgeA.M_hasVertex0 {
			A1 := edgeA.M_vertex0
			B1 := A
			e1 := B2Vec2Sub(B1, A1)
			u1 := B2Vec2Dot(e1, B2Vec2Sub(B1, Q))

			// Is the circle in Region AB of the previous edge?
			if u1 > 0.0 {
				return
			}
		}

		cf.IndexA = 0
		cf.TypeA = B2ContactFeature_Type.E_vertex
		manifold.PointCount = 1
		manifold.Type = B2Manifold_Type.E_circles
		manifold.LocalNormal.SetZero()
		manifold.LocalPoint = P
		manifold.Points[0].Id.SetKey(0)
		manifold.Points[0].Id.IndexA = cf.IndexA
		manifold.Points[0].Id.IndexB = cf.IndexB
		manifold.Points[0].Id.TypeA = cf.TypeA
		manifold.Points[0].Id.TypeB = cf.TypeB
		manifold.Points[0].LocalPoint = circleB.M_p
		return
	}

	// Region B
	if u <= 0.0 {
		P := B
		d := B2Vec2Sub(Q, P)
		dd := B2Vec2Dot(d, d)
		if dd > radius*radius {
			return
		}

		// Is there an edge connected to B?
		if edgeA.M_hasVertex3 {
			B2 := edgeA.M_vertex3
			A2 := B
			e2 := B2Vec2Sub(B2, A2)
			v2 := B2Vec2Dot(e2, B2Vec2Sub(Q, A2))

			// Is the circle in Region AB of the next edge?
			if v2 > 0.0 {
				return
			}
		}

		cf.IndexA = 1
		cf.TypeA = B2ContactFeature_Type.E_vertex
		manifold.PointCount = 1
		manifold.Type = B2Manifold_Type.E_circles
		manifold.LocalNormal.SetZero()
		manifold.LocalPoint = P
		manifold.Points[0].Id.SetKey(0)
		manifold.Points[0].Id.IndexA = cf.IndexA
		manifold.Points[0].Id.IndexB = cf.IndexB
		manifold.Points[0].Id.TypeA = cf.TypeA
		manifold.Points[0].Id.TypeB = cf.TypeB
		manifold.Points[0].LocalPoint = circleB.M_p
		return
	}

	// Region AB
	den := B2Vec2Dot(e, e)
	B2Assert(den > 0.0)
	P := B2Vec2MulScalar(1.0/den, B2Vec2Add(B2Vec2MulScalar(u, A), B2Vec2MulScalar(v, B)))
	d := B2Vec2Sub(Q, P)
	dd := B2Vec2Dot(d, d)
	if dd > radius*radius {
		return
	}

	n := MakeB2Vec2(-e.Y, e.X)
	if B2Vec2Dot(n, B2Vec2Sub(Q, A)) < 0.0 {
		n.Set(-n.X, -n.Y)
	}
	n.Normalize()

	cf.IndexA = 0
	cf.TypeA = B2ContactFeature_Type.E_face
	manifold.PointCount = 1
	manifold.Type = B2Manifold_Type.E_faceA
	manifold.LocalNormal = n
	manifold.LocalPoint = A
	manifold.Points[0].Id.SetKey(0)
	manifold.Points[0].Id.IndexA = cf.IndexA
	manifold.Points[0].Id.IndexB = cf.IndexB
	manifold.Points[0].Id.TypeA = cf.TypeA
	manifold.Points[0].Id.TypeB = cf.TypeB
	manifold.Points[0].LocalPoint = circleB.M_p
}

// This structure is used to keep track of the best separating axis.
var B2EPAxis_Type = struct {
	E_unknown uint8
	E_edgeA   uint8
	E_edgeB   uint8
}{
	E_unknown: 0,
	E_edgeA:   1,
	E_edgeB:   2,
}

type B2EPAxis struct {
	Type       uint8
	Index      int
	Separation float64
}

func MakeB2EPAxis() B2EPAxis {
	return B2EPAxis{}
}

// This holds polygon B expressed in frame A.
type B2TempPolygon struct {
	Vertices [B2_maxPolygonVertices]B2Vec2
	Normals  [B2_maxPolygonVertices]B2Vec2
	Count    int
}

// Reference face used for clipping
type B2ReferenceFace struct {
	I1, I2 int

	V1, V2 B2Vec2

	Normal B2Vec2

	SideNormal1 B2Vec2
	SideOffset1 float64

	SideNormal2 B2Vec2
	SideOffset2 float64
}

func MakeB2ReferenceFace() B2ReferenceFace {
	return B2ReferenceFace{}
}

var B2EPCollider_VertexType = struct {
	E_isolated uint8
	E_concave  uint8
	E_convex   uint8
}{
	E_isolated: 0,
	E_concave:  1,
	E_convex:   2,
}

// This class collides and edge and a polygon, taking into account edge adjacency.
type B2EPCollider struct {
	M_polygonB B2TempPolygon

	M_xf                            B2Transform
	M_centroidB                     B2Vec2
	M_v0, M_v1, M_v2, M_v3          B2Vec2
	M_normal0, M_normal1, M_normal2 B2Vec2
	M_normal                        B2Vec2
	M_type1, M_type2                uint8
	M_lowerLimit, M_upperLimit      B2Vec2
	M_radius                        float64
	M_front                         bool
}

func MakeB2EPCollider() B2EPCollider {
	return B2EPCollider{}
}

// Algorithm:
// 1. Classify v1 and v2
// 2. Classify polygon centroid as front or back
// 3. Flip normal if necessary
// 4. Initialize normal range to [-pi, pi] about face normal
// 5. Adjust normal range according to adjacent edges
// 6. Visit each separating axes, only accept axes within the range
// 7. Return if _any_ axis indicates separation
// 8. Clip
func (collider *B2EPCollider) Collide(manifold *B2Manifold, edgeA *B2EdgeShape, xfA B2Transform, polygonB *B2PolygonShape, xfB B2Transform) {

	collider.M_xf = B2TransformMulT(xfA, xfB)

	collider.M_centroidB = B2TransformVec2Mul(collider.M_xf, polygonB.M_centroid)

	collider.M_v0 = edgeA.M_vertex0
	collider.M_v1 = edgeA.M_vertex1
	collider.M_v2 = edgeA.M_vertex2
	collider.M_v3 = edgeA.M_vertex3

	hasVertex0 := edgeA.M_hasVertex0
	hasVertex3 := edgeA.M_hasVertex3

	edge1 := B2Vec2Sub(collider.M_v2, collider.M_v1)
	edge1.Normalize()
	collider.M_normal1.Set(edge1.Y, -edge1.X)
	offset1 := B2Vec2Dot(collider.M_normal1, B2Vec2Sub(collider.M_centroidB, collider.M_v1))
	offset0 := 0.0
	offset2 := 0.0
	convex1 := false
	convex2 := false

	// Is there a preceding edge?
	if hasVertex0 {
		edge0 := B2Vec2Sub(collider.M_v1, collider.M_v0)
		edge0.Normalize()
		collider.M_normal0.Set(edge0.Y, -edge0.X)
		convex1 = B2Vec2Cross(edge0, edge1) >= 0.0
		offset0 = B2Vec2Dot(collider.M_normal0, B2Vec2Sub(collider.M_centroidB, collider.M_v0))
	}

	// Is there a following edge?
	if hasVertex3 {
		edge2 := B2Vec2Sub(collider.M_v3, collider.M_v2)
		edge2.Normalize()
		collider.M_normal2.Set(edge2.Y, -edge2.X)
		convex2 = B2Vec2Cross(edge1, edge2) > 0.0
		offset2 = B2Vec2Dot(collider.M_normal2, B2Vec2Sub(collider.M_centroidB, collider.M_v2))
	}

	// Determine front or back collision. Determine collision normal limits.
	if hasVertex0 && hasVertex3 {
		if convex1 && convex2 {
			collider.M_front = offset0 >= 0.0 || offset1 >= 0.0 || offset2 >= 0.0
			if collider.M_front {
				collider.M_normal = collider.M_normal1
				collider.M_lowerLimit = collider.M_normal0
				collider.M_upperLimit = collider.M_normal2
			} else {
				collider.M_normal = collider.M_normal1.OperatorNegate()
				collider.M_lowerLimit = collider.M_normal1.OperatorNegate()
				collider.M_upperLimit = collider.M_normal1.OperatorNegate()
			}
		} else if convex1 {
			collider.M_front = offset0 >= 0.0 || (offset1 >= 0.0 && offset2 >= 0.0)
			if collider.M_front {
				collider.M_normal = collider.M_normal1
				collider.M_lowerLimit = collider.M_normal0
				collider.M_upperLimit = collider.M_normal1
			} else {
				collider.M_normal = collider.M_normal1.OperatorNegate()
				collider.M_lowerLimit = collider.M_normal2.OperatorNegate()
				collider.M_upperLimit = collider.M_normal1.OperatorNegate()
			}
		} else if convex2 {
			collider.M_front = offset2 >= 0.0 || (offset0 >= 0.0 && offset1 >= 0.0)
			if collider.M_front {
				collider.M_normal = collider.M_normal1
				collider.M_lowerLimit = collider.M_normal1
				collider.M_upperLimit = collider.M_normal2
			} else {
				collider.M_normal = collider.M_normal1.OperatorNegate()
				collider.M_lowerLimit = collider.M_normal1.OperatorNegate()
				collider.M_upperLimit = collider.M_normal0.OperatorNegate()
			}
		} else {
			collider.M_front = offset0 >= 0.0 && offset1 >= 0.0 && offset2 >= 0.0
			if collider.M_front {
				collider.M_normal = collider.M_normal1
				collider.M_lowerLimit = collider.M_normal1
				collider.M_upperLimit = collider.M_normal1
			} else {
				collider.M_normal = collider.M_normal1.OperatorNegate()
				collider.M_lowerLimit = collider.M_normal2.OperatorNegate()
				collider.M_upperLimit = collider.M_normal0.OperatorNegate()
			}
		}
	} else if hasVertex0 {
		if convex1 {
			collider.M_front = offset0 >= 0.0 || offset1 >= 0.0
			if collider.M_front {
				collider.M_normal = collider.M_normal1
				collider.M_lowerLimit = collider.M_normal0
				collider.M_upperLimit = collider.M_normal1.OperatorNegate()
			} else {
				collider.M_normal = collider.M_normal1.OperatorNegate()
				collider.M_lowerLimit = collider.M_normal1
				collider.M_upperLimit = collider.M_normal1.OperatorNegate()
			}
		} else {
			collider.M_front = offset0 >= 0.0 && offset1 >= 0.0
			if collider.M_front {
				collider.M_normal = collider.M_normal1
				collider.M_lowerLimit = collider.M_normal1
				collider.M_upperLimit = collider.M_normal1.OperatorNegate()
			} else {
				collider.M_normal = collider.M_normal1.OperatorNegate()
				collider.M_lowerLimit = collider.M_normal1
				collider.M_upperLimit = collider.M_normal0.OperatorNegate()
			}
		}
	} else if hasVertex3 {
		if convex2 {
			collider.M_front = offset1 >= 0.0 || offset2 >= 0.0
			if collider.M_front {
				collider.M_normal = collider.M_normal1
				collider.M_lowerLimit = collider.M_normal1.OperatorNegate()
				collider.M_upperLimit = collider.M_normal2
			} else {
				collider.M_normal = collider.M_normal1.OperatorNegate()
				collider.M_lowerLimit = collider.M_normal1.OperatorNegate()
				collider.M_upperLimit = collider.M_normal1
			}
		} else {
			collider.M_front = offset1 >= 0.0 && offset2 >= 0.0
			if collider.M_front {
				collider.M_normal = collider.M_normal1
				collider.M_lowerLimit = collider.M_normal1.OperatorNegate()
				collider.M_upperLimit = collider.M_normal1
			} else {
				collider.M_normal = collider.M_normal1.OperatorNegate()
				collider.M_lowerLimit = collider.M_normal2.OperatorNegate()
				collider.M_upperLimit = collider.M_normal1
			}
		}
	} else {
		collider.M_front = offset1 >= 0.0
		if collider.M_front {
			collider.M_normal = collider.M_normal1
			collider.M_lowerLimit = collider.M_normal1.OperatorNegate()
			collider.M_upperLimit = collider.M_normal1.OperatorNegate()
		} else {
			collider.M_normal = collider.M_normal1.OperatorNegate()
			collider.M_lowerLimit = collider.M_normal1
			collider.M_upperLimit = collider.M_normal1
		}
	}

	// Get polygonB in frameA
	collider.M_polygonB.Count = polygonB.M_count
	for i := 0; i < polygonB.M_count; i++ {
		collider.M_polygonB.Vertices[i] = B2TransformVec2Mul(collider.M_xf, polygonB.M_vertices[i])
		collider.M_polygonB.Normals[i] = B2RotVec2Mul(collider.M_xf.Q, polygonB.M_normals[i])
	}

	collider.M_radius = polygonB.M_radius + edgeA.M_radius

	manifold.PointCount = 0

	edgeAxis := collider.ComputeEdgeSeparation()

	// If no valid normal can be found than this edge should not collide.
	if edgeAxis.Type == B2EPAxis_Type.E_unknown {
		return
	}

	if edgeAxis.Separation > collider.M_radius {
		return
	}

	polygonAxis := collider.ComputePolygonSeparation()
	if polygonAxis.Type != B2EPAxis_Type.E_unknown && polygonAxis.Separation > collider.M_radius {
		return
	}

	// Use hysteresis for jitter reduction.
	k_relativeTol := 0.98
	k_absoluteTol := 0.001

	primaryAxis := MakeB2EPAxis()
	if polygonAxis.Type == B2EPAxis_Type.E_unknown {
		primaryAxis = edgeAxis
	} else if polygonAxis.Separation > k_relativeTol*edgeAxis.Separation+k_absoluteTol {
		primaryAxis = polygonAxis
	} else {
		primaryAxis = edgeAxis
	}

	ie := make([]B2ClipVertex, 2)
	rf := MakeB2ReferenceFace()
	if primaryAxis.Type == B2EPAxis_Type.E_edgeA {
		manifold.Type = B2Manifold_Type.E_faceA

		// Search for the polygon normal that is most anti-parallel to the edge normal.
		bestIndex := 0
		bestValue := B2Vec2Dot(collider.M_normal, collider.M_polygonB.Normals[0])
		for i := 1; i < collider.M_polygonB.Count; i++ {
			value := B2Vec2Dot(collider.M_normal, collider.M_polygonB.Normals[i])
			if value < bestValue {
				bestValue = value
				bestIndex = i
			}
		}

		i1 := bestIndex
		i2 := 0
		if i1+1 < collider.M_polygonB.Count {
			i2 = i1 + 1
		}

		ie[0].V = collider.M_polygonB.Vertices[i1]
		ie[0].Id.IndexA = 0
		ie[0].Id.IndexB = uint8(i1)
		ie[0].Id.TypeA = B2ContactFeature_Type.E_face
		ie[0].Id.TypeB = B2ContactFeature_Type.E_vertex

		ie[1].V = collider.M_polygonB.Vertices[i2]
		ie[1].Id.IndexA = 0
		ie[1].Id.IndexB = uint8(i2)
		ie[1].Id.TypeA = B2ContactFeature_Type.E_face
		ie[1].Id.TypeB = B2ContactFeature_Type.E_vertex

		if collider.M_front {
			rf.I1 = 0
			rf.I2 = 1
			rf.V1 = collider.M_v1
			rf.V2 = collider.M_v2
			rf.Normal = collider.M_normal1
		} else {
			rf.I1 = 1
			rf.I2 = 0
			rf.V1 = collider.M_v2
			rf.V2 = collider.M_v1
			rf.Normal = collider.M_normal1.OperatorNegate()
		}
	} else {
		manifold.Type = B2Manifold_Type.E_faceB

		ie[0].V = collider.M_v1
		ie[0].Id.IndexA = 0
		ie[0].Id.IndexB = uint8(primaryAxis.Index)
		ie[0].Id.TypeA = B2ContactFeature_Type.E_vertex
		ie[0].Id.TypeB = B2ContactFeature_Type.E_face

		ie[1].V = collider.M_v2
		ie[1].Id.IndexA = 0
		ie[1].Id.IndexB = uint8(primaryAxis.Index)
		ie[1].Id.TypeA = B2ContactFeature_Type.E_vertex
		ie[1].Id.TypeB = B2ContactFeature_Type.E_face

		rf.I1 = primaryAxis.Index
		if rf.I1+1 < collider.M_polygonB.Count {
			rf.I2 = rf.I1 + 1
		} else {
			rf.I2 = 0
		}

		rf.V1 = collider.M_polygonB.Vertices[rf.I1]
		rf.V2 = collider.M_polygonB.Vertices[rf.I2]
		rf.Normal = collider.M_polygonB.Normals[rf.I1]
	}

	rf.SideNormal1.Set(rf.Normal.Y, -rf.Normal.X)
	rf.SideNormal2 = rf.SideNormal1.OperatorNegate()
	rf.SideOffset1 = B2Vec2Dot(rf.SideNormal1, rf.V1)
	rf.SideOffset2 = B2Vec2Dot(rf.SideNormal2, rf.V2)

	// Clip incident edge against extruded edge1 side edges.
	clipPoints1 := make([]B2ClipVertex, 2)
	clipPoints2 := make([]B2ClipVertex, 2)
	np := 0

	// Clip to box side 1
	np = B2ClipSegmentToLine(clipPoints1, ie, rf.SideNormal1, rf.SideOffset1, rf.I1)

	if np < B2_maxManifoldPoints {
		return
	}

	// Clip to negative box side 1
	np = B2ClipSegmentToLine(clipPoints2, clipPoints1, rf.SideNormal2, rf.SideOffset2, rf.I2)

	if np < B2_maxManifoldPoints {
		return
	}

	// Now clipPoints2 contains the clipped points.
	if primaryAxis.Type == B2EPAxis_Type.E_edgeA {
		manifold.LocalNormal = rf.Normal
		manifold.LocalPoint = rf.V1
	} else {
		manifold.LocalNormal = polygonB.M_normals[rf.I1]
		manifold.LocalPoint = polygonB.M_vertices[rf.I1]
	}

	pointCount := 0
	for i := 0; i < B2_maxManifoldPoints; i++ {
		separation := 0.0

		separation = B2Vec2Dot(rf.Normal, B2Vec2Sub(clipPoints2[i].V, rf.V1))

		if separation <= collider.M_radius {
			cp := &manifold.Points[pointCount]

			if primaryAxis.Type == B2EPAxis_Type.E_edgeA {
				cp.LocalPoint = B2TransformVec2MulT(collider.M_xf, clipPoints2[i].V)
				cp.Id = clipPoints2[i].Id
			} else {
				cp.LocalPoint = clipPoints2[i].V
				cp.Id.TypeA = clipPoints2[i].Id.TypeB
				cp.Id.TypeB = clipPoints2[i].Id.TypeA
				cp.Id.IndexA = clipPoints2[i].Id.IndexB
				cp.Id.IndexB = clipPoints2[i].Id.IndexA
			}

			pointCount++
		}
	}

	manifold.PointCount = pointCount
}

func (collider *B2EPCollider) ComputeEdgeSeparation() B2EPAxis {
	axis := MakeB2EPAxis()
	axis.Type = B2EPAxis_Type.E_edgeA
	if collider.M_front {
		axis.Index = 0
	} else {
		axis.Index = 1
	}
	axis.Separation = B2_maxFloat

	for i := 0; i < collider.M_polygonB.Count; i++ {
		s := B2Vec2Dot(collider.M_normal, B2Vec2Sub(collider.M_polygonB.Vertices[i], collider.M_v1))
		if s < axis.Separation {
			axis.Separation = s
		}
	}

	return axis
}

func (collider *B2EPCollider) ComputePolygonSeparation() B2EPAxis {

	axis := MakeB2EPAxis()
	axis.Type = B2EPAxis_Type.E_unknown
	axis.Index = -1
	axis.Separation = -B2_maxFloat

	perp := MakeB2Vec2(-collider.M_normal.Y, collider.M_normal.X)

	for i := 0; i < collider.M_polygonB.Count; i++ {
		n := collider.M_polygonB.Normals[i].OperatorNegate()

		s1 := B2Vec2Dot(n, B2Vec2Sub(collider.M_polygonB.Vertices[i], collider.M_v1))
		s2 := B2Vec2Dot(n, B2Vec2Sub(collider.M_polygonB.Vertices[i], collider.M_v2))
		s := math.Min(s1, s2)

		if s > collider.M_radius {
			// No collision
			axis.Type = B2EPAxis_Type.E_edgeB
			axis.Index = i
			axis.Separation = s
			return axis
		}

		// Adjacency
		if B2Vec2Dot(n, perp) >= 0.0 {
			if B2Vec2Dot(B2Vec2Sub(n, collider.M_upperLimit), collider.M_normal) < -B2_angularSlop {
				continue
			}
		} else {
			if B2Vec2Dot(B2Vec2Sub(n, collider.M_lowerLimit), collider.M_normal) < -B2_angularSlop {
				continue
			}
		}

		if s > axis.Separation {
			axis.Type = B2EPAxis_Type.E_edgeB
			axis.Index = i
			axis.Separation = s
		}
	}

	return axis
}

func B2CollideEdgeAndPolygon(manifold *B2Manifold, edgeA *B2EdgeShape, xfA B2Transform, polygonB *B2PolygonShape, xfB B2Transform) {
	collider := MakeB2EPCollider()
	collider.Collide(manifold, edgeA, xfA, polygonB, xfB)
}
