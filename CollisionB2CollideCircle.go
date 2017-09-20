package box2d

func B2CollideCircles(manifold *B2Manifold, circleA *B2CircleShape, xfA B2Transform, circleB *B2CircleShape, xfB B2Transform) {

	manifold.PointCount = 0

	pA := B2TransformVec2Mul(xfA, circleA.M_p)
	pB := B2TransformVec2Mul(xfB, circleB.M_p)

	d := B2Vec2Sub(pB, pA)
	distSqr := B2Vec2Dot(d, d)
	rA := circleA.M_radius
	rB := circleB.M_radius
	radius := rA + rB
	if distSqr > radius*radius {
		return
	}

	manifold.Type = B2Manifold_Type.E_circles
	manifold.LocalPoint = circleA.M_p
	manifold.LocalNormal.SetZero()
	manifold.PointCount = 1

	manifold.Points[0].LocalPoint = circleB.M_p
	manifold.Points[0].Id.SetKey(0)
}

func B2CollidePolygonAndCircle(manifold *B2Manifold, polygonA *B2PolygonShape, xfA B2Transform, circleB *B2CircleShape, xfB B2Transform) {

	manifold.PointCount = 0

	// Compute circle position in the frame of the polygon.
	c := B2TransformVec2Mul(xfB, circleB.M_p)
	cLocal := B2TransformVec2MulT(xfA, c)

	// Find the min separating edge.
	normalIndex := 0
	separation := -B2_maxFloat
	radius := polygonA.M_radius + circleB.M_radius
	vertexCount := polygonA.M_count
	vertices := polygonA.M_vertices
	normals := polygonA.M_normals

	for i := 0; i < vertexCount; i++ {
		s := B2Vec2Dot(normals[i], B2Vec2Sub(cLocal, vertices[i]))

		if s > radius {
			// Early out.
			return
		}

		if s > separation {
			separation = s
			normalIndex = i
		}
	}

	// Vertices that subtend the incident face.
	vertIndex1 := normalIndex
	vertIndex2 := 0
	if vertIndex1+1 < vertexCount {
		vertIndex2 = vertIndex1 + 1
	}

	v1 := vertices[vertIndex1]
	v2 := vertices[vertIndex2]

	// If the center is inside the polygon ...
	if separation < B2_epsilon {
		manifold.PointCount = 1
		manifold.Type = B2Manifold_Type.E_faceA
		manifold.LocalNormal = normals[normalIndex]
		manifold.LocalPoint = B2Vec2MulScalar(0.5, B2Vec2Add(v1, v2))
		manifold.Points[0].LocalPoint = circleB.M_p
		manifold.Points[0].Id.SetKey(0)
		return
	}

	// Compute barycentric coordinates
	u1 := B2Vec2Dot(B2Vec2Sub(cLocal, v1), B2Vec2Sub(v2, v1))
	u2 := B2Vec2Dot(B2Vec2Sub(cLocal, v2), B2Vec2Sub(v1, v2))
	if u1 <= 0.0 {
		if B2Vec2DistanceSquared(cLocal, v1) > radius*radius {
			return
		}

		manifold.PointCount = 1
		manifold.Type = B2Manifold_Type.E_faceA
		manifold.LocalNormal = B2Vec2Sub(cLocal, v1)
		manifold.LocalNormal.Normalize()
		manifold.LocalPoint = v1
		manifold.Points[0].LocalPoint = circleB.M_p
		manifold.Points[0].Id.SetKey(0)
	} else if u2 <= 0.0 {
		if B2Vec2DistanceSquared(cLocal, v2) > radius*radius {
			return
		}

		manifold.PointCount = 1
		manifold.Type = B2Manifold_Type.E_faceA
		manifold.LocalNormal = B2Vec2Sub(cLocal, v2)
		manifold.LocalNormal.Normalize()
		manifold.LocalPoint = v2
		manifold.Points[0].LocalPoint = circleB.M_p
		manifold.Points[0].Id.SetKey(0)
	} else {
		faceCenter := B2Vec2MulScalar(0.5, B2Vec2Add(v1, v2))
		s := B2Vec2Dot(B2Vec2Sub(cLocal, faceCenter), normals[vertIndex1])
		if s > radius {
			return
		}

		manifold.PointCount = 1
		manifold.Type = B2Manifold_Type.E_faceA
		manifold.LocalNormal = normals[vertIndex1]
		manifold.LocalPoint = faceCenter
		manifold.Points[0].LocalPoint = circleB.M_p
		manifold.Points[0].Id.SetKey(0)
	}
}
