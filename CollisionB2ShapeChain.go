package box2d

/// A chain shape is a free form sequence of line segments.
/// The chain has two-sided collision, so you can use inside and outside collision.
/// Therefore, you may use any winding order.
/// Since there may be many vertices, they are allocated using b2Alloc.
/// Connectivity information is used to create smooth collisions.
/// WARNING: The chain will not collide properly if there are self-intersections.

/// A circle shape.
type B2ChainShape struct {
	B2Shape

	/// The vertices. Owned by this class.
	M_vertices []B2Vec2

	/// The vertex count.
	M_count int

	M_prevVertex    B2Vec2
	M_nextVertex    B2Vec2
	M_hasPrevVertex bool
	M_hasNextVertex bool
}

func MakeB2ChainShape() B2ChainShape {
	return B2ChainShape{
		B2Shape: B2Shape{
			M_type:   B2Shape_Type.E_chain,
			M_radius: B2_polygonRadius,
		},
		M_vertices:      nil,
		M_count:         0,
		M_hasPrevVertex: false,
		M_hasNextVertex: false,
	}
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// B2ChainShape.cpp
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

func (chain *B2ChainShape) Destroy() {
	chain.Clear()
}

func (chain *B2ChainShape) Clear() {
	chain.M_vertices = nil
	chain.M_count = 0
}

func (chain *B2ChainShape) CreateLoop(vertices []B2Vec2, count int) {
	B2Assert(chain.M_vertices == nil && chain.M_count == 0)
	B2Assert(count >= 3)
	if count < 3 {
		return
	}

	for i := 1; i < count; i++ {
		v1 := vertices[i-1]
		v2 := vertices[i]
		// If the code crashes here, it means your vertices are too close together.
		B2Assert(B2Vec2DistanceSquared(v1, v2) > B2_linearSlop*B2_linearSlop)
	}

	chain.M_count = count + 1
	chain.M_vertices = make([]B2Vec2, chain.M_count)
	for i, vertice := range vertices {
		chain.M_vertices[i] = vertice
	}

	chain.M_vertices[count] = chain.M_vertices[0]
	chain.M_prevVertex = chain.M_vertices[chain.M_count-2]
	chain.M_nextVertex = chain.M_vertices[1]
	chain.M_hasPrevVertex = true
	chain.M_hasNextVertex = true
}

func (chain *B2ChainShape) CreateChain(vertices []B2Vec2, count int) {
	B2Assert(chain.M_vertices == nil && chain.M_count == 0)
	B2Assert(count >= 2)
	for i := 1; i < count; i++ {
		// If the code crashes here, it means your vertices are too close together.
		B2Assert(B2Vec2DistanceSquared(vertices[i-1], vertices[i]) > B2_linearSlop*B2_linearSlop)
	}

	chain.M_count = count
	chain.M_vertices = make([]B2Vec2, count)
	for i, vertice := range vertices {
		chain.M_vertices[i] = vertice
	}

	chain.M_hasPrevVertex = false
	chain.M_hasNextVertex = false

	chain.M_prevVertex.SetZero()
	chain.M_nextVertex.SetZero()
}

func (chain *B2ChainShape) SetPrevVertex(prevVertex B2Vec2) {
	chain.M_prevVertex = prevVertex
	chain.M_hasPrevVertex = true
}

func (chain *B2ChainShape) SetNextVertex(nextVertex B2Vec2) {
	chain.M_nextVertex = nextVertex
	chain.M_hasNextVertex = true
}

func (chain B2ChainShape) Clone() B2ShapeInterface {

	clone := MakeB2ChainShape()
	clone.CreateChain(chain.M_vertices, chain.M_count)
	clone.M_prevVertex = chain.M_prevVertex
	clone.M_nextVertex = chain.M_nextVertex
	clone.M_hasPrevVertex = chain.M_hasPrevVertex
	clone.M_hasNextVertex = chain.M_hasNextVertex

	return &clone
}

func (chain B2ChainShape) GetChildCount() int {
	// edge count = vertex count - 1
	return chain.M_count - 1
}

func (chain B2ChainShape) GetChildEdge(edge *B2EdgeShape, index int) {
	B2Assert(0 <= index && index < chain.M_count-1)

	edge.M_type = B2Shape_Type.E_edge
	edge.M_radius = chain.M_radius

	edge.M_vertex1 = chain.M_vertices[index+0]
	edge.M_vertex2 = chain.M_vertices[index+1]

	if index > 0 {
		edge.M_vertex0 = chain.M_vertices[index-1]
		edge.M_hasVertex0 = true
	} else {
		edge.M_vertex0 = chain.M_prevVertex
		edge.M_hasVertex0 = chain.M_hasPrevVertex
	}

	if index < chain.M_count-2 {
		edge.M_vertex3 = chain.M_vertices[index+2]
		edge.M_hasVertex3 = true
	} else {
		edge.M_vertex3 = chain.M_nextVertex
		edge.M_hasVertex3 = chain.M_hasNextVertex
	}
}

func (chain B2ChainShape) TestPoint(xf B2Transform, p B2Vec2) bool {
	return false
}

func (chain B2ChainShape) RayCast(output *B2RayCastOutput, input B2RayCastInput, xf B2Transform, childIndex int) bool {
	B2Assert(childIndex < chain.M_count)

	edgeShape := MakeB2EdgeShape()

	i1 := childIndex
	i2 := childIndex + 1
	if i2 == chain.M_count {
		i2 = 0
	}

	edgeShape.M_vertex1 = chain.M_vertices[i1]
	edgeShape.M_vertex2 = chain.M_vertices[i2]

	return edgeShape.RayCast(output, input, xf, 0)
}

func (chain B2ChainShape) ComputeAABB(aabb *B2AABB, xf B2Transform, childIndex int) {
	B2Assert(childIndex < chain.M_count)

	i1 := childIndex
	i2 := childIndex + 1
	if i2 == chain.M_count {
		i2 = 0
	}

	v1 := B2TransformVec2Mul(xf, chain.M_vertices[i1])
	v2 := B2TransformVec2Mul(xf, chain.M_vertices[i2])

	aabb.LowerBound = B2Vec2Min(v1, v2)
	aabb.UpperBound = B2Vec2Max(v1, v2)
}

func (chain B2ChainShape) ComputeMass(massData *B2MassData, density float64) {
	massData.Mass = 0.0
	massData.Center.SetZero()
	massData.I = 0.0
}
