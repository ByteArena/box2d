package box2d

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// B2Distance.h
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

/// A distance proxy is used by the GJK algorithm.
/// It encapsulates any shape.
type B2DistanceProxy struct {
	M_buffer   [2]B2Vec2
	M_vertices []B2Vec2 // is a memory blob using pointer arithmetic in original implementation
	M_count    int
	M_radius   float64
}

func MakeB2DistanceProxy() B2DistanceProxy {
	return B2DistanceProxy{
		M_vertices: make([]B2Vec2, 0),
		M_count:    0,
		M_radius:   0.0,
	}
}

func NewB2DistanceProxy() *B2DistanceProxy {
	res := MakeB2DistanceProxy()
	return &res
}

/// Used to warm start b2Distance.
/// Set count to zero on first call.
type B2SimplexCache struct {
	Metric float64 ///< length or area
	Count  int
	IndexA [3]int ///< vertices on shape A
	IndexB [3]int ///< vertices on shape B
}

func MakeB2SimplexCache() B2SimplexCache {
	return B2SimplexCache{
		Metric: 0,
		Count:  0,
		IndexA: [3]int{}, ///< vertices on shape A
		IndexB: [3]int{}, ///< vertices on shape B
	}
}

func NewB2SimplexCache() *B2SimplexCache {
	res := MakeB2SimplexCache()
	return &res
}

/// Input for b2Distance.
/// You have to option to use the shape radii
/// in the computation. Even
type B2DistanceInput struct {
	ProxyA     B2DistanceProxy
	ProxyB     B2DistanceProxy
	TransformA B2Transform
	TransformB B2Transform
	UseRadii   bool
}

func MakeB2DistanceInput() B2DistanceInput {
	return B2DistanceInput{
		ProxyA:     MakeB2DistanceProxy(),
		ProxyB:     MakeB2DistanceProxy(),
		TransformA: MakeB2Transform(),
		TransformB: MakeB2Transform(),
		UseRadii:   false,
	}
}

func NewB2DistanceInput() *B2DistanceInput {
	res := MakeB2DistanceInput()
	return &res
}

/// Output for b2Distance.
type B2DistanceOutput struct {
	PointA     B2Vec2 ///< closest point on shapeA
	PointB     B2Vec2 ///< closest point on shapeB
	Distance   float64
	Iterations int ///< number of GJK iterations used
}

func MakeB2DistanceOutput() B2DistanceOutput {
	return B2DistanceOutput{
		PointA:     MakeB2Vec2(0, 0),
		PointB:     MakeB2Vec2(0, 0),
		Distance:   0,
		Iterations: 0,
	}
}

func NewB2DistanceOutput() *B2DistanceOutput {
	res := MakeB2DistanceOutput()
	return &res
}

// //////////////////////////////////////////////////////////////////////////

func (p B2DistanceProxy) GetVertexCount() int {
	return p.M_count
}

func (p B2DistanceProxy) GetVertex(index int) B2Vec2 {
	B2Assert(0 <= index && index < p.M_count)
	return p.M_vertices[index]
}

func (p B2DistanceProxy) GetSupport(d B2Vec2) int {
	bestIndex := 0
	bestValue := B2Vec2Dot(p.M_vertices[0], d)
	for i := 1; i < p.M_count; i++ {
		value := B2Vec2Dot(p.M_vertices[i], d)
		if value > bestValue {
			bestIndex = i
			bestValue = value
		}
	}

	return bestIndex
}

func (p B2DistanceProxy) GetSupportVertex(d B2Vec2) B2Vec2 {
	bestIndex := 0
	bestValue := B2Vec2Dot(p.M_vertices[0], d)

	for i := 1; i < p.M_count; i++ {
		value := B2Vec2Dot(p.M_vertices[i], d)
		if value > bestValue {
			bestIndex = i
			bestValue = value
		}
	}

	return p.M_vertices[bestIndex]
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// B2Distance.cpp
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// GJK using Voronoi regions (Christer Ericson) and Barycentric coordinates.
var b2_gjkCalls, b2_gjkIters, b2_gjkMaxIters int

func (p *B2DistanceProxy) Set(shape B2ShapeInterface, index int) {
	switch shape.GetType() {
	case B2Shape_Type.E_circle:
		{
			circle := (shape).(*B2CircleShape)
			p.M_vertices = []B2Vec2{circle.M_p}
			p.M_count = 1
			p.M_radius = circle.M_radius
		}
		break

	case B2Shape_Type.E_polygon:
		{
			polygon := shape.(*B2PolygonShape)
			p.M_vertices = polygon.M_vertices[:]
			p.M_count = polygon.M_count
			p.M_radius = polygon.M_radius
		}
		break

	case B2Shape_Type.E_chain:
		{
			chain := shape.(*B2ChainShape)
			B2Assert(0 <= index && index < chain.M_count)

			p.M_buffer[0] = chain.M_vertices[index]
			if index+1 < chain.M_count {
				p.M_buffer[1] = chain.M_vertices[index+1]
			} else {
				p.M_buffer[1] = chain.M_vertices[0]
			}

			p.M_vertices = p.M_buffer[:]
			p.M_count = 2
			p.M_radius = chain.M_radius
		}
		break

	case B2Shape_Type.E_edge:
		{
			edge := shape.(*B2EdgeShape)
			p.M_vertices = []B2Vec2{edge.M_vertex1, edge.M_vertex2}
			p.M_count = 2
			p.M_radius = edge.M_radius
		}
		break

	default:
		B2Assert(false)
	}
}

type B2SimplexVertex struct {
	WA     B2Vec2  // support point in proxyA
	WB     B2Vec2  // support point in proxyB
	W      B2Vec2  // wB - wA
	A      float64 // barycentric coordinate for closest point
	IndexA int     // wA index
	IndexB int     // wB index
}

func MakeB2SimplexVertex() B2SimplexVertex {
	return B2SimplexVertex{
		WA:     MakeB2Vec2(0, 0),
		WB:     MakeB2Vec2(0, 0),
		W:      MakeB2Vec2(0, 0),
		A:      0,
		IndexA: 0,
		IndexB: 0,
	}
}

func NewB2SimplexVertex() *B2SimplexVertex {
	res := MakeB2SimplexVertex()
	return &res
}

type B2Simplex struct {
	//M_v1, M_v2, M_v3 *B2SimplexVertex
	M_vs    [3]B2SimplexVertex
	M_count int
}

func MakeB2Simplex() B2Simplex {
	return B2Simplex{
		M_vs: [3]B2SimplexVertex{
			MakeB2SimplexVertex(),
			MakeB2SimplexVertex(),
			MakeB2SimplexVertex(),
		},
	}
}

func NewB2Simplex() *B2Simplex {
	res := MakeB2Simplex()
	return &res
}

func (simplex *B2Simplex) ReadCache(cache *B2SimplexCache, proxyA *B2DistanceProxy, transformA B2Transform, proxyB *B2DistanceProxy, transformB B2Transform) {
	B2Assert(cache.Count <= 3)

	// Copy data from cache.
	simplex.M_count = cache.Count
	vertices := &simplex.M_vs
	for i := 0; i < simplex.M_count; i++ {
		v := &vertices[i]
		v.IndexA = cache.IndexA[i]
		v.IndexB = cache.IndexB[i]
		wALocal := proxyA.GetVertex(v.IndexA)
		wBLocal := proxyB.GetVertex(v.IndexB)
		v.WA = B2TransformVec2Mul(transformA, wALocal)
		v.WB = B2TransformVec2Mul(transformB, wBLocal)
		v.W = B2Vec2Sub(v.WB, v.WA)
		v.A = 0.0
	}

	// Compute the new simplex metric, if it is substantially different than
	// old metric then flush the simplex.
	if simplex.M_count > 1 {
		metric1 := cache.Metric
		metric2 := simplex.GetMetric()
		if metric2 < 0.5*metric1 || 2.0*metric1 < metric2 || metric2 < B2_epsilon {
			// Reset the simplex.
			simplex.M_count = 0
		}
	}

	// If the cache is empty or invalid ...
	if simplex.M_count == 0 {
		v := &vertices[0]
		v.IndexA = 0
		v.IndexB = 0
		wALocal := proxyA.GetVertex(0)
		wBLocal := proxyB.GetVertex(0)
		v.WA = B2TransformVec2Mul(transformA, wALocal)
		v.WB = B2TransformVec2Mul(transformB, wBLocal)
		v.W = B2Vec2Sub(v.WB, v.WA)
		v.A = 1.0
		simplex.M_count = 1
	}
}

func (simplex B2Simplex) WriteCache(cache *B2SimplexCache) {
	cache.Metric = simplex.GetMetric()
	cache.Count = simplex.M_count
	vertices := &simplex.M_vs
	for i := 0; i < simplex.M_count; i++ {
		cache.IndexA[i] = vertices[i].IndexA
		cache.IndexB[i] = vertices[i].IndexB
	}
}

func (simplex B2Simplex) GetSearchDirection() B2Vec2 {
	switch simplex.M_count {
	case 1:
		return simplex.M_vs[0].W.OperatorNegate()

	case 2:
		{
			e12 := B2Vec2Sub(simplex.M_vs[1].W, simplex.M_vs[0].W)
			sgn := B2Vec2Cross(e12, simplex.M_vs[0].W.OperatorNegate())
			if sgn > 0.0 {
				// Origin is left of e12.
				return B2Vec2CrossScalarVector(1.0, e12)
			} else {
				// Origin is right of e12.
				return B2Vec2CrossVectorScalar(e12, 1.0)
			}
		}

	default:
		B2Assert(false)
		return B2Vec2_zero
	}
}

func (simplex B2Simplex) GetClosestPoint() B2Vec2 {
	switch simplex.M_count {
	case 0:
		B2Assert(false)
		return B2Vec2_zero

	case 1:
		return simplex.M_vs[0].W

	case 2:
		return B2Vec2Add(
			B2Vec2MulScalar(
				simplex.M_vs[0].A,
				simplex.M_vs[0].W,
			),
			B2Vec2MulScalar(
				simplex.M_vs[1].A,
				simplex.M_vs[1].W,
			),
		)

	case 3:
		return B2Vec2_zero

	default:
		B2Assert(false)
		return B2Vec2_zero
	}
}

func (simplex B2Simplex) GetWitnessPoints(pA *B2Vec2, pB *B2Vec2) {
	switch simplex.M_count {
	case 0:
		B2Assert(false)
		break

	case 1:
		*pA = simplex.M_vs[0].WA
		*pB = simplex.M_vs[0].WB
		break

	case 2:
		*pA = B2Vec2Add(
			B2Vec2MulScalar(simplex.M_vs[0].A, simplex.M_vs[0].WA),
			B2Vec2MulScalar(simplex.M_vs[1].A, simplex.M_vs[1].WA),
		)
		*pB = B2Vec2Add(
			B2Vec2MulScalar(simplex.M_vs[0].A, simplex.M_vs[0].WB),
			B2Vec2MulScalar(simplex.M_vs[1].A, simplex.M_vs[1].WB),
		)
		break

	case 3:
		*pA = B2Vec2Add(
			B2Vec2Add(
				B2Vec2MulScalar(simplex.M_vs[0].A, simplex.M_vs[0].WA),
				B2Vec2MulScalar(simplex.M_vs[1].A, simplex.M_vs[1].WA),
			),
			B2Vec2MulScalar(simplex.M_vs[2].A, simplex.M_vs[2].WA),
		)
		*pB = *pA
		break

	default:
		B2Assert(false)
		break
	}
}

func (simplex B2Simplex) GetMetric() float64 {
	switch simplex.M_count {
	case 0:
		B2Assert(false)
		return 0.0

	case 1:
		return 0.0

	case 2:
		return B2Vec2Distance(simplex.M_vs[0].W, simplex.M_vs[1].W)

	case 3:
		return B2Vec2Cross(
			B2Vec2Sub(simplex.M_vs[1].W, simplex.M_vs[0].W),
			B2Vec2Sub(simplex.M_vs[2].W, simplex.M_vs[0].W),
		)

	default:
		B2Assert(false)
		return 0.0
	}
}

////////////////////////////////////////////////////

// Solve a line segment using barycentric coordinates.
func (simplex *B2Simplex) Solve2() {
	w1 := simplex.M_vs[0].W
	w2 := simplex.M_vs[1].W
	e12 := B2Vec2Sub(w2, w1)

	// w1 region
	d12_2 := -B2Vec2Dot(w1, e12)
	if d12_2 <= 0.0 {
		// a2 <= 0, so we clamp it to 0
		simplex.M_vs[0].A = 1.0
		simplex.M_count = 1
		return
	}

	// w2 region
	d12_1 := B2Vec2Dot(w2, e12)
	if d12_1 <= 0.0 {
		// a1 <= 0, so we clamp it to 0
		simplex.M_vs[1].A = 1.0
		simplex.M_count = 1
		simplex.M_vs[0] = simplex.M_vs[1]
		return
	}

	// Must be in e12 region.
	inv_d12 := 1.0 / (d12_1 + d12_2)
	simplex.M_vs[0].A = d12_1 * inv_d12
	simplex.M_vs[1].A = d12_2 * inv_d12
	simplex.M_count = 2
}

// // Possible regions:
// // - points[2]
// // - edge points[0]-points[2]
// // - edge points[1]-points[2]
// // - inside the triangle
func (simplex *B2Simplex) Solve3() {

	w1 := simplex.M_vs[0].W
	w2 := simplex.M_vs[1].W
	w3 := simplex.M_vs[2].W

	// Edge12
	// [1      1     ][a1] = [1]
	// [w1.e12 w2.e12][a2] = [0]
	// a3 = 0
	e12 := B2Vec2Sub(w2, w1)
	w1e12 := B2Vec2Dot(w1, e12)
	w2e12 := B2Vec2Dot(w2, e12)
	d12_1 := w2e12
	d12_2 := -w1e12

	// Edge13
	// [1      1     ][a1] = [1]
	// [w1.e13 w3.e13][a3] = [0]
	// a2 = 0
	e13 := B2Vec2Sub(w3, w1)
	w1e13 := B2Vec2Dot(w1, e13)
	w3e13 := B2Vec2Dot(w3, e13)
	d13_1 := w3e13
	d13_2 := -w1e13

	// Edge23
	// [1      1     ][a2] = [1]
	// [w2.e23 w3.e23][a3] = [0]
	// a1 = 0
	e23 := B2Vec2Sub(w3, w2)
	w2e23 := B2Vec2Dot(w2, e23)
	w3e23 := B2Vec2Dot(w3, e23)
	d23_1 := w3e23
	d23_2 := -w2e23

	// Triangle123
	n123 := B2Vec2Cross(e12, e13)

	d123_1 := n123 * B2Vec2Cross(w2, w3)
	d123_2 := n123 * B2Vec2Cross(w3, w1)
	d123_3 := n123 * B2Vec2Cross(w1, w2)

	// w1 region
	if d12_2 <= 0.0 && d13_2 <= 0.0 {
		simplex.M_vs[0].A = 1.0
		simplex.M_count = 1
		return
	}

	// e12
	if d12_1 > 0.0 && d12_2 > 0.0 && d123_3 <= 0.0 {
		inv_d12 := 1.0 / (d12_1 + d12_2)
		simplex.M_vs[0].A = d12_1 * inv_d12
		simplex.M_vs[1].A = d12_2 * inv_d12
		simplex.M_count = 2
		return
	}

	// e13
	if d13_1 > 0.0 && d13_2 > 0.0 && d123_2 <= 0.0 {
		inv_d13 := 1.0 / (d13_1 + d13_2)
		simplex.M_vs[0].A = d13_1 * inv_d13
		simplex.M_vs[2].A = d13_2 * inv_d13
		simplex.M_count = 2
		simplex.M_vs[1] = simplex.M_vs[2]
		return
	}

	// w2 region
	if d12_1 <= 0.0 && d23_2 <= 0.0 {
		simplex.M_vs[1].A = 1.0
		simplex.M_count = 1
		simplex.M_vs[0] = simplex.M_vs[1]
		return
	}

	// w3 region
	if d13_1 <= 0.0 && d23_1 <= 0.0 {
		simplex.M_vs[2].A = 1.0
		simplex.M_count = 1
		simplex.M_vs[0] = simplex.M_vs[2]
		return
	}

	// e23
	if d23_1 > 0.0 && d23_2 > 0.0 && d123_1 <= 0.0 {
		inv_d23 := 1.0 / (d23_1 + d23_2)
		simplex.M_vs[1].A = d23_1 * inv_d23
		simplex.M_vs[2].A = d23_2 * inv_d23
		simplex.M_count = 2
		simplex.M_vs[0] = simplex.M_vs[2]
		return
	}

	// Must be in triangle123
	inv_d123 := 1.0 / (d123_1 + d123_2 + d123_3)
	simplex.M_vs[0].A = d123_1 * inv_d123
	simplex.M_vs[1].A = d123_2 * inv_d123
	simplex.M_vs[2].A = d123_3 * inv_d123
	simplex.M_count = 3
}

func B2Distance(output *B2DistanceOutput, cache *B2SimplexCache, input *B2DistanceInput) {
	b2_gjkCalls++

	proxyA := &input.ProxyA
	proxyB := &input.ProxyB

	transformA := input.TransformA
	transformB := input.TransformB

	// Initialize the simplex.
	simplex := MakeB2Simplex()
	simplex.ReadCache(cache, proxyA, transformA, proxyB, transformB)

	// Get simplex vertices as an array.
	vertices := &simplex.M_vs
	k_maxIters := 20

	// These store the vertices of the last simplex so that we
	// can check for duplicates and prevent cycling.
	saveA := make([]int, 3)
	saveB := make([]int, 3)
	saveCount := 0

	// Main iteration loop.
	iter := 0
	for iter < k_maxIters {
		// Copy simplex so we can identify duplicates.
		saveCount = simplex.M_count
		for i := 0; i < saveCount; i++ {
			saveA[i] = vertices[i].IndexA
			saveB[i] = vertices[i].IndexB
		}

		switch simplex.M_count {
		case 1:
			break

		case 2:
			simplex.Solve2()
			break

		case 3:
			simplex.Solve3()
			break

		default:
			B2Assert(false)
		}

		// If we have 3 points, then the origin is in the corresponding triangle.
		if simplex.M_count == 3 {
			break
		}

		// Get search direction.
		d := simplex.GetSearchDirection()

		// Ensure the search direction is numerically fit.
		if d.LengthSquared() < B2_epsilon*B2_epsilon {
			// The origin is probably contained by a line segment
			// or triangle. Thus the shapes are overlapped.

			// We can't return zero here even though there may be overlap.
			// In case the simplex is a point, segment, or triangle it is difficult
			// to determine if the origin is contained in the CSO or very close to it.
			break
		}

		// Compute a tentative new simplex vertex using support points.
		vertex := &vertices[simplex.M_count]
		vertex.IndexA = proxyA.GetSupport(
			B2RotVec2MulT(transformA.Q, d.OperatorNegate()),
		)
		vertex.WA = B2TransformVec2Mul(transformA, proxyA.GetVertex(vertex.IndexA))
		// b2Vec2 wBLocal;
		vertex.IndexB = proxyB.GetSupport(B2RotVec2MulT(transformB.Q, d))
		vertex.WB = B2TransformVec2Mul(transformB, proxyB.GetVertex(vertex.IndexB))
		vertex.W = B2Vec2Sub(vertex.WB, vertex.WA)

		// Iteration count is equated to the number of support point calls.
		iter++
		b2_gjkIters++

		// Check for duplicate support points. This is the main termination criteria.
		duplicate := false
		for i := 0; i < saveCount; i++ {
			if vertex.IndexA == saveA[i] && vertex.IndexB == saveB[i] {
				duplicate = true
				break
			}
		}

		// If we found a duplicate support point we must exit to avoid cycling.
		if duplicate {
			break
		}

		// New vertex is ok and needed.
		simplex.M_count++
	}

	if iter > b2_gjkMaxIters {
		b2_gjkMaxIters = iter
	}

	// Prepare output.
	simplex.GetWitnessPoints(&output.PointA, &output.PointB)
	output.Distance = B2Vec2Distance(output.PointA, output.PointB)
	output.Iterations = iter

	// // Cache the simplex.
	simplex.WriteCache(cache)

	// // Apply radii if requested.
	if input.UseRadii {
		rA := proxyA.M_radius
		rB := proxyB.M_radius

		if output.Distance > rA+rB && output.Distance > B2_epsilon {
			// Shapes are still no overlapped.
			// Move the witness points to the outer surface.
			output.Distance -= rA + rB
			normal := B2Vec2Sub(output.PointB, output.PointA)
			normal.Normalize()
			output.PointA.OperatorPlusInplace(
				B2Vec2MulScalar(rA, normal),
			)
			output.PointB.OperatorMinusInplace(
				B2Vec2MulScalar(rB, normal),
			)
		} else {
			// Shapes are overlapped when radii are considered.
			// Move the witness points to the middle.
			p := B2Vec2MulScalar(
				0.5,
				B2Vec2Add(output.PointA, output.PointB),
			)
			output.PointA = p
			output.PointB = p
			output.Distance = 0.0
		}
	}
}
