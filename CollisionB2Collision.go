package box2d

import (
	"math"
)

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// B2Collision.h
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

const B2_nullFeature uint8 = math.MaxUint8

var B2ContactFeature_Type = struct {
	E_vertex uint8
	E_face   uint8
}{
	E_vertex: 0,
	E_face:   1,
}

/// The features that intersect to form the contact point
/// This must be 4 bytes or less.
type B2ContactFeature struct {
	IndexA uint8 ///< Feature index on shapeA
	IndexB uint8 ///< Feature index on shapeB
	TypeA  uint8 ///< The feature type on shapeA
	TypeB  uint8 ///< The feature type on shapeB
}

func MakeB2ContactFeature() B2ContactFeature {
	return B2ContactFeature{}
}

type B2ContactID B2ContactFeature

/// Contact ids to facilitate warm starting.
///< Used to quickly compare contact ids.
func (v B2ContactID) Key() uint32 {
	var key uint32 = 0
	key |= uint32(v.IndexA)
	key |= uint32(v.IndexB) << 8
	key |= uint32(v.TypeA) << 16
	key |= uint32(v.TypeB) << 24
	return key
}

func (v *B2ContactID) SetKey(key uint32) {
	(*v).IndexA = uint8(key & 0xFF)
	(*v).IndexB = byte(key >> 8 & 0xFF)
	(*v).TypeA = byte(key >> 16 & 0xFF)
	(*v).TypeB = byte(key >> 24 & 0xFF)
}

/// A manifold point is a contact point belonging to a contact
/// manifold. It holds details related to the geometry and dynamics
/// of the contact points.
/// The local point usage depends on the manifold type:
/// -e_circles: the local center of circleB
/// -e_faceA: the local center of cirlceB or the clip point of polygonB
/// -e_faceB: the clip point of polygonA
/// This structure is stored across time steps, so we keep it small.
/// Note: the impulses are used for internal caching and may not
/// provide reliable contact forces, especially for high speed collisions.
type B2ManifoldPoint struct {
	LocalPoint     B2Vec2      ///< usage depends on manifold type
	NormalImpulse  float64     ///< the non-penetration impulse
	TangentImpulse float64     ///< the friction impulse
	Id             B2ContactID ///< uniquely identifies a contact point between two shapes
}

/// A manifold for two touching convex shapes.
/// Box2D supports multiple types of contact:
/// - clip point versus plane with radius
/// - point versus point with radius (circles)
/// The local point usage depends on the manifold type:
/// -e_circles: the local center of circleA
/// -e_faceA: the center of faceA
/// -e_faceB: the center of faceB
/// Similarly the local normal usage:
/// -e_circles: not used
/// -e_faceA: the normal on polygonA
/// -e_faceB: the normal on polygonB
/// We store contacts in this way so that position correction can
/// account for movement, which is critical for continuous physics.
/// All contact scenarios must be expressed in one of these types.
/// This structure is stored across time steps, so we keep it small.

var B2Manifold_Type = struct {
	E_circles uint8
	E_faceA   uint8
	E_faceB   uint8
}{
	E_circles: 0,
	E_faceA:   1,
	E_faceB:   2,
}

type B2Manifold struct {
	Points      [B2_maxManifoldPoints]B2ManifoldPoint ///< the points of contact
	LocalNormal B2Vec2                                ///< not use for Type::e_points
	LocalPoint  B2Vec2                                ///< usage depends on manifold type
	Type        uint8                                 // B2Manifold_Type
	PointCount  int                                   ///< the number of manifold points
}

func NewB2Manifold() *B2Manifold {
	return &B2Manifold{}
}

/// This is used to compute the current state of a contact manifold.
type B2WorldManifold struct {
	Normal      B2Vec2                        ///< world vector pointing from A to B
	Points      [B2_maxManifoldPoints]B2Vec2  ///< world contact point (point of intersection)
	Separations [B2_maxManifoldPoints]float64 ///< a negative value indicates overlap, in meters
}

func MakeB2WorldManifold() B2WorldManifold {
	return B2WorldManifold{}
}

var B2PointState = struct {
	B2_nullState    uint8 ///< point does not exist
	B2_addState     uint8 ///< point was added in the update
	B2_persistState uint8 ///< point persisted across the update
	B2_removeState  uint8 ///< point was removed in the update
}{
	B2_nullState:    0,
	B2_addState:     1,
	B2_persistState: 2,
	B2_removeState:  3,
}

/// Used for computing contact manifolds.
type B2ClipVertex struct {
	V  B2Vec2
	Id B2ContactID
}

/// Ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
type B2RayCastInput struct {
	P1, P2      B2Vec2
	MaxFraction float64
}

func MakeB2RayCastInput() B2RayCastInput {
	return B2RayCastInput{
		P1:          MakeB2Vec2(0, 0),
		P2:          MakeB2Vec2(0, 0),
		MaxFraction: 0,
	}
}

func NewB2RayCastInput() *B2RayCastInput {
	res := MakeB2RayCastInput()
	return &res
}

/// Ray-cast output data. The ray hits at p1 + fraction * (p2 - p1), where p1 and p2
/// come from b2RayCastInput.
type B2RayCastOutput struct {
	Normal   B2Vec2
	Fraction float64
}

func MakeB2RayCastOutput() B2RayCastOutput {
	return B2RayCastOutput{
		Normal:   MakeB2Vec2(0, 0),
		Fraction: 0,
	}
}

/// An axis aligned bounding box.
type B2AABB struct {
	LowerBound B2Vec2 ///< the lower vertex
	UpperBound B2Vec2 ///< the upper vertex
}

func MakeB2AABB() B2AABB {
	return B2AABB{
		LowerBound: MakeB2Vec2(0, 0),
		UpperBound: MakeB2Vec2(0, 0),
	}
}

func NewB2AABB() *B2AABB {
	res := MakeB2AABB()
	return &res
}

/// Get the center of the AABB.
func (bb B2AABB) GetCenter() B2Vec2 {
	return B2Vec2MulScalar(
		0.5,
		B2Vec2Add(bb.LowerBound, bb.UpperBound),
	)
}

/// Get the extents of the AABB (half-widths).
func (bb B2AABB) GetExtents() B2Vec2 {
	return B2Vec2MulScalar(
		0.5,
		B2Vec2Sub(bb.UpperBound, bb.LowerBound),
	)
}

/// Get the perimeter length
func (bb B2AABB) GetPerimeter() float64 {
	wx := bb.UpperBound.X - bb.LowerBound.X
	wy := bb.UpperBound.Y - bb.LowerBound.Y
	return 2.0 * (wx + wy)
}

/// Combine an AABB into this one.
func (bb *B2AABB) CombineInPlace(aabb B2AABB) {
	bb.LowerBound = B2Vec2Min(bb.LowerBound, aabb.LowerBound)
	bb.UpperBound = B2Vec2Max(bb.UpperBound, aabb.UpperBound)
}

/// Combine two AABBs into this one.
func (bb *B2AABB) CombineTwoInPlace(aabb1, aabb2 B2AABB) {
	bb.LowerBound = B2Vec2Min(aabb1.LowerBound, aabb2.LowerBound)
	bb.UpperBound = B2Vec2Max(aabb1.UpperBound, aabb2.UpperBound)
}

/// Does this aabb contain the provided AABB.
func (bb B2AABB) Contains(aabb B2AABB) bool {

	return (bb.LowerBound.X <= aabb.LowerBound.X &&
		bb.LowerBound.Y <= aabb.LowerBound.Y &&
		aabb.UpperBound.X <= bb.UpperBound.X &&
		aabb.UpperBound.Y <= bb.UpperBound.Y)
}

func (bb B2AABB) IsValid() bool {
	d := B2Vec2Sub(bb.UpperBound, bb.LowerBound)
	valid := d.X >= 0.0 && d.Y >= 0.0
	valid = valid && bb.LowerBound.IsValid() && bb.UpperBound.IsValid()
	return valid
}

func (bb B2AABB) Clone() B2AABB {
	clone := MakeB2AABB()
	clone.LowerBound = bb.LowerBound.Clone()
	clone.UpperBound = bb.UpperBound.Clone()

	return clone
}

func B2TestOverlapBoundingBoxes(a, b B2AABB) bool {

	d1 := B2Vec2Sub(b.LowerBound, a.UpperBound)
	d2 := B2Vec2Sub(a.LowerBound, b.UpperBound)

	if d1.X > 0.0 || d1.Y > 0.0 {
		return false
	}

	if d2.X > 0.0 || d2.Y > 0.0 {
		return false
	}

	return true
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// B2Collision.cpp
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

func (wm *B2WorldManifold) Initialize(manifold *B2Manifold, xfA B2Transform, radiusA float64, xfB B2Transform, radiusB float64) {
	if manifold.PointCount == 0 {
		return
	}

	switch manifold.Type {
	case B2Manifold_Type.E_circles:
		{
			wm.Normal.Set(1.0, 0.0)
			pointA := B2TransformVec2Mul(xfA, manifold.LocalPoint)
			pointB := B2TransformVec2Mul(xfB, manifold.Points[0].LocalPoint)
			if B2Vec2DistanceSquared(pointA, pointB) > B2_epsilon*B2_epsilon {
				wm.Normal = B2Vec2Sub(pointB, pointA)
				wm.Normal.Normalize()
			}

			cA := B2Vec2Add(pointA, B2Vec2MulScalar(radiusA, wm.Normal))
			cB := B2Vec2Sub(pointB, B2Vec2MulScalar(radiusB, wm.Normal))

			wm.Points[0] = B2Vec2MulScalar(0.5, B2Vec2Add(cA, cB))
			wm.Separations[0] = B2Vec2Dot(B2Vec2Sub(cB, cA), wm.Normal)
		}
		break

	case B2Manifold_Type.E_faceA:
		{
			wm.Normal = B2RotVec2Mul(xfA.Q, manifold.LocalNormal)
			planePoint := B2TransformVec2Mul(xfA, manifold.LocalPoint)

			for i := 0; i < manifold.PointCount; i++ {
				clipPoint := B2TransformVec2Mul(xfB, manifold.Points[i].LocalPoint)
				cA := B2Vec2Add(
					clipPoint,
					B2Vec2MulScalar(
						radiusA-B2Vec2Dot(
							B2Vec2Sub(clipPoint, planePoint),
							wm.Normal,
						),
						wm.Normal,
					),
				)
				cB := B2Vec2Sub(clipPoint, B2Vec2MulScalar(radiusB, wm.Normal))
				wm.Points[i] = B2Vec2MulScalar(0.5, B2Vec2Add(cA, cB))
				wm.Separations[i] = B2Vec2Dot(
					B2Vec2Sub(cB, cA),
					wm.Normal,
				)
			}
		}
		break

	case B2Manifold_Type.E_faceB:
		{
			wm.Normal = B2RotVec2Mul(xfB.Q, manifold.LocalNormal)
			planePoint := B2TransformVec2Mul(xfB, manifold.LocalPoint)

			for i := 0; i < manifold.PointCount; i++ {
				clipPoint := B2TransformVec2Mul(xfA, manifold.Points[i].LocalPoint)
				cB := B2Vec2Add(clipPoint, B2Vec2MulScalar(
					radiusB-B2Vec2Dot(
						B2Vec2Sub(clipPoint, planePoint),
						wm.Normal,
					), wm.Normal,
				))
				cA := B2Vec2Sub(clipPoint, B2Vec2MulScalar(radiusA, wm.Normal))
				wm.Points[i] = B2Vec2MulScalar(0.5, B2Vec2Add(cA, cB))
				wm.Separations[i] = B2Vec2Dot(
					B2Vec2Sub(cA, cB),
					wm.Normal,
				)
			}

			// Ensure normal points from A to B.
			wm.Normal = wm.Normal.OperatorNegate()
		}
		break
	}
}

func B2GetPointStates(state1 *[B2_maxManifoldPoints]uint8, state2 *[B2_maxManifoldPoints]uint8, manifold1 B2Manifold, manifold2 B2Manifold) {

	for i := 0; i < B2_maxManifoldPoints; i++ {
		state1[i] = B2PointState.B2_nullState
		state2[i] = B2PointState.B2_nullState
	}

	// Detect persists and removes.
	for i := 0; i < manifold1.PointCount; i++ {
		id := manifold1.Points[i].Id

		state1[i] = B2PointState.B2_removeState

		for j := 0; j < manifold2.PointCount; j++ {
			if manifold2.Points[j].Id.Key() == id.Key() {
				state1[i] = B2PointState.B2_persistState
				break
			}
		}
	}

	// Detect persists and adds.
	for i := 0; i < manifold2.PointCount; i++ {
		id := manifold2.Points[i].Id

		state2[i] = B2PointState.B2_addState

		for j := 0; j < manifold1.PointCount; j++ {
			if manifold1.Points[j].Id.Key() == id.Key() {
				state2[i] = B2PointState.B2_persistState
				break
			}
		}
	}
}

// From Real-time Collision Detection, p179.
func (bb B2AABB) RayCast(output *B2RayCastOutput, input B2RayCastInput) bool {
	tmin := -B2_maxFloat
	tmax := B2_maxFloat

	p := input.P1
	d := B2Vec2Sub(input.P2, input.P1)
	absD := B2Vec2Abs(d)

	normal := MakeB2Vec2(0, 0)

	for i := 0; i < 2; i++ {
		if absD.OperatorIndexGet(i) < B2_epsilon {
			// Parallel.
			if p.OperatorIndexGet(i) < bb.LowerBound.OperatorIndexGet(i) || bb.UpperBound.OperatorIndexGet(i) < p.OperatorIndexGet(i) {
				return false
			}
		} else {
			inv_d := 1.0 / d.OperatorIndexGet(i)
			t1 := (bb.LowerBound.OperatorIndexGet(i) - p.OperatorIndexGet(i)) * inv_d
			t2 := (bb.UpperBound.OperatorIndexGet(i) - p.OperatorIndexGet(i)) * inv_d

			// Sign of the normal vector.
			s := -1.0

			if t1 > t2 {
				t1, t2 = t2, t1
				s = 1.0
			}

			// Push the min up
			if t1 > tmin {
				normal.SetZero()
				normal.OperatorIndexSet(i, s)
				tmin = t1
			}

			// Pull the max down
			tmax = math.Min(tmax, t2)

			if tmin > tmax {
				return false
			}
		}
	}

	// Does the ray start inside the box?
	// Does the ray intersect beyond the max fraction?
	if tmin < 0.0 || input.MaxFraction < tmin {
		return false
	}

	// Intersection.
	output.Fraction = tmin
	output.Normal = normal
	return true
}

// Sutherland-Hodgman clipping.
func B2ClipSegmentToLine(vOut []B2ClipVertex, vIn []B2ClipVertex, normal B2Vec2, offset float64, vertexIndexA int) int {

	// Start with no output points
	numOut := 0

	// Calculate the distance of end points to the line
	distance0 := B2Vec2Dot(normal, vIn[0].V) - offset
	distance1 := B2Vec2Dot(normal, vIn[1].V) - offset

	// If the points are behind the plane
	if distance0 <= 0.0 {
		vOut[numOut] = vIn[0]
		numOut++
	}

	if distance1 <= 0.0 {
		vOut[numOut] = vIn[1]
		numOut++
	}

	// If the points are on different sides of the plane
	if distance0*distance1 < 0.0 {
		// Find intersection point of edge and plane
		interp := distance0 / (distance0 - distance1)
		vOut[numOut].V = B2Vec2Add(
			vIn[0].V,
			B2Vec2MulScalar(interp, B2Vec2Sub(vIn[1].V, vIn[0].V)),
		)

		// VertexA is hitting edgeB.
		vOut[numOut].Id.IndexA = uint8(vertexIndexA)
		vOut[numOut].Id.IndexB = vIn[0].Id.IndexB
		vOut[numOut].Id.TypeA = B2ContactFeature_Type.E_vertex
		vOut[numOut].Id.TypeB = B2ContactFeature_Type.E_face
		numOut++
	}

	return numOut
}

func B2TestOverlapShapes(shapeA B2ShapeInterface, indexA int, shapeB B2ShapeInterface, indexB int, xfA B2Transform, xfB B2Transform) bool {
	input := MakeB2DistanceInput()
	input.ProxyA.Set(shapeA, indexA)
	input.ProxyB.Set(shapeB, indexB)
	input.TransformA = xfA
	input.TransformB = xfB
	input.UseRadii = true

	cache := MakeB2SimplexCache()
	cache.Count = 0

	output := MakeB2DistanceOutput()

	B2Distance(&output, &cache, &input)

	return output.Distance < 10.0*B2_epsilon
}
