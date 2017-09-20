package box2d

import (
	"math"
)

/// A circle shape.
type B2CircleShape struct {
	B2Shape
	/// Position
	M_p B2Vec2
}

func MakeB2CircleShape() B2CircleShape {
	return B2CircleShape{
		B2Shape: B2Shape{
			M_type:   B2Shape_Type.E_circle,
			M_radius: 0.0,
		},
		M_p: MakeB2Vec2(0, 0),
	}
}

func NewB2CircleShape() *B2CircleShape {
	res := MakeB2CircleShape()
	return &res
}

///////////////////////////////////////////////////////////////////////////////

func (shape B2CircleShape) Clone() B2ShapeInterface {
	clone := NewB2CircleShape()
	clone.M_radius = shape.M_radius
	clone.M_p = shape.M_p
	return clone
}

func (shape B2CircleShape) GetChildCount() int {
	return 1
}

func (shape B2CircleShape) TestPoint(transform B2Transform, p B2Vec2) bool {
	center := B2Vec2Add(transform.P, B2RotVec2Mul(transform.Q, shape.M_p))
	d := B2Vec2Sub(p, center)
	return B2Vec2Dot(d, d) <= shape.M_radius*shape.M_radius
}

// Collision Detection in Interactive 3D Environments by Gino van den Bergen
// From Section 3.1.2
// x = s + a * r
// norm(x) = radius
func (shape B2CircleShape) RayCast(output *B2RayCastOutput, input B2RayCastInput, transform B2Transform, childIndex int) bool {
	// B2_NOT_USED(childIndex);

	position := B2Vec2Add(transform.P, B2RotVec2Mul(transform.Q, shape.M_p))
	s := B2Vec2Sub(input.P1, position)
	b := B2Vec2Dot(s, s) - shape.M_radius*shape.M_radius

	// Solve quadratic equation.
	r := B2Vec2Sub(input.P2, input.P1)
	c := B2Vec2Dot(s, r)
	rr := B2Vec2Dot(r, r)
	sigma := c*c - rr*b

	// Check for negative discriminant and short segment.
	if sigma < 0.0 || rr < B2_epsilon {
		return false
	}

	// Find the point of intersection of the line with the circle.
	a := -(c + math.Sqrt(sigma))

	// Is the intersection point on the segment?
	if 0.0 <= a && a <= input.MaxFraction*rr {
		a /= rr
		output.Fraction = a
		output.Normal = B2Vec2Add(s, B2Vec2MulScalar(a, r))
		output.Normal.Normalize()
		return true
	}

	return false
}

func (shape B2CircleShape) ComputeAABB(aabb *B2AABB, transform B2Transform, childIndex int) {
	//B2_NOT_USED(childIndex);

	p := B2Vec2Add(transform.P, B2RotVec2Mul(transform.Q, shape.M_p))
	aabb.LowerBound.Set(p.X-shape.M_radius, p.Y-shape.M_radius)
	aabb.UpperBound.Set(p.X+shape.M_radius, p.Y+shape.M_radius)
}

func (shape B2CircleShape) ComputeMass(massData *B2MassData, density float64) {
	massData.Mass = density * B2_pi * shape.M_radius * shape.M_radius
	massData.Center = shape.M_p

	// inertia about the local origin
	massData.I = massData.Mass * (0.5*shape.M_radius*shape.M_radius + B2Vec2Dot(shape.M_p, shape.M_p))
}

func (shape B2CircleShape) Destroy() {}
