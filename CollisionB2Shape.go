package box2d

/// This holds the mass data computed for a shape.
type B2MassData struct {
	/// The mass of the shape, usually in kilograms.
	Mass float64

	/// The position of the shape's centroid relative to the shape's origin.
	Center B2Vec2

	/// The rotational inertia of the shape about the local origin.
	I float64
}

func MakeMassData() B2MassData {
	return B2MassData{
		Mass:   0.0,
		Center: MakeB2Vec2(0, 0),
		I:      0.0,
	}
}

func NewMassData() *B2MassData {
	res := MakeMassData()
	return &res
}

/// A shape is used for collision detection. You can create a shape however you like.
/// Shapes used for simulation in b2World are created automatically when a b2Fixture
/// is created. Shapes may encapsulate a one or more child shapes.

var B2Shape_Type = struct {
	E_circle    uint8
	E_edge      uint8
	E_polygon   uint8
	E_chain     uint8
	E_typeCount uint8
}{
	E_circle:    0,
	E_edge:      1,
	E_polygon:   2,
	E_chain:     3,
	E_typeCount: 4,
}

type B2ShapeInterface interface {
	Destroy()

	/// Clone the concrete shape using the provided allocator.
	Clone() B2ShapeInterface

	/// Get the type of this shape. You can use this to down cast to the concrete shape.
	/// @return the shape type.
	GetType() uint8

	/// Get the type of this shape. You can use this to down cast to the concrete shape.
	/// @return the shape type.
	GetRadius() float64

	/// Get the number of child primitives.
	GetChildCount() int

	/// Test a point for containment in this shape. This only works for convex shapes.
	/// @param xf the shape world transform.
	/// @param p a point in world coordinates.
	TestPoint(xf B2Transform, p B2Vec2) bool

	/// Cast a ray against a child shape.
	/// @param output the ray-cast results.
	/// @param input the ray-cast input parameters.
	/// @param transform the transform to be applied to the shape.
	/// @param childIndex the child shape index
	RayCast(output *B2RayCastOutput, input B2RayCastInput, transform B2Transform, childIndex int) bool

	/// Given a transform, compute the associated axis aligned bounding box for a child shape.
	/// @param aabb returns the axis aligned box.
	/// @param xf the world transform of the shape.
	/// @param childIndex the child shape
	ComputeAABB(aabb *B2AABB, xf B2Transform, childIndex int)

	/// Compute the mass properties of this shape using its dimensions and density.
	/// The inertia tensor is computed about the local origin.
	/// @param massData returns the mass data for this shape.
	/// @param density the density in kilograms per meter squared.
	ComputeMass(massData *B2MassData, density float64)
}

type B2Shape struct {
	M_type uint8

	/// Radius of a shape. For polygonal shapes this must be b2_polygonRadius. There is no support for
	/// making rounded polygons.
	M_radius float64
}

func (shape B2Shape) GetType() uint8 {
	return shape.M_type
}

func (shape B2Shape) GetRadius() float64 {
	return shape.M_radius
}

//@addedgo
func (shape *B2Shape) SetRadius(r float64) {
	shape.M_radius = r
}
