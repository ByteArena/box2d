package box2d

type B2PolygonContact struct {
	B2Contact
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// B2ContactPolygon.go
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

func B2PolygonContact_Create(fixtureA *B2Fixture, indexA int, fixtureB *B2Fixture, indexB int) B2ContactInterface {
	B2Assert(fixtureA.GetType() == B2Shape_Type.E_polygon)
	B2Assert(fixtureB.GetType() == B2Shape_Type.E_polygon)
	res := &B2PolygonContact{
		B2Contact: MakeB2Contact(fixtureA, 0, fixtureB, 0),
	}

	return res
}

func B2PolygonContact_Destroy(contact B2ContactInterface) { // should be a pointer
}

func (contact *B2PolygonContact) Evaluate(manifold *B2Manifold, xfA B2Transform, xfB B2Transform) {
	B2CollidePolygons(
		manifold,
		contact.GetFixtureA().GetShape().(*B2PolygonShape), xfA,
		contact.GetFixtureB().GetShape().(*B2PolygonShape), xfB,
	)
}
