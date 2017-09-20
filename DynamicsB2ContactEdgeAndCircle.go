package box2d

type B2EdgeAndCircleContact struct {
	B2Contact
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// B2ContactEdgeAndCircle.cpp
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

func B2EdgeAndCircleContact_Create(fixtureA *B2Fixture, indexA int, fixtureB *B2Fixture, indexB int) B2ContactInterface {
	B2Assert(fixtureA.GetType() == B2Shape_Type.E_edge)
	B2Assert(fixtureB.GetType() == B2Shape_Type.E_circle)
	res := &B2EdgeAndCircleContact{
		B2Contact: MakeB2Contact(fixtureA, 0, fixtureB, 0),
	}

	return res
}

func B2EdgeAndCircleContact_Destroy(contact B2ContactInterface) { // should be a pointer
}

func (contact *B2EdgeAndCircleContact) Evaluate(manifold *B2Manifold, xfA B2Transform, xfB B2Transform) {
	B2CollideEdgeAndCircle(
		manifold,
		contact.GetFixtureA().GetShape().(*B2EdgeShape), xfA,
		contact.GetFixtureB().GetShape().(*B2CircleShape), xfB,
	)
}
