package box2d

type B2CircleContact struct {
	B2Contact
}

func B2CircleContact_Create(fixtureA *B2Fixture, indexA int, fixtureB *B2Fixture, indexB int) B2ContactInterface {
	B2Assert(fixtureA.GetType() == B2Shape_Type.E_circle)
	B2Assert(fixtureB.GetType() == B2Shape_Type.E_circle)
	res := &B2CircleContact{
		B2Contact: MakeB2Contact(fixtureA, 0, fixtureB, 0),
	}

	return res
}

func B2CircleContact_Destroy(contact B2ContactInterface) { // should be a pointer
}

func (contact *B2CircleContact) Evaluate(manifold *B2Manifold, xfA B2Transform, xfB B2Transform) {
	B2CollideCircles(
		manifold,
		contact.GetFixtureA().GetShape().(*B2CircleShape), xfA,
		contact.GetFixtureB().GetShape().(*B2CircleShape), xfB,
	)
}
