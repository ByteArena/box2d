package box2d

type B2ChainAndPolygonContact struct {
	B2Contact
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// B2ChainAndPolygonContact.cpp
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

func B2ChainAndPolygonContact_Create(fixtureA *B2Fixture, indexA int, fixtureB *B2Fixture, indexB int) B2ContactInterface {
	B2Assert(fixtureA.GetType() == B2Shape_Type.E_chain)
	B2Assert(fixtureB.GetType() == B2Shape_Type.E_polygon)
	res := &B2ChainAndPolygonContact{
		B2Contact: MakeB2Contact(fixtureA, indexA, fixtureB, indexB),
	}

	return res
}

func B2ChainAndPolygonContact_Destroy(contact B2ContactInterface) { // should be a pointer
}

func (contact *B2ChainAndPolygonContact) Evaluate(manifold *B2Manifold, xfA B2Transform, xfB B2Transform) {
	chain := contact.GetFixtureA().GetShape().(*B2ChainShape)
	edge := MakeB2EdgeShape()
	chain.GetChildEdge(&edge, contact.M_indexA)
	B2CollideEdgeAndPolygon(manifold, &edge, xfA, contact.GetFixtureB().GetShape().(*B2PolygonShape), xfB)
}
