package box2d

import (
	"math"
)

/// Friction mixing law. The idea is to allow either fixture to drive the friction to zero.
/// For example, anything slides on ice.
func B2MixFriction(friction1, friction2 float64) float64 {
	return math.Sqrt(friction1 * friction2)
}

/// Restitution mixing law. The idea is allow for anything to bounce off an inelastic surface.
/// For example, a superball bounces on anything.
func B2MixRestitution(restitution1, restitution2 float64) float64 {
	if restitution1 > restitution2 {
		return restitution1
	}

	return restitution2
}

type B2ContactCreateFcn func(fixtureA *B2Fixture, indexA int, fixtureB *B2Fixture, indexB int) B2ContactInterface // returned contact should be a pointer
type B2ContactDestroyFcn func(contact B2ContactInterface)                                                         // contact should be a pointer

type B2ContactRegister struct {
	CreateFcn  B2ContactCreateFcn
	DestroyFcn B2ContactDestroyFcn
	Primary    bool
}

/// A contact edge is used to connect bodies and contacts together
/// in a contact graph where each body is a node and each contact
/// is an edge. A contact edge belongs to a doubly linked list
/// maintained in each attached body. Each contact has two contact
/// nodes, one for each attached body.
type B2ContactEdge struct {
	Other   *B2Body            ///< provides quick access to the other body attached.
	Contact B2ContactInterface ///< the contact
	Prev    *B2ContactEdge     ///< the previous contact edge in the body's contact list
	Next    *B2ContactEdge     ///< the next contact edge in the body's contact list
}

func NewB2ContactEdge() *B2ContactEdge {
	return &B2ContactEdge{}
}

var B2Contact_Flag = struct {
	// Used when crawling contact graph when forming islands.
	E_islandFlag uint32

	// Set when the shapes are touching.
	E_touchingFlag uint32

	// This contact can be disabled (by user)
	E_enabledFlag uint32

	// This contact needs filtering because a fixture filter was changed.
	E_filterFlag uint32

	// This bullet contact had a TOI event
	E_bulletHitFlag uint32

	// This contact has a valid TOI in m_toi
	E_toiFlag uint32
}{
	E_islandFlag:    0x0001,
	E_touchingFlag:  0x0002,
	E_enabledFlag:   0x0004,
	E_filterFlag:    0x0008,
	E_bulletHitFlag: 0x0010,
	E_toiFlag:       0x0020,
}

// /// The class manages contact between two shapes. A contact exists for each overlapping
// /// AABB in the broad-phase (except if filtered). Therefore a contact object may exist
// /// that has no contact points.
var s_registers [][]B2ContactRegister
var s_initialized = false

type B2ContactInterface interface {
	GetFlags() uint32
	SetFlags(flags uint32)

	GetPrev() B2ContactInterface
	SetPrev(prev B2ContactInterface)

	GetNext() B2ContactInterface
	SetNext(prev B2ContactInterface)

	GetNodeA() *B2ContactEdge
	SetNodeA(node *B2ContactEdge)

	GetNodeB() *B2ContactEdge
	SetNodeB(node *B2ContactEdge)

	GetFixtureA() *B2Fixture
	SetFixtureA(fixture *B2Fixture)

	GetFixtureB() *B2Fixture
	SetFixtureB(fixture *B2Fixture)

	GetChildIndexA() int
	SetChildIndexA(index int)

	GetChildIndexB() int
	SetChildIndexB(index int)

	GetManifold() *B2Manifold
	SetManifold(manifold *B2Manifold)

	GetTOICount() int
	SetTOICount(toiCount int)

	GetTOI() float64
	SetTOI(toiCount float64)

	GetFriction() float64
	SetFriction(friction float64)
	ResetFriction()

	GetRestitution() float64
	SetRestitution(restitution float64)
	ResetRestitution()

	GetTangentSpeed() float64
	SetTangentSpeed(tangentSpeed float64)

	IsTouching() bool
	IsEnabled() bool
	SetEnabled(bool)

	Evaluate(manifold *B2Manifold, xfA B2Transform, xfB B2Transform)

	FlagForFiltering()

	GetWorldManifold(worldManifold *B2WorldManifold)
}

type B2Contact struct {
	M_flags uint32

	// World pool and list pointers.
	M_prev B2ContactInterface //should be backed by a pointer
	M_next B2ContactInterface //should be backed by a pointer

	// Nodes for connecting bodies.
	M_nodeA *B2ContactEdge
	M_nodeB *B2ContactEdge

	M_fixtureA *B2Fixture
	M_fixtureB *B2Fixture

	M_indexA int
	M_indexB int

	M_manifold *B2Manifold

	M_toiCount     int
	M_toi          float64
	M_friction     float64
	M_restitution  float64
	M_tangentSpeed float64
}

func (contact B2Contact) GetFlags() uint32 {
	return contact.M_flags
}

func (contact *B2Contact) SetFlags(flags uint32) {
	contact.M_flags = flags
}

func (contact B2Contact) GetPrev() B2ContactInterface {
	return contact.M_prev
}

func (contact *B2Contact) SetPrev(prev B2ContactInterface) {
	contact.M_prev = prev
}

func (contact B2Contact) GetNext() B2ContactInterface {
	return contact.M_next
}

func (contact *B2Contact) SetNext(next B2ContactInterface) {
	contact.M_next = next
}

func (contact B2Contact) GetNodeA() *B2ContactEdge {
	return contact.M_nodeA
}

func (contact *B2Contact) SetNodeA(node *B2ContactEdge) {
	contact.M_nodeA = node
}

func (contact B2Contact) GetNodeB() *B2ContactEdge {
	return contact.M_nodeB
}

func (contact *B2Contact) SetNodeB(node *B2ContactEdge) {
	contact.M_nodeB = node
}

func (contact B2Contact) GetFixtureA() *B2Fixture {
	return contact.M_fixtureA
}

func (contact *B2Contact) SetFixtureA(fixture *B2Fixture) {
	contact.M_fixtureA = fixture
}

func (contact B2Contact) GetFixtureB() *B2Fixture {
	return contact.M_fixtureB
}

func (contact *B2Contact) SetFixtureB(fixture *B2Fixture) {
	contact.M_fixtureB = fixture
}

func (contact B2Contact) GetChildIndexA() int {
	return contact.M_indexA
}

func (contact *B2Contact) SetChildIndexA(index int) {
	contact.M_indexA = index
}

func (contact B2Contact) GetChildIndexB() int {
	return contact.M_indexB
}

func (contact *B2Contact) SetChildIndexB(index int) {
	contact.M_indexB = index
}

func (contact B2Contact) GetManifold() *B2Manifold {
	return contact.M_manifold
}

func (contact *B2Contact) SetManifold(manifold *B2Manifold) {
	contact.M_manifold = manifold
}

func (contact B2Contact) GetTOICount() int {
	return contact.M_toiCount
}

func (contact *B2Contact) SetTOICount(toiCount int) {
	contact.M_toiCount = toiCount
}

func (contact B2Contact) GetTOI() float64 {
	return contact.M_toi
}

func (contact *B2Contact) SetTOI(toi float64) {
	contact.M_toi = toi
}

func (contact B2Contact) GetFriction() float64 {
	return contact.M_friction
}

func (contact *B2Contact) SetFriction(friction float64) {
	contact.M_friction = friction
}

func (contact *B2Contact) ResetFriction() {
	contact.M_friction = B2MixFriction(contact.M_fixtureA.M_friction, contact.M_fixtureB.M_friction)
}

func (contact B2Contact) GetRestitution() float64 {
	return contact.M_restitution
}

func (contact *B2Contact) SetRestitution(restitution float64) {
	contact.M_restitution = restitution
}

func (contact *B2Contact) ResetRestitution() {
	contact.M_restitution = B2MixRestitution(contact.M_fixtureA.M_restitution, contact.M_fixtureB.M_restitution)
}

func (contact B2Contact) GetTangentSpeed() float64 {
	return contact.M_tangentSpeed
}

func (contact *B2Contact) SetTangentSpeed(speed float64) {
	contact.M_tangentSpeed = speed
}

func (contact B2Contact) GetWorldManifold(worldManifold *B2WorldManifold) {
	bodyA := contact.M_fixtureA.GetBody()
	bodyB := contact.M_fixtureB.GetBody()
	shapeA := contact.M_fixtureA.GetShape()
	shapeB := contact.M_fixtureB.GetShape()

	worldManifold.Initialize(contact.M_manifold, bodyA.GetTransform(), shapeA.GetRadius(), bodyB.GetTransform(), shapeB.GetRadius())
}

func (contact *B2Contact) SetEnabled(flag bool) {
	if flag {
		contact.M_flags |= B2Contact_Flag.E_enabledFlag
	} else {
		contact.M_flags &= ^B2Contact_Flag.E_enabledFlag
	}
}

func (contact B2Contact) IsEnabled() bool {
	return (contact.M_flags & B2Contact_Flag.E_enabledFlag) == B2Contact_Flag.E_enabledFlag
}

func (contact B2Contact) IsTouching() bool {
	return (contact.M_flags & B2Contact_Flag.E_touchingFlag) == B2Contact_Flag.E_touchingFlag
}

func (contact *B2Contact) FlagForFiltering() {
	contact.M_flags |= B2Contact_Flag.E_filterFlag
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// B2Contact.cpp
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

func B2ContactInitializeRegisters() {
	s_registers = make([][]B2ContactRegister, B2Shape_Type.E_typeCount)
	for i := 0; i < int(B2Shape_Type.E_typeCount); i++ {
		s_registers[i] = make([]B2ContactRegister, B2Shape_Type.E_typeCount)
	}

	AddType(B2CircleContact_Create, B2CircleContact_Destroy, B2Shape_Type.E_circle, B2Shape_Type.E_circle)
	AddType(B2PolygonAndCircleContact_Create, B2PolygonAndCircleContact_Destroy, B2Shape_Type.E_polygon, B2Shape_Type.E_circle)
	AddType(B2PolygonContact_Create, B2PolygonContact_Destroy, B2Shape_Type.E_polygon, B2Shape_Type.E_polygon)
	AddType(B2EdgeAndCircleContact_Create, B2EdgeAndCircleContact_Destroy, B2Shape_Type.E_edge, B2Shape_Type.E_circle)
	AddType(B2EdgeAndPolygonContact_Create, B2EdgeAndPolygonContact_Destroy, B2Shape_Type.E_edge, B2Shape_Type.E_polygon)
	AddType(B2ChainAndCircleContact_Create, B2ChainAndCircleContact_Destroy, B2Shape_Type.E_chain, B2Shape_Type.E_circle)
	AddType(B2ChainAndPolygonContact_Create, B2ChainAndPolygonContact_Destroy, B2Shape_Type.E_chain, B2Shape_Type.E_polygon)
}

func AddType(createFcn B2ContactCreateFcn, destroyFcn B2ContactDestroyFcn, type1 uint8, type2 uint8) {
	B2Assert(0 <= type1 && type1 < B2Shape_Type.E_typeCount)
	B2Assert(0 <= type2 && type2 < B2Shape_Type.E_typeCount)

	s_registers[type1][type2].CreateFcn = createFcn
	s_registers[type1][type2].DestroyFcn = destroyFcn
	s_registers[type1][type2].Primary = true

	if type1 != type2 {
		s_registers[type2][type1].CreateFcn = createFcn
		s_registers[type2][type1].DestroyFcn = destroyFcn
		s_registers[type2][type1].Primary = false
	}
}

func B2ContactFactory(fixtureA *B2Fixture, indexA int, fixtureB *B2Fixture, indexB int) B2ContactInterface { // returned contact should be a pointer

	if s_initialized == false {
		B2ContactInitializeRegisters()
		s_initialized = true
	}

	type1 := fixtureA.GetType()
	type2 := fixtureB.GetType()

	B2Assert(0 <= type1 && type1 < B2Shape_Type.E_typeCount)
	B2Assert(0 <= type2 && type2 < B2Shape_Type.E_typeCount)

	createFcn := s_registers[type1][type2].CreateFcn
	if createFcn != nil {
		if s_registers[type1][type2].Primary {
			return createFcn(fixtureA, indexA, fixtureB, indexB)
		} else {
			return createFcn(fixtureB, indexB, fixtureA, indexA)
		}
	}

	return nil
}

func B2ContactDestroy(contact B2ContactInterface) {
	B2Assert(s_initialized == true)

	fixtureA := contact.GetFixtureA()
	fixtureB := contact.GetFixtureB()

	if contact.GetManifold().PointCount > 0 && fixtureA.IsSensor() == false && fixtureB.IsSensor() == false {
		fixtureA.GetBody().SetAwake(true)
		fixtureB.GetBody().SetAwake(true)
	}

	typeA := fixtureA.GetType()
	typeB := fixtureB.GetType()

	B2Assert(0 <= typeA && typeB < B2Shape_Type.E_typeCount)

	destroyFcn := s_registers[typeA][typeB].DestroyFcn
	destroyFcn(contact)
}

func MakeB2Contact(fA *B2Fixture, indexA int, fB *B2Fixture, indexB int) B2Contact {

	contact := B2Contact{}
	contact.M_flags = B2Contact_Flag.E_enabledFlag

	contact.M_fixtureA = fA
	contact.M_fixtureB = fB

	contact.M_indexA = indexA
	contact.M_indexB = indexB

	contact.M_manifold = NewB2Manifold()
	contact.M_manifold.PointCount = 0

	contact.M_prev = nil
	contact.M_next = nil

	contact.M_nodeA = NewB2ContactEdge()

	contact.M_nodeA.Contact = nil
	contact.M_nodeA.Prev = nil
	contact.M_nodeA.Next = nil
	contact.M_nodeA.Other = nil

	contact.M_nodeB = NewB2ContactEdge()

	contact.M_nodeB.Contact = nil
	contact.M_nodeB.Prev = nil
	contact.M_nodeB.Next = nil
	contact.M_nodeB.Other = nil

	contact.M_toiCount = 0

	contact.M_friction = B2MixFriction(contact.M_fixtureA.M_friction, contact.M_fixtureB.M_friction)
	contact.M_restitution = B2MixRestitution(contact.M_fixtureA.M_restitution, contact.M_fixtureB.M_restitution)

	contact.M_tangentSpeed = 0.0

	return contact
}

// Update the contact manifold and touching status.
// Note: do not assume the fixture AABBs are overlapping or are valid.
func B2ContactUpdate(contact B2ContactInterface, listener B2ContactListenerInterface) {
	oldManifold := *contact.GetManifold()

	// Re-enable this contact.
	contact.SetFlags(contact.GetFlags() | B2Contact_Flag.E_enabledFlag)

	touching := false
	wasTouching := (contact.GetFlags() & B2Contact_Flag.E_touchingFlag) == B2Contact_Flag.E_touchingFlag

	sensorA := contact.GetFixtureA().IsSensor()
	sensorB := contact.GetFixtureB().IsSensor()
	sensor := sensorA || sensorB

	bodyA := contact.GetFixtureA().GetBody()
	bodyB := contact.GetFixtureB().GetBody()
	xfA := bodyA.GetTransform()
	xfB := bodyB.GetTransform()

	// Is this contact a sensor?
	if sensor {
		shapeA := contact.GetFixtureA().GetShape()
		shapeB := contact.GetFixtureB().GetShape()
		touching = B2TestOverlapShapes(shapeA, contact.GetChildIndexA(), shapeB, contact.GetChildIndexB(), xfA, xfB)

		// Sensors don't generate manifolds.
		contact.GetManifold().PointCount = 0
	} else {
		// *B2Contact is extended by specialized contact structs and mentionned by B2ContactInterface but not implemented on specialized structs
		// Thus when
		//spew.Dump("AVANT", contact.GetManifold())
		contact.Evaluate(contact.GetManifold(), xfA, xfB) // should be evaluated on specialisations of contact (like CircleContact)
		//spew.Dump("APRES", contact.GetManifold())
		touching = contact.GetManifold().PointCount > 0

		// Match old contact ids to new contact ids and copy the
		// stored impulses to warm start the solver.
		for i := 0; i < contact.GetManifold().PointCount; i++ {
			mp2 := &contact.GetManifold().Points[i]
			mp2.NormalImpulse = 0.0
			mp2.TangentImpulse = 0.0
			id2 := mp2.Id

			for j := 0; j < oldManifold.PointCount; j++ {
				mp1 := &oldManifold.Points[j]

				if mp1.Id.Key() == id2.Key() {
					mp2.NormalImpulse = mp1.NormalImpulse
					mp2.TangentImpulse = mp1.TangentImpulse
					break
				}
			}
		}

		if touching != wasTouching {
			bodyA.SetAwake(true)
			bodyB.SetAwake(true)
		}
	}

	if touching {
		contact.SetFlags(contact.GetFlags() | B2Contact_Flag.E_touchingFlag)
	} else {
		contact.SetFlags(contact.GetFlags() & ^B2Contact_Flag.E_touchingFlag)
	}

	if wasTouching == false && touching == true && listener != nil {
		listener.BeginContact(contact)
	}

	if wasTouching == true && touching == false && listener != nil {
		listener.EndContact(contact)
	}

	if sensor == false && touching && listener != nil {
		listener.PreSolve(contact, oldManifold)
	}
}
