package box2d

type B2ContactManager struct {
	M_broadPhase      B2BroadPhase
	M_contactList     B2ContactInterface
	M_contactCount    int
	M_contactFilter   B2ContactFilterInterface
	M_contactListener B2ContactListenerInterface
}

var b2_defaultFilter B2ContactFilterInterface
var b2_defaultListener B2ContactListenerInterface

func MakeB2ContactManager() B2ContactManager {
	return B2ContactManager{
		M_broadPhase:      MakeB2BroadPhase(),
		M_contactList:     nil,
		M_contactCount:    0,
		M_contactFilter:   b2_defaultFilter,
		M_contactListener: b2_defaultListener,
	}
}

func NewB2ContactManager() *B2ContactManager {
	res := MakeB2ContactManager()
	return &res
}

func (mgr *B2ContactManager) Destroy(c B2ContactInterface) {
	fixtureA := c.GetFixtureA()
	fixtureB := c.GetFixtureB()
	bodyA := fixtureA.GetBody()
	bodyB := fixtureB.GetBody()

	if mgr.M_contactListener != nil && c.IsTouching() {
		mgr.M_contactListener.EndContact(c)
	}

	// Remove from the world.
	if c.GetPrev() != nil {
		c.GetPrev().SetNext(c.GetNext())
	}

	if c.GetNext() != nil {
		c.GetNext().SetPrev(c.GetPrev())
	}

	if c == mgr.M_contactList {
		mgr.M_contactList = c.GetNext()
	}

	// Remove from body 1
	if c.GetNodeA().Prev != nil {
		c.GetNodeA().Prev.Next = c.GetNodeA().Next
	}

	if c.GetNodeA().Next != nil {
		c.GetNodeA().Next.Prev = c.GetNodeA().Prev
	}

	if c.GetNodeA() == bodyA.M_contactList {
		bodyA.M_contactList = c.GetNodeA().Next
	}

	// Remove from body 2
	if c.GetNodeB().Prev != nil {
		c.GetNodeB().Prev.Next = c.GetNodeB().Next
	}

	if c.GetNodeB().Next != nil {
		c.GetNodeB().Next.Prev = c.GetNodeB().Prev
	}

	if c.GetNodeB() == bodyB.M_contactList {
		bodyB.M_contactList = c.GetNodeB().Next
	}

	// Call the factory.
	B2ContactDestroy(c)
	mgr.M_contactCount--
}

// This is the top level collision call for the time step. Here
// all the narrow phase collision is processed for the world
// contact list.
func (mgr *B2ContactManager) Collide() {
	// Update awake contacts.
	c := mgr.M_contactList

	for c != nil {
		fixtureA := c.GetFixtureA()
		fixtureB := c.GetFixtureB()
		indexA := c.GetChildIndexA()
		indexB := c.GetChildIndexB()
		bodyA := fixtureA.GetBody()
		bodyB := fixtureB.GetBody()

		// Is this contact flagged for filtering?
		if (c.GetFlags() & B2Contact_Flag.E_filterFlag) != 0x0000 {
			// Should these bodies collide?
			if bodyB.ShouldCollide(bodyA) == false {
				cNuke := c
				c = cNuke.GetNext()
				mgr.Destroy(cNuke)
				continue
			}

			// Check user filtering.
			if mgr.M_contactFilter != nil && mgr.M_contactFilter.ShouldCollide(fixtureA, fixtureB) == false {
				cNuke := c
				c = cNuke.GetNext()
				mgr.Destroy(cNuke)
				continue
			}

			// Clear the filtering flag.
			c.SetFlags(c.GetFlags() & ^B2Contact_Flag.E_filterFlag)
		}

		activeA := bodyA.IsAwake() && bodyA.M_type != B2BodyType.B2_staticBody
		activeB := bodyB.IsAwake() && bodyB.M_type != B2BodyType.B2_staticBody

		// At least one body must be awake and it must be dynamic or kinematic.
		if activeA == false && activeB == false {
			c = c.GetNext()
			continue
		}

		proxyIdA := fixtureA.M_proxies[indexA].ProxyId
		proxyIdB := fixtureB.M_proxies[indexB].ProxyId
		overlap := mgr.M_broadPhase.TestOverlap(proxyIdA, proxyIdB)

		// Here we destroy contacts that cease to overlap in the broad-phase.
		if overlap == false {
			cNuke := c
			c = cNuke.GetNext()
			mgr.Destroy(cNuke)
			continue
		}

		// The contact persists.
		B2ContactUpdate(c, mgr.M_contactListener)
		c = c.GetNext()
	}
}

func (mgr *B2ContactManager) FindNewContacts() {
	mgr.M_broadPhase.UpdatePairs(mgr.AddPair)
}

func (mgr *B2ContactManager) AddPair(proxyUserDataA interface{}, proxyUserDataB interface{}) {

	proxyA := proxyUserDataA.(*B2FixtureProxy)
	proxyB := proxyUserDataB.(*B2FixtureProxy)

	fixtureA := proxyA.Fixture
	fixtureB := proxyB.Fixture

	indexA := proxyA.ChildIndex
	indexB := proxyB.ChildIndex

	bodyA := fixtureA.GetBody()
	bodyB := fixtureB.GetBody()

	// Are the fixtures on the same body?
	if bodyA == bodyB {
		return
	}

	// TODO_ERIN use a hash table to remove a potential bottleneck when both
	// bodies have a lot of contacts.
	// Does a contact already exist?
	edge := bodyB.GetContactList()
	for edge != nil {
		if edge.Other == bodyA {
			fA := edge.Contact.GetFixtureA()
			fB := edge.Contact.GetFixtureB()
			iA := edge.Contact.GetChildIndexA()
			iB := edge.Contact.GetChildIndexB()

			if fA == fixtureA && fB == fixtureB && iA == indexA && iB == indexB {
				// A contact already exists.
				return
			}

			if fA == fixtureB && fB == fixtureA && iA == indexB && iB == indexA {
				// A contact already exists.
				return
			}
		}

		edge = edge.Next
	}

	// Does a joint override collision? Is at least one body dynamic?
	if bodyB.ShouldCollide(bodyA) == false {
		return
	}

	// Check user filtering.
	if mgr.M_contactFilter != nil && mgr.M_contactFilter.ShouldCollide(fixtureA, fixtureB) == false {
		return
	}

	// Call the factory.
	c := B2ContactFactory(fixtureA, indexA, fixtureB, indexB)
	if c == nil {
		return
	}

	// Contact creation may swap fixtures.
	fixtureA = c.GetFixtureA()
	fixtureB = c.GetFixtureB()
	indexA = c.GetChildIndexA()
	indexB = c.GetChildIndexB()
	bodyA = fixtureA.GetBody()
	bodyB = fixtureB.GetBody()

	// Insert into the world.
	c.SetPrev(nil)
	c.SetNext(mgr.M_contactList)
	if mgr.M_contactList != nil {
		mgr.M_contactList.SetPrev(c)
	}
	mgr.M_contactList = c

	// Connect to island graph.

	// Connect to body A
	// fmt.Printf("getNode(): %p\n", c.GetNodeA())
	// fmt.Printf("getNode(): %p\n", c.GetNodeA())
	// fmt.Printf("getNode(): %p\n", c.GetNodeA())

	c.GetNodeA().Contact = c
	c.GetNodeA().Other = bodyB

	c.GetNodeA().Prev = nil
	c.GetNodeA().Next = bodyA.M_contactList
	if bodyA.M_contactList != nil {
		bodyA.M_contactList.Prev = c.GetNodeA()
	}
	bodyA.M_contactList = c.GetNodeA()

	// Connect to body B
	c.GetNodeB().Contact = c
	c.GetNodeB().Other = bodyA

	c.GetNodeB().Prev = nil
	c.GetNodeB().Next = bodyB.M_contactList
	if bodyB.M_contactList != nil {
		bodyB.M_contactList.Prev = c.GetNodeB()
	}
	bodyB.M_contactList = c.GetNodeB()

	// Wake up the bodies
	if fixtureA.IsSensor() == false && fixtureB.IsSensor() == false {
		bodyA.SetAwake(true)
		bodyB.SetAwake(true)
	}

	mgr.M_contactCount++
}
