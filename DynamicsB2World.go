package box2d

import (
	"fmt"
	"math"
)

/// The world class manages all physics entities, dynamic simulation,
/// and asynchronous queries. The world also contains efficient memory
/// management facilities.

var B2World_Flags = struct {
	E_newFixture  int
	E_locked      int
	E_clearForces int
}{
	E_newFixture:  0x0001,
	E_locked:      0x0002,
	E_clearForces: 0x0004,
}

// /// The world class manages all physics entities, dynamic simulation,
// /// and asynchronous queries. The world also contains efficient memory
// /// management facilities.
type B2World struct {
	M_flags int

	M_contactManager B2ContactManager

	M_bodyList  *B2Body          // linked list
	M_jointList B2JointInterface // has to be backed by pointer

	M_bodyCount  int
	M_jointCount int

	M_gravity    B2Vec2
	M_allowSleep bool

	M_destructionListener B2DestructionListenerInterface
	//G_debugDraw           *B2Draw

	// This is used to compute the time step ratio to
	// support a variable time step.
	M_inv_dt0 float64

	// These are for debugging the solver.
	M_warmStarting      bool
	M_continuousPhysics bool
	M_subStepping       bool

	M_stepComplete bool

	M_profile B2Profile
}

func (world B2World) GetBodyList() *B2Body {
	return world.M_bodyList
}

func (world B2World) GetJointList() B2JointInterface { // returns a pointer
	return world.M_jointList
}

func (world B2World) GetContactList() B2ContactInterface { // returns a pointer
	return world.M_contactManager.M_contactList
}

func (world B2World) GetBodyCount() int {
	return world.M_bodyCount
}

func (world B2World) GetJointCount() int {
	return world.M_jointCount
}

func (world B2World) GetContactCount() int {
	return world.M_contactManager.M_contactCount
}

func (world *B2World) SetGravity(gravity B2Vec2) {
	world.M_gravity = gravity
}

func (world B2World) GetGravity() B2Vec2 {
	return world.M_gravity
}

func (world B2World) IsLocked() bool {
	return (world.M_flags & B2World_Flags.E_locked) == B2World_Flags.E_locked
}

func (world *B2World) SetAutoClearForces(flag bool) {
	if flag {
		world.M_flags |= B2World_Flags.E_clearForces
	} else {
		world.M_flags &= ^B2World_Flags.E_clearForces
	}
}

/// Get the flag that controls automatic clearing of forces after each time step.
func (world B2World) GetAutoClearForces() bool {
	return (world.M_flags & B2World_Flags.E_clearForces) == B2World_Flags.E_clearForces
}

func (world B2World) GetContactManager() B2ContactManager {
	return world.M_contactManager
}

func (world B2World) GetProfile() B2Profile {
	return world.M_profile
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// B2World.cpp
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

func MakeB2World(gravity B2Vec2) B2World {

	world := B2World{}

	world.M_destructionListener = nil
	//world.G_debugDraw = nil

	world.M_bodyList = nil
	world.M_jointList = nil

	world.M_bodyCount = 0
	world.M_jointCount = 0

	world.M_warmStarting = true
	world.M_continuousPhysics = true
	world.M_subStepping = false

	world.M_stepComplete = true

	world.M_allowSleep = true
	world.M_gravity = gravity

	world.M_flags = B2World_Flags.E_clearForces

	world.M_inv_dt0 = 0.0

	world.M_contactManager = MakeB2ContactManager()

	return world
}

func (world *B2World) Destroy() {

	// Some shapes allocate using b2Alloc.
	b := world.M_bodyList
	for b != nil {
		bNext := b.M_next

		f := b.M_fixtureList
		for f != nil {
			fNext := f.M_next
			f.M_proxyCount = 0
			f.Destroy()
			f = fNext
		}

		b = bNext
	}
}

func (world *B2World) SetDestructionListener(listener B2DestructionListenerInterface) {
	world.M_destructionListener = listener
}

func (world *B2World) SetContactFilter(filter B2ContactFilterInterface) {
	world.M_contactManager.M_contactFilter = filter
}

func (world *B2World) SetContactListener(listener B2ContactListenerInterface) {
	world.M_contactManager.M_contactListener = listener
}

// void (world *B2World) SetDebugDraw(b2Draw* debugDraw)
// {
// 	g_debugDraw = debugDraw;
// }

func (world *B2World) CreateBody(def *B2BodyDef) *B2Body {
	B2Assert(world.IsLocked() == false)

	if world.IsLocked() {
		return nil
	}

	b := NewB2Body(def, world)

	// Add to world doubly linked list.
	b.M_prev = nil
	b.M_next = world.M_bodyList
	if world.M_bodyList != nil {
		world.M_bodyList.M_prev = b
	}
	world.M_bodyList = b
	world.M_bodyCount++

	return b
}

func (world *B2World) DestroyBody(b *B2Body) {
	B2Assert(world.M_bodyCount > 0)
	B2Assert(world.IsLocked() == false)

	if world.IsLocked() {
		return
	}

	// Delete the attached joints.
	je := b.M_jointList
	for je != nil {
		je0 := je
		je = je.Next

		if world.M_destructionListener != nil {
			world.M_destructionListener.SayGoodbyeToJoint(je0.Joint)
		}

		world.DestroyJoint(je0.Joint)

		b.M_jointList = je
	}
	b.M_jointList = nil

	// Delete the attached contacts.
	ce := b.M_contactList
	for ce != nil {
		ce0 := ce
		ce = ce.Next
		world.M_contactManager.Destroy(ce0.Contact)
	}
	b.M_contactList = nil

	// Delete the attached fixtures. This destroys broad-phase proxies.
	f := b.M_fixtureList
	for f != nil {
		f0 := f
		f = f.M_next

		if world.M_destructionListener != nil {
			world.M_destructionListener.SayGoodbyeToFixture(f0)
		}

		f0.DestroyProxies(&world.M_contactManager.M_broadPhase)
		f0.Destroy()

		b.M_fixtureList = f
		b.M_fixtureCount -= 1
	}

	b.M_fixtureList = nil
	b.M_fixtureCount = 0

	// Remove world body list.
	if b.M_prev != nil {
		b.M_prev.M_next = b.M_next
	}

	if b.M_next != nil {
		b.M_next.M_prev = b.M_prev
	}

	if b == world.M_bodyList {
		world.M_bodyList = b.M_next
	}

	world.M_bodyCount--
}

func (world *B2World) CreateJoint(def *B2JointDef) B2JointInterface {
	B2Assert(world.IsLocked() == false)
	if world.IsLocked() {
		return nil
	}

	j := B2JointCreate(def)

	// Connect to the world list.
	j.SetPrev(nil)
	j.SetNext(world.M_jointList)
	if world.M_jointList != nil {
		world.M_jointList.SetPrev(j)
	}
	world.M_jointList = j
	world.M_jointCount++

	// Connect to the bodies' doubly linked lists.
	j.GetEdgeA().Joint = j
	j.GetEdgeA().Other = j.GetBodyB()
	j.GetEdgeA().Prev = nil
	j.GetEdgeA().Next = j.GetBodyA().M_jointList
	if j.GetBodyA().M_jointList != nil {
		j.GetBodyA().M_jointList.Prev = j.GetEdgeA()
	}

	j.GetBodyA().M_jointList = j.GetEdgeA()

	j.GetEdgeB().Joint = j
	j.GetEdgeB().Other = j.GetBodyA()
	j.GetEdgeB().Prev = nil
	j.GetEdgeB().Next = j.GetBodyB().M_jointList
	if j.GetBodyB().M_jointList != nil {
		j.GetBodyB().M_jointList.Prev = j.GetEdgeB()
	}
	j.GetBodyB().M_jointList = j.GetEdgeB()

	bodyA := def.BodyA
	bodyB := def.BodyB

	// If the joint prevents collisions, then flag any contacts for filtering.
	if def.CollideConnected == false {
		edge := bodyB.GetContactList()
		for edge != nil {
			if edge.Other == bodyA {
				// Flag the contact for filtering at the next time step (where either
				// body is awake).
				edge.Contact.FlagForFiltering()
			}

			edge = edge.Next
		}
	}

	// Note: creating a joint doesn't wake the bodies.

	return j
}

func (world *B2World) DestroyJoint(j B2JointInterface) { // j backed by pointer
	B2Assert(world.IsLocked() == false)
	if world.IsLocked() {
		return
	}

	collideConnected := j.IsCollideConnected()

	// Remove from the doubly linked list.
	if j.GetPrev() != nil {
		j.GetPrev().SetNext(j.GetNext())
	}

	if j.GetNext() != nil {
		j.GetNext().SetPrev(j.GetPrev())
	}

	if j == world.M_jointList {
		world.M_jointList = j.GetNext()
	}

	// Disconnect from island graph.
	bodyA := j.GetBodyA()
	bodyB := j.GetBodyB()

	// Wake up connected bodies.
	bodyA.SetAwake(true)
	bodyB.SetAwake(true)

	// Remove from body 1.
	if j.GetEdgeA().Prev != nil {
		j.GetEdgeA().Prev.Next = j.GetEdgeA().Next
	}

	if j.GetEdgeA().Next != nil {
		j.GetEdgeA().Next.Prev = j.GetEdgeA().Prev
	}

	if j.GetEdgeA() == bodyA.M_jointList {
		bodyA.M_jointList = j.GetEdgeA().Next
	}

	j.GetEdgeA().Prev = nil
	j.GetEdgeA().Next = nil

	// Remove from body 2
	if j.GetEdgeB().Prev != nil {
		j.GetEdgeB().Prev.Next = j.GetEdgeB().Next
	}

	if j.GetEdgeB().Next != nil {
		j.GetEdgeB().Next.Prev = j.GetEdgeB().Prev
	}

	if j.GetEdgeB() == bodyB.M_jointList {
		bodyB.M_jointList = j.GetEdgeB().Next
	}

	j.GetEdgeB().Prev = nil
	j.GetEdgeB().Next = nil

	B2JointDestroy(j)

	B2Assert(world.M_jointCount > 0)
	world.M_jointCount--

	// If the joint prevents collisions, then flag any contacts for filtering.
	if collideConnected == false {
		edge := bodyB.GetContactList()
		for edge != nil {
			if edge.Other == bodyA {
				// Flag the contact for filtering at the next time step (where either
				// body is awake).
				edge.Contact.FlagForFiltering()
			}

			edge = edge.Next
		}
	}
}

func (world *B2World) SetAllowSleeping(flag bool) {
	if flag == world.M_allowSleep {
		return
	}

	world.M_allowSleep = flag
	if world.M_allowSleep == false {
		for b := world.M_bodyList; b != nil; b = b.M_next {
			b.SetAwake(true)
		}
	}
}

// Find islands, integrate and solve constraints, solve position constraints
func (world *B2World) Solve(step B2TimeStep) {
	world.M_profile.SolveInit = 0.0
	world.M_profile.SolveVelocity = 0.0
	world.M_profile.SolvePosition = 0.0

	// Size the island for the worst case.
	island := MakeB2Island(
		world.M_bodyCount,
		world.M_contactManager.M_contactCount,
		world.M_jointCount,
		world.M_contactManager.M_contactListener,
	)

	// Clear all the island flags.
	for b := world.M_bodyList; b != nil; b = b.M_next {
		b.M_flags &= ^B2Body_Flags.E_islandFlag
	}
	for c := world.M_contactManager.M_contactList; c != nil; c = c.GetNext() {
		c.SetFlags(c.GetFlags() & ^B2Body_Flags.E_islandFlag)
	}

	for j := world.M_jointList; j != nil; j = j.GetNext() {
		j.SetIslandFlag(false)
	}

	// Build and simulate all awake islands.
	stackSize := world.M_bodyCount
	stack := make([]*B2Body, stackSize)

	for seed := world.M_bodyList; seed != nil; seed = seed.M_next {
		if (seed.M_flags & B2Body_Flags.E_islandFlag) != 0x0000 {
			continue
		}

		if seed.IsAwake() == false || seed.IsActive() == false {
			continue
		}

		// The seed can be dynamic or kinematic.
		if seed.GetType() == B2BodyType.B2_staticBody {
			continue
		}

		// Reset island and stack.
		island.Clear()
		stackCount := 0
		stack[stackCount] = seed
		stackCount++
		seed.M_flags |= B2Body_Flags.E_islandFlag

		// Perform a depth first search (DFS) on the constraint graph.
		for stackCount > 0 {
			// Grab the next body off the stack and add it to the island.
			stackCount--
			b := stack[stackCount]
			B2Assert(b.IsActive() == true)
			island.AddBody(b)

			// Make sure the body is awake (without resetting sleep timer).
			b.M_flags |= B2Body_Flags.E_awakeFlag

			// To keep islands as small as possible, we don't
			// propagate islands across static bodies.
			if b.GetType() == B2BodyType.B2_staticBody {
				continue
			}

			// Search all contacts connected to this body.
			for ce := b.M_contactList; ce != nil; ce = ce.Next {
				contact := ce.Contact

				// Has this contact already been added to an island?
				if (contact.GetFlags() & B2Body_Flags.E_islandFlag) != 0x0000 {
					continue
				}

				// Is this contact solid and touching?
				if contact.IsEnabled() == false || contact.IsTouching() == false {
					continue
				}

				// Skip sensors.
				sensorA := contact.GetFixtureA().M_isSensor
				sensorB := contact.GetFixtureB().M_isSensor

				if sensorA || sensorB {
					continue
				}

				island.AddContact(contact)
				contact.SetFlags(contact.GetFlags() | B2Body_Flags.E_islandFlag)

				other := ce.Other

				// Was the other body already added to this island?
				if (other.M_flags & B2Body_Flags.E_islandFlag) != 0x0000 {
					continue
				}

				B2Assert(stackCount < stackSize)
				stack[stackCount] = other
				stackCount++
				other.M_flags |= B2Body_Flags.E_islandFlag
			}

			// Search all joints connect to this body.
			for je := b.M_jointList; je != nil; je = je.Next {

				if je.Joint.GetIslandFlag() == true {
					continue
				}

				other := je.Other

				// Don't simulate joints connected to inactive bodies.
				if other.IsActive() == false {
					continue
				}

				island.Add(je.Joint)
				je.Joint.SetIslandFlag(true)

				if other.M_flags&B2Body_Flags.E_islandFlag != 0x0000 {
					continue
				}

				B2Assert(stackCount < stackSize)
				stack[stackCount] = other
				stackCount++
				other.M_flags |= B2Body_Flags.E_islandFlag
			}
		}

		profile := MakeB2Profile()
		island.Solve(&profile, step, world.M_gravity, world.M_allowSleep)
		world.M_profile.SolveInit += profile.SolveInit
		world.M_profile.SolveVelocity += profile.SolveVelocity
		world.M_profile.SolvePosition += profile.SolvePosition

		// Post solve cleanup.
		for i := 0; i < island.M_bodyCount; i++ {
			// Allow static bodies to participate in other islands.
			b := island.M_bodies[i]
			if b.GetType() == B2BodyType.B2_staticBody {
				b.M_flags &= ^B2Body_Flags.E_islandFlag
			}
		}
	}

	stack = nil

	{
		timer := MakeB2Timer()

		// Synchronize fixtures, check for out of range bodies.
		for b := world.M_bodyList; b != nil; b = b.GetNext() {
			// If a body was not in an island then it did not move.
			if (b.M_flags & B2Body_Flags.E_islandFlag) == 0 {
				continue
			}

			if b.GetType() == B2BodyType.B2_staticBody {
				continue
			}

			// Update fixtures (for broad-phase).
			b.SynchronizeFixtures()
		}

		// Look for new contacts.
		world.M_contactManager.FindNewContacts()
		world.M_profile.Broadphase = timer.GetMilliseconds()
	}
}

// Find TOI contacts and solve them.
func (world *B2World) SolveTOI(step B2TimeStep) {

	island := MakeB2Island(2*B2_maxTOIContacts, B2_maxTOIContacts, 0, world.M_contactManager.M_contactListener)

	if world.M_stepComplete {
		for b := world.M_bodyList; b != nil; b = b.M_next {
			b.M_flags &= ^B2Body_Flags.E_islandFlag
			b.M_sweep.Alpha0 = 0.0
		}

		for c := world.M_contactManager.M_contactList; c != nil; c = c.GetNext() {
			// Invalidate TOI
			c.SetFlags(c.GetFlags() & ^(B2Contact_Flag.E_toiFlag | B2Contact_Flag.E_islandFlag))
			c.SetTOICount(0)
			c.SetTOI(1.0)
		}
	}

	// Find TOI events and solve them.
	for {
		// Find the first TOI.
		var minContact B2ContactInterface = nil // has to be a pointer
		minAlpha := 1.0

		for c := world.M_contactManager.M_contactList; c != nil; c = c.GetNext() {

			// Is this contact disabled?
			if c.IsEnabled() == false {
				continue
			}

			// Prevent excessive sub-stepping.
			if c.GetTOICount() > B2_maxSubSteps {
				continue
			}

			alpha := 1.0
			if (c.GetFlags() & B2Contact_Flag.E_toiFlag) != 0x0000 {
				// This contact has a valid cached TOI.
				alpha = c.GetTOI()
			} else {
				fA := c.GetFixtureA()
				fB := c.GetFixtureB()

				// Is there a sensor?
				if fA.IsSensor() || fB.IsSensor() {
					continue
				}

				bA := fA.GetBody()
				bB := fB.GetBody()

				typeA := bA.M_type
				typeB := bB.M_type
				B2Assert(typeA == B2BodyType.B2_dynamicBody || typeB == B2BodyType.B2_dynamicBody)

				activeA := bA.IsAwake() && typeA != B2BodyType.B2_staticBody
				activeB := bB.IsAwake() && typeB != B2BodyType.B2_staticBody

				// Is at least one body active (awake and dynamic or kinematic)?
				if activeA == false && activeB == false {
					continue
				}

				collideA := bA.IsBullet() || typeA != B2BodyType.B2_dynamicBody
				collideB := bB.IsBullet() || typeB != B2BodyType.B2_dynamicBody

				// Are these two non-bullet dynamic bodies?
				if collideA == false && collideB == false {
					continue
				}

				// Compute the TOI for this contact.
				// Put the sweeps onto the same time interval.
				alpha0 := bA.M_sweep.Alpha0

				if bA.M_sweep.Alpha0 < bB.M_sweep.Alpha0 {
					alpha0 = bB.M_sweep.Alpha0
					bA.M_sweep.Advance(alpha0)
				} else if bB.M_sweep.Alpha0 < bA.M_sweep.Alpha0 {
					alpha0 = bA.M_sweep.Alpha0
					bB.M_sweep.Advance(alpha0)
				}

				B2Assert(alpha0 < 1.0)

				indexA := c.GetChildIndexA()
				indexB := c.GetChildIndexB()

				// Compute the time of impact in interval [0, minTOI]
				input := MakeB2TOIInput()
				input.ProxyA.Set(fA.GetShape(), indexA)
				input.ProxyB.Set(fB.GetShape(), indexB)
				input.SweepA = bA.M_sweep
				input.SweepB = bB.M_sweep
				input.TMax = 1.0

				output := MakeB2TOIOutput()
				B2TimeOfImpact(&output, &input)

				// Beta is the fraction of the remaining portion of the .
				beta := output.T
				if output.State == B2TOIOutput_State.E_touching {
					alpha = math.Min(alpha0+(1.0-alpha0)*beta, 1.0)
				} else {
					alpha = 1.0
				}

				c.SetTOI(alpha)
				c.SetFlags(c.GetFlags() | B2Contact_Flag.E_toiFlag)
			}

			if alpha < minAlpha {
				// This is the minimum TOI found so far.
				minContact = c
				minAlpha = alpha
			}
		}

		if minContact == nil || 1.0-10.0*B2_epsilon < minAlpha {
			// No more TOI events. Done!
			world.M_stepComplete = true
			break
		}

		// Advance the bodies to the TOI.
		fA := minContact.GetFixtureA()
		fB := minContact.GetFixtureB()
		bA := fA.GetBody()
		bB := fB.GetBody()

		backup1 := bA.M_sweep
		backup2 := bB.M_sweep

		bA.Advance(minAlpha)
		bB.Advance(minAlpha)

		// The TOI contact likely has some new contact points.
		B2ContactUpdate(minContact, world.M_contactManager.M_contactListener)
		minContact.SetFlags(minContact.GetFlags() & ^B2Contact_Flag.E_toiFlag)
		minContact.SetTOICount(minContact.GetTOICount() + 1)

		// Is the contact solid?
		if minContact.IsEnabled() == false || minContact.IsTouching() == false {
			// Restore the sweeps.
			minContact.SetEnabled(false)
			bA.M_sweep = backup1
			bB.M_sweep = backup2
			bA.SynchronizeTransform()
			bB.SynchronizeTransform()
			continue
		}

		bA.SetAwake(true)
		bB.SetAwake(true)

		// Build the island
		island.Clear()
		island.AddBody(bA)
		island.AddBody(bB)
		island.AddContact(minContact)

		bA.M_flags |= B2Body_Flags.E_islandFlag
		bB.M_flags |= B2Body_Flags.E_islandFlag
		minContact.SetFlags(minContact.GetFlags() | B2Contact_Flag.E_islandFlag)

		// Get contacts on bodyA and bodyB.
		bodies := [2]*B2Body{bA, bB}

		for i := 0; i < 2; i++ {
			body := bodies[i]
			if body.M_type == B2BodyType.B2_dynamicBody {
				for ce := body.M_contactList; ce != nil; ce = ce.Next {
					if island.M_bodyCount == island.M_bodyCapacity {
						break
					}

					if island.M_contactCount == island.M_contactCapacity {
						break
					}

					contact := ce.Contact

					// Has this contact already been added to the island?
					if (contact.GetFlags() & B2Contact_Flag.E_islandFlag) != 0x0000 {
						continue
					}

					// Only add static, kinematic, or bullet bodies.
					other := ce.Other
					if other.M_type == B2BodyType.B2_dynamicBody && body.IsBullet() == false && other.IsBullet() == false {
						continue
					}

					// Skip sensors.
					sensorA := contact.GetFixtureA().M_isSensor
					sensorB := contact.GetFixtureB().M_isSensor
					if sensorA || sensorB {
						continue
					}

					// Tentatively advance the body to the TOI.
					backup := other.M_sweep
					if (other.M_flags & B2Body_Flags.E_islandFlag) == 0 {
						other.Advance(minAlpha)
					}

					// Update the contact points
					B2ContactUpdate(contact, world.M_contactManager.M_contactListener)

					// Was the contact disabled by the user?
					if contact.IsEnabled() == false {
						other.M_sweep = backup
						other.SynchronizeTransform()
						continue
					}

					// Are there contact points?
					if contact.IsTouching() == false {
						other.M_sweep = backup
						other.SynchronizeTransform()
						continue
					}

					// Add the contact to the island
					contact.SetFlags(contact.GetFlags() | B2Contact_Flag.E_islandFlag)
					island.AddContact(contact)

					// Has the other body already been added to the island?
					if (other.M_flags & B2Body_Flags.E_islandFlag) != 0x0000 {
						continue
					}

					// Add the other body to the island.
					other.M_flags |= B2Body_Flags.E_islandFlag

					if other.M_type != B2BodyType.B2_staticBody {
						other.SetAwake(true)
					}

					island.AddBody(other)
				}
			}
		}

		subStep := MakeB2TimeStep()
		subStep.Dt = (1.0 - minAlpha) * step.Dt
		subStep.Inv_dt = 1.0 / subStep.Dt
		subStep.DtRatio = 1.0
		subStep.PositionIterations = 20
		subStep.VelocityIterations = step.VelocityIterations
		subStep.WarmStarting = false
		island.SolveTOI(subStep, bA.M_islandIndex, bB.M_islandIndex)

		// Reset island flags and synchronize broad-phase proxies.
		for i := 0; i < island.M_bodyCount; i++ {
			body := island.M_bodies[i]
			body.M_flags &= ^B2Body_Flags.E_islandFlag

			if body.M_type != B2BodyType.B2_dynamicBody {
				continue
			}

			body.SynchronizeFixtures()

			// Invalidate all contact TOIs on this displaced body.
			for ce := body.M_contactList; ce != nil; ce = ce.Next {
				ce.Contact.SetFlags(ce.Contact.GetFlags() & ^(B2Contact_Flag.E_toiFlag | B2Contact_Flag.E_islandFlag))
			}
		}

		// Commit fixture proxy movements to the broad-phase so that new contacts are created.
		// Also, some contacts can be destroyed.
		world.M_contactManager.FindNewContacts()

		if world.M_subStepping {
			world.M_stepComplete = false
			break
		}
	}
}

func (world *B2World) Step(dt float64, velocityIterations int, positionIterations int) {
	stepTimer := MakeB2Timer()

	// If new fixtures were added, we need to find the new contacts.
	if (world.M_flags & B2World_Flags.E_newFixture) != 0x0000 {
		world.M_contactManager.FindNewContacts()
		world.M_flags &= ^B2World_Flags.E_newFixture
	}

	world.M_flags |= B2World_Flags.E_locked

	step := MakeB2TimeStep()
	step.Dt = dt
	step.VelocityIterations = velocityIterations
	step.PositionIterations = positionIterations
	if dt > 0.0 {
		step.Inv_dt = 1.0 / dt
	} else {
		step.Inv_dt = 0.0
	}

	step.DtRatio = world.M_inv_dt0 * dt

	step.WarmStarting = world.M_warmStarting

	// Update contacts. This is where some contacts are destroyed.
	{
		timer := MakeB2Timer()
		world.M_contactManager.Collide()
		world.M_profile.Collide = timer.GetMilliseconds()
	}

	// Integrate velocities, solve velocity constraints, and integrate positions.
	if world.M_stepComplete && step.Dt > 0.0 {
		timer := MakeB2Timer()
		world.Solve(step)
		world.M_profile.Solve = timer.GetMilliseconds()
	}

	// Handle TOI events.
	if world.M_continuousPhysics && step.Dt > 0.0 {
		timer := MakeB2Timer()
		world.SolveTOI(step)
		world.M_profile.SolveTOI = timer.GetMilliseconds()
	}

	if step.Dt > 0.0 {
		world.M_inv_dt0 = step.Inv_dt
	}

	if (world.M_flags & B2World_Flags.E_clearForces) != 0x0000 {
		world.ClearForces()
	}

	world.M_flags &= ^B2World_Flags.E_locked

	world.M_profile.Step = stepTimer.GetMilliseconds()
}

func (world *B2World) ClearForces() {
	for body := world.M_bodyList; body != nil; body = body.GetNext() {
		body.M_force.SetZero()
		body.M_torque = 0.0
	}
}

type B2WorldQueryWrapper struct {
	BroadPhase *B2BroadPhase
	Callback   B2BroadPhaseQueryCallback
}

func MakeB2WorldQueryWrapper() B2WorldQueryWrapper {
	return B2WorldQueryWrapper{}
}

func (query *B2WorldQueryWrapper) QueryCallback(proxyId int) bool {
	proxy := query.BroadPhase.GetUserData(proxyId).(*B2FixtureProxy)
	return query.Callback(proxy.Fixture)
}

func (world *B2World) QueryAABB(callback B2BroadPhaseQueryCallback, aabb B2AABB) {
	wrapper := MakeB2WorldQueryWrapper()
	wrapper.BroadPhase = &world.M_contactManager.M_broadPhase
	wrapper.Callback = callback
	world.M_contactManager.M_broadPhase.Query(wrapper.QueryCallback, aabb)
}

func (world *B2World) RayCast(callback B2RaycastCallback, point1 B2Vec2, point2 B2Vec2) {

	// B2TreeRayCastCallback
	wrapper := func(input B2RayCastInput, nodeId int) float64 {

		userData := world.M_contactManager.M_broadPhase.GetUserData(nodeId)
		proxy := userData.(*B2FixtureProxy)
		fixture := proxy.Fixture
		index := proxy.ChildIndex
		output := MakeB2RayCastOutput()
		hit := fixture.RayCast(&output, input, index)

		if hit {
			fraction := output.Fraction
			point := B2Vec2Add(B2Vec2MulScalar((1.0-fraction), input.P1), B2Vec2MulScalar(fraction, input.P2))
			return callback(fixture, point, output.Normal, fraction)
		}

		return input.MaxFraction
	}

	input := MakeB2RayCastInput()
	input.MaxFraction = 1.0
	input.P1 = point1
	input.P2 = point2
	world.M_contactManager.M_broadPhase.RayCast(wrapper, input)
}

// void (world *B2World) DrawShape(b2Fixture* fixture, const b2Transform& xf, const b2Color& color)
// {
// 	switch (fixture.GetType())
// 	{
// 	case b2Shape::e_circle:
// 		{
// 			b2CircleShape* circle = (b2CircleShape*)fixture.GetShape();

// 			b2Vec2 center = b2Mul(xf, circle.m_p);
// 			float64 radius = circle.m_radius;
// 			b2Vec2 axis = b2Mul(xf.q, b2Vec2(1.0, 0.0));

// 			g_debugDraw.DrawSolidCircle(center, radius, axis, color);
// 		}
// 		break;

// 	case b2Shape::e_edge:
// 		{
// 			b2EdgeShape* edge = (b2EdgeShape*)fixture.GetShape();
// 			b2Vec2 v1 = b2Mul(xf, edge.m_vertex1);
// 			b2Vec2 v2 = b2Mul(xf, edge.m_vertex2);
// 			g_debugDraw.DrawSegment(v1, v2, color);
// 		}
// 		break;

// 	case b2Shape::e_chain:
// 		{
// 			b2ChainShape* chain = (b2ChainShape*)fixture.GetShape();
// 			int count = chain.m_count;
// 			const b2Vec2* vertices = chain.m_vertices;

// 			b2Color ghostColor(0.75f * color.r, 0.75f * color.g, 0.75f * color.b, color.a);

// 			b2Vec2 v1 = b2Mul(xf, vertices[0]);
// 			g_debugDraw.DrawPoint(v1, 4.0, color);

// 			if (chain.m_hasPrevVertex)
// 			{
// 				b2Vec2 vp = b2Mul(xf, chain.m_prevVertex);
// 				g_debugDraw.DrawSegment(vp, v1, ghostColor);
// 				g_debugDraw.DrawCircle(vp, 0.1f, ghostColor);
// 			}

// 			for (int i = 1; i < count; ++i)
// 			{
// 				b2Vec2 v2 = b2Mul(xf, vertices[i]);
// 				g_debugDraw.DrawSegment(v1, v2, color);
// 				g_debugDraw.DrawPoint(v2, 4.0, color);
// 				v1 = v2;
// 			}

// 			if (chain.m_hasNextVertex)
// 			{
// 				b2Vec2 vn = b2Mul(xf, chain.m_nextVertex);
// 				g_debugDraw.DrawSegment(v1, vn, ghostColor);
// 				g_debugDraw.DrawCircle(vn, 0.1f, ghostColor);
// 			}
// 		}
// 		break;

// 	case b2Shape::e_polygon:
// 		{
// 			b2PolygonShape* poly = (b2PolygonShape*)fixture.GetShape();
// 			int vertexCount = poly.m_count;
// 			b2Assert(vertexCount <= b2_maxPolygonVertices);
// 			b2Vec2 vertices[b2_maxPolygonVertices];

// 			for (int i = 0; i < vertexCount; ++i)
// 			{
// 				vertices[i] = b2Mul(xf, poly.m_vertices[i]);
// 			}

// 			g_debugDraw.DrawSolidPolygon(vertices, vertexCount, color);
// 		}
// 		break;

//     default:
//         break;
// 	}
// }

// void (world *B2World) DrawJoint(b2Joint* joint)
// {
// 	b2Body* bodyA = joint.GetBodyA();
// 	b2Body* bodyB = joint.GetBodyB();
// 	const b2Transform& xf1 = bodyA.GetTransform();
// 	const b2Transform& xf2 = bodyB.GetTransform();
// 	b2Vec2 x1 = xf1.p;
// 	b2Vec2 x2 = xf2.p;
// 	b2Vec2 p1 = joint.GetAnchorA();
// 	b2Vec2 p2 = joint.GetAnchorB();

// 	b2Color color(0.5f, 0.8f, 0.8f);

// 	switch (joint.GetType())
// 	{
// 	case e_distanceJoint:
// 		g_debugDraw.DrawSegment(p1, p2, color);
// 		break;

// 	case e_pulleyJoint:
// 		{
// 			b2PulleyJoint* pulley = (b2PulleyJoint*)joint;
// 			b2Vec2 s1 = pulley.GetGroundAnchorA();
// 			b2Vec2 s2 = pulley.GetGroundAnchorB();
// 			g_debugDraw.DrawSegment(s1, p1, color);
// 			g_debugDraw.DrawSegment(s2, p2, color);
// 			g_debugDraw.DrawSegment(s1, s2, color);
// 		}
// 		break;

// 	case e_mouseJoint:
// 		// don't draw this
// 		break;

// 	default:
// 		g_debugDraw.DrawSegment(x1, p1, color);
// 		g_debugDraw.DrawSegment(p1, p2, color);
// 		g_debugDraw.DrawSegment(x2, p2, color);
// 	}
// }

// void (world *B2World) DrawDebugData()
// {
// 	if (g_debugDraw == nullptr)
// 	{
// 		return;
// 	}

// 	uint flags = g_debugDraw.GetFlags();

// 	if (flags & b2Draw::e_shapeBit)
// 	{
// 		for (b2Body* b = m_bodyList; b; b = b.GetNext())
// 		{
// 			const b2Transform& xf = b.GetTransform();
// 			for (b2Fixture* f = b.GetFixtureList(); f; f = f.GetNext())
// 			{
// 				if (b.IsActive() == false)
// 				{
// 					DrawShape(f, xf, b2Color(0.5f, 0.5f, 0.3f));
// 				}
// 				else if (b.GetType() == b2_staticBody)
// 				{
// 					DrawShape(f, xf, b2Color(0.5f, 0.9f, 0.5f));
// 				}
// 				else if (b.GetType() == b2_kinematicBody)
// 				{
// 					DrawShape(f, xf, b2Color(0.5f, 0.5f, 0.9f));
// 				}
// 				else if (b.IsAwake() == false)
// 				{
// 					DrawShape(f, xf, b2Color(0.6f, 0.6f, 0.6f));
// 				}
// 				else
// 				{
// 					DrawShape(f, xf, b2Color(0.9f, 0.7f, 0.7f));
// 				}
// 			}
// 		}
// 	}

// 	if (flags & b2Draw::e_jointBit)
// 	{
// 		for (b2Joint* j = m_jointList; j; j = j.GetNext())
// 		{
// 			DrawJoint(j);
// 		}
// 	}

// 	if (flags & b2Draw::e_pairBit)
// 	{
// 		b2Color color(0.3f, 0.9f, 0.9f);
// 		for (b2Contact* c = m_contactManager.m_contactList; c; c = c.GetNext())
// 		{
// 			//b2Fixture* fixtureA = c.GetFixtureA();
// 			//b2Fixture* fixtureB = c.GetFixtureB();

// 			//b2Vec2 cA = fixtureA.GetAABB().GetCenter();
// 			//b2Vec2 cB = fixtureB.GetAABB().GetCenter();

// 			//g_debugDraw.DrawSegment(cA, cB, color);
// 		}
// 	}

// 	if (flags & b2Draw::e_aabbBit)
// 	{
// 		b2Color color(0.9f, 0.3f, 0.9f);
// 		b2BroadPhase* bp = &m_contactManager.m_broadPhase;

// 		for (b2Body* b = m_bodyList; b; b = b.GetNext())
// 		{
// 			if (b.IsActive() == false)
// 			{
// 				continue;
// 			}

// 			for (b2Fixture* f = b.GetFixtureList(); f; f = f.GetNext())
// 			{
// 				for (int i = 0; i < f.m_proxyCount; ++i)
// 				{
// 					b2FixtureProxy* proxy = f.m_proxies + i;
// 					b2AABB aabb = bp.GetFatAABB(proxy.proxyId);
// 					b2Vec2 vs[4];
// 					vs[0].Set(aabb.lowerBound.x, aabb.lowerBound.y);
// 					vs[1].Set(aabb.upperBound.x, aabb.lowerBound.y);
// 					vs[2].Set(aabb.upperBound.x, aabb.upperBound.y);
// 					vs[3].Set(aabb.lowerBound.x, aabb.upperBound.y);

// 					g_debugDraw.DrawPolygon(vs, 4, color);
// 				}
// 			}
// 		}
// 	}

// 	if (flags & b2Draw::e_centerOfMassBit)
// 	{
// 		for (b2Body* b = m_bodyList; b; b = b.GetNext())
// 		{
// 			b2Transform xf = b.GetTransform();
// 			xf.p = b.GetWorldCenter();
// 			g_debugDraw.DrawTransform(xf);
// 		}
// 	}
// }

func (world B2World) GetProxyCount() int {
	return world.M_contactManager.M_broadPhase.GetProxyCount()
}

func (world B2World) GetTreeHeight() int {
	return world.M_contactManager.M_broadPhase.GetTreeHeight()
}

func (world B2World) GetTreeBalance() int {
	return world.M_contactManager.M_broadPhase.GetTreeBalance()
}

func (world B2World) GetTreeQuality() float64 {
	return world.M_contactManager.M_broadPhase.GetTreeQuality()
}

func (world *B2World) ShiftOrigin(newOrigin B2Vec2) {

	B2Assert((world.M_flags & B2World_Flags.E_locked) == 0)
	if (world.M_flags & B2World_Flags.E_locked) == B2World_Flags.E_locked {
		return
	}

	for b := world.M_bodyList; b != nil; b = b.M_next {
		b.M_xf.P.OperatorMinusInplace(newOrigin)
		b.M_sweep.C0.OperatorMinusInplace(newOrigin)
		b.M_sweep.C.OperatorMinusInplace(newOrigin)
	}

	for j := world.M_jointList; j != nil; j = j.GetNext() {
		j.ShiftOrigin(newOrigin)
	}

	world.M_contactManager.M_broadPhase.ShiftOrigin(newOrigin)
}

func (world *B2World) Dump() {
	if (world.M_flags & B2World_Flags.E_locked) == B2World_Flags.E_locked {
		return
	}

	fmt.Print(fmt.Printf("b2Vec2 g(%.15lef, %.15lef);\n", world.M_gravity.X, world.M_gravity.Y))
	fmt.Print("m_world.SetGravity(g);\n")

	fmt.Print(fmt.Printf("b2Body** bodies = (b2Body**)b2Alloc(%d * sizeof(b2Body*));\n", world.M_bodyCount))
	//fmt.Print("b2Joint** joints = (b2Joint**)b2Alloc(%d * sizeof(b2Joint*));\n", m_jointCount)
	i := 0
	for b := world.M_bodyList; b != nil; b = b.M_next {
		b.M_islandIndex = i
		b.Dump()
		i++
	}

	i = 0
	for j := world.M_jointList; j != nil; j = j.GetNext() {
		j.SetIndex(i)
		i++
	}

	// First pass on joints, skip gear joints.
	for j := world.M_jointList; j != nil; j = j.GetNext() {
		if j.GetType() == B2JointType.E_gearJoint {
			continue
		}

		fmt.Print("{\n")
		j.Dump()
		fmt.Print("}\n")
	}

	// Second pass on joints, only gear joints.
	for j := world.M_jointList; j != nil; j = j.GetNext() {
		if j.GetType() != B2JointType.E_gearJoint {
			continue
		}

		fmt.Print("{\n")
		j.Dump()
		fmt.Print("}\n")
	}

	fmt.Print("b2Free(joints);\n")
	fmt.Print("b2Free(bodies);\n")
	fmt.Print("joints = nullptr;\n")
	fmt.Print("bodies = nullptr;\n")
}
