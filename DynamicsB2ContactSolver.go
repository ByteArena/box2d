package box2d

import (
	"math"
)

type B2VelocityConstraintPoint struct {
	RA             B2Vec2
	RB             B2Vec2
	NormalImpulse  float64
	TangentImpulse float64
	NormalMass     float64
	TangentMass    float64
	VelocityBias   float64
}

type B2ContactVelocityConstraint struct {
	Points             [B2_maxManifoldPoints]B2VelocityConstraintPoint
	Normal             B2Vec2
	NormalMass         B2Mat22
	K                  B2Mat22
	IndexA             int
	IndexB             int
	InvMassA, InvMassB float64
	InvIA, InvIB       float64
	Friction           float64
	Restitution        float64
	TangentSpeed       float64
	PointCount         int
	ContactIndex       int
}

type B2ContactSolverDef struct {
	Step       B2TimeStep
	Contacts   []B2ContactInterface // has to be backed by pointers
	Count      int
	Positions  []B2Position
	Velocities []B2Velocity
}

func MakeB2ContactSolverDef() B2ContactSolverDef {
	return B2ContactSolverDef{
		Contacts:   make([]B2ContactInterface, 0),
		Positions:  make([]B2Position, 0),
		Velocities: make([]B2Velocity, 0),
	}
}

type B2ContactSolver struct {
	M_step                B2TimeStep
	M_positions           []B2Position
	M_velocities          []B2Velocity
	M_positionConstraints []B2ContactPositionConstraint
	M_velocityConstraints []B2ContactVelocityConstraint
	M_contacts            []B2ContactInterface // has to be backed by pointers
	M_count               int
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// B2ContactSolver
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// // Solver debugging is normally disabled because the block solver sometimes has to deal with a poorly conditioned effective mass matrix.
const B2_DEBUG_SOLVER = 0

var g_blockSolve = true

type B2ContactPositionConstraint struct {
	LocalPoints                [B2_maxManifoldPoints]B2Vec2
	LocalNormal                B2Vec2
	LocalPoint                 B2Vec2
	IndexA                     int
	IndexB                     int
	InvMassA, InvMassB         float64
	LocalCenterA, LocalCenterB B2Vec2
	InvIA, InvIB               float64
	Type                       uint8
	RadiusA, RadiusB           float64
	PointCount                 int
}

func MakeB2ContactSolver(def *B2ContactSolverDef) B2ContactSolver {
	solver := B2ContactSolver{}

	solver.M_step = def.Step
	solver.M_count = def.Count
	solver.M_positionConstraints = make([]B2ContactPositionConstraint, solver.M_count)
	solver.M_velocityConstraints = make([]B2ContactVelocityConstraint, solver.M_count)
	solver.M_positions = def.Positions
	solver.M_velocities = def.Velocities
	solver.M_contacts = def.Contacts

	// Initialize position independent portions of the constraints.
	for i := 0; i < solver.M_count; i++ {
		contact := solver.M_contacts[i]

		fixtureA := contact.GetFixtureA()
		fixtureB := contact.GetFixtureB()
		shapeA := fixtureA.GetShape()
		shapeB := fixtureB.GetShape()
		radiusA := shapeA.GetRadius()
		radiusB := shapeB.GetRadius()
		bodyA := fixtureA.GetBody()
		bodyB := fixtureB.GetBody()
		manifold := contact.GetManifold()

		pointCount := manifold.PointCount
		B2Assert(pointCount > 0)

		vc := &solver.M_velocityConstraints[i]
		vc.Friction = contact.GetFriction()
		vc.Restitution = contact.GetRestitution()
		vc.TangentSpeed = contact.GetTangentSpeed()
		vc.IndexA = bodyA.M_islandIndex
		vc.IndexB = bodyB.M_islandIndex
		vc.InvMassA = bodyA.M_invMass
		vc.InvMassB = bodyB.M_invMass
		vc.InvIA = bodyA.M_invI
		vc.InvIB = bodyB.M_invI
		vc.ContactIndex = i
		vc.PointCount = pointCount
		vc.K.SetZero()
		vc.NormalMass.SetZero()

		pc := &solver.M_positionConstraints[i]
		pc.IndexA = bodyA.M_islandIndex
		pc.IndexB = bodyB.M_islandIndex
		pc.InvMassA = bodyA.M_invMass
		pc.InvMassB = bodyB.M_invMass
		pc.LocalCenterA = bodyA.M_sweep.LocalCenter
		pc.LocalCenterB = bodyB.M_sweep.LocalCenter
		pc.InvIA = bodyA.M_invI
		pc.InvIB = bodyB.M_invI
		pc.LocalNormal = manifold.LocalNormal
		pc.LocalPoint = manifold.LocalPoint
		pc.PointCount = pointCount
		pc.RadiusA = radiusA
		pc.RadiusB = radiusB
		pc.Type = manifold.Type

		for j := 0; j < pointCount; j++ {
			cp := &manifold.Points[j]
			vcp := &vc.Points[j]

			if solver.M_step.WarmStarting {
				vcp.NormalImpulse = solver.M_step.DtRatio * cp.NormalImpulse
				vcp.TangentImpulse = solver.M_step.DtRatio * cp.TangentImpulse
			} else {
				vcp.NormalImpulse = 0.0
				vcp.TangentImpulse = 0.0
			}

			vcp.RA.SetZero()
			vcp.RB.SetZero()
			vcp.NormalMass = 0.0
			vcp.TangentMass = 0.0
			vcp.VelocityBias = 0.0

			pc.LocalPoints[j] = cp.LocalPoint
		}
	}

	return solver
}

func (solver *B2ContactSolver) Destroy() {
}

// Initialize position dependent portions of the velocity constraints.
func (solver *B2ContactSolver) InitializeVelocityConstraints() {
	for i := 0; i < solver.M_count; i++ {
		vc := &solver.M_velocityConstraints[i]
		pc := &solver.M_positionConstraints[i]

		radiusA := pc.RadiusA
		radiusB := pc.RadiusB
		manifold := solver.M_contacts[vc.ContactIndex].GetManifold()

		indexA := vc.IndexA
		indexB := vc.IndexB

		mA := vc.InvMassA
		mB := vc.InvMassB
		iA := vc.InvIA
		iB := vc.InvIB
		localCenterA := pc.LocalCenterA
		localCenterB := pc.LocalCenterB

		cA := solver.M_positions[indexA].C
		aA := solver.M_positions[indexA].A
		vA := solver.M_velocities[indexA].V
		wA := solver.M_velocities[indexA].W

		cB := solver.M_positions[indexB].C
		aB := solver.M_positions[indexB].A
		vB := solver.M_velocities[indexB].V
		wB := solver.M_velocities[indexB].W

		B2Assert(manifold.PointCount > 0)

		xfA := MakeB2Transform()
		xfB := MakeB2Transform()
		xfA.Q.Set(aA)
		xfB.Q.Set(aB)
		xfA.P = B2Vec2Sub(cA, B2RotVec2Mul(xfA.Q, localCenterA))
		xfB.P = B2Vec2Sub(cB, B2RotVec2Mul(xfB.Q, localCenterB))

		worldManifold := MakeB2WorldManifold()
		worldManifold.Initialize(manifold, xfA, radiusA, xfB, radiusB)

		vc.Normal = worldManifold.Normal

		pointCount := vc.PointCount
		for j := 0; j < pointCount; j++ {
			vcp := &vc.Points[j]

			vcp.RA = B2Vec2Sub(worldManifold.Points[j], cA)
			vcp.RB = B2Vec2Sub(worldManifold.Points[j], cB)

			rnA := B2Vec2Cross(vcp.RA, vc.Normal)
			rnB := B2Vec2Cross(vcp.RB, vc.Normal)

			kNormal := mA + mB + iA*rnA*rnA + iB*rnB*rnB

			if kNormal > 0.0 {
				vcp.NormalMass = 1.0 / kNormal
			} else {
				vcp.NormalMass = 0.0
			}

			tangent := B2Vec2CrossVectorScalar(vc.Normal, 1.0)

			rtA := B2Vec2Cross(vcp.RA, tangent)
			rtB := B2Vec2Cross(vcp.RB, tangent)

			kTangent := mA + mB + iA*rtA*rtA + iB*rtB*rtB

			if kTangent > 0.0 {
				vcp.TangentMass = 1.0 / kTangent
			} else {
				vcp.TangentMass = 0.0
			}

			// Setup a velocity bias for restitution.
			vcp.VelocityBias = 0.0
			vRel := B2Vec2Dot(
				vc.Normal,
				B2Vec2Sub(
					B2Vec2Sub(
						B2Vec2Add(
							vB,
							B2Vec2CrossScalarVector(wB, vcp.RB),
						),
						vA),
					B2Vec2CrossScalarVector(wA, vcp.RA),
				),
			)
			if vRel < -B2_velocityThreshold {
				vcp.VelocityBias = -vc.Restitution * vRel
			}
		}

		// If we have two points, then prepare the block solver.
		if vc.PointCount == 2 && g_blockSolve {
			vcp1 := &vc.Points[0]
			vcp2 := &vc.Points[1]

			rn1A := B2Vec2Cross(vcp1.RA, vc.Normal)
			rn1B := B2Vec2Cross(vcp1.RB, vc.Normal)
			rn2A := B2Vec2Cross(vcp2.RA, vc.Normal)
			rn2B := B2Vec2Cross(vcp2.RB, vc.Normal)

			k11 := mA + mB + iA*rn1A*rn1A + iB*rn1B*rn1B
			k22 := mA + mB + iA*rn2A*rn2A + iB*rn2B*rn2B
			k12 := mA + mB + iA*rn1A*rn2A + iB*rn1B*rn2B

			// Ensure a reasonable condition number.
			k_maxConditionNumber := 1000.0
			if k11*k11 < k_maxConditionNumber*(k11*k22-k12*k12) {
				// K is safe to invert.
				vc.K.Ex.Set(k11, k12)
				vc.K.Ey.Set(k12, k22)
				vc.NormalMass = vc.K.GetInverse()
			} else {
				// The constraints are redundant, just use one.
				// TODO_ERIN use deepest?
				vc.PointCount = 1
			}
		}
	}
}

func (solver *B2ContactSolver) WarmStart() {
	// Warm start.
	for i := 0; i < solver.M_count; i++ {
		vc := &solver.M_velocityConstraints[i]

		indexA := vc.IndexA
		indexB := vc.IndexB
		mA := vc.InvMassA
		iA := vc.InvIA
		mB := vc.InvMassB
		iB := vc.InvIB
		pointCount := vc.PointCount

		vA := solver.M_velocities[indexA].V
		wA := solver.M_velocities[indexA].W
		vB := solver.M_velocities[indexB].V
		wB := solver.M_velocities[indexB].W

		normal := vc.Normal
		tangent := B2Vec2CrossVectorScalar(normal, 1.0)

		for j := 0; j < pointCount; j++ {
			vcp := &vc.Points[j]
			P := B2Vec2Add(B2Vec2MulScalar(vcp.NormalImpulse, normal), B2Vec2MulScalar(vcp.TangentImpulse, tangent))
			wA -= iA * B2Vec2Cross(vcp.RA, P)
			vA.OperatorMinusInplace(B2Vec2MulScalar(mA, P))
			wB += iB * B2Vec2Cross(vcp.RB, P)
			vB.OperatorPlusInplace(B2Vec2MulScalar(mB, P))
		}

		solver.M_velocities[indexA].V = vA
		solver.M_velocities[indexA].W = wA
		solver.M_velocities[indexB].V = vB
		solver.M_velocities[indexB].W = wB
	}
}

func (solver *B2ContactSolver) SolveVelocityConstraints() {
	for i := 0; i < solver.M_count; i++ {
		vc := &solver.M_velocityConstraints[i]

		indexA := vc.IndexA
		indexB := vc.IndexB
		mA := vc.InvMassA
		iA := vc.InvIA
		mB := vc.InvMassB
		iB := vc.InvIB
		pointCount := vc.PointCount

		vA := solver.M_velocities[indexA].V
		wA := solver.M_velocities[indexA].W
		vB := solver.M_velocities[indexB].V
		wB := solver.M_velocities[indexB].W

		normal := vc.Normal
		tangent := B2Vec2CrossVectorScalar(normal, 1.0)
		friction := vc.Friction

		B2Assert(pointCount == 1 || pointCount == 2)

		// Solve tangent constraints first because non-penetration is more important
		// than friction.
		for j := 0; j < pointCount; j++ {
			vcp := &vc.Points[j]

			// Relative velocity at contact
			dv := B2Vec2Add(
				vB,
				B2Vec2Sub(
					B2Vec2Sub(
						B2Vec2CrossScalarVector(wB, vcp.RB),
						vA,
					),
					B2Vec2CrossScalarVector(wA, vcp.RA),
				),
			)

			// Compute tangent force
			vt := B2Vec2Dot(dv, tangent) - vc.TangentSpeed
			lambda := vcp.TangentMass * (-vt)

			// b2Clamp the accumulated force
			maxFriction := friction * vcp.NormalImpulse
			newImpulse := B2FloatClamp(vcp.TangentImpulse+lambda, -maxFriction, maxFriction)
			lambda = newImpulse - vcp.TangentImpulse
			vcp.TangentImpulse = newImpulse

			// Apply contact impulse
			P := B2Vec2MulScalar(lambda, tangent)

			vA.OperatorMinusInplace(B2Vec2MulScalar(mA, P))
			wA -= iA * B2Vec2Cross(vcp.RA, P)

			vB.OperatorPlusInplace(B2Vec2MulScalar(mB, P))
			wB += iB * B2Vec2Cross(vcp.RB, P)
		}

		// Solve normal constraints
		if pointCount == 1 || g_blockSolve == false {
			for j := 0; j < pointCount; j++ {
				vcp := &vc.Points[j]

				// Relative velocity at contact
				dv := B2Vec2Add(
					vB,
					B2Vec2Sub(
						B2Vec2Sub(
							B2Vec2CrossScalarVector(wB, vcp.RB),
							vA,
						),
						B2Vec2CrossScalarVector(wA, vcp.RA),
					),
				)

				// Compute normal impulse
				vn := B2Vec2Dot(dv, normal)
				lambda := -vcp.NormalMass * (vn - vcp.VelocityBias)

				// b2Clamp the accumulated impulse
				newImpulse := math.Max(vcp.NormalImpulse+lambda, 0.0)
				lambda = newImpulse - vcp.NormalImpulse
				vcp.NormalImpulse = newImpulse

				// Apply contact impulse
				P := B2Vec2MulScalar(lambda, normal)
				vA.OperatorMinusInplace(B2Vec2MulScalar(mA, P))
				wA -= iA * B2Vec2Cross(vcp.RA, P)

				vB.OperatorPlusInplace(B2Vec2MulScalar(mB, P))
				wB += iB * B2Vec2Cross(vcp.RB, P)
			}
		} else {
			// Block solver developed in collaboration with Dirk Gregorius (back in 01/07 on Box2D_Lite).
			// Build the mini LCP for this contact patch
			//
			// vn = A * x + b, vn >= 0, x >= 0 and vn_i * x_i = 0 with i = 1..2
			//
			// A = J * W * JT and J = ( -n, -r1 x n, n, r2 x n )
			// b = vn0 - velocityBias
			//
			// The system is solved using the "Total enumeration method" (s. Murty). The complementary constraint vn_i * x_i
			// implies that we must have in any solution either vn_i = 0 or x_i = 0. So for the 2D contact problem the cases
			// vn1 = 0 and vn2 = 0, x1 = 0 and x2 = 0, x1 = 0 and vn2 = 0, x2 = 0 and vn1 = 0 need to be tested. The first valid
			// solution that satisfies the problem is chosen.
			//
			// In order to account of the accumulated impulse 'a' (because of the iterative nature of the solver which only requires
			// that the accumulated impulse is clamped and not the incremental impulse) we change the impulse variable (x_i).
			//
			// Substitute:
			//
			// x = a + d
			//
			// a := old total impulse
			// x := new total impulse
			// d := incremental impulse
			//
			// For the current iteration we extend the formula for the incremental impulse
			// to compute the new total impulse:
			//
			// vn = A * d + b
			//    = A * (x - a) + b
			//    = A * x + b - A * a
			//    = A * x + b'
			// b' = b - A * a;

			cp1 := &vc.Points[0]
			cp2 := &vc.Points[1]

			a := MakeB2Vec2(cp1.NormalImpulse, cp2.NormalImpulse)
			B2Assert(a.X >= 0.0 && a.Y >= 0.0)

			// Relative velocity at contact
			dv1 := B2Vec2Add(vB, B2Vec2Sub(B2Vec2Sub(B2Vec2CrossScalarVector(wB, cp1.RB), vA), B2Vec2CrossScalarVector(wA, cp1.RA)))
			dv2 := B2Vec2Add(vB, B2Vec2Sub(B2Vec2Sub(B2Vec2CrossScalarVector(wB, cp2.RB), vA), B2Vec2CrossScalarVector(wA, cp2.RA)))

			// Compute normal velocity
			vn1 := B2Vec2Dot(dv1, normal)
			vn2 := B2Vec2Dot(dv2, normal)

			b := MakeB2Vec2(0, 0)
			b.X = vn1 - cp1.VelocityBias
			b.Y = vn2 - cp2.VelocityBias

			// Compute b'
			b.OperatorMinusInplace(B2Vec2Mat22Mul(vc.K, a))

			const k_errorTol = 0.001
			// B2_NOT_USED(k_errorTol);

			for {
				//
				// Case 1: vn = 0
				//
				// 0 = A * x + b'
				//
				// Solve for x:
				//
				// x = - inv(A) * b'
				//
				x := B2Vec2Mat22Mul(vc.NormalMass, b).OperatorNegate()

				if x.X >= 0.0 && x.Y >= 0.0 {
					// Get the incremental impulse
					d := B2Vec2Sub(x, a)

					// Apply incremental impulse
					P1 := B2Vec2MulScalar(d.X, normal)
					P2 := B2Vec2MulScalar(d.Y, normal)
					vA.OperatorMinusInplace(B2Vec2MulScalar(mA, B2Vec2Add(P1, P2)))
					wA -= iA * (B2Vec2Cross(cp1.RA, P1) + B2Vec2Cross(cp2.RA, P2))

					vB.OperatorPlusInplace(B2Vec2MulScalar(mB, B2Vec2Add(P1, P2)))
					wB += iB * (B2Vec2Cross(cp1.RB, P1) + B2Vec2Cross(cp2.RB, P2))

					// Accumulate
					cp1.NormalImpulse = x.X
					cp2.NormalImpulse = x.Y

					if B2_DEBUG_SOLVER == 1 {
						// Postconditions
						dv1 = B2Vec2Add(
							vB,
							B2Vec2Sub(
								B2Vec2Sub(
									B2Vec2CrossScalarVector(wB, cp1.RB),
									vA,
								),
								B2Vec2CrossScalarVector(wA, cp1.RA),
							),
						)
						dv2 = B2Vec2Add(
							vB,
							B2Vec2Sub(
								B2Vec2Sub(
									B2Vec2CrossScalarVector(wB, cp2.RB),
									vA,
								),
								B2Vec2CrossScalarVector(wA, cp2.RA),
							),
						)

						// Compute normal velocity
						vn1 = B2Vec2Dot(dv1, normal)
						vn2 = B2Vec2Dot(dv2, normal)

						B2Assert(math.Abs(vn1-cp1.VelocityBias) < k_errorTol)
						B2Assert(math.Abs(vn2-cp2.VelocityBias) < k_errorTol)
					}
					break
				}

				//
				// Case 2: vn1 = 0 and x2 = 0
				//
				//   0 = a11 * x1 + a12 * 0 + b1'
				// vn2 = a21 * x1 + a22 * 0 + b2'
				//
				x.X = -cp1.NormalMass * b.X
				x.Y = 0.0
				vn1 = 0.0
				vn2 = vc.K.Ex.Y*x.X + b.Y
				if x.X >= 0.0 && vn2 >= 0.0 {
					// Get the incremental impulse
					d := B2Vec2Sub(x, a)

					// Apply incremental impulse
					P1 := B2Vec2MulScalar(d.X, normal)
					P2 := B2Vec2MulScalar(d.Y, normal)
					vA.OperatorMinusInplace(B2Vec2MulScalar(mA, B2Vec2Add(P1, P2)))
					wA -= iA * (B2Vec2Cross(cp1.RA, P1) + B2Vec2Cross(cp2.RA, P2))

					vB.OperatorPlusInplace(B2Vec2MulScalar(mB, B2Vec2Add(P1, P2)))
					wB += iB * (B2Vec2Cross(cp1.RB, P1) + B2Vec2Cross(cp2.RB, P2))

					// Accumulate
					cp1.NormalImpulse = x.X
					cp2.NormalImpulse = x.Y

					if B2_DEBUG_SOLVER == 1 {
						// Postconditions
						dv1 = B2Vec2Add(vB, B2Vec2Sub(B2Vec2Sub(B2Vec2CrossScalarVector(wB, cp1.RB), vA), B2Vec2CrossScalarVector(wA, cp1.RA)))

						// Compute normal velocity
						vn1 = B2Vec2Dot(dv1, normal)

						B2Assert(math.Abs(vn1-cp1.VelocityBias) < k_errorTol)
					}
					break
				}

				//
				// Case 3: vn2 = 0 and x1 = 0
				//
				// vn1 = a11 * 0 + a12 * x2 + b1'
				//   0 = a21 * 0 + a22 * x2 + b2'
				//
				x.X = 0.0
				x.Y = -cp2.NormalMass * b.Y
				vn1 = vc.K.Ey.X*x.Y + b.X
				vn2 = 0.0

				if x.Y >= 0.0 && vn1 >= 0.0 {
					// Resubstitute for the incremental impulse
					d := B2Vec2Sub(x, a)

					// Apply incremental impulse
					P1 := B2Vec2MulScalar(d.X, normal)
					P2 := B2Vec2MulScalar(d.Y, normal)
					vA.OperatorMinusInplace(B2Vec2MulScalar(mA, B2Vec2Add(P1, P2)))
					wA -= iA * (B2Vec2Cross(cp1.RA, P1) + B2Vec2Cross(cp2.RA, P2))

					vB.OperatorPlusInplace(B2Vec2MulScalar(mB, B2Vec2Add(P1, P2)))
					wB += iB * (B2Vec2Cross(cp1.RB, P1) + B2Vec2Cross(cp2.RB, P2))

					// Accumulate
					cp1.NormalImpulse = x.X
					cp2.NormalImpulse = x.Y

					if B2_DEBUG_SOLVER == 1 {
						// Postconditions
						dv2 = B2Vec2Add(vB, B2Vec2Sub(B2Vec2Sub(B2Vec2CrossScalarVector(wB, cp2.RB), vA), B2Vec2CrossScalarVector(wA, cp2.RA)))

						// Compute normal velocity
						vn2 = B2Vec2Dot(dv2, normal)

						B2Assert(math.Abs(vn2-cp2.VelocityBias) < k_errorTol)
					}

					break
				}

				//
				// Case 4: x1 = 0 and x2 = 0
				//
				// vn1 = b1
				// vn2 = b2;
				x.X = 0.0
				x.Y = 0.0
				vn1 = b.X
				vn2 = b.Y

				if vn1 >= 0.0 && vn2 >= 0.0 {
					// Resubstitute for the incremental impulse
					d := B2Vec2Sub(x, a)

					// Apply incremental impulse
					P1 := B2Vec2MulScalar(d.X, normal)
					P2 := B2Vec2MulScalar(d.Y, normal)
					vA.OperatorMinusInplace(B2Vec2MulScalar(mA, B2Vec2Add(P1, P2)))
					wA -= iA * (B2Vec2Cross(cp1.RA, P1) + B2Vec2Cross(cp2.RA, P2))

					vB.OperatorPlusInplace(B2Vec2MulScalar(mB, B2Vec2Add(P1, P2)))
					wB += iB * (B2Vec2Cross(cp1.RB, P1) + B2Vec2Cross(cp2.RB, P2))

					// Accumulate
					cp1.NormalImpulse = x.X
					cp2.NormalImpulse = x.Y

					break
				}

				// No solution, give up. This is hit sometimes, but it doesn't seem to matter.
				break
			}
		}

		solver.M_velocities[indexA].V = vA
		solver.M_velocities[indexA].W = wA
		solver.M_velocities[indexB].V = vB
		solver.M_velocities[indexB].W = wB
	}
}

func (solver *B2ContactSolver) StoreImpulses() {
	for i := 0; i < solver.M_count; i++ {
		vc := &solver.M_velocityConstraints[i]
		manifold := solver.M_contacts[vc.ContactIndex].GetManifold()

		for j := 0; j < vc.PointCount; j++ {
			manifold.Points[j].NormalImpulse = vc.Points[j].NormalImpulse
			manifold.Points[j].TangentImpulse = vc.Points[j].TangentImpulse
		}
	}
}

type B2PositionSolverManifold struct {
	Normal     B2Vec2
	Point      B2Vec2
	Separation float64
}

func MakeB2PositionSolverManifold() B2PositionSolverManifold {
	return B2PositionSolverManifold{}
}

func (solvermanifold *B2PositionSolverManifold) Initialize(pc *B2ContactPositionConstraint, xfA B2Transform, xfB B2Transform, index int) {

	B2Assert(pc.PointCount > 0)

	switch pc.Type {
	case B2Manifold_Type.E_circles:
		{
			pointA := B2TransformVec2Mul(xfA, pc.LocalPoint)
			pointB := B2TransformVec2Mul(xfB, pc.LocalPoints[0])
			solvermanifold.Normal = B2Vec2Sub(pointB, pointA)
			solvermanifold.Normal.Normalize()
			solvermanifold.Point = B2Vec2MulScalar(0.5, B2Vec2Add(pointA, pointB))
			solvermanifold.Separation = B2Vec2Dot(B2Vec2Sub(pointB, pointA), solvermanifold.Normal) - pc.RadiusA - pc.RadiusB
		}
		break

	case B2Manifold_Type.E_faceA:
		{
			solvermanifold.Normal = B2RotVec2Mul(xfA.Q, pc.LocalNormal)
			planePoint := B2TransformVec2Mul(xfA, pc.LocalPoint)

			clipPoint := B2TransformVec2Mul(xfB, pc.LocalPoints[index])
			solvermanifold.Separation = B2Vec2Dot(B2Vec2Sub(clipPoint, planePoint), solvermanifold.Normal) - pc.RadiusA - pc.RadiusB
			solvermanifold.Point = clipPoint
		}
		break

	case B2Manifold_Type.E_faceB:
		{
			solvermanifold.Normal = B2RotVec2Mul(xfB.Q, pc.LocalNormal)
			planePoint := B2TransformVec2Mul(xfB, pc.LocalPoint)

			clipPoint := B2TransformVec2Mul(xfA, pc.LocalPoints[index])
			solvermanifold.Separation = B2Vec2Dot(B2Vec2Sub(clipPoint, planePoint), solvermanifold.Normal) - pc.RadiusA - pc.RadiusB
			solvermanifold.Point = clipPoint

			// Ensure normal points from A to B
			solvermanifold.Normal = solvermanifold.Normal.OperatorNegate()
		}
		break
	}
}

// Sequential solver.
func (solver *B2ContactSolver) SolvePositionConstraints() bool {

	minSeparation := 0.0

	for i := 0; i < solver.M_count; i++ {
		pc := &solver.M_positionConstraints[i]

		indexA := pc.IndexA
		indexB := pc.IndexB
		localCenterA := pc.LocalCenterA
		mA := pc.InvMassA
		iA := pc.InvIA
		localCenterB := pc.LocalCenterB
		mB := pc.InvMassB
		iB := pc.InvIB
		pointCount := pc.PointCount

		cA := solver.M_positions[indexA].C
		aA := solver.M_positions[indexA].A

		cB := solver.M_positions[indexB].C
		aB := solver.M_positions[indexB].A

		// Solve normal constraints
		for j := 0; j < pointCount; j++ {
			xfA := MakeB2Transform()
			xfB := MakeB2Transform()

			xfA.Q.Set(aA)
			xfB.Q.Set(aB)
			xfA.P = B2Vec2Sub(cA, B2RotVec2Mul(xfA.Q, localCenterA))
			xfB.P = B2Vec2Sub(cB, B2RotVec2Mul(xfB.Q, localCenterB))

			psm := MakeB2PositionSolverManifold()
			psm.Initialize(pc, xfA, xfB, j)
			normal := psm.Normal

			point := psm.Point
			separation := psm.Separation

			rA := B2Vec2Sub(point, cA)
			rB := B2Vec2Sub(point, cB)

			// Track max constraint error.
			minSeparation = math.Min(minSeparation, separation)

			// Prevent large corrections and allow slop.
			C := B2FloatClamp(B2_baumgarte*(separation+B2_linearSlop), -B2_maxLinearCorrection, 0.0)

			// Compute the effective mass.
			rnA := B2Vec2Cross(rA, normal)
			rnB := B2Vec2Cross(rB, normal)
			K := mA + mB + iA*rnA*rnA + iB*rnB*rnB

			// Compute normal impulse
			impulse := 0.0
			if K > 0.0 {
				impulse = -C / K
			}

			P := B2Vec2MulScalar(impulse, normal)

			cA.OperatorMinusInplace(B2Vec2MulScalar(mA, P))
			aA -= iA * B2Vec2Cross(rA, P)

			cB.OperatorPlusInplace(B2Vec2MulScalar(mB, P))
			aB += iB * B2Vec2Cross(rB, P)
		}

		solver.M_positions[indexA].C = cA
		solver.M_positions[indexA].A = aA

		solver.M_positions[indexB].C = cB
		solver.M_positions[indexB].A = aB
	}

	// We can't expect minSpeparation >= -b2_linearSlop because we don't
	// push the separation above -b2_linearSlop.
	return minSeparation >= -3.0*B2_linearSlop
}

// Sequential position solver for position constraints.
func (solver *B2ContactSolver) SolveTOIPositionConstraints(toiIndexA int, toiIndexB int) bool {

	minSeparation := 0.0

	for i := 0; i < solver.M_count; i++ {
		pc := &solver.M_positionConstraints[i]

		indexA := pc.IndexA
		indexB := pc.IndexB
		localCenterA := pc.LocalCenterA
		localCenterB := pc.LocalCenterB
		pointCount := pc.PointCount

		mA := 0.0
		iA := 0.0
		if indexA == toiIndexA || indexA == toiIndexB {
			mA = pc.InvMassA
			iA = pc.InvIA
		}

		mB := 0.0
		iB := 0.0
		if indexB == toiIndexA || indexB == toiIndexB {
			mB = pc.InvMassB
			iB = pc.InvIB
		}

		cA := solver.M_positions[indexA].C
		aA := solver.M_positions[indexA].A

		cB := solver.M_positions[indexB].C
		aB := solver.M_positions[indexB].A

		// Solve normal constraints
		for j := 0; j < pointCount; j++ {
			xfA := MakeB2Transform()
			xfB := MakeB2Transform()

			xfA.Q.Set(aA)
			xfB.Q.Set(aB)
			xfB.P = B2Vec2Sub(cB, B2RotVec2Mul(xfB.Q, localCenterB))
			xfA.P = B2Vec2Sub(cA, B2RotVec2Mul(xfA.Q, localCenterA))

			psm := MakeB2PositionSolverManifold()
			psm.Initialize(pc, xfA, xfB, j)
			normal := psm.Normal

			point := psm.Point
			separation := psm.Separation

			rA := B2Vec2Sub(point, cA)
			rB := B2Vec2Sub(point, cB)

			// Track max constraint error.
			minSeparation = math.Min(minSeparation, separation)

			// Prevent large corrections and allow slop.
			C := B2FloatClamp(B2_toiBaugarte*(separation+B2_linearSlop), -B2_maxLinearCorrection, 0.0)

			// Compute the effective mass.
			rnA := B2Vec2Cross(rA, normal)
			rnB := B2Vec2Cross(rB, normal)
			K := mA + mB + iA*rnA*rnA + iB*rnB*rnB

			// Compute normal impulse
			impulse := 0.0
			if K > 0.0 {
				impulse = -C / K
			}

			P := B2Vec2MulScalar(impulse, normal)

			cA.OperatorMinusInplace(B2Vec2MulScalar(mA, P))
			aA -= iA * B2Vec2Cross(rA, P)

			cB.OperatorPlusInplace(B2Vec2MulScalar(mB, P))
			aB += iB * B2Vec2Cross(rB, P)
		}

		solver.M_positions[indexA].C = cA
		solver.M_positions[indexA].A = aA

		solver.M_positions[indexB].C = cB
		solver.M_positions[indexB].A = aB
	}

	// We can't expect minSpeparation >= -b2_linearSlop because we don't
	// push the separation above -b2_linearSlop.
	return minSeparation >= -1.5*B2_linearSlop
}
