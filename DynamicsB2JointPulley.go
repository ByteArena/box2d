package box2d

import (
	"fmt"
	"math"
)

const b2_minPulleyLength = 2.0

/// Pulley joint definition. This requires two ground anchors,
/// two dynamic body anchor points, and a pulley ratio.
type B2PulleyJointDef struct {
	B2JointDef

	/// The first ground anchor in world coordinates. This point never moves.
	GroundAnchorA B2Vec2

	/// The second ground anchor in world coordinates. This point never moves.
	GroundAnchorB B2Vec2

	/// The local anchor point relative to bodyA's origin.
	LocalAnchorA B2Vec2

	/// The local anchor point relative to bodyB's origin.
	LocalAnchorB B2Vec2

	/// The a reference length for the segment attached to bodyA.
	LengthA float64

	/// The a reference length for the segment attached to bodyB.
	LengthB float64

	/// The pulley ratio, used to simulate a block-and-tackle.
	Ratio float64
}

func MakeB2PulleyJointDef() B2PulleyJointDef {
	res := B2PulleyJointDef{
		B2JointDef: MakeB2JointDef(),
	}

	res.Type = B2JointType.E_pulleyJoint
	res.GroundAnchorA.Set(-1.0, 1.0)
	res.GroundAnchorB.Set(1.0, 1.0)
	res.LocalAnchorA.Set(-1.0, 0.0)
	res.LocalAnchorB.Set(1.0, 0.0)
	res.LengthA = 0.0
	res.LengthB = 0.0
	res.Ratio = 1.0
	res.CollideConnected = true

	return res
}

/// The pulley joint is connected to two bodies and two fixed ground points.
/// The pulley supports a ratio such that:
/// length1 + ratio * length2 <= constant
/// Yes, the force transmitted is scaled by the ratio.
/// Warning: the pulley joint can get a bit squirrelly by itself. They often
/// work better when combined with prismatic joints. You should also cover the
/// the anchor points with static shapes to prevent one side from going to
/// zero length.
type B2PulleyJoint struct {
	*B2Joint

	M_groundAnchorA B2Vec2
	M_groundAnchorB B2Vec2
	M_lengthA       float64
	M_lengthB       float64

	// Solver shared
	M_localAnchorA B2Vec2
	M_localAnchorB B2Vec2
	M_constant     float64
	M_ratio        float64
	M_impulse      float64

	// Solver temp
	M_indexA       int
	M_indexB       int
	M_uA           B2Vec2
	M_uB           B2Vec2
	M_rA           B2Vec2
	M_rB           B2Vec2
	M_localCenterA B2Vec2
	M_localCenterB B2Vec2
	M_invMassA     float64
	M_invMassB     float64
	M_invIA        float64
	M_invIB        float64
	M_mass         float64
}

// Pulley:
// length1 = norm(p1 - s1)
// length2 = norm(p2 - s2)
// C0 = (length1 + ratio * length2)_initial
// C = C0 - (length1 + ratio * length2)
// u1 = (p1 - s1) / norm(p1 - s1)
// u2 = (p2 - s2) / norm(p2 - s2)
// Cdot = -dot(u1, v1 + cross(w1, r1)) - ratio * dot(u2, v2 + cross(w2, r2))
// J = -[u1 cross(r1, u1) ratio * u2  ratio * cross(r2, u2)]
// K = J * invM * JT
//   = invMass1 + invI1 * cross(r1, u1)^2 + ratio^2 * (invMass2 + invI2 * cross(r2, u2)^2)

func (def *B2PulleyJointDef) Initialize(bA *B2Body, bB *B2Body, groundA B2Vec2, groundB B2Vec2, anchorA B2Vec2, anchorB B2Vec2, r float64) {
	def.BodyA = bA
	def.BodyB = bB
	def.GroundAnchorA = groundA
	def.GroundAnchorB = groundB
	def.LocalAnchorA = def.BodyA.GetLocalPoint(anchorA)
	def.LocalAnchorB = def.BodyB.GetLocalPoint(anchorB)
	dA := B2Vec2Sub(anchorA, groundA)
	def.LengthA = dA.Length()
	dB := B2Vec2Sub(anchorB, groundB)
	def.LengthB = dB.Length()
	def.Ratio = r
	B2Assert(def.Ratio > B2_epsilon)
}

func MakeB2PulleyJoint(def *B2PulleyJointDef) *B2PulleyJoint {
	res := B2PulleyJoint{
		B2Joint: MakeB2Joint(def),
	}

	res.M_groundAnchorA = def.GroundAnchorA
	res.M_groundAnchorB = def.GroundAnchorB
	res.M_localAnchorA = def.LocalAnchorA
	res.M_localAnchorB = def.LocalAnchorB

	res.M_lengthA = def.LengthA
	res.M_lengthB = def.LengthB

	B2Assert(def.Ratio != 0.0)
	res.M_ratio = def.Ratio

	res.M_constant = def.LengthA + res.M_ratio*def.LengthB

	res.M_impulse = 0.0

	return &res
}

func (joint *B2PulleyJoint) InitVelocityConstraints(data B2SolverData) {
	joint.M_indexA = joint.M_bodyA.M_islandIndex
	joint.M_indexB = joint.M_bodyB.M_islandIndex
	joint.M_localCenterA = joint.M_bodyA.M_sweep.LocalCenter
	joint.M_localCenterB = joint.M_bodyB.M_sweep.LocalCenter
	joint.M_invMassA = joint.M_bodyA.M_invMass
	joint.M_invMassB = joint.M_bodyB.M_invMass
	joint.M_invIA = joint.M_bodyA.M_invI
	joint.M_invIB = joint.M_bodyB.M_invI

	cA := data.Positions[joint.M_indexA].C
	aA := data.Positions[joint.M_indexA].A
	vA := data.Velocities[joint.M_indexA].V
	wA := data.Velocities[joint.M_indexA].W

	cB := data.Positions[joint.M_indexB].C
	aB := data.Positions[joint.M_indexB].A
	vB := data.Velocities[joint.M_indexB].V
	wB := data.Velocities[joint.M_indexB].W

	qA := MakeB2RotFromAngle(aA)
	qB := MakeB2RotFromAngle(aB)

	joint.M_rA = B2RotVec2Mul(qA, B2Vec2Sub(joint.M_localAnchorA, joint.M_localCenterA))
	joint.M_rB = B2RotVec2Mul(qB, B2Vec2Sub(joint.M_localAnchorB, joint.M_localCenterB))

	// Get the pulley axes.
	joint.M_uA = B2Vec2Sub(B2Vec2Add(cA, joint.M_rA), joint.M_groundAnchorA)
	joint.M_uB = B2Vec2Sub(B2Vec2Add(cB, joint.M_rB), joint.M_groundAnchorB)

	lengthA := joint.M_uA.Length()
	lengthB := joint.M_uB.Length()

	if lengthA > 10.0*B2_linearSlop {
		joint.M_uA.OperatorScalarMulInplace(1.0 / lengthA)
	} else {
		joint.M_uA.SetZero()
	}

	if lengthB > 10.0*B2_linearSlop {
		joint.M_uB.OperatorScalarMulInplace(1.0 / lengthB)
	} else {
		joint.M_uB.SetZero()
	}

	// Compute effective mass.
	ruA := B2Vec2Cross(joint.M_rA, joint.M_uA)
	ruB := B2Vec2Cross(joint.M_rB, joint.M_uB)

	mA := joint.M_invMassA + joint.M_invIA*ruA*ruA
	mB := joint.M_invMassB + joint.M_invIB*ruB*ruB

	joint.M_mass = mA + joint.M_ratio*joint.M_ratio*mB

	if joint.M_mass > 0.0 {
		joint.M_mass = 1.0 / joint.M_mass
	}

	if data.Step.WarmStarting {
		// Scale impulses to support variable time steps.
		joint.M_impulse *= data.Step.DtRatio

		// Warm starting.
		PA := B2Vec2MulScalar(-(joint.M_impulse), joint.M_uA)
		PB := B2Vec2MulScalar(-joint.M_ratio*joint.M_impulse, joint.M_uB)

		vA.OperatorPlusInplace(B2Vec2MulScalar(joint.M_invMassA, PA))
		wA += joint.M_invIA * B2Vec2Cross(joint.M_rA, PA)
		vB.OperatorPlusInplace(B2Vec2MulScalar(joint.M_invMassB, PB))
		wB += joint.M_invIB * B2Vec2Cross(joint.M_rB, PB)
	} else {
		joint.M_impulse = 0.0
	}

	data.Velocities[joint.M_indexA].V = vA
	data.Velocities[joint.M_indexA].W = wA
	data.Velocities[joint.M_indexB].V = vB
	data.Velocities[joint.M_indexB].W = wB
}

func (joint *B2PulleyJoint) SolveVelocityConstraints(data B2SolverData) {
	vA := data.Velocities[joint.M_indexA].V
	wA := data.Velocities[joint.M_indexA].W
	vB := data.Velocities[joint.M_indexB].V
	wB := data.Velocities[joint.M_indexB].W

	vpA := B2Vec2Add(vA, B2Vec2CrossScalarVector(wA, joint.M_rA))
	vpB := B2Vec2Add(vB, B2Vec2CrossScalarVector(wB, joint.M_rB))

	Cdot := -B2Vec2Dot(joint.M_uA, vpA) - joint.M_ratio*B2Vec2Dot(joint.M_uB, vpB)
	impulse := -joint.M_mass * Cdot
	joint.M_impulse += impulse

	PA := B2Vec2MulScalar(-impulse, joint.M_uA)
	PB := B2Vec2MulScalar(-joint.M_ratio*impulse, joint.M_uB)
	vA.OperatorPlusInplace(B2Vec2MulScalar(joint.M_invMassA, PA))
	wA += joint.M_invIA * B2Vec2Cross(joint.M_rA, PA)
	vB.OperatorPlusInplace(B2Vec2MulScalar(joint.M_invMassB, PB))
	wB += joint.M_invIB * B2Vec2Cross(joint.M_rB, PB)

	data.Velocities[joint.M_indexA].V = vA
	data.Velocities[joint.M_indexA].W = wA
	data.Velocities[joint.M_indexB].V = vB
	data.Velocities[joint.M_indexB].W = wB
}

func (joint *B2PulleyJoint) SolvePositionConstraints(data B2SolverData) bool {
	cA := data.Positions[joint.M_indexA].C
	aA := data.Positions[joint.M_indexA].A
	cB := data.Positions[joint.M_indexB].C
	aB := data.Positions[joint.M_indexB].A

	qA := MakeB2RotFromAngle(aA)
	qB := MakeB2RotFromAngle(aB)

	rA := B2RotVec2Mul(qA, B2Vec2Sub(joint.M_localAnchorA, joint.M_localCenterA))
	rB := B2RotVec2Mul(qB, B2Vec2Sub(joint.M_localAnchorB, joint.M_localCenterB))

	// Get the pulley axes.
	uA := B2Vec2Sub(B2Vec2Add(cA, rA), joint.M_groundAnchorA)
	uB := B2Vec2Sub(B2Vec2Add(cB, rB), joint.M_groundAnchorB)

	lengthA := uA.Length()
	lengthB := uB.Length()

	if lengthA > 10.0*B2_linearSlop {
		uA.OperatorScalarMulInplace(1.0 / lengthA)
	} else {
		uA.SetZero()
	}

	if lengthB > 10.0*B2_linearSlop {
		uB.OperatorScalarMulInplace(1.0 / lengthB)
	} else {
		uB.SetZero()
	}

	// Compute effective mass.
	ruA := B2Vec2Cross(rA, uA)
	ruB := B2Vec2Cross(rB, uB)

	mA := joint.M_invMassA + joint.M_invIA*ruA*ruA
	mB := joint.M_invMassB + joint.M_invIB*ruB*ruB

	mass := mA + joint.M_ratio*joint.M_ratio*mB

	if mass > 0.0 {
		mass = 1.0 / mass
	}

	C := joint.M_constant - lengthA - joint.M_ratio*lengthB
	linearError := math.Abs(C)

	impulse := -mass * C

	PA := B2Vec2MulScalar(-impulse, uA)
	PB := B2Vec2MulScalar(-joint.M_ratio*impulse, uB)

	cA.OperatorPlusInplace(B2Vec2MulScalar(joint.M_invMassA, PA))
	aA += joint.M_invIA * B2Vec2Cross(rA, PA)
	cB.OperatorPlusInplace(B2Vec2MulScalar(joint.M_invMassB, PB))
	aB += joint.M_invIB * B2Vec2Cross(rB, PB)

	data.Positions[joint.M_indexA].C = cA
	data.Positions[joint.M_indexA].A = aA
	data.Positions[joint.M_indexB].C = cB
	data.Positions[joint.M_indexB].A = aB

	return linearError < B2_linearSlop
}

func (joint B2PulleyJoint) GetAnchorA() B2Vec2 {
	return joint.M_bodyA.GetWorldPoint(joint.M_localAnchorA)
}

func (joint B2PulleyJoint) GetAnchorB() B2Vec2 {
	return joint.M_bodyB.GetWorldPoint(joint.M_localAnchorB)
}

func (joint B2PulleyJoint) GetReactionForce(inv_dt float64) B2Vec2 {
	P := B2Vec2MulScalar(joint.M_impulse, joint.M_uB)
	return B2Vec2MulScalar(inv_dt, P)
}

func (joint B2PulleyJoint) GetReactionTorque(inv_dt float64) float64 {
	return 0.0
}

func (joint B2PulleyJoint) GetGroundAnchorA() B2Vec2 {
	return joint.M_groundAnchorA
}

func (joint B2PulleyJoint) GetGroundAnchorB() B2Vec2 {
	return joint.M_groundAnchorB
}

func (joint B2PulleyJoint) GetLengthA() float64 {
	return joint.M_lengthA
}

func (joint B2PulleyJoint) GetLengthB() float64 {
	return joint.M_lengthB
}

func (joint B2PulleyJoint) GetRatio() float64 {
	return joint.M_ratio
}

func (joint B2PulleyJoint) GetCurrentLengthA() float64 {
	p := joint.M_bodyA.GetWorldPoint(joint.M_localAnchorA)
	s := joint.M_groundAnchorA
	d := B2Vec2Sub(p, s)
	return d.Length()
}

func (joint B2PulleyJoint) GetCurrentLengthB() float64 {
	p := joint.M_bodyB.GetWorldPoint(joint.M_localAnchorB)
	s := joint.M_groundAnchorB
	d := B2Vec2Sub(p, s)
	return d.Length()
}

func (joint *B2PulleyJoint) Dump() {
	indexA := joint.M_bodyA.M_islandIndex
	indexB := joint.M_bodyB.M_islandIndex

	fmt.Printf("  b2PulleyJointDef jd;\n")
	fmt.Printf("  jd.bodyA = bodies[%d];\n", indexA)
	fmt.Printf("  jd.bodyB = bodies[%d];\n", indexB)
	fmt.Printf("  jd.collideConnected = bool(%v);\n", joint.M_collideConnected)
	fmt.Printf("  jd.groundAnchorA.Set(%.15f, %.15f);\n", joint.M_groundAnchorA.X, joint.M_groundAnchorA.Y)
	fmt.Printf("  jd.groundAnchorB.Set(%.15f, %.15f);\n", joint.M_groundAnchorB.X, joint.M_groundAnchorB.Y)
	fmt.Printf("  jd.localAnchorA.Set(%.15f, %.15f);\n", joint.M_localAnchorA.X, joint.M_localAnchorA.Y)
	fmt.Printf("  jd.localAnchorB.Set(%.15f, %.15f);\n", joint.M_localAnchorB.X, joint.M_localAnchorB.Y)
	fmt.Printf("  jd.lengthA = %.15f;\n", joint.M_lengthA)
	fmt.Printf("  jd.lengthB = %.15f;\n", joint.M_lengthB)
	fmt.Printf("  jd.ratio = %.15f;\n", joint.M_ratio)
	fmt.Printf("  joints[%d] = m_world.CreateJoint(&jd);\n", joint.M_index)
}

func (joint *B2PulleyJoint) ShiftOrigin(newOrigin B2Vec2) {
	joint.M_groundAnchorA.OperatorMinusInplace(newOrigin)
	joint.M_groundAnchorB.OperatorMinusInplace(newOrigin)
}
