package box2d

import (
	"fmt"
)

/// Friction joint definition.
type B2FrictionJointDef struct {
	B2JointDef

	/// The local anchor point relative to bodyA's origin.
	LocalAnchorA B2Vec2

	/// The local anchor point relative to bodyB's origin.
	LocalAnchorB B2Vec2

	/// The maximum friction force in N.
	MaxForce float64

	/// The maximum friction torque in N-m.
	MaxTorque float64
}

func MakeB2FrictionJointDef() B2FrictionJointDef {
	res := B2FrictionJointDef{
		B2JointDef: MakeB2JointDef(),
	}

	res.Type = B2JointType.E_frictionJoint
	res.LocalAnchorA.SetZero()
	res.LocalAnchorB.SetZero()
	res.MaxForce = 0.0
	res.MaxTorque = 0.0

	return res
}

/// Friction joint. This is used for top-down friction.
/// It provides 2D translational friction and angular friction.
type B2FrictionJoint struct {
	*B2Joint

	M_localAnchorA B2Vec2
	M_localAnchorB B2Vec2

	// Solver shared
	M_linearImpulse  B2Vec2
	M_angularImpulse float64
	M_maxForce       float64
	M_maxTorque      float64

	// Solver temp
	M_indexA       int
	M_indexB       int
	M_rA           B2Vec2
	M_rB           B2Vec2
	M_localCenterA B2Vec2
	M_localCenterB B2Vec2
	M_invMassA     float64
	M_invMassB     float64
	M_invIA        float64
	M_invIB        float64
	M_linearMass   B2Mat22
	M_angularMass  float64
}

/// The local anchor point relative to bodyA's origin.
func (joint B2FrictionJoint) GetLocalAnchorA() B2Vec2 {
	return joint.M_localAnchorA
}

/// The local anchor point relative to bodyB's origin.
func (joint B2FrictionJoint) GetLocalAnchorB() B2Vec2 {
	return joint.M_localAnchorB
}

// Point-to-point constraint
// Cdot = v2 - v1
//      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
// J = [-I -r1_skew I r2_skew ]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

// Angle constraint
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
// K = invI1 + invI2

func (joint *B2FrictionJointDef) Initialize(bA *B2Body, bB *B2Body, anchor B2Vec2) {
	joint.BodyA = bA
	joint.BodyB = bB
	joint.LocalAnchorA = joint.BodyA.GetLocalPoint(anchor)
	joint.LocalAnchorB = joint.BodyB.GetLocalPoint(anchor)
}

func MakeB2FrictionJoint(def *B2FrictionJointDef) *B2FrictionJoint {
	res := B2FrictionJoint{
		B2Joint: MakeB2Joint(def),
	}

	res.M_localAnchorA = def.LocalAnchorA
	res.M_localAnchorB = def.LocalAnchorB

	res.M_linearImpulse.SetZero()
	res.M_angularImpulse = 0.0

	res.M_maxForce = def.MaxForce
	res.M_maxTorque = def.MaxTorque

	return &res
}

func (joint *B2FrictionJoint) InitVelocityConstraints(data B2SolverData) {

	joint.M_indexA = joint.M_bodyA.M_islandIndex
	joint.M_indexB = joint.M_bodyB.M_islandIndex
	joint.M_localCenterA = joint.M_bodyA.M_sweep.LocalCenter
	joint.M_localCenterB = joint.M_bodyB.M_sweep.LocalCenter
	joint.M_invMassA = joint.M_bodyA.M_invMass
	joint.M_invMassB = joint.M_bodyB.M_invMass
	joint.M_invIA = joint.M_bodyA.M_invI
	joint.M_invIB = joint.M_bodyB.M_invI

	aA := data.Positions[joint.M_indexA].A
	vA := data.Velocities[joint.M_indexA].V
	wA := data.Velocities[joint.M_indexA].W

	aB := data.Positions[joint.M_indexB].A
	vB := data.Velocities[joint.M_indexB].V
	wB := data.Velocities[joint.M_indexB].W

	qA := MakeB2RotFromAngle(aA)
	qB := MakeB2RotFromAngle(aB)

	// Compute the effective mass matrix.
	joint.M_rA = B2RotVec2Mul(qA, B2Vec2Sub(joint.M_localAnchorA, joint.M_localCenterA))
	joint.M_rB = B2RotVec2Mul(qB, B2Vec2Sub(joint.M_localAnchorB, joint.M_localCenterB))

	// J = [-I -r1_skew I r2_skew]
	//     [ 0       -1 0       1]
	// r_skew = [-ry; rx]

	// Matlab
	// K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
	//     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
	//     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]

	mA := joint.M_invMassA
	mB := joint.M_invMassB
	iA := joint.M_invIA
	iB := joint.M_invIB

	var K B2Mat22
	K.Ex.X = mA + mB + iA*joint.M_rA.Y*joint.M_rA.Y + iB*joint.M_rB.Y*joint.M_rB.Y
	K.Ex.Y = -iA*joint.M_rA.X*joint.M_rA.Y - iB*joint.M_rB.X*joint.M_rB.Y
	K.Ey.X = K.Ex.Y
	K.Ey.Y = mA + mB + iA*joint.M_rA.X*joint.M_rA.X + iB*joint.M_rB.X*joint.M_rB.X

	joint.M_linearMass = K.GetInverse()

	joint.M_angularMass = iA + iB
	if joint.M_angularMass > 0.0 {
		joint.M_angularMass = 1.0 / joint.M_angularMass
	}

	if data.Step.WarmStarting {
		// Scale impulses to support a variable time step.
		joint.M_linearImpulse.OperatorScalarMulInplace(data.Step.DtRatio)
		joint.M_angularImpulse *= data.Step.DtRatio

		P := MakeB2Vec2(joint.M_linearImpulse.X, joint.M_linearImpulse.Y)
		vA.OperatorMinusInplace(B2Vec2MulScalar(mA, P))
		wA -= iA * (B2Vec2Cross(joint.M_rA, P) + joint.M_angularImpulse)
		vB.OperatorPlusInplace(B2Vec2MulScalar(mB, P))
		wB += iB * (B2Vec2Cross(joint.M_rB, P) + joint.M_angularImpulse)
	} else {
		joint.M_linearImpulse.SetZero()
		joint.M_angularImpulse = 0.0
	}

	data.Velocities[joint.M_indexA].V = vA
	data.Velocities[joint.M_indexA].W = wA
	data.Velocities[joint.M_indexB].V = vB
	data.Velocities[joint.M_indexB].W = wB
}

func (joint *B2FrictionJoint) SolveVelocityConstraints(data B2SolverData) {
	vA := data.Velocities[joint.M_indexA].V
	wA := data.Velocities[joint.M_indexA].W
	vB := data.Velocities[joint.M_indexB].V
	wB := data.Velocities[joint.M_indexB].W

	mA := joint.M_invMassA
	mB := joint.M_invMassB
	iA := joint.M_invIA
	iB := joint.M_invIB

	h := data.Step.Dt

	// Solve angular friction
	{
		Cdot := wB - wA
		impulse := -joint.M_angularMass * Cdot

		oldImpulse := joint.M_angularImpulse
		maxImpulse := h * joint.M_maxTorque
		joint.M_angularImpulse = B2FloatClamp(joint.M_angularImpulse+impulse, -maxImpulse, maxImpulse)
		impulse = joint.M_angularImpulse - oldImpulse

		wA -= iA * impulse
		wB += iB * impulse
	}

	// Solve linear friction
	{
		Cdot := B2Vec2Sub(B2Vec2Sub(B2Vec2Add(vB, B2Vec2CrossScalarVector(wB, joint.M_rB)), vA), B2Vec2CrossScalarVector(wA, joint.M_rA))

		impulse := B2Vec2Mat22Mul(joint.M_linearMass, Cdot).OperatorNegate()
		oldImpulse := joint.M_linearImpulse
		joint.M_linearImpulse.OperatorPlusInplace(impulse)

		maxImpulse := h * joint.M_maxForce

		if joint.M_linearImpulse.LengthSquared() > maxImpulse*maxImpulse {
			joint.M_linearImpulse.Normalize()
			joint.M_linearImpulse.OperatorScalarMulInplace(maxImpulse)
		}

		impulse = B2Vec2Sub(joint.M_linearImpulse, oldImpulse)

		vA.OperatorMinusInplace(B2Vec2MulScalar(mA, impulse))
		wA -= iA * B2Vec2Cross(joint.M_rA, impulse)

		vB.OperatorPlusInplace(B2Vec2MulScalar(mB, impulse))
		wB += iB * B2Vec2Cross(joint.M_rB, impulse)
	}

	data.Velocities[joint.M_indexA].V = vA
	data.Velocities[joint.M_indexA].W = wA
	data.Velocities[joint.M_indexB].V = vB
	data.Velocities[joint.M_indexB].W = wB
}

func (joint *B2FrictionJoint) SolvePositionConstraints(data B2SolverData) bool {
	return true
}

func (joint B2FrictionJoint) GetAnchorA() B2Vec2 {
	return joint.M_bodyA.GetWorldPoint(joint.M_localAnchorA)
}

func (joint B2FrictionJoint) GetAnchorB() B2Vec2 {
	return joint.M_bodyB.GetWorldPoint(joint.M_localAnchorB)
}

func (joint B2FrictionJoint) GetReactionForce(inv_dt float64) B2Vec2 {
	return B2Vec2MulScalar(inv_dt, joint.M_linearImpulse)
}

func (joint B2FrictionJoint) GetReactionTorque(inv_dt float64) float64 {
	return inv_dt * joint.M_angularImpulse
}

func (joint *B2FrictionJoint) SetMaxForce(force float64) {
	B2Assert(B2IsValid(force) && force >= 0.0)
	joint.M_maxForce = force
}

func (joint B2FrictionJoint) GetMaxForce() float64 {
	return joint.M_maxForce
}

func (joint *B2FrictionJoint) SetMaxTorque(torque float64) {
	B2Assert(B2IsValid(torque) && torque >= 0.0)
	joint.M_maxTorque = torque
}

func (joint B2FrictionJoint) GetMaxTorque() float64 {
	return joint.M_maxTorque
}

func (joint *B2FrictionJoint) Dump() {
	indexA := joint.M_bodyA.M_islandIndex
	indexB := joint.M_bodyB.M_islandIndex

	fmt.Printf("  b2FrictionJointDef jd;\n")
	fmt.Printf("  jd.bodyA = bodies[%d];\n", indexA)
	fmt.Printf("  jd.bodyB = bodies[%d];\n", indexB)
	fmt.Printf("  jd.collideConnected = bool(%v);\n", joint.M_collideConnected)
	fmt.Printf("  jd.localAnchorA.Set(%.15f, %.15f);\n", joint.M_localAnchorA.X, joint.M_localAnchorA.Y)
	fmt.Printf("  jd.localAnchorB.Set(%.15f, %.15f);\n", joint.M_localAnchorB.X, joint.M_localAnchorB.Y)
	fmt.Printf("  jd.maxForce = %.15f;\n", joint.M_maxForce)
	fmt.Printf("  jd.maxTorque = %.15f;\n", joint.M_maxTorque)
	fmt.Printf("  joints[%d] = m_world.CreateJoint(&jd);\n", joint.M_index)
}
