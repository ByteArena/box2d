package box2d

import (
	"fmt"
)

/// Mouse joint definition. This requires a world target point,
/// tuning parameters, and the time step.
type B2MouseJointDef struct {
	B2JointDef

	/// The initial world target point. This is assumed
	/// to coincide with the body anchor initially.
	Target B2Vec2

	/// The maximum constraint force that can be exerted
	/// to move the candidate body. Usually you will express
	/// as some multiple of the weight (multiplier * mass * gravity).
	MaxForce float64

	/// The response speed.
	FrequencyHz float64

	/// The damping ratio. 0 = no damping, 1 = critical damping.
	DampingRatio float64
}

func MakeB2MouseJointDef() B2MouseJointDef {
	res := B2MouseJointDef{
		B2JointDef: MakeB2JointDef(),
	}

	res.Type = B2JointType.E_mouseJoint
	res.Target.Set(0.0, 0.0)
	res.MaxForce = 0.0
	res.FrequencyHz = 5.0
	res.DampingRatio = 0.7

	return res
}

/// A mouse joint is used to make a point on a body track a
/// specified world point. This a soft constraint with a maximum
/// force. This allows the constraint to stretch and without
/// applying huge forces.
/// NOTE: this joint is not documented in the manual because it was
/// developed to be used in the testbed. If you want to learn how to
/// use the mouse joint, look at the testbed.
type B2MouseJoint struct {
	*B2Joint

	M_localAnchorB B2Vec2
	M_targetA      B2Vec2
	M_frequencyHz  float64
	M_dampingRatio float64
	M_beta         float64

	// Solver shared
	M_impulse  B2Vec2
	M_maxForce float64
	M_gamma    float64

	// Solver temp
	M_indexA       int
	M_indexB       int
	M_rB           B2Vec2
	M_localCenterB B2Vec2
	M_invMassB     float64
	M_invIB        float64
	M_mass         B2Mat22
	M_C            B2Vec2
}

/// The mouse joint does not support dumping.
func (def *B2MouseJoint) Dump() {
	fmt.Printf("Mouse joint dumping is not supported.\n")
}

// p = attached point, m = mouse point
// C = p - m
// Cdot = v
//      = v + cross(w, r)
// J = [I r_skew]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

func MakeB2MouseJoint(def *B2MouseJointDef) *B2MouseJoint {
	res := B2MouseJoint{
		B2Joint: MakeB2Joint(def),
	}

	B2Assert(def.Target.IsValid())
	B2Assert(B2IsValid(def.MaxForce) && def.MaxForce >= 0.0)
	B2Assert(B2IsValid(def.FrequencyHz) && def.FrequencyHz >= 0.0)
	B2Assert(B2IsValid(def.DampingRatio) && def.DampingRatio >= 0.0)

	res.M_targetA = def.Target
	res.M_localAnchorB = B2TransformVec2MulT(res.M_bodyB.GetTransform(), res.M_targetA)

	res.M_maxForce = def.MaxForce
	res.M_impulse.SetZero()

	res.M_frequencyHz = def.FrequencyHz
	res.M_dampingRatio = def.DampingRatio

	res.M_beta = 0.0
	res.M_gamma = 0.0

	return &res
}

func (joint *B2MouseJoint) SetTarget(target B2Vec2) {
	if target != joint.M_targetA {
		joint.M_bodyB.SetAwake(true)
		joint.M_targetA = target
	}
}

func (joint B2MouseJoint) GetTarget() B2Vec2 {
	return joint.M_targetA
}

func (joint *B2MouseJoint) SetMaxForce(force float64) {
	joint.M_maxForce = force
}

func (joint B2MouseJoint) GetMaxForce() float64 {
	return joint.M_maxForce
}

func (joint *B2MouseJoint) SetFrequency(hz float64) {
	joint.M_frequencyHz = hz
}

func (joint B2MouseJoint) GetFrequency() float64 {
	return joint.M_frequencyHz
}

func (joint *B2MouseJoint) SetDampingRatio(ratio float64) {
	joint.M_dampingRatio = ratio
}

func (joint B2MouseJoint) GetDampingRatio() float64 {
	return joint.M_dampingRatio
}

func (joint *B2MouseJoint) InitVelocityConstraints(data B2SolverData) {
	joint.M_indexB = joint.M_bodyB.M_islandIndex
	joint.M_localCenterB = joint.M_bodyB.M_sweep.LocalCenter
	joint.M_invMassB = joint.M_bodyB.M_invMass
	joint.M_invIB = joint.M_bodyB.M_invI

	cB := data.Positions[joint.M_indexB].C
	aB := data.Positions[joint.M_indexB].A
	vB := data.Velocities[joint.M_indexB].V
	wB := data.Velocities[joint.M_indexB].W

	qB := MakeB2RotFromAngle(aB)

	mass := joint.M_bodyB.GetMass()

	// Frequency
	omega := 2.0 * B2_pi * joint.M_frequencyHz

	// Damping coefficient
	d := 2.0 * mass * joint.M_dampingRatio * omega

	// Spring stiffness
	k := mass * (omega * omega)

	// magic formulas
	// gamma has units of inverse mass.
	// beta has units of inverse time.
	h := data.Step.Dt
	B2Assert(d+h*k > B2_epsilon)
	joint.M_gamma = h * (d + h*k)
	if joint.M_gamma != 0.0 {
		joint.M_gamma = 1.0 / joint.M_gamma
	}
	joint.M_beta = h * k * joint.M_gamma

	// Compute the effective mass matrix.
	joint.M_rB = B2RotVec2Mul(qB, B2Vec2Sub(joint.M_localAnchorB, joint.M_localCenterB))

	// K    = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
	//      = [1/m1+1/m2     0    ] + invI1 * [r1.y*r1.y -r1.x*r1.y] + invI2 * [r1.y*r1.y -r1.x*r1.y]
	//        [    0     1/m1+1/m2]           [-r1.x*r1.y r1.x*r1.x]           [-r1.x*r1.y r1.x*r1.x]
	var K B2Mat22
	K.Ex.X = joint.M_invMassB + joint.M_invIB*joint.M_rB.Y*joint.M_rB.Y + joint.M_gamma
	K.Ex.Y = -joint.M_invIB * joint.M_rB.X * joint.M_rB.Y
	K.Ey.X = K.Ex.Y
	K.Ey.Y = joint.M_invMassB + joint.M_invIB*joint.M_rB.X*joint.M_rB.X + joint.M_gamma

	joint.M_mass = K.GetInverse()

	joint.M_C = B2Vec2Sub(B2Vec2Add(cB, joint.M_rB), joint.M_targetA)
	joint.M_C.OperatorScalarMulInplace(joint.M_beta)

	// Cheat with some damping
	wB *= 0.98

	if data.Step.WarmStarting {
		joint.M_impulse.OperatorScalarMulInplace(data.Step.DtRatio)
		vB.OperatorPlusInplace(B2Vec2MulScalar(joint.M_invMassB, joint.M_impulse))
		wB += joint.M_invIB * B2Vec2Cross(joint.M_rB, joint.M_impulse)
	} else {
		joint.M_impulse.SetZero()
	}

	data.Velocities[joint.M_indexB].V = vB
	data.Velocities[joint.M_indexB].W = wB
}

func (joint *B2MouseJoint) SolveVelocityConstraints(data B2SolverData) {

	vB := data.Velocities[joint.M_indexB].V
	wB := data.Velocities[joint.M_indexB].W

	// Cdot = v + cross(w, r)
	Cdot := B2Vec2Add(vB, B2Vec2CrossScalarVector(wB, joint.M_rB))
	impulse := B2Vec2Mat22Mul(joint.M_mass, (B2Vec2Add(B2Vec2Add(Cdot, joint.M_C), B2Vec2MulScalar(joint.M_gamma, joint.M_impulse))).OperatorNegate())

	oldImpulse := joint.M_impulse
	joint.M_impulse.OperatorPlusInplace(impulse)
	maxImpulse := data.Step.Dt * joint.M_maxForce
	if joint.M_impulse.LengthSquared() > maxImpulse*maxImpulse {
		joint.M_impulse.OperatorScalarMulInplace(maxImpulse / joint.M_impulse.Length())
	}
	impulse = B2Vec2Sub(joint.M_impulse, oldImpulse)

	vB.OperatorPlusInplace(B2Vec2MulScalar(joint.M_invMassB, impulse))
	wB += joint.M_invIB * B2Vec2Cross(joint.M_rB, impulse)

	data.Velocities[joint.M_indexB].V = vB
	data.Velocities[joint.M_indexB].W = wB
}

func (joint *B2MouseJoint) SolvePositionConstraints(data B2SolverData) bool {
	return true
}

func (joint B2MouseJoint) GetAnchorA() B2Vec2 {
	return joint.M_targetA
}

func (joint B2MouseJoint) GetAnchorB() B2Vec2 {
	return joint.M_bodyB.GetWorldPoint(joint.M_localAnchorB)
}

func (joint B2MouseJoint) GetReactionForce(inv_dt float64) B2Vec2 {
	return B2Vec2MulScalar(inv_dt, joint.M_impulse)
}

func (joint B2MouseJoint) GetReactionTorque(inv_dt float64) float64 {
	return inv_dt * 0.0
}

func (joint *B2MouseJoint) ShiftOrigin(newOrigin B2Vec2) {
	joint.M_targetA.OperatorMinusInplace(newOrigin)
}
