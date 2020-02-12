package box2d

import (
	"fmt"
	"math"
)

/// Wheel joint definition. This requires defining a line of
/// motion using an axis and an anchor point. The definition uses local
/// anchor points and a local axis so that the initial configuration
/// can violate the constraint slightly. The joint translation is zero
/// when the local anchor points coincide in world space. Using local
/// anchors and a local axis helps when saving and loading a game.
type B2WheelJointDef struct {
	B2JointDef

	/// The local anchor point relative to bodyA's origin.
	LocalAnchorA B2Vec2

	/// The local anchor point relative to bodyB's origin.
	LocalAnchorB B2Vec2

	/// The local translation axis in bodyA.
	LocalAxisA B2Vec2

	/// Enable/disable the joint motor.
	EnableMotor bool

	/// The maximum motor torque, usually in N-m.
	MaxMotorTorque float64

	/// The desired motor speed in radians per second.
	MotorSpeed float64

	/// Suspension frequency, zero indicates no suspension
	FrequencyHz float64

	/// Suspension damping ratio, one indicates critical damping
	DampingRatio float64
}

func MakeB2WheelJointDef() B2WheelJointDef {
	res := B2WheelJointDef{
		B2JointDef: MakeB2JointDef(),
	}

	res.Type = B2JointType.E_wheelJoint
	res.LocalAnchorA.SetZero()
	res.LocalAnchorB.SetZero()
	res.LocalAxisA.Set(1.0, 0.0)
	res.EnableMotor = false
	res.MaxMotorTorque = 0.0
	res.MotorSpeed = 0.0
	res.FrequencyHz = 2.0
	res.DampingRatio = 0.7

	return res
}

/// A wheel joint. This joint provides two degrees of freedom: translation
/// along an axis fixed in bodyA and rotation in the plane. In other words, it is a point to
/// line constraint with a rotational motor and a linear spring/damper.
/// This joint is designed for vehicle suspensions.
type B2WheelJoint struct {
	*B2Joint

	M_frequencyHz  float64
	M_dampingRatio float64

	// Solver shared
	M_localAnchorA B2Vec2
	M_localAnchorB B2Vec2
	M_localXAxisA  B2Vec2
	M_localYAxisA  B2Vec2

	M_impulse       float64
	M_motorImpulse  float64
	M_springImpulse float64

	M_maxMotorTorque float64
	M_motorSpeed     float64
	M_enableMotor    bool

	// Solver temp
	M_indexA       int
	M_indexB       int
	M_localCenterA B2Vec2
	M_localCenterB B2Vec2
	M_invMassA     float64
	M_invMassB     float64
	M_invIA        float64
	M_invIB        float64

	M_ax  B2Vec2
	M_ay  B2Vec2
	M_sAx float64
	M_sBx float64
	M_sAy float64
	M_sBy float64

	M_mass       float64
	M_motorMass  float64
	M_springMass float64

	M_bias  float64
	M_gamma float64
}

/// The local anchor point relative to bodyA's origin.
func (joint B2WheelJoint) GetLocalAnchorA() B2Vec2 {
	return joint.M_localAnchorA
}

/// The local anchor point relative to bodyB's origin.
func (joint B2WheelJoint) GetLocalAnchorB() B2Vec2 {
	return joint.M_localAnchorB
}

/// The local joint axis relative to bodyA.
func (joint B2WheelJoint) GetLocalAxisA() B2Vec2 {
	return joint.M_localXAxisA
}

func (joint B2WheelJoint) GetMotorSpeed() float64 {
	return joint.M_motorSpeed
}

func (joint B2WheelJoint) GetMaxMotorTorque() float64 {
	return joint.M_maxMotorTorque
}

func (joint *B2WheelJoint) SetSpringFrequencyHz(hz float64) {
	joint.M_frequencyHz = hz
}

func (joint B2WheelJoint) GetSpringFrequencyHz() float64 {
	return joint.M_frequencyHz
}

func (joint *B2WheelJoint) SetSpringDampingRatio(ratio float64) {
	joint.M_dampingRatio = ratio
}

func (joint B2WheelJoint) GetSpringDampingRatio() float64 {
	return joint.M_dampingRatio
}

// Linear constraint (point-to-line)
// d = pB - pA = xB + rB - xA - rA
// C = dot(ay, d)
// Cdot = dot(d, cross(wA, ay)) + dot(ay, vB + cross(wB, rB) - vA - cross(wA, rA))
//      = -dot(ay, vA) - dot(cross(d + rA, ay), wA) + dot(ay, vB) + dot(cross(rB, ay), vB)
// J = [-ay, -cross(d + rA, ay), ay, cross(rB, ay)]

// Spring linear constraint
// C = dot(ax, d)
// Cdot = = -dot(ax, vA) - dot(cross(d + rA, ax), wA) + dot(ax, vB) + dot(cross(rB, ax), vB)
// J = [-ax -cross(d+rA, ax) ax cross(rB, ax)]

// Motor rotational constraint
// Cdot = wB - wA
// J = [0 0 -1 0 0 1]

func (def *B2WheelJointDef) Initialize(bA *B2Body, bB *B2Body, anchor B2Vec2, axis B2Vec2) {
	def.BodyA = bA
	def.BodyB = bB
	def.LocalAnchorA = def.BodyA.GetLocalPoint(anchor)
	def.LocalAnchorB = def.BodyB.GetLocalPoint(anchor)
	def.LocalAxisA = def.BodyA.GetLocalVector(axis)
}

func MakeB2WheelJoint(def *B2WheelJointDef) *B2WheelJoint {
	res := B2WheelJoint{
		B2Joint: MakeB2Joint(def),
	}

	res.M_localAnchorA = def.LocalAnchorA
	res.M_localAnchorB = def.LocalAnchorB
	res.M_localXAxisA = def.LocalAxisA
	res.M_localYAxisA = B2Vec2CrossScalarVector(1.0, res.M_localXAxisA)

	res.M_mass = 0.0
	res.M_impulse = 0.0
	res.M_motorMass = 0.0
	res.M_motorImpulse = 0.0
	res.M_springMass = 0.0
	res.M_springImpulse = 0.0

	res.M_maxMotorTorque = def.MaxMotorTorque
	res.M_motorSpeed = def.MotorSpeed
	res.M_enableMotor = def.EnableMotor

	res.M_frequencyHz = def.FrequencyHz
	res.M_dampingRatio = def.DampingRatio

	res.M_bias = 0.0
	res.M_gamma = 0.0

	res.M_ax.SetZero()
	res.M_ay.SetZero()

	return &res
}

func (joint *B2WheelJoint) InitVelocityConstraints(data B2SolverData) {

	joint.M_indexA = joint.M_bodyA.M_islandIndex
	joint.M_indexB = joint.M_bodyB.M_islandIndex
	joint.M_localCenterA = joint.M_bodyA.M_sweep.LocalCenter
	joint.M_localCenterB = joint.M_bodyB.M_sweep.LocalCenter
	joint.M_invMassA = joint.M_bodyA.M_invMass
	joint.M_invMassB = joint.M_bodyB.M_invMass
	joint.M_invIA = joint.M_bodyA.M_invI
	joint.M_invIB = joint.M_bodyB.M_invI

	mA := joint.M_invMassA
	mB := joint.M_invMassB
	iA := joint.M_invIA
	iB := joint.M_invIB

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

	// Compute the effective masses.
	rA := B2RotVec2Mul(qA, B2Vec2Sub(joint.M_localAnchorA, joint.M_localCenterA))
	rB := B2RotVec2Mul(qB, B2Vec2Sub(joint.M_localAnchorB, joint.M_localCenterB))
	d := B2Vec2Sub(B2Vec2Sub(B2Vec2Add(cB, rB), cA), rA)

	// Point to line constraint
	{
		joint.M_ay = B2RotVec2Mul(qA, joint.M_localYAxisA)
		joint.M_sAy = B2Vec2Cross(B2Vec2Add(d, rA), joint.M_ay)
		joint.M_sBy = B2Vec2Cross(rB, joint.M_ay)

		joint.M_mass = mA + mB + iA*joint.M_sAy*joint.M_sAy + iB*joint.M_sBy*joint.M_sBy

		if joint.M_mass > 0.0 {
			joint.M_mass = 1.0 / joint.M_mass
		}
	}

	// Spring constraint
	joint.M_springMass = 0.0
	joint.M_bias = 0.0
	joint.M_gamma = 0.0
	if joint.M_frequencyHz > 0.0 {
		joint.M_ax = B2RotVec2Mul(qA, joint.M_localXAxisA)
		joint.M_sAx = B2Vec2Cross(B2Vec2Add(d, rA), joint.M_ax)
		joint.M_sBx = B2Vec2Cross(rB, joint.M_ax)

		invMass := mA + mB + iA*joint.M_sAx*joint.M_sAx + iB*joint.M_sBx*joint.M_sBx

		if invMass > 0.0 {
			joint.M_springMass = 1.0 / invMass

			C := B2Vec2Dot(d, joint.M_ax)

			// Frequency
			omega := 2.0 * B2_pi * joint.M_frequencyHz

			// Damping coefficient
			damp := 2.0 * joint.M_springMass * joint.M_dampingRatio * omega

			// Spring stiffness
			k := joint.M_springMass * omega * omega

			// magic formulas
			h := data.Step.Dt
			joint.M_gamma = h * (damp + h*k)
			if joint.M_gamma > 0.0 {
				joint.M_gamma = 1.0 / joint.M_gamma
			}

			joint.M_bias = C * h * k * joint.M_gamma

			joint.M_springMass = invMass + joint.M_gamma
			if joint.M_springMass > 0.0 {
				joint.M_springMass = 1.0 / joint.M_springMass
			}
		}
	} else {
		joint.M_springImpulse = 0.0
	}

	// Rotational motor
	if joint.M_enableMotor {
		joint.M_motorMass = iA + iB
		if joint.M_motorMass > 0.0 {
			joint.M_motorMass = 1.0 / joint.M_motorMass
		}
	} else {
		joint.M_motorMass = 0.0
		joint.M_motorImpulse = 0.0
	}

	if data.Step.WarmStarting {
		// Account for variable time step.
		joint.M_impulse *= data.Step.DtRatio
		joint.M_springImpulse *= data.Step.DtRatio
		joint.M_motorImpulse *= data.Step.DtRatio

		P := B2Vec2Add(B2Vec2MulScalar(joint.M_impulse, joint.M_ay), B2Vec2MulScalar(joint.M_springImpulse, joint.M_ax))
		LA := joint.M_impulse*joint.M_sAy + joint.M_springImpulse*joint.M_sAx + joint.M_motorImpulse
		LB := joint.M_impulse*joint.M_sBy + joint.M_springImpulse*joint.M_sBx + joint.M_motorImpulse

		vA.OperatorMinusInplace(B2Vec2MulScalar(joint.M_invMassA, P))
		wA -= joint.M_invIA * LA

		vB.OperatorPlusInplace(B2Vec2MulScalar(joint.M_invMassB, P))
		wB += joint.M_invIB * LB
	} else {
		joint.M_impulse = 0.0
		joint.M_springImpulse = 0.0
		joint.M_motorImpulse = 0.0
	}

	data.Velocities[joint.M_indexA].V = vA
	data.Velocities[joint.M_indexA].W = wA
	data.Velocities[joint.M_indexB].V = vB
	data.Velocities[joint.M_indexB].W = wB
}

func (joint *B2WheelJoint) SolveVelocityConstraints(data B2SolverData) {
	mA := joint.M_invMassA
	mB := joint.M_invMassB
	iA := joint.M_invIA
	iB := joint.M_invIB

	vA := data.Velocities[joint.M_indexA].V
	wA := data.Velocities[joint.M_indexA].W
	vB := data.Velocities[joint.M_indexB].V
	wB := data.Velocities[joint.M_indexB].W

	// Solve spring constraint
	{
		Cdot := B2Vec2Dot(joint.M_ax, B2Vec2Sub(vB, vA)) + joint.M_sBx*wB - joint.M_sAx*wA
		impulse := -joint.M_springMass * (Cdot + joint.M_bias + joint.M_gamma*joint.M_springImpulse)
		joint.M_springImpulse += impulse

		P := B2Vec2MulScalar(impulse, joint.M_ax)
		LA := impulse * joint.M_sAx
		LB := impulse * joint.M_sBx

		vA.OperatorMinusInplace(B2Vec2MulScalar(mA, P))
		wA -= iA * LA

		vB.OperatorPlusInplace(B2Vec2MulScalar(mB, P))
		wB += iB * LB
	}

	// Solve rotational motor constraint
	{
		Cdot := wB - wA - joint.M_motorSpeed
		impulse := -joint.M_motorMass * Cdot

		oldImpulse := joint.M_motorImpulse
		maxImpulse := data.Step.Dt * joint.M_maxMotorTorque
		joint.M_motorImpulse = B2FloatClamp(joint.M_motorImpulse+impulse, -maxImpulse, maxImpulse)
		impulse = joint.M_motorImpulse - oldImpulse

		wA -= iA * impulse
		wB += iB * impulse
	}

	// Solve point to line constraint
	{
		Cdot := B2Vec2Dot(joint.M_ay, B2Vec2Sub(vB, vA)) + joint.M_sBy*wB - joint.M_sAy*wA
		impulse := -joint.M_mass * Cdot
		joint.M_impulse += impulse

		P := B2Vec2MulScalar(impulse, joint.M_ay)
		LA := impulse * joint.M_sAy
		LB := impulse * joint.M_sBy

		vA.OperatorMinusInplace(B2Vec2MulScalar(mA, P))
		wA -= iA * LA

		vB.OperatorPlusInplace(B2Vec2MulScalar(mB, P))
		wB += iB * LB
	}

	data.Velocities[joint.M_indexA].V = vA
	data.Velocities[joint.M_indexA].W = wA
	data.Velocities[joint.M_indexB].V = vB
	data.Velocities[joint.M_indexB].W = wB
}

func (joint *B2WheelJoint) SolvePositionConstraints(data B2SolverData) bool {
	cA := data.Positions[joint.M_indexA].C
	aA := data.Positions[joint.M_indexA].A
	cB := data.Positions[joint.M_indexB].C
	aB := data.Positions[joint.M_indexB].A

	qA := MakeB2RotFromAngle(aA)
	qB := MakeB2RotFromAngle(aB)

	rA := B2RotVec2Mul(qA, B2Vec2Sub(joint.M_localAnchorA, joint.M_localCenterA))
	rB := B2RotVec2Mul(qB, B2Vec2Sub(joint.M_localAnchorB, joint.M_localCenterB))
	d := B2Vec2Sub(B2Vec2Add(B2Vec2Sub(cB, cA), rB), rA)

	ay := B2RotVec2Mul(qA, joint.M_localYAxisA)

	sAy := B2Vec2Cross(B2Vec2Add(d, rA), ay)
	sBy := B2Vec2Cross(rB, ay)

	C := B2Vec2Dot(d, ay)

	k := joint.M_invMassA + joint.M_invMassB + joint.M_invIA*joint.M_sAy*joint.M_sAy + joint.M_invIB*joint.M_sBy*joint.M_sBy

	impulse := 0.0
	if k != 0.0 {
		impulse = -C / k
	} else {
		impulse = 0.0
	}

	P := B2Vec2MulScalar(impulse, ay)
	LA := impulse * sAy
	LB := impulse * sBy

	cA.OperatorMinusInplace(B2Vec2MulScalar(joint.M_invMassA, P))
	aA -= joint.M_invIA * LA
	cB.OperatorPlusInplace(B2Vec2MulScalar(joint.M_invMassB, P))
	aB += joint.M_invIB * LB

	data.Positions[joint.M_indexA].C = cA
	data.Positions[joint.M_indexA].A = aA
	data.Positions[joint.M_indexB].C = cB
	data.Positions[joint.M_indexB].A = aB

	return math.Abs(C) <= B2_linearSlop
}

func (joint B2WheelJoint) GetAnchorA() B2Vec2 {
	return joint.M_bodyA.GetWorldPoint(joint.M_localAnchorA)
}

func (joint B2WheelJoint) GetAnchorB() B2Vec2 {
	return joint.M_bodyB.GetWorldPoint(joint.M_localAnchorB)
}

func (joint B2WheelJoint) GetReactionForce(inv_dt float64) B2Vec2 {
	return B2Vec2MulScalar(inv_dt, B2Vec2Add(B2Vec2MulScalar(joint.M_impulse, joint.M_ay), B2Vec2MulScalar(joint.M_springImpulse, joint.M_ax)))
}

func (joint B2WheelJoint) GetReactionTorque(inv_dt float64) float64 {
	return inv_dt * joint.M_motorImpulse
}

func (joint B2WheelJoint) GetJointTranslation() float64 {
	bA := joint.M_bodyA
	bB := joint.M_bodyB

	pA := bA.GetWorldPoint(joint.M_localAnchorA)
	pB := bB.GetWorldPoint(joint.M_localAnchorB)
	d := B2Vec2Sub(pB, pA)
	axis := bA.GetWorldVector(joint.M_localXAxisA)

	translation := B2Vec2Dot(d, axis)
	return translation
}

func (joint B2WheelJoint) GetJointLinearSpeed() float64 {
	bA := joint.M_bodyA
	bB := joint.M_bodyB

	rA := B2RotVec2Mul(bA.M_xf.Q, B2Vec2Sub(joint.M_localAnchorA, bA.M_sweep.LocalCenter))
	rB := B2RotVec2Mul(bB.M_xf.Q, B2Vec2Sub(joint.M_localAnchorB, bB.M_sweep.LocalCenter))
	p1 := B2Vec2Add(bA.M_sweep.C, rA)
	p2 := B2Vec2Add(bB.M_sweep.C, rB)
	d := B2Vec2Sub(p2, p1)
	axis := B2RotVec2Mul(bA.M_xf.Q, joint.M_localXAxisA)

	vA := bA.M_linearVelocity
	vB := bB.M_linearVelocity
	wA := bA.M_angularVelocity
	wB := bB.M_angularVelocity

	speed := B2Vec2Dot(d, B2Vec2CrossScalarVector(wA, axis)) + B2Vec2Dot(axis, B2Vec2Sub(B2Vec2Sub(B2Vec2Add(vB, B2Vec2CrossScalarVector(wB, rB)), vA), B2Vec2CrossScalarVector(wA, rA)))
	return speed
}

func (joint B2WheelJoint) GetJointAngle() float64 {
	bA := joint.M_bodyA
	bB := joint.M_bodyB
	return bB.M_sweep.A - bA.M_sweep.A
}

func (joint B2WheelJoint) GetJointAngularSpeed() float64 {
	wA := joint.M_bodyA.M_angularVelocity
	wB := joint.M_bodyB.M_angularVelocity
	return wB - wA
}

func (joint B2WheelJoint) IsMotorEnabled() bool {
	return joint.M_enableMotor
}

func (joint *B2WheelJoint) EnableMotor(flag bool) {
	if flag != joint.M_enableMotor {
		joint.M_bodyA.SetAwake(true)
		joint.M_bodyB.SetAwake(true)
		joint.M_enableMotor = flag
	}
}

func (joint *B2WheelJoint) SetMotorSpeed(speed float64) {
	if speed != joint.M_motorSpeed {
		joint.M_bodyA.SetAwake(true)
		joint.M_bodyB.SetAwake(true)
		joint.M_motorSpeed = speed
	}
}

func (joint *B2WheelJoint) SetMaxMotorTorque(torque float64) {
	if torque != joint.M_maxMotorTorque {
		joint.M_bodyA.SetAwake(true)
		joint.M_bodyB.SetAwake(true)
		joint.M_maxMotorTorque = torque
	}
}

func (joint B2WheelJoint) GetMotorTorque(inv_dt float64) float64 {
	return inv_dt * joint.M_motorImpulse
}

func (joint *B2WheelJoint) Dump() {
	indexA := joint.M_bodyA.M_islandIndex
	indexB := joint.M_bodyB.M_islandIndex

	fmt.Printf("  b2WheelJointDef jd;\n")
	fmt.Printf("  jd.bodyA = bodies[%d];\n", indexA)
	fmt.Printf("  jd.bodyB = bodies[%d];\n", indexB)
	fmt.Printf("  jd.collideConnected = bool(%v);\n", joint.M_collideConnected)
	fmt.Printf("  jd.localAnchorA.Set(%.15f, %.15f);\n", joint.M_localAnchorA.X, joint.M_localAnchorA.Y)
	fmt.Printf("  jd.localAnchorB.Set(%.15f, %.15f);\n", joint.M_localAnchorB.X, joint.M_localAnchorB.Y)
	fmt.Printf("  jd.localAxisA.Set(%.15f, %.15f);\n", joint.M_localXAxisA.X, joint.M_localXAxisA.Y)
	fmt.Printf("  jd.enableMotor = bool(%v);\n", joint.M_enableMotor)
	fmt.Printf("  jd.motorSpeed = %.15f;\n", joint.M_motorSpeed)
	fmt.Printf("  jd.maxMotorTorque = %.15f;\n", joint.M_maxMotorTorque)
	fmt.Printf("  jd.frequencyHz = %.15f;\n", joint.M_frequencyHz)
	fmt.Printf("  jd.dampingRatio = %.15f;\n", joint.M_dampingRatio)
	fmt.Printf("  joints[%d] = m_world.CreateJoint(&jd);\n", joint.M_index)
}
