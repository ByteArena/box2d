package box2d

import (
	"fmt"
	"math"
)

/// Distance joint definition. This requires defining an
/// anchor point on both bodies and the non-zero length of the
/// distance joint. The definition uses local anchor points
/// so that the initial configuration can violate the constraint
/// slightly. This helps when saving and loading a game.
/// @warning Do not use a zero or short length.
type B2DistanceJointDef struct {
	B2JointDef

	/// The local anchor point relative to bodyA's origin.
	LocalAnchorA B2Vec2

	/// The local anchor point relative to bodyB's origin.
	LocalAnchorB B2Vec2

	/// The natural length between the anchor points.
	Length float64

	/// The mass-spring-damper frequency in Hertz. A value of 0
	/// disables softness.
	FrequencyHz float64

	/// The damping ratio. 0 = no damping, 1 = critical damping.
	DampingRatio float64
}

func MakeB2DistanceJointDef() B2DistanceJointDef {
	res := B2DistanceJointDef{
		B2JointDef: MakeB2JointDef(),
	}

	res.Type = B2JointType.E_distanceJoint
	res.LocalAnchorA.Set(0.0, 0.0)
	res.LocalAnchorB.Set(0.0, 0.0)
	res.Length = 1.0
	res.FrequencyHz = 0.0
	res.DampingRatio = 0.0

	return res
}

/// A distance joint constrains two points on two bodies
/// to remain at a fixed distance from each other. You can view
/// this as a massless, rigid rod.
type B2DistanceJoint struct {
	*B2Joint

	M_frequencyHz  float64
	M_dampingRatio float64
	M_bias         float64

	// Solver shared
	M_localAnchorA B2Vec2
	M_localAnchorB B2Vec2
	M_gamma        float64
	M_impulse      float64
	M_length       float64

	// Solver temp
	M_indexA       int
	M_indexB       int
	M_u            B2Vec2
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

/// The local anchor point relative to bodyA's origin.
func (joint B2DistanceJoint) GetLocalAnchorA() B2Vec2 {
	return joint.M_localAnchorA
}

/// The local anchor point relative to bodyB's origin.
func (joint B2DistanceJoint) GetLocalAnchorB() B2Vec2 {
	return joint.M_localAnchorB
}

func (joint *B2DistanceJoint) SetLength(length float64) {
	joint.M_length = length
}

func (joint B2DistanceJoint) GetLength() float64 {
	return joint.M_length
}

func (joint *B2DistanceJoint) SetFrequency(hz float64) {
	joint.M_frequencyHz = hz
}

func (joint B2DistanceJoint) GetFrequency() float64 {
	return joint.M_frequencyHz
}

func (joint *B2DistanceJoint) SetDampingRatio(ratio float64) {
	joint.M_dampingRatio = ratio
}

func (joint B2DistanceJoint) GetDampingRatio() float64 {
	return joint.M_dampingRatio
}

// 1-D constrained system
// m (v2 - v1) = lambda
// v2 + (beta/h) * x1 + gamma * lambda = 0, gamma has units of inverse mass.
// x2 = x1 + h * v2

// 1-D mass-damper-spring system
// m (v2 - v1) + h * d * v2 + h * k *

// C = norm(p2 - p1) - L
// u = (p2 - p1) / norm(p2 - p1)
// Cdot = dot(u, v2 + cross(w2, r2) - v1 - cross(w1, r1))
// J = [-u -cross(r1, u) u cross(r2, u)]
// K = J * invM * JT
//   = invMass1 + invI1 * cross(r1, u)^2 + invMass2 + invI2 * cross(r2, u)^2

func (joint *B2DistanceJointDef) Initialize(b1 *B2Body, b2 *B2Body, anchor1 B2Vec2, anchor2 B2Vec2) {
	joint.BodyA = b1
	joint.BodyB = b2
	joint.LocalAnchorA = joint.BodyA.GetLocalPoint(anchor1)
	joint.LocalAnchorB = joint.BodyB.GetLocalPoint(anchor2)
	d := B2Vec2Sub(anchor2, anchor1)
	joint.Length = d.Length()
}

func MakeB2DistanceJoint(def *B2DistanceJointDef) *B2DistanceJoint {
	res := B2DistanceJoint{
		B2Joint: MakeB2Joint(def),
	}

	res.M_localAnchorA = def.LocalAnchorA
	res.M_localAnchorB = def.LocalAnchorB
	res.M_length = def.Length
	res.M_frequencyHz = def.FrequencyHz
	res.M_dampingRatio = def.DampingRatio
	res.M_impulse = 0.0
	res.M_gamma = 0.0
	res.M_bias = 0.0

	return &res
}

func (joint *B2DistanceJoint) InitVelocityConstraints(data B2SolverData) {
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
	joint.M_u = B2Vec2Sub(B2Vec2Sub(B2Vec2Add(cB, joint.M_rB), cA), joint.M_rA)

	// Handle singularity.
	length := joint.M_u.Length()
	if length > B2_linearSlop {
		joint.M_u.OperatorScalarMulInplace(1.0 / length)
	} else {
		joint.M_u.Set(0.0, 0.0)
	}

	crAu := B2Vec2Cross(joint.M_rA, joint.M_u)
	crBu := B2Vec2Cross(joint.M_rB, joint.M_u)
	invMass := joint.M_invMassA + joint.M_invIA*crAu*crAu + joint.M_invMassB + joint.M_invIB*crBu*crBu

	// Compute the effective mass matrix.
	if invMass != 0.0 {
		joint.M_mass = 1.0 / invMass
	} else {
		joint.M_mass = 0
	}

	if joint.M_frequencyHz > 0.0 {
		C := length - joint.M_length

		// Frequency
		omega := 2.0 * B2_pi * joint.M_frequencyHz

		// Damping coefficient
		d := 2.0 * joint.M_mass * joint.M_dampingRatio * omega

		// Spring stiffness
		k := joint.M_mass * omega * omega

		// magic formulas
		h := data.Step.Dt
		joint.M_gamma = h * (d + h*k)
		if joint.M_gamma != 0.0 {
			joint.M_gamma = 1.0 / joint.M_gamma
		} else {
			joint.M_gamma = 0.0
		}
		joint.M_bias = C * h * k * joint.M_gamma

		invMass += joint.M_gamma
		if invMass != 0.0 {
			joint.M_mass = 1.0 / invMass
		} else {
			joint.M_mass = 0.0
		}
	} else {
		joint.M_gamma = 0.0
		joint.M_bias = 0.0
	}

	if data.Step.WarmStarting {
		// Scale the impulse to support a variable time step.
		joint.M_impulse *= data.Step.DtRatio

		P := B2Vec2MulScalar(joint.M_impulse, joint.M_u)
		vA.OperatorMinusInplace(B2Vec2MulScalar(joint.M_invMassA, P))
		wA -= joint.M_invIA * B2Vec2Cross(joint.M_rA, P)
		vB.OperatorPlusInplace(B2Vec2MulScalar(joint.M_invMassB, P))
		wB += joint.M_invIB * B2Vec2Cross(joint.M_rB, P)
	} else {
		joint.M_impulse = 0.0
	}

	// Note: mutation on value, not ref; but OK because Velocities is an array
	data.Velocities[joint.M_indexA].V = vA
	data.Velocities[joint.M_indexA].W = wA
	data.Velocities[joint.M_indexB].V = vB
	data.Velocities[joint.M_indexB].W = wB
}

func (joint *B2DistanceJoint) SolveVelocityConstraints(data B2SolverData) {
	vA := data.Velocities[joint.M_indexA].V
	wA := data.Velocities[joint.M_indexA].W
	vB := data.Velocities[joint.M_indexB].V
	wB := data.Velocities[joint.M_indexB].W

	// Cdot = dot(u, v + cross(w, r))
	vpA := B2Vec2Add(vA, B2Vec2CrossScalarVector(wA, joint.M_rA))
	vpB := B2Vec2Add(vB, B2Vec2CrossScalarVector(wB, joint.M_rB))
	Cdot := B2Vec2Dot(joint.M_u, B2Vec2Sub(vpB, vpA))

	impulse := -joint.M_mass * (Cdot + joint.M_bias + joint.M_gamma*joint.M_impulse)
	joint.M_impulse += impulse

	P := B2Vec2MulScalar(impulse, joint.M_u)
	vA.OperatorMinusInplace(B2Vec2MulScalar(joint.M_invMassA, P))
	wA -= joint.M_invIA * B2Vec2Cross(joint.M_rA, P)
	vB.OperatorPlusInplace(B2Vec2MulScalar(joint.M_invMassB, P))
	wB += joint.M_invIB * B2Vec2Cross(joint.M_rB, P)

	// Note: mutation on value, not ref; but OK because Velocities is an array
	data.Velocities[joint.M_indexA].V = vA
	data.Velocities[joint.M_indexA].W = wA
	data.Velocities[joint.M_indexB].V = vB
	data.Velocities[joint.M_indexB].W = wB
}

func (joint *B2DistanceJoint) SolvePositionConstraints(data B2SolverData) bool {
	if joint.M_frequencyHz > 0.0 {
		// There is no position correction for soft distance constraints.
		return true
	}

	cA := data.Positions[joint.M_indexA].C
	aA := data.Positions[joint.M_indexA].A
	cB := data.Positions[joint.M_indexB].C
	aB := data.Positions[joint.M_indexB].A

	qA := MakeB2RotFromAngle(aA)
	qB := MakeB2RotFromAngle(aB)

	rA := B2RotVec2Mul(qA, B2Vec2Sub(joint.M_localAnchorA, joint.M_localCenterA))
	rB := B2RotVec2Mul(qB, B2Vec2Sub(joint.M_localAnchorB, joint.M_localCenterB))
	u := B2Vec2Sub(B2Vec2Sub(B2Vec2Add(cB, rB), cA), rA)

	length := u.Normalize()
	C := length - joint.M_length
	C = B2FloatClamp(C, -B2_maxLinearCorrection, B2_maxLinearCorrection)

	impulse := -joint.M_mass * C
	P := B2Vec2MulScalar(impulse, u)

	cA.OperatorMinusInplace(B2Vec2MulScalar(joint.M_invMassA, P))
	aA -= joint.M_invIA * B2Vec2Cross(rA, P)
	cB.OperatorPlusInplace(B2Vec2MulScalar(joint.M_invMassB, P))
	aB += joint.M_invIB * B2Vec2Cross(rB, P)

	// Note: mutation on value, not ref; but OK because Positions is an array
	data.Positions[joint.M_indexA].C = cA
	data.Positions[joint.M_indexA].A = aA
	data.Positions[joint.M_indexB].C = cB
	data.Positions[joint.M_indexB].A = aB

	return math.Abs(C) < B2_linearSlop
}

func (joint B2DistanceJoint) GetAnchorA() B2Vec2 {
	return joint.M_bodyA.GetWorldPoint(joint.M_localAnchorA)
}

func (joint B2DistanceJoint) GetAnchorB() B2Vec2 {
	return joint.M_bodyB.GetWorldPoint(joint.M_localAnchorB)
}

func (joint B2DistanceJoint) GetReactionForce(inv_dt float64) B2Vec2 {
	return B2Vec2MulScalar((inv_dt * joint.M_impulse), joint.M_u)
}

func (joint B2DistanceJoint) GetReactionTorque(inv_dt float64) float64 {
	return 0.0
}

func (joint B2DistanceJoint) Dump() {
	indexA := joint.M_bodyA.M_islandIndex
	indexB := joint.M_bodyB.M_islandIndex

	fmt.Printf("  b2DistanceJointDef jd;\n")
	fmt.Printf("  jd.bodyA = bodies[%d];\n", indexA)
	fmt.Printf("  jd.bodyB = bodies[%d];\n", indexB)
	fmt.Printf("  jd.collideConnected = bool(%d);\n", joint.M_collideConnected)
	fmt.Printf("  jd.localAnchorA.Set(%.15lef, %.15lef);\n", joint.M_localAnchorA.X, joint.M_localAnchorA.Y)
	fmt.Printf("  jd.localAnchorB.Set(%.15lef, %.15lef);\n", joint.M_localAnchorB.X, joint.M_localAnchorB.Y)
	fmt.Printf("  jd.length = %.15lef;\n", joint.M_length)
	fmt.Printf("  jd.frequencyHz = %.15lef;\n", joint.M_frequencyHz)
	fmt.Printf("  jd.dampingRatio = %.15lef;\n", joint.M_dampingRatio)
	fmt.Printf("  joints[%d] = m_world.CreateJoint(&jd);\n", joint.M_index)
}
