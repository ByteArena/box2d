package box2d

import (
	"math"
)

///
type B2RopeDef struct {

	///
	Vertices []B2Vec2

	///
	Count int

	///
	Masses []float64

	///
	Gravity B2Vec2

	///
	Damping float64

	/// Stretching stiffness
	K2 float64

	/// Bending stiffness. Values above 0.5 can make the simulation blow up.
	K3 float64
}

func MakeB2RopeDef() B2RopeDef {
	res := B2RopeDef{}

	res.Vertices = nil
	res.Count = 0
	res.Masses = nil
	res.Gravity.SetZero()
	res.Damping = 0.1
	res.K2 = 0.9
	res.K3 = 0.1

	return res
}

///
type B2Rope struct {
	M_count int
	M_ps    []B2Vec2
	M_p0s   []B2Vec2
	M_vs    []B2Vec2

	M_ims []float64

	M_Ls []float64
	M_as []float64

	M_gravity B2Vec2
	M_damping float64

	M_k2 float64
	M_k3 float64
}

///
func (rope B2Rope) GetVertexCount() int {
	return rope.M_count
}

///
func (rope B2Rope) GetVertices() []B2Vec2 {
	return rope.M_ps
}

func MakeB2Rope() B2Rope {

	res := B2Rope{}

	res.M_count = 0
	res.M_ps = nil
	res.M_p0s = nil
	res.M_vs = nil
	res.M_ims = nil
	res.M_Ls = nil
	res.M_as = nil
	res.M_gravity.SetZero()
	res.M_k2 = 1.0
	res.M_k3 = 0.1

	return res
}

func (rope *B2Rope) Destroy() {
	rope.M_ps = nil
	rope.M_p0s = nil
	rope.M_vs = nil
	rope.M_ims = nil
	rope.M_Ls = nil
	rope.M_as = nil
}

func (rope *B2Rope) Initialize(def *B2RopeDef) {
	B2Assert(def.Count >= 3)
	rope.M_count = def.Count
	rope.M_ps = make([]B2Vec2, rope.M_count)
	rope.M_p0s = make([]B2Vec2, rope.M_count)
	rope.M_vs = make([]B2Vec2, rope.M_count)
	rope.M_ims = make([]float64, rope.M_count)

	for i := 0; i < rope.M_count; i++ {
		rope.M_ps[i] = def.Vertices[i]
		rope.M_p0s[i] = def.Vertices[i]
		rope.M_vs[i].SetZero()

		m := def.Masses[i]
		if m > 0.0 {
			rope.M_ims[i] = 1.0 / m
		} else {
			rope.M_ims[i] = 0.0
		}
	}

	count2 := rope.M_count - 1
	count3 := rope.M_count - 2
	rope.M_Ls = make([]float64, count2)
	rope.M_as = make([]float64, count3)

	for i := 0; i < count2; i++ {
		p1 := rope.M_ps[i]
		p2 := rope.M_ps[i+1]
		rope.M_Ls[i] = B2Vec2Distance(p1, p2)
	}

	for i := 0; i < count3; i++ {
		p1 := rope.M_ps[i]
		p2 := rope.M_ps[i+1]
		p3 := rope.M_ps[i+2]

		d1 := B2Vec2Sub(p2, p1)
		d2 := B2Vec2Sub(p3, p2)

		a := B2Vec2Cross(d1, d2)
		b := B2Vec2Dot(d1, d2)

		rope.M_as[i] = math.Atan2(a, b)
	}

	rope.M_gravity = def.Gravity
	rope.M_damping = def.Damping
	rope.M_k2 = def.K2
	rope.M_k3 = def.K3
}

func (rope *B2Rope) Step(h float64, iterations int) {
	if h == 0.0 {
		return
	}

	d := math.Exp(-h * rope.M_damping)

	for i := 0; i < rope.M_count; i++ {
		rope.M_p0s[i] = rope.M_ps[i]
		if rope.M_ims[i] > 0.0 {
			rope.M_vs[i].OperatorPlusInplace(B2Vec2MulScalar(h, rope.M_gravity))
		}
		rope.M_vs[i].OperatorScalarMulInplace(d)
		rope.M_ps[i].OperatorPlusInplace(B2Vec2MulScalar(h, rope.M_vs[i]))
	}

	for i := 0; i < iterations; i++ {
		rope.SolveC2()
		rope.SolveC3()
		rope.SolveC2()
	}

	inv_h := 1.0 / h
	for i := 0; i < rope.M_count; i++ {
		rope.M_vs[i] = B2Vec2MulScalar(inv_h, B2Vec2Sub(rope.M_ps[i], rope.M_p0s[i]))
	}
}

func (rope *B2Rope) SolveC2() {
	count2 := rope.M_count - 1

	for i := 0; i < count2; i++ {
		p1 := rope.M_ps[i]
		p2 := rope.M_ps[i+1]

		d := B2Vec2Sub(p2, p1)
		L := d.Normalize()

		im1 := rope.M_ims[i]
		im2 := rope.M_ims[i+1]

		if im1+im2 == 0.0 {
			continue
		}

		s1 := im1 / (im1 + im2)
		s2 := im2 / (im1 + im2)

		p1.OperatorMinusInplace(B2Vec2MulScalar(rope.M_k2*s1*(rope.M_Ls[i]-L), d))
		p2.OperatorPlusInplace(B2Vec2MulScalar(rope.M_k2*s2*(rope.M_Ls[i]-L), d))

		rope.M_ps[i] = p1
		rope.M_ps[i+1] = p2
	}
}

func (rope *B2Rope) SetAngle(angle float64) {
	count3 := rope.M_count - 2
	for i := 0; i < count3; i++ {
		rope.M_as[i] = angle
	}
}

func (rope *B2Rope) SolveC3() {
	count3 := rope.M_count - 2

	for i := 0; i < count3; i++ {
		p1 := rope.M_ps[i]
		p2 := rope.M_ps[i+1]
		p3 := rope.M_ps[i+2]

		m1 := rope.M_ims[i]
		m2 := rope.M_ims[i+1]
		m3 := rope.M_ims[i+2]

		d1 := B2Vec2Sub(p2, p1)
		d2 := B2Vec2Sub(p3, p2)

		L1sqr := d1.LengthSquared()
		L2sqr := d2.LengthSquared()

		if L1sqr*L2sqr == 0.0 {
			continue
		}

		a := B2Vec2Cross(d1, d2)
		b := B2Vec2Dot(d1, d2)

		angle := math.Atan2(a, b)

		Jd1 := B2Vec2MulScalar((-1.0 / L1sqr), d1.Skew())
		Jd2 := B2Vec2MulScalar((1.0 / L2sqr), d2.Skew())

		J1 := Jd1.OperatorNegate()
		J2 := B2Vec2Sub(Jd1, Jd2)
		J3 := Jd2

		mass := m1*B2Vec2Dot(J1, J1) + m2*B2Vec2Dot(J2, J2) + m3*B2Vec2Dot(J3, J3)
		if mass == 0.0 {
			continue
		}

		mass = 1.0 / mass

		C := angle - rope.M_as[i]

		for C > B2_pi {
			angle -= 2 * B2_pi
			C = angle - rope.M_as[i]
		}

		for C < -B2_pi {
			angle += 2.0 * B2_pi
			C = angle - rope.M_as[i]
		}

		impulse := -rope.M_k3 * mass * C

		p1.OperatorPlusInplace(B2Vec2MulScalar((m1 * impulse), J1))
		p2.OperatorPlusInplace(B2Vec2MulScalar((m2 * impulse), J2))
		p3.OperatorPlusInplace(B2Vec2MulScalar((m3 * impulse), J3))

		rope.M_ps[i] = p1
		rope.M_ps[i+1] = p2
		rope.M_ps[i+2] = p3
	}
}

// void b2Rope::Draw(b2Draw* draw) const
// {
// 	b2Color c(0.4f, 0.5f, 0.7f);

// 	for (int32 i = 0; i < m_count - 1; ++i)
// 	{
// 		draw.DrawSegment(m_ps[i], m_ps[i+1], c);
// 	}
// }
