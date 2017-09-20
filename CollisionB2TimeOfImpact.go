package box2d

import (
	"math"
)

/// Input parameters for b2TimeOfImpact
type B2TOIInput struct {
	ProxyA B2DistanceProxy
	ProxyB B2DistanceProxy
	SweepA B2Sweep
	SweepB B2Sweep
	TMax   float64 // defines sweep interval [0, tMax]
}

func MakeB2TOIInput() B2TOIInput {
	return B2TOIInput{}
}

// Output parameters for b2TimeOfImpact.

var B2TOIOutput_State = struct {
	E_unknown    uint8
	E_failed     uint8
	E_overlapped uint8
	E_touching   uint8
	E_separated  uint8
}{
	E_unknown:    1,
	E_failed:     2,
	E_overlapped: 3,
	E_touching:   4,
	E_separated:  5,
}

type B2TOIOutput struct {
	State uint8
	T     float64
}

func MakeB2TOIOutput() B2TOIOutput {
	return B2TOIOutput{}
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// B2TimeOfImpact.cpp
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

var B2_toiTime, B2_toiMaxTime float64
var B2_toiCalls, B2_toiIters, B2_toiMaxIters int
var B2_toiRootIters, B2_toiMaxRootIters int

var B2SeparationFunction_Type = struct {
	E_points uint8
	E_faceA  uint8
	E_faceB  uint8
}{
	E_points: 0,
	E_faceA:  1,
	E_faceB:  2,
}

//
type B2SeparationFunction struct {
	M_proxyA           *B2DistanceProxy
	M_proxyB           *B2DistanceProxy
	M_sweepA, M_sweepB B2Sweep
	M_type             uint8
	M_localPoint       B2Vec2
	M_axis             B2Vec2
}

// TODO_ERIN might not need to return the separation
func (sepfunc *B2SeparationFunction) Initialize(cache *B2SimplexCache, proxyA *B2DistanceProxy, sweepA B2Sweep, proxyB *B2DistanceProxy, sweepB B2Sweep, t1 float64) float64 {

	sepfunc.M_proxyA = proxyA
	sepfunc.M_proxyB = proxyB
	count := cache.Count
	B2Assert(0 < count && count < 3)

	sepfunc.M_sweepA = sweepA
	sepfunc.M_sweepB = sweepB

	xfA := MakeB2Transform()
	xfB := MakeB2Transform()
	sepfunc.M_sweepA.GetTransform(&xfA, t1)
	sepfunc.M_sweepB.GetTransform(&xfB, t1)

	if count == 1 {
		sepfunc.M_type = B2SeparationFunction_Type.E_points
		localPointA := sepfunc.M_proxyA.GetVertex(cache.IndexA[0])
		localPointB := sepfunc.M_proxyB.GetVertex(cache.IndexB[0])
		pointA := B2TransformVec2Mul(xfA, localPointA)
		pointB := B2TransformVec2Mul(xfB, localPointB)
		sepfunc.M_axis = B2Vec2Sub(pointB, pointA)
		s := sepfunc.M_axis.Normalize()
		return s
	} else if cache.IndexA[0] == cache.IndexA[1] {
		// Two points on B and one on A.
		sepfunc.M_type = B2SeparationFunction_Type.E_faceB
		localPointB1 := proxyB.GetVertex(cache.IndexB[0])
		localPointB2 := proxyB.GetVertex(cache.IndexB[1])

		sepfunc.M_axis = B2Vec2CrossVectorScalar(
			B2Vec2Sub(localPointB2, localPointB1),
			1.0,
		)

		sepfunc.M_axis.Normalize()
		normal := B2RotVec2Mul(xfB.Q, sepfunc.M_axis)

		sepfunc.M_localPoint = B2Vec2MulScalar(0.5, B2Vec2Add(localPointB1, localPointB2))
		pointB := B2TransformVec2Mul(xfB, sepfunc.M_localPoint)

		localPointA := proxyA.GetVertex(cache.IndexA[0])
		pointA := B2TransformVec2Mul(xfA, localPointA)

		s := B2Vec2Dot(B2Vec2Sub(pointA, pointB), normal)
		if s < 0.0 {
			sepfunc.M_axis = sepfunc.M_axis.OperatorNegate()
			s = -s
		}

		return s
	} else {
		// Two points on A and one or two points on B.
		sepfunc.M_type = B2SeparationFunction_Type.E_faceA
		localPointA1 := sepfunc.M_proxyA.GetVertex(cache.IndexA[0])
		localPointA2 := sepfunc.M_proxyA.GetVertex(cache.IndexA[1])

		sepfunc.M_axis = B2Vec2CrossVectorScalar(B2Vec2Sub(localPointA2, localPointA1), 1.0)
		sepfunc.M_axis.Normalize()
		normal := B2RotVec2Mul(xfA.Q, sepfunc.M_axis)

		sepfunc.M_localPoint = B2Vec2MulScalar(0.5, B2Vec2Add(localPointA1, localPointA2))
		pointA := B2TransformVec2Mul(xfA, sepfunc.M_localPoint)

		localPointB := sepfunc.M_proxyB.GetVertex(cache.IndexB[0])
		pointB := B2TransformVec2Mul(xfB, localPointB)

		s := B2Vec2Dot(B2Vec2Sub(pointB, pointA), normal)
		if s < 0.0 {
			sepfunc.M_axis = sepfunc.M_axis.OperatorNegate()
			s = -s
		}

		return s
	}
}

//
func (sepfunc *B2SeparationFunction) FindMinSeparation(indexA *int, indexB *int, t float64) float64 {

	xfA := MakeB2Transform()
	xfB := MakeB2Transform()

	sepfunc.M_sweepA.GetTransform(&xfA, t)
	sepfunc.M_sweepB.GetTransform(&xfB, t)

	switch sepfunc.M_type {
	case B2SeparationFunction_Type.E_points:
		{
			axisA := B2RotVec2MulT(xfA.Q, sepfunc.M_axis)
			axisB := B2RotVec2MulT(xfB.Q, sepfunc.M_axis.OperatorNegate())

			*indexA = sepfunc.M_proxyA.GetSupport(axisA)
			*indexB = sepfunc.M_proxyB.GetSupport(axisB)

			localPointA := sepfunc.M_proxyA.GetVertex(*indexA)
			localPointB := sepfunc.M_proxyB.GetVertex(*indexB)

			pointA := B2TransformVec2Mul(xfA, localPointA)
			pointB := B2TransformVec2Mul(xfB, localPointB)

			separation := B2Vec2Dot(B2Vec2Sub(pointB, pointA), sepfunc.M_axis)
			return separation
		}

	case B2SeparationFunction_Type.E_faceA:
		{
			normal := B2RotVec2Mul(xfA.Q, sepfunc.M_axis)
			pointA := B2TransformVec2Mul(xfA, sepfunc.M_localPoint)

			axisB := B2RotVec2MulT(xfB.Q, normal.OperatorNegate())

			*indexA = -1
			*indexB = sepfunc.M_proxyB.GetSupport(axisB)

			localPointB := sepfunc.M_proxyB.GetVertex(*indexB)
			pointB := B2TransformVec2Mul(xfB, localPointB)

			separation := B2Vec2Dot(B2Vec2Sub(pointB, pointA), normal)
			return separation
		}

	case B2SeparationFunction_Type.E_faceB:
		{
			normal := B2RotVec2Mul(xfB.Q, sepfunc.M_axis)
			pointB := B2TransformVec2Mul(xfB, sepfunc.M_localPoint)

			axisA := B2RotVec2MulT(xfA.Q, normal.OperatorNegate())

			*indexB = -1
			*indexA = sepfunc.M_proxyA.GetSupport(axisA)

			localPointA := sepfunc.M_proxyA.GetVertex(*indexA)
			pointA := B2TransformVec2Mul(xfA, localPointA)

			separation := B2Vec2Dot(B2Vec2Sub(pointA, pointB), normal)
			return separation
		}

	default:
		B2Assert(false)
		*indexA = -1
		*indexB = -1
		return 0.0
	}
}

//
func (sepfunc *B2SeparationFunction) Evaluate(indexA int, indexB int, t float64) float64 {

	xfA := MakeB2Transform()
	xfB := MakeB2Transform()

	sepfunc.M_sweepA.GetTransform(&xfA, t)
	sepfunc.M_sweepB.GetTransform(&xfB, t)

	switch sepfunc.M_type {
	case B2SeparationFunction_Type.E_points:
		{
			localPointA := sepfunc.M_proxyA.GetVertex(indexA)
			localPointB := sepfunc.M_proxyB.GetVertex(indexB)

			pointA := B2TransformVec2Mul(xfA, localPointA)
			pointB := B2TransformVec2Mul(xfB, localPointB)
			separation := B2Vec2Dot(B2Vec2Sub(pointB, pointA), sepfunc.M_axis)

			return separation
		}

	case B2SeparationFunction_Type.E_faceA:
		{
			normal := B2RotVec2Mul(xfA.Q, sepfunc.M_axis)
			pointA := B2TransformVec2Mul(xfA, sepfunc.M_localPoint)

			localPointB := sepfunc.M_proxyB.GetVertex(indexB)
			pointB := B2TransformVec2Mul(xfB, localPointB)

			separation := B2Vec2Dot(B2Vec2Sub(pointB, pointA), normal)
			return separation
		}

	case B2SeparationFunction_Type.E_faceB:
		{
			normal := B2RotVec2Mul(xfB.Q, sepfunc.M_axis)
			pointB := B2TransformVec2Mul(xfB, sepfunc.M_localPoint)

			localPointA := sepfunc.M_proxyA.GetVertex(indexA)
			pointA := B2TransformVec2Mul(xfA, localPointA)

			separation := B2Vec2Dot(B2Vec2Sub(pointA, pointB), normal)
			return separation
		}

	default:
		B2Assert(false)
		return 0.0
	}
}

/// Compute the upper bound on time before two shapes penetrate. Time is represented as
/// a fraction between [0,tMax]. This uses a swept separating axis and may miss some intermediate,
/// non-tunneling collision. If you change the time interval, you should call this function
/// again.
/// Note: use b2Distance to compute the contact point and normal at the time of impact.
// CCD via the local separating axis method. This seeks progression
// by computing the largest time at which separation is maintained.
func B2TimeOfImpact(output *B2TOIOutput, input *B2TOIInput) {

	timer := MakeB2Timer()

	B2_toiCalls++

	output.State = B2TOIOutput_State.E_unknown
	output.T = input.TMax

	proxyA := &input.ProxyA
	proxyB := &input.ProxyB

	sweepA := input.SweepA
	sweepB := input.SweepB

	// Large rotations can make the root finder fail, so we normalize the
	// sweep angles.
	sweepA.Normalize()
	sweepB.Normalize()

	tMax := input.TMax

	totalRadius := proxyA.M_radius + proxyB.M_radius
	target := math.Max(B2_linearSlop, totalRadius-3.0*B2_linearSlop)
	tolerance := 0.25 * B2_linearSlop
	B2Assert(target > tolerance)

	t1 := 0.0
	k_maxIterations := 20 // TODO_ERIN b2Settings
	iter := 0

	// Prepare input for distance query.
	cache := MakeB2SimplexCache()
	cache.Count = 0
	distanceInput := MakeB2DistanceInput()
	distanceInput.ProxyA = input.ProxyA
	distanceInput.ProxyB = input.ProxyB
	distanceInput.UseRadii = false

	// The outer loop progressively attempts to compute new separating axes.
	// This loop terminates when an axis is repeated (no progress is made).
	for {

		xfA := MakeB2Transform()
		xfB := MakeB2Transform()

		sweepA.GetTransform(&xfA, t1)
		sweepB.GetTransform(&xfB, t1)

		// Get the distance between shapes. We can also use the results
		// to get a separating axis.
		distanceInput.TransformA = xfA
		distanceInput.TransformB = xfB
		distanceOutput := MakeB2DistanceOutput()
		B2Distance(&distanceOutput, &cache, &distanceInput)

		// If the shapes are overlapped, we give up on continuous collision.
		if distanceOutput.Distance <= 0.0 {
			// Failure!
			output.State = B2TOIOutput_State.E_overlapped
			output.T = 0.0
			break
		}

		if distanceOutput.Distance < target+tolerance {
			// Victory!
			output.State = B2TOIOutput_State.E_touching
			output.T = t1
			break
		}

		// Initialize the separating axis.
		var fcn B2SeparationFunction
		fcn.Initialize(&cache, proxyA, sweepA, proxyB, sweepB, t1)

		// Compute the TOI on the separating axis. We do this by successively
		// resolving the deepest point. This loop is bounded by the number of vertices.
		done := false
		t2 := tMax
		pushBackIter := 0
		for {
			// Find the deepest point at t2. Store the witness point indices.
			var indexA, indexB int
			s2 := fcn.FindMinSeparation(&indexA, &indexB, t2)

			// Is the final configuration separated?
			if s2 > target+tolerance {
				// Victory!
				output.State = B2TOIOutput_State.E_separated
				output.T = tMax
				done = true
				break
			}

			// Has the separation reached tolerance?
			if s2 > target-tolerance {
				// Advance the sweeps
				t1 = t2
				break
			}

			// Compute the initial separation of the witness points.
			s1 := fcn.Evaluate(indexA, indexB, t1)

			// Check for initial overlap. This might happen if the root finder
			// runs out of iterations.
			if s1 < target-tolerance {
				output.State = B2TOIOutput_State.E_failed
				output.T = t1
				done = true
				break
			}

			// Check for touching
			if s1 <= target+tolerance {
				// Victory! t1 should hold the TOI (could be 0.0).
				output.State = B2TOIOutput_State.E_touching
				output.T = t1
				done = true
				break
			}

			// Compute 1D root of: f(x) - target = 0
			rootIterCount := 0
			a1 := t1
			a2 := t2

			for {
				// Use a mix of the secant rule and bisection.
				t := 0.0

				if (rootIterCount & 1) != 0x0000 {
					// Secant rule to improve convergence.
					t = a1 + (target-s1)*(a2-a1)/(s2-s1)
				} else {
					// Bisection to guarantee progress.
					t = 0.5 * (a1 + a2)
				}

				rootIterCount++
				B2_toiRootIters++

				s := fcn.Evaluate(indexA, indexB, t)

				if math.Abs(s-target) < tolerance {
					// t2 holds a tentative value for t1
					t2 = t
					break
				}

				// Ensure we continue to bracket the root.
				if s > target {
					a1 = t
					s1 = s
				} else {
					a2 = t
					s2 = s
				}

				if rootIterCount == 50 {
					break
				}
			}

			B2_toiMaxRootIters = MaxInt(B2_toiMaxRootIters, rootIterCount)

			pushBackIter++

			if pushBackIter == B2_maxPolygonVertices {
				break
			}
		}

		iter++
		B2_toiIters++

		if done {
			break
		}

		if iter == k_maxIterations {
			// Root finder got stuck. Semi-victory.
			output.State = B2TOIOutput_State.E_failed
			output.T = t1
			break
		}
	}

	B2_toiMaxIters = MaxInt(B2_toiMaxIters, iter)

	time := timer.GetMilliseconds()
	B2_toiMaxTime = math.Max(B2_toiMaxTime, time)
	B2_toiTime += time
}
