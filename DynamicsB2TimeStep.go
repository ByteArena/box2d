package box2d

/// Profiling data. Times are in milliseconds.
type B2Profile struct {
	Step          float64
	Collide       float64
	Solve         float64
	SolveInit     float64
	SolveVelocity float64
	SolvePosition float64
	Broadphase    float64
	SolveTOI      float64
}

func MakeB2Profile() B2Profile {
	return B2Profile{}
}

/// This is an internal structure.
type B2TimeStep struct {
	Dt                 float64 // time step
	Inv_dt             float64 // inverse time step (0 if dt == 0).
	DtRatio            float64 // dt * inv_dt0
	VelocityIterations int
	PositionIterations int
	WarmStarting       bool
}

func MakeB2TimeStep() B2TimeStep {
	return B2TimeStep{}
}

/// This is an internal structure.
type B2Position struct {
	C B2Vec2
	A float64
}

/// This is an internal structure.
type B2Velocity struct {
	V B2Vec2
	W float64
}

/// Solver Data
type B2SolverData struct {
	Step       B2TimeStep
	Positions  []B2Position
	Velocities []B2Velocity
}

func MakeB2SolverData() B2SolverData {
	return B2SolverData{}
}
