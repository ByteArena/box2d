package box2d_test

import (
	"fmt"
	"math"
	"sort"
	"testing"

	"github.com/ByteArena/box2d"
	"github.com/pmezard/go-difflib/difflib"
)

func TestCPPCompliance(t *testing.T) {

	// Define the gravity vector.
	gravity := box2d.MakeB2Vec2(0.0, -10.0)

	// Construct a world object, which will hold and simulate the rigid bodies.
	world := box2d.MakeB2World(gravity)

	characters := make(map[string]*box2d.B2Body)

	// Ground body
	{
		bd := box2d.MakeB2BodyDef()
		ground := world.CreateBody(&bd)

		shape := box2d.MakeB2EdgeShape()
		shape.Set(box2d.MakeB2Vec2(-20.0, 0.0), box2d.MakeB2Vec2(20.0, 0.0))
		ground.CreateFixture(&shape, 0.0)
		characters["00_ground"] = ground
	}

	// Collinear edges with no adjacency information.
	// This shows the problematic case where a box shape can hit
	// an internal vertex.
	{
		bd := box2d.MakeB2BodyDef()
		ground := world.CreateBody(&bd)

		shape := box2d.MakeB2EdgeShape()
		shape.Set(box2d.MakeB2Vec2(-8.0, 1.0), box2d.MakeB2Vec2(-6.0, 1.0))
		ground.CreateFixture(&shape, 0.0)
		shape.Set(box2d.MakeB2Vec2(-6.0, 1.0), box2d.MakeB2Vec2(-4.0, 1.0))
		ground.CreateFixture(&shape, 0.0)
		shape.Set(box2d.MakeB2Vec2(-4.0, 1.0), box2d.MakeB2Vec2(-2.0, 1.0))
		ground.CreateFixture(&shape, 0.0)
		characters["01_colinearground"] = ground
	}

	// Chain shape
	{
		bd := box2d.MakeB2BodyDef()
		bd.Angle = 0.25 * box2d.B2_pi
		ground := world.CreateBody(&bd)

		vs := make([]box2d.B2Vec2, 4)
		vs[0].Set(5.0, 7.0)
		vs[1].Set(6.0, 8.0)
		vs[2].Set(7.0, 8.0)
		vs[3].Set(8.0, 7.0)
		shape := box2d.MakeB2ChainShape()
		shape.CreateChain(vs, 4)
		ground.CreateFixture(&shape, 0.0)
		characters["02_chainshape"] = ground
	}

	// Square tiles. This shows that adjacency shapes may
	// have non-smooth  There is no solution
	// to this problem.
	{
		bd := box2d.MakeB2BodyDef()
		ground := world.CreateBody(&bd)

		shape := box2d.MakeB2PolygonShape()
		shape.SetAsBoxFromCenterAndAngle(1.0, 1.0, box2d.MakeB2Vec2(4.0, 3.0), 0.0)
		ground.CreateFixture(&shape, 0.0)
		shape.SetAsBoxFromCenterAndAngle(1.0, 1.0, box2d.MakeB2Vec2(6.0, 3.0), 0.0)
		ground.CreateFixture(&shape, 0.0)
		shape.SetAsBoxFromCenterAndAngle(1.0, 1.0, box2d.MakeB2Vec2(8.0, 3.0), 0.0)
		ground.CreateFixture(&shape, 0.0)
		characters["03_squaretiles"] = ground
	}

	// Square made from an edge loop. Collision should be smooth.
	{
		bd := box2d.MakeB2BodyDef()
		ground := world.CreateBody(&bd)

		vs := make([]box2d.B2Vec2, 4)
		vs[0].Set(-1.0, 3.0)
		vs[1].Set(1.0, 3.0)
		vs[2].Set(1.0, 5.0)
		vs[3].Set(-1.0, 5.0)
		shape := box2d.MakeB2ChainShape()
		shape.CreateLoop(vs, 4)
		ground.CreateFixture(&shape, 0.0)
		characters["04_edgeloopsquare"] = ground
	}

	// Edge loop. Collision should be smooth.
	{
		bd := box2d.MakeB2BodyDef()
		bd.Position.Set(-10.0, 4.0)
		ground := world.CreateBody(&bd)

		vs := make([]box2d.B2Vec2, 10)
		vs[0].Set(0.0, 0.0)
		vs[1].Set(6.0, 0.0)
		vs[2].Set(6.0, 2.0)
		vs[3].Set(4.0, 1.0)
		vs[4].Set(2.0, 2.0)
		vs[5].Set(0.0, 2.0)
		vs[6].Set(-2.0, 2.0)
		vs[7].Set(-4.0, 3.0)
		vs[8].Set(-6.0, 2.0)
		vs[9].Set(-6.0, 0.0)
		shape := box2d.MakeB2ChainShape()
		shape.CreateLoop(vs, 10)
		ground.CreateFixture(&shape, 0.0)
		characters["05_edgelooppoly"] = ground
	}

	// Square character 1
	{
		bd := box2d.MakeB2BodyDef()
		bd.Position.Set(-3.0, 8.0)
		bd.Type = box2d.B2BodyType.B2_dynamicBody
		bd.FixedRotation = true
		bd.AllowSleep = false

		body := world.CreateBody(&bd)

		shape := box2d.MakeB2PolygonShape()
		shape.SetAsBox(0.5, 0.5)

		fd := box2d.MakeB2FixtureDef()
		fd.Shape = &shape
		fd.Density = 20.0
		body.CreateFixtureFromDef(&fd)
		characters["06_squarecharacter1"] = body
	}

	// Square character 2
	{
		bd := box2d.MakeB2BodyDef()
		bd.Position.Set(-5.0, 5.0)
		bd.Type = box2d.B2BodyType.B2_dynamicBody
		bd.FixedRotation = true
		bd.AllowSleep = false

		body := world.CreateBody(&bd)

		shape := box2d.MakeB2PolygonShape()
		shape.SetAsBox(0.25, 0.25)

		fd := box2d.MakeB2FixtureDef()
		fd.Shape = &shape
		fd.Density = 20.0
		body.CreateFixtureFromDef(&fd)
		characters["07_squarecharacter2"] = body
	}

	// Hexagon character
	{
		bd := box2d.MakeB2BodyDef()
		bd.Position.Set(-5.0, 8.0)
		bd.Type = box2d.B2BodyType.B2_dynamicBody
		bd.FixedRotation = true
		bd.AllowSleep = false

		body := world.CreateBody(&bd)

		angle := 0.0
		delta := box2d.B2_pi / 3.0
		vertices := make([]box2d.B2Vec2, 6)
		for i := 0; i < 6; i++ {
			vertices[i].Set(0.5*math.Cos(angle), 0.5*math.Sin(angle))
			angle += delta
		}

		shape := box2d.MakeB2PolygonShape()
		shape.Set(vertices, 6)

		fd := box2d.MakeB2FixtureDef()
		fd.Shape = &shape
		fd.Density = 20.0
		body.CreateFixtureFromDef(&fd)
		characters["08_hexagoncharacter"] = body
	}

	// Circle character
	{
		bd := box2d.MakeB2BodyDef()
		bd.Position.Set(3.0, 5.0)
		bd.Type = box2d.B2BodyType.B2_dynamicBody
		bd.FixedRotation = true
		bd.AllowSleep = false

		body := world.CreateBody(&bd)

		shape := box2d.MakeB2CircleShape()
		shape.M_radius = 0.5

		fd := box2d.MakeB2FixtureDef()
		fd.Shape = &shape
		fd.Density = 20.0
		body.CreateFixtureFromDef(&fd)
		characters["09_circlecharacter1"] = body
	}

	// Circle character
	{
		bd := box2d.MakeB2BodyDef()
		bd.Position.Set(-7.0, 6.0)
		bd.Type = box2d.B2BodyType.B2_dynamicBody
		bd.AllowSleep = false

		body := world.CreateBody(&bd)

		shape := box2d.MakeB2CircleShape()
		shape.M_radius = 0.25

		fd := box2d.MakeB2FixtureDef()
		fd.Shape = &shape
		fd.Density = 20.0
		fd.Friction = 1.0
		body.CreateFixtureFromDef(&fd)

		characters["10_circlecharacter2"] = body
	}

	// Prepare for simulation. Typically we use a time step of 1/60 of a
	// second (60Hz) and 10 iterations. This provides a high quality simulation
	// in most game scenarios.
	timeStep := 1.0 / 60.0
	velocityIterations := 8
	positionIterations := 3

	output := ""

	characterNames := make([]string, 0)
	for k, _ := range characters {
		characterNames = append(characterNames, k)
	}

	sort.Strings(characterNames)

	// This is our little game loop.
	for i := 0; i < 60; i++ {
		// Instruct the world to perform a single step of simulation.
		// It is generally best to keep the time step and iterations fixed.
		//runtime.Breakpoint()
		world.Step(timeStep, velocityIterations, positionIterations)

		// Now print the position and angle of the body.
		for _, name := range characterNames {
			character := characters[name]
			position := character.GetPosition()
			angle := character.GetAngle()
			msg := fmt.Sprintf("%v(%s): %4.3f %4.3f %4.3f\n", i, name, position.X, position.Y, angle)
			fmt.Print(msg)
			output += msg
		}
	}

	if output != expected {

		diff := difflib.UnifiedDiff{
			A:        difflib.SplitLines(expected),
			B:        difflib.SplitLines(output),
			FromFile: "Expected",
			ToFile:   "Current",
			Context:  0,
		}
		text, _ := difflib.GetUnifiedDiffString(diff)
		t.Fatalf("NOT Matching c++ reference. Failure: \n%s", text)
	}
}
