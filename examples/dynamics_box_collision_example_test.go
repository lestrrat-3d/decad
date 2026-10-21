package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/dynamics"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// The dynamics package simulates rigid bodies that fall, collide and bounce.
// Use it to see how parts behave under motion, for example whether a dropped
// part bounces off a surface. Here a box moving down at 100 mm/s hits a fixed
// floor. Step runs the simulation forward and returns the impact and the
// velocity after it.
func Example_dynamics_box_collision() {
	ctx := context.Background()
	doc := decad.New()
	// Two bodies: a fixed floor, and a 10 mm box above it. The box starts
	// with its bottom 10 mm above the floor's top face.
	floor, err := extrudeBlock(doc, -20, -20, 20, 20, -10, 10)
	if err != nil {
		fmt.Printf("failed to build floor: %s\n", err)
		return
	}
	mover, err := extrudeBlock(doc, -5, -5, 5, 5, 0, 10)
	if err != nil {
		fmt.Printf("failed to build moving box: %s\n", err)
		return
	}
	startPose, err := r3.Translation(r3.Vec{Z: 10})
	if err != nil {
		fmt.Printf("failed to place moving box: %s\n", err)
		return
	}
	// First confirm the geometry is valid: the boxes must start apart.
	// ContactPair asks how two bodies, each in a given pose, relate.
	contactRequest := decad.ContactRequest{
		PointResolution: units.Millimeters(1e-6), NormalResolution: units.Radians(1e-6),
	}
	initial, err := doc.ContactPair(ctx, floor, mover, r3.Identity(), startPose, contactRequest)
	if err != nil {
		fmt.Printf("failed to check initial contact: %s\n", err)
		return
	}
	if initial.Relation != decad.ContactSeparated {
		fmt.Println("the boxes do not start separated")
		return
	}

	// Describe the world. The floor is Fixed and never moves. The box is
	// Dynamic, so it needs a density to get a mass. Restitution 0.5 means it
	// rebounds at half its impact speed, and Friction 0 keeps this a pure
	// vertical bounce. The step settings below are tolerances, which are fine
	// for most uses.
	density := units.KilogramsPerCubicMillimeter(0.001)
	material := dynamics.Material{Restitution: units.Scalar(0.5), Friction: units.Scalar(0)}
	simulation, err := dynamics.NewWorld(ctx, doc, dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{
			{Body: floor, Role: dynamics.Fixed, Material: material},
			{Body: mover, Role: dynamics.Dynamic, Density: &density, Material: material},
		},
		Step: dynamics.StepConfig{
			Contact: contactRequest, TimeResolution: units.Seconds(2e-9),
			ContactSlop:             units.Millimeters(1e-6),
			VelocityResidual:        units.MillimetersPerSecond(1e-6),
			AngularVelocityResidual: units.RadiansPerSecond(1e-6),
			ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
			PenetrationResidual:     units.Millimeters(1e-6),
			ImpactSpeed:             units.MillimetersPerSecond(0),
			MaxPoseEvaluations:      128, MaxIterations: 8, MaxEvents: 2, MaxPairSweeps: 4096,
		},
	})
	if err != nil {
		fmt.Printf("failed to create simulation: %s\n", err)
		return
	}
	// The starting state: the floor is still, and the box moves down
	// (negative Z) at 100 mm/s.
	zeroVelocity := dynamics.QuantityVec{
		X: units.MillimetersPerSecond(0), Y: units.MillimetersPerSecond(0), Z: units.MillimetersPerSecond(0),
	}
	zeroSpin := dynamics.QuantityVec{
		X: units.RadiansPerSecond(0), Y: units.RadiansPerSecond(0), Z: units.RadiansPerSecond(0),
	}
	state, err := simulation.NewState([]dynamics.BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity, AngularVelocity: zeroSpin},
		{Body: mover, Pose: startPose, LinearVelocity: dynamics.QuantityVec{
			X: zeroVelocity.X, Y: zeroVelocity.Y, Z: units.MillimetersPerSecond(-100),
		}, AngularVelocity: zeroSpin},
	})
	if err != nil {
		fmt.Printf("failed to create state: %s\n", err)
		return
	}
	// Turn gravity off so the only motion is the box's initial velocity, then
	// run for up to 0.2 s. The box crosses the 10 mm gap in 0.1 s and hits.
	zeroGravity := dynamics.QuantityVec{
		X: units.MillimetersPerSecondSquared(0), Y: units.MillimetersPerSecondSquared(0),
		Z: units.MillimetersPerSecondSquared(0),
	}
	report, err := simulation.Step(ctx, state, dynamics.StepInput{Gravity: zeroGravity}, units.Seconds(0.2))
	if err != nil {
		fmt.Printf("failed to step: %s\n", err)
		return
	}
	if report.Status != dynamics.Advanced || len(report.Events) != 1 || report.Events[0].Kind != dynamics.ContactImpact {
		fmt.Println("the impact was not resolved")
		return
	}
	final, ok := report.Next.Body(mover)
	if !ok {
		fmt.Println("the moving box has no final state")
		return
	}
	fmt.Printf("impact at %.2f s; rebound at %.0f mm/s\n",
		report.Events[0].Time.Base(), final.LinearVelocity.Z.Base())
	// Output:
	// impact at 0.10 s; rebound at 50 mm/s
}
