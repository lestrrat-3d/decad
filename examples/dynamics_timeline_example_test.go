package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/dynamics"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// A Timeline records a simulation as it advances, so you can replay any moment
// afterwards. Advance runs the next step and keeps the result; Sample returns
// the state at any earlier time. Use it to step through a motion and then
// inspect it. Here a box is dropped on a floor and bounces three times.
func Example_dynamics_timeline() {
	ctx := context.Background()
	doc := decad.New()
	// Two bodies: a fixed floor with its top at z = 0, and a 10 mm box that
	// starts 4 mm above it.
	corners := [2][6]float64{{-20, -20, 20, 20, -10, 10}, {-5, -5, 5, 5, 0, 10}}
	var bodies [2]*decad.Body
	for i, c := range corners {
		body, err := extrudeBlock(doc, c[0], c[1], c[2], c[3], c[4], c[5])
		if err != nil {
			fmt.Printf("failed to build body %d: %s\n", i, err)
			return
		}
		bodies[i] = body
	}
	// Describe the world as in Example_dynamics_box_collision: the floor is
	// Fixed, the box is Dynamic, and the restitution of 0.5 makes it rebound at
	// half its impact speed.
	density := units.KilogramsPerCubicMillimeter(0.001)
	material := dynamics.Material{Restitution: units.Scalar(0.5), Friction: units.Scalar(0)}
	config := dynamics.WorldConfig{Step: dynamics.StepConfig{
		Contact: decad.ContactRequest{
			PointResolution: units.Millimeters(1e-6), NormalResolution: units.Radians(1e-6),
		},
		TimeResolution: units.Seconds(1e-9), ContactSlop: units.Millimeters(1e-6),
		VelocityResidual:        units.MillimetersPerSecond(1e-6),
		AngularVelocityResidual: units.RadiansPerSecond(1e-6),
		ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
		PenetrationResidual:     units.Millimeters(1e-6),
		// An impact slower than 16 mm/s no longer bounces. This is what lets the
		// box settle instead of bouncing forever.
		ImpactSpeed:        units.MillimetersPerSecond(16),
		MaxPoseEvaluations: 128, MaxIterations: 64, MaxEvents: 8, MaxPairSweeps: 4096,
	}}
	for i, body := range bodies {
		entry := dynamics.RigidBody{Body: body, Role: dynamics.Fixed, Material: material}
		if i == 1 {
			entry.Role, entry.Density = dynamics.Dynamic, &density
		}
		config.Bodies = append(config.Bodies, entry)
	}
	world, err := dynamics.NewWorld(ctx, doc, config)
	if err != nil {
		fmt.Printf("failed to create world: %s\n", err)
		return
	}
	// Everything starts at rest, with the box raised 4 mm above the floor.
	still := dynamics.QuantityVec{
		X: units.MillimetersPerSecond(0), Y: units.MillimetersPerSecond(0), Z: units.MillimetersPerSecond(0),
	}
	noSpin := dynamics.QuantityVec{
		X: units.RadiansPerSecond(0), Y: units.RadiansPerSecond(0), Z: units.RadiansPerSecond(0),
	}
	raised, err := r3.Translation(r3.Vec{Z: 4})
	if err != nil {
		fmt.Printf("failed to place the box: %s\n", err)
		return
	}
	var entries []dynamics.BodyState
	for i, body := range bodies {
		pose := r3.Identity()
		if i == 1 {
			pose = raised
		}
		entries = append(entries, dynamics.BodyState{Body: body, Pose: pose, LinearVelocity: still,
			AngularVelocity: noSpin})
	}
	start, err := world.NewState(entries)
	if err != nil {
		fmt.Printf("failed to create state: %s\n", err)
		return
	}
	// A Timeline starts from this state and keeps every step it advances.
	timeline, err := dynamics.NewTimeline(world, start)
	if err != nil {
		fmt.Printf("failed to create timeline: %s\n", err)
		return
	}
	// Gravity of -8192 mm/s² kicks the box by exactly -256 mm/s per 1/32 s step.
	input := dynamics.StepInput{Gravity: dynamics.QuantityVec{X: units.MillimetersPerSecondSquared(0),
		Y: units.MillimetersPerSecondSquared(0), Z: units.MillimetersPerSecondSquared(-8192)}}
	dt := units.Seconds(1.0 / 32)
	// Advance three steps of 1/32 s each. A step can contain impacts; each one
	// is solved at the moment it happens. Print when each bounce happened and
	// how fast the box left the floor.
	for step := range 3 {
		begin := timeline.End()
		report, err := timeline.Advance(ctx, input, dt)
		if err != nil {
			fmt.Printf("failed to advance: %s\n", err)
			return
		}
		if report.Status != dynamics.Advanced || len(report.Events) != 1 {
			fmt.Printf("step %d did not bounce once\n", step)
			return
		}
		event := report.Events[0]
		fmt.Printf("bounce at %.6f s, leaving at %.0f mm/s\n",
			begin.Base()+event.Time.Base(), event.PostVelocityB.Z.Base())
	}
	// Sample replays the state at any earlier time without running the
	// simulation again. Here: the box's height after one and after three steps.
	for _, at := range []float64{1.0 / 32, 3.0 / 32} {
		state, err := timeline.Sample(units.Seconds(at))
		if err != nil {
			fmt.Printf("failed to sample: %s\n", err)
			return
		}
		entry, _ := state.Body(bodies[1])
		fmt.Printf("height at %.6f s: %.3f mm\n", at, entry.Pose.Translation().Z)
	}
	// Output:
	// bounce at 0.015625 s, leaving at 128 mm/s
	// bounce at 0.046875 s, leaving at 64 mm/s
	// bounce at 0.067708 s, leaving at 96 mm/s
	// height at 0.031250 s: 2.000 mm
	// height at 0.093750 s: 2.500 mm
}
