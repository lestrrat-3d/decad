package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/dynamics"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// A box dropped onto a floor bounces through three steps of a Timeline. The
// world holds four bodies, so every step takes the scheduled step: each
// bounce is an event inside the step, solved and certified where it happens.
// The timeline replays any time up to its certified end.
func Example_dynamics_timeline() {
	ctx := context.Background()
	doc := decad.New()
	box := func(x0, y0, x1, y1, z0, height float64) (*decad.Body, error) {
		world := sketch.NewWorld()
		plane, err := world.CreateOffsetPlane(world.XY(), z0)
		if err != nil {
			return nil, err
		}
		s, err := world.CreateSketch(plane)
		if err != nil {
			return nil, err
		}
		rect := s.CreateRectangle(x0, y0, x1, y1)
		s.Fix(rect.A)
		if _, err := s.Solve(ctx); err != nil {
			return nil, err
		}
		//nolint:contextcheck // Extrude has no context parameter.
		return doc.Extrude(s, s.Profiles()[0], decad.Distance{
			D: units.Millimeters(height), Dir: decad.Along,
		})
	}
	// A floor with its top at z = 0, a 10 mm box above it, and two parked
	// fixed boxes far away.
	corners := [4][6]float64{{-20, -20, 20, 20, -10, 10}, {-5, -5, 5, 5, 0, 10},
		{500, 0, 510, 10, 0, 10}, {600, 0, 610, 10, 0, 10}}
	var bodies [4]*decad.Body
	for i, c := range corners {
		body, err := box(c[0], c[1], c[2], c[3], c[4], c[5])
		if err != nil {
			fmt.Printf("failed to build body %d: %s\n", i, err)
			return
		}
		bodies[i] = body
	}
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
		// An incoming speed of 16 mm/s or less no longer bounces.
		ImpactSpeed:        units.MillimetersPerSecond(16),
		MaxPoseEvaluations: 128, MaxIterations: 64, MaxEvents: 8,
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
	timeline, err := dynamics.NewTimeline(world, start)
	if err != nil {
		fmt.Printf("failed to create timeline: %s\n", err)
		return
	}
	// Gravity of -8192 mm/s² kicks the box by exactly -256 mm/s per 1/32 s step.
	input := dynamics.StepInput{Gravity: dynamics.QuantityVec{X: units.MillimetersPerSecondSquared(0),
		Y: units.MillimetersPerSecondSquared(0), Z: units.MillimetersPerSecondSquared(-8192)}}
	dt := units.Seconds(1.0 / 32)
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
