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

// A dynamic box falls onto a fixed box. The geometry query proves their
// initial separation; Step finds the impact and computes the rebound.
func Example_dynamics_boxCollision() {
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

	floor, err := box(-20, -20, 20, 20, -10, 10)
	if err != nil {
		fmt.Printf("failed to build floor: %s\n", err)
		return
	}
	mover, err := box(-5, -5, 5, 5, 0, 10)
	if err != nil {
		fmt.Printf("failed to build moving box: %s\n", err)
		return
	}
	startPose, err := r3.Translation(r3.Vec{Z: 10})
	if err != nil {
		fmt.Printf("failed to place moving box: %s\n", err)
		return
	}
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
			MaxPoseEvaluations:      128, MaxIterations: 8, MaxEvents: 2,
		},
	})
	if err != nil {
		fmt.Printf("failed to create simulation: %s\n", err)
		return
	}
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
