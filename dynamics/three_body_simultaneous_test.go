package dynamics_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/dynamics"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestThreeBodySimultaneousCornerImpact(t *testing.T) {
	doc := decad.New()
	floor := makeBox(t, doc, -20, -20, 20, 20, -10, 10)
	box := makeBox(t, doc, 0, 0, 10, 10, 0, 10)
	wall := makeBox(t, doc, 10, 0, 20, 10, 0, 20)
	originalBodies := doc.Bodies()
	density := units.KilogramsPerCubicMillimeter(0.001)
	material := dynamics.Material{Restitution: units.Scalar(0.5), Friction: units.Scalar(0)}
	step := dynamics.StepConfig{
		Contact: decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
			NormalResolution: units.Radians(1e-6)},
		TimeResolution: units.Seconds(1e-9), ContactSlop: units.Millimeters(1e-6),
		VelocityResidual:        units.MillimetersPerSecond(1e-6),
		AngularVelocityResidual: units.RadiansPerSecond(1e-6),
		ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
		PenetrationResidual:     units.Millimeters(1e-6), ImpactSpeed: units.MillimetersPerSecond(0),
		MaxPoseEvaluations: 128, MaxIterations: 8, MaxEvents: 4, MaxPairSweeps: 4096,
	}
	w, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{
			{Body: floor, Role: dynamics.Fixed, Material: material},
			{Body: box, Role: dynamics.Dynamic, Density: &density, Material: material},
			{Body: wall, Role: dynamics.Fixed, Material: material},
		},
		Excluded: []dynamics.BodyPair{{A: floor, B: wall}},
		Step:     step,
	})
	require.NoError(t, err)
	start, err := w.NewState([]dynamics.BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: box, Pose: r3.Identity(), LinearVelocity: dynamics.QuantityVec{
			X: units.MillimetersPerSecond(100), Y: units.MillimetersPerSecond(0),
			Z: units.MillimetersPerSecond(-100)}, AngularVelocity: zeroAngular(t)},
		{Body: wall, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	report, err := w.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(0.1))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Len(t, report.Events, 2)
	require.Equal(t, dynamics.BodyPair{A: floor, B: box}, report.Events[0].Pair)
	require.Equal(t, dynamics.BodyPair{A: box, B: wall}, report.Events[1].Pair)
	// Both initial touches take restitution 0.5 like any contact
	// (docs/multibody-dynamics-design.md §6.5): each face delivers
	// 1.5·100 kg·mm/s and the box leaves the corner at (−50, 0, 50) mm/s.
	for _, event := range report.Events {
		require.InDelta(t, 150, event.NormalImpulse.Base(), 1e-6)
		require.NotEmpty(t, event.Manifold.Points)
		require.NotNil(t, event.Solver)
	}
	endBox, ok := report.Next.Body(box)
	require.True(t, ok)
	require.Equal(t, r3.Vec{X: -5, Z: 5}, endBox.Pose.Translation())
	require.Equal(t, dynamics.QuantityVec{X: units.MillimetersPerSecond(-50), Y: units.MillimetersPerSecond(0),
		Z: units.MillimetersPerSecond(50)}, endBox.LinearVelocity)
	require.NotNil(t, report.Conservation)
	require.InDelta(t, -150, report.Conservation.ContactImpulse.Value.X.Base(), 1e-6)
	require.InDelta(t, 150, report.Conservation.ContactImpulse.Value.Z.Base(), 1e-6)
	replayed, err := report.Trace.Sample(units.Seconds(0.1))
	require.NoError(t, err)
	require.Equal(t, report.Next.Entries(), replayed.Entries())

	gravity := zeroAcceleration()
	gravity.X = units.MillimetersPerSecondSquared(1000)
	gravity.Z = units.MillimetersPerSecondSquared(-1000)
	report, err = w.Step(t.Context(), *report.Next, dynamics.StepInput{Gravity: gravity}, units.Seconds(0.1))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	// The kick turns the box back into the corner, which it reaches exactly
	// at the step end and leaves again at half its (50, 0, −50) mm/s.
	require.Len(t, report.Events, 2)
	for _, event := range report.Events {
		require.InDelta(t, .1, event.Time.Base(), 1e-9)
		require.InDelta(t, 75, event.NormalImpulse.Base(), 1e-6)
	}
	endBox, ok = report.Next.Body(box)
	require.True(t, ok)
	require.Equal(t, r3.Identity(), endBox.Pose)
	// Fused multiply-adds (arm64) move the solve's last ulps, so the
	// velocity compares within 1e-12 of the exact (−25, 0, 25) mm/s.
	require.InDelta(t, -25, endBox.LinearVelocity.X.Base(), 1e-12)
	require.InDelta(t, 0, endBox.LinearVelocity.Y.Base(), 1e-12)
	require.InDelta(t, 25, endBox.LinearVelocity.Z.Base(), 1e-12)

	report, err = w.Step(t.Context(), *report.Next,
		dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(0.1))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Empty(t, report.Events)

	reversed, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{
			{Body: wall, Role: dynamics.Fixed, Material: material},
			{Body: floor, Role: dynamics.Fixed, Material: material},
			{Body: box, Role: dynamics.Dynamic, Density: &density, Material: material},
		},
		Excluded: []dynamics.BodyPair{{A: floor, B: wall}}, Step: step,
	})
	require.NoError(t, err)
	reversedStart, err := reversed.NewState(start.Entries())
	require.NoError(t, err)
	reversedReport, err := reversed.Step(t.Context(), reversedStart,
		dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(0.1))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, reversedReport.Status, "%+v", reversedReport.Diagnostics)
	require.Len(t, reversedReport.Events, 2)
	require.Equal(t, dynamics.BodyPair{A: wall, B: box}, reversedReport.Events[0].Pair)
	require.Equal(t, dynamics.BodyPair{A: floor, B: box}, reversedReport.Events[1].Pair)
	reversedBox, ok := reversedReport.Next.Body(box)
	require.True(t, ok)
	require.Equal(t, dynamics.QuantityVec{X: units.MillimetersPerSecond(-50), Y: units.MillimetersPerSecond(0),
		Z: units.MillimetersPerSecond(50)}, reversedBox.LinearVelocity)
	require.Equal(t, originalBodies, doc.Bodies())

	step.MaxEvents = 2
	limited, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{
			{Body: floor, Role: dynamics.Fixed, Material: material},
			{Body: box, Role: dynamics.Dynamic, Density: &density, Material: material},
			{Body: wall, Role: dynamics.Fixed, Material: material},
		},
		Excluded: []dynamics.BodyPair{{A: floor, B: wall}}, Step: step,
	})
	require.NoError(t, err)
	limitedStart, err := limited.NewState(start.Entries())
	require.NoError(t, err)
	limitedReport, err := limited.Step(t.Context(), limitedStart,
		dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(0.1))
	require.NoError(t, err)
	require.Equal(t, dynamics.Undecided, limitedReport.Status)
	require.Nil(t, limitedReport.Next)
}
