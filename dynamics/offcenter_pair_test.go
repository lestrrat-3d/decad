package dynamics_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/dynamics"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestOffcenterDynamicPairReboundsWithCertifiedSpin(t *testing.T) {
	doc := decad.New()
	a := makeBox(t, doc, -5, -5, 5, 5, -10, 10)
	b := makeBox(t, doc, 0, -5, 10, 5, 0, 10)
	request := decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
		NormalResolution: units.Radians(1e-6)}
	contact, err := doc.ContactPair(t.Context(), a, b, r3.Identity(), r3.Identity(), request)
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, contact.Relation)
	require.Len(t, contact.Manifold.Points, 4)
	for _, point := range contact.Manifold.Points {
		require.Equal(t, r3.Vec{Z: 1}, point.Normal.Value)
		require.Equal(t, point.OnA.Value, point.OnB.Value)
	}

	density := units.KilogramsPerCubicMillimeter(.001)
	material := dynamics.Material{Restitution: units.Scalar(.5), Friction: units.Scalar(0)}
	config := dynamics.StepConfig{Contact: request, TimeResolution: units.Seconds(1e-9),
		ContactSlop: units.Millimeters(1e-6), VelocityResidual: units.MillimetersPerSecond(1e-6),
		AngularVelocityResidual: units.RadiansPerSecond(1e-6),
		ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
		PenetrationResidual:     units.Millimeters(1e-6), ImpactSpeed: units.MillimetersPerSecond(0),
		MaxPoseEvaluations: 128, MaxIterations: 64, MaxEvents: 2, MaxPairSweeps: 4096}
	world, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{{Body: a, Role: dynamics.Dynamic, Density: &density, Material: material},
			{Body: b, Role: dynamics.Dynamic, Density: &density, Material: material}}, Step: config})
	require.NoError(t, err)
	initial := []dynamics.BodyState{
		{Body: a, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: b, Pose: r3.Identity(), LinearVelocity: dynamics.QuantityVec{
			X: units.MillimetersPerSecond(0), Y: units.MillimetersPerSecond(0),
			Z: units.MillimetersPerSecond(-100)}, AngularVelocity: zeroAngular(t)},
	}
	start, err := world.NewState(initial)
	require.NoError(t, err)
	report, err := world.Step(t.Context(), start,
		dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(.1))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.NotNil(t, report.Next)
	require.Len(t, report.Events, 1)
	require.NotNil(t, report.Conservation)
	event := report.Events[0]
	require.Equal(t, dynamics.ContactImpact, event.Kind)
	require.Len(t, event.PointImpulses, 4)
	require.InDelta(t, 600.0/11, event.NormalImpulse.Base(), 1e-9)
	require.InDelta(t, 90.0/11, event.PostAngularVelocityA.Y.Base(), 1e-9)
	require.InDelta(t, event.PostAngularVelocityA.Y.Base(), event.PostAngularVelocityB.Y.Base(), 1e-12)
	require.Greater(t, event.PostAngularVelocityA.Y.Base(), 0.0)
	require.Zero(t, event.TangentImpulse.X.Base())
	require.Zero(t, event.TangentImpulse.Y.Base())
	for _, impulse := range event.PointImpulses {
		require.InDelta(t, 150.0/11, impulse.Normal.Base(), 1e-9)
		require.Zero(t, impulse.Tangent.X.Base())
		require.Zero(t, impulse.Tangent.Y.Base())
	}
	require.LessOrEqual(t, event.Solver.NormalResidual.Base(), config.VelocityResidual.Base())
	require.GreaterOrEqual(t, event.Solver.AngularUpper.Base(), event.PostAngularVelocityA.Y.Base())
	require.InDelta(t, -100, report.Conservation.Input.LinearMomentum.Value.Z.Base(), 1e-9)
	require.InDelta(t, -100, report.Conservation.Completion.LinearMomentum.Value.Z.Base(), 1e-6)
	require.InDelta(t, 500, report.Conservation.Input.AngularMomentum.Value.Y.Base(), 1e-6)
	require.InDelta(t, 500, report.Conservation.Completion.AngularMomentum.Value.Y.Base(), 1e-5)
	require.Less(t, report.Conservation.Completion.KineticEnergy.Value.Base(),
		report.Conservation.Input.KineticEnergy.Value.Base())
	toDrift := func(body *decad.Body, linear, angular dynamics.QuantityVec) decad.RigidDriftSegment {
		mass, massErr := body.MassProperties(t.Context(), density)
		require.NoError(t, massErr)
		return decad.RigidDriftSegment{From: r3.Identity(), Center: mass.Center.Value,
			LinearVelocity: linear, AngularVelocity: angular, Duration: units.Seconds(.1)}
	}
	departure, err := doc.SweepPair(t.Context(), a, b,
		toDrift(a, event.PostVelocityA, event.PostAngularVelocityA),
		toDrift(b, event.PostVelocityB, event.PostAngularVelocityB), decad.SweepRequest{
			ContactRequest: request, StartPolicy: decad.ContinueSeparatingTouch,
			TimeResolution: units.Seconds(1e-9), MaxPoseEvaluations: 128})
	require.NoError(t, err)
	require.Equal(t, decad.SweepDepartedClear, departure.Outcome)
	require.NotNil(t, departure.Departure)
	for _, body := range []*decad.Body{a, b} {
		entry, ok := report.Next.Body(body)
		require.True(t, ok)
		require.InDelta(t, 90.0/11, entry.AngularVelocity.Y.Base(), 1e-9)
		require.Greater(t, math.Abs(entry.Pose.ApplyDir(r3.Vec{X: 1}).Z), .1)
	}
	endA, _ := report.Next.Body(a)
	endB, _ := report.Next.Body(b)
	final, err := doc.ContactPair(t.Context(), a, b, endA.Pose, endB.Pose, request)
	require.NoError(t, err)
	require.Equal(t, decad.ContactSeparated, final.Relation)
	require.Greater(t, final.Gap.Value.Base()-final.Gap.Bound.Base(), 0.0)
	replayed, err := report.Trace.Sample(units.Seconds(.1))
	require.NoError(t, err)
	replayedA, _ := replayed.Body(a)
	replayedB, _ := replayed.Body(b)
	require.Equal(t, endA, replayedA)
	require.Equal(t, endB, replayedB)

	fixedWorld, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{{Body: a, Role: dynamics.Fixed, Material: material},
			{Body: b, Role: dynamics.Dynamic, Density: &density, Material: material}}, Step: config})
	require.NoError(t, err)
	fixedStart, err := fixedWorld.NewState(initial)
	require.NoError(t, err)
	refused, err := fixedWorld.Step(t.Context(), fixedStart,
		dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(.1))
	require.NoError(t, err)
	require.Equal(t, dynamics.Undecided, refused.Status)
	require.Nil(t, refused.Next)
	reversed, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{{Body: b, Role: dynamics.Dynamic, Density: &density, Material: material},
			{Body: a, Role: dynamics.Dynamic, Density: &density, Material: material}}, Step: config})
	require.NoError(t, err)
	reverseStart, err := reversed.NewState(initial)
	require.NoError(t, err)
	reverseReport, err := reversed.Step(t.Context(), reverseStart,
		dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(.1))
	require.NoError(t, err)
	require.Equal(t, dynamics.Undecided, reverseReport.Status)
	require.Nil(t, reverseReport.Next)
}
