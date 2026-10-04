package dynamics_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/dynamics"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestThreeBodyImpactWithThirdClear(t *testing.T) {
	doc := decad.New()
	floor := makeBox(t, doc, -20, -20, 20, 20, -10, 10)
	box := makeBox(t, doc, -5, -5, 5, 5, 0, 10)
	// A fictitious one-second continuation of the incoming velocity reaches
	// this body; the actual short correction and outgoing drift stay clear.
	remote := makeBox(t, doc, -5, -5, 5, 5, -50, 10)
	density := units.KilogramsPerCubicMillimeter(0.001)
	material := dynamics.Material{Restitution: units.Scalar(0.5), Friction: units.Scalar(0)}
	w, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{
			{Body: floor, Role: dynamics.Fixed, Material: material},
			{Body: box, Role: dynamics.Dynamic, Density: &density, Material: material},
			{Body: remote, Role: dynamics.Fixed, Material: material},
		},
		Step: dynamics.StepConfig{
			Contact: decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
				NormalResolution: units.Radians(1e-6)},
			TimeResolution: units.Seconds(1e-9), ContactSlop: units.Millimeters(1e-6),
			VelocityResidual:        units.MillimetersPerSecond(1e-6),
			AngularVelocityResidual: units.RadiansPerSecond(1e-6),
			ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
			PenetrationResidual:     units.Millimeters(1e-6), ImpactSpeed: units.MillimetersPerSecond(0),
			MaxPoseEvaluations: 128, MaxIterations: 8, MaxEvents: 2,
		},
	})
	require.NoError(t, err)
	startPose, err := r3.Translation(r3.Vec{Z: 10})
	require.NoError(t, err)
	start, err := w.NewState([]dynamics.BodyState{
		{Body: remote, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: box, Pose: startPose, LinearVelocity: dynamics.QuantityVec{
			X: units.MillimetersPerSecond(0), Y: units.MillimetersPerSecond(0),
			Z: units.MillimetersPerSecond(-100)}, AngularVelocity: zeroAngular(t)},
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	report, err := w.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(0.2))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Len(t, report.Events, 1)
	require.Equal(t, dynamics.BodyPair{A: floor, B: box}, report.Events[0].Pair)
	require.Len(t, report.Next.Entries(), 3)
	require.NotNil(t, report.Conservation)
	require.InDelta(t, 50, report.Conservation.Completion.LinearMomentum.Value.Z.Base(), 1e-6)
	endBox, found := report.Next.Body(box)
	require.True(t, found)
	require.InDelta(t, 5, endBox.Pose.Translation().Z, 2e-6)
	require.InDelta(t, 50, endBox.LinearVelocity.Z.Base(), 1e-6)
	endRemote, found := report.Next.Body(remote)
	require.True(t, found)
	require.Equal(t, r3.Identity(), endRemote.Pose)
	replayed, err := report.Trace.Sample(units.Seconds(0.2))
	require.NoError(t, err)
	require.Equal(t, report.Next.Entries(), replayed.Entries())
}

func TestThreeBodyRejectsSecondContact(t *testing.T) {
	doc := decad.New()
	floor := makeBox(t, doc, -20, -20, 20, 20, -10, 10)
	box := makeBox(t, doc, -5, -5, 5, 5, 0, 10)
	barrier := makeBox(t, doc, -5, -5, 5, 5, 2, 5)
	density := units.KilogramsPerCubicMillimeter(0.001)
	material := dynamics.Material{Restitution: units.Scalar(0.5), Friction: units.Scalar(0)}
	config := dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{
			{Body: floor, Role: dynamics.Fixed, Material: material},
			{Body: box, Role: dynamics.Dynamic, Density: &density, Material: material},
			{Body: barrier, Role: dynamics.Fixed, Material: material},
		},
		Step: dynamics.StepConfig{
			Contact: decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
				NormalResolution: units.Radians(1e-6)},
			TimeResolution: units.Seconds(1e-9), ContactSlop: units.Millimeters(1e-6),
			VelocityResidual:        units.MillimetersPerSecond(1e-6),
			AngularVelocityResidual: units.RadiansPerSecond(1e-6),
			ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
			PenetrationResidual:     units.Millimeters(1e-6), ImpactSpeed: units.MillimetersPerSecond(0),
			MaxPoseEvaluations: 128, MaxIterations: 8, MaxEvents: 2,
		},
	}
	w, err := dynamics.NewWorld(t.Context(), doc, config)
	require.NoError(t, err)
	startPose, err := r3.Translation(r3.Vec{Z: 10})
	require.NoError(t, err)
	state, err := w.NewState([]dynamics.BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: box, Pose: startPose, LinearVelocity: dynamics.QuantityVec{
			X: units.MillimetersPerSecond(0), Y: units.MillimetersPerSecond(0),
			Z: units.MillimetersPerSecond(-100)}, AngularVelocity: zeroAngular(t)},
		{Body: barrier, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	report, err := w.Step(t.Context(), state, dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(0.2))
	require.NoError(t, err)
	require.Equal(t, dynamics.Undecided, report.Status)
	require.Nil(t, report.Next)
	require.Equal(t, dynamics.BodyPair{A: box, B: barrier}, report.Diagnostics[0].Pair)

	config.Excluded = []dynamics.BodyPair{{A: barrier, B: box}}
	w, err = dynamics.NewWorld(t.Context(), doc, config)
	require.NoError(t, err)
	state, err = w.NewState(state.Entries())
	require.NoError(t, err)
	report, err = w.Step(t.Context(), state, dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(0.2))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Equal(t, []dynamics.BodyPair{{A: box, B: barrier}}, report.Excluded)

	// The initial path clears this upper body, but the floor rebound reaches it.
	upper := makeBox(t, doc, -5, -5, 5, 5, 21, 5)
	config.Excluded = nil
	config.Bodies[2].Body = upper
	w, err = dynamics.NewWorld(t.Context(), doc, config)
	require.NoError(t, err)
	entries := state.Entries()
	entries[2].Body = upper
	state, err = w.NewState(entries)
	require.NoError(t, err)
	report, err = w.Step(t.Context(), state, dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(0.4))
	require.NoError(t, err)
	require.Equal(t, dynamics.Undecided, report.Status)
	require.Nil(t, report.Next)
	require.Contains(t, report.Diagnostics[0].Reason, "third-body pair lacks a clear response path")
	require.Equal(t, dynamics.BodyPair{A: box, B: upper}, report.Diagnostics[0].Pair)
}
