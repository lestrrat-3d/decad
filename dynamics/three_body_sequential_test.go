package dynamics_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/dynamics"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestThreeBodySequentialBoxImpacts(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	floor := makeBox(t, doc, -20, -20, 20, 20, -10, 10)
	box := makeBox(t, doc, -5, -5, 5, 5, 0, 10)
	ceiling := makeBox(t, doc, -20, -20, 20, 20, 21, 5)
	density := units.KilogramsPerCubicMillimeter(.001)
	material := dynamics.Material{Restitution: units.Scalar(.5), Friction: units.Scalar(0)}
	config := dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{
			{Body: floor, Role: dynamics.Fixed, Material: material},
			{Body: box, Role: dynamics.Dynamic, Density: &density, Material: material},
			{Body: ceiling, Role: dynamics.Fixed, Material: material},
		},
		Step: dynamics.StepConfig{
			Contact: decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
				NormalResolution: units.Radians(1e-6)},
			TimeResolution: units.Seconds(1e-9), ContactSlop: units.Millimeters(1e-6),
			VelocityResidual:        units.MillimetersPerSecond(1e-6),
			AngularVelocityResidual: units.RadiansPerSecond(1e-6),
			ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
			PenetrationResidual:     units.Millimeters(1e-6),
			ImpactSpeed:             units.MillimetersPerSecond(0),
			MaxPoseEvaluations:      128, MaxIterations: 8, MaxEvents: 3, MaxPairSweeps: 4096,
		},
	}
	w, err := dynamics.NewWorld(t.Context(), doc, config)
	require.NoError(t, err)
	startPose, err := r3.Translation(r3.Vec{Z: 10})
	require.NoError(t, err)
	start, err := w.NewState([]dynamics.BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: box, Pose: startPose, LinearVelocity: dynamics.QuantityVec{
			X: units.MillimetersPerSecond(0), Y: units.MillimetersPerSecond(0),
			Z: units.MillimetersPerSecond(-100),
		}, AngularVelocity: zeroAngular(t)},
		{Body: ceiling, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	req := config.Step.Contact
	floorContact, err := doc.ContactPair(t.Context(), floor, box, r3.Identity(), r3.Identity(), req)
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, floorContact.Relation)
	require.Len(t, floorContact.Manifold.Points, 4)
	ceilingPose, err := r3.Translation(r3.Vec{Z: 11})
	require.NoError(t, err)
	ceilingContact, err := doc.ContactPair(t.Context(), box, ceiling, ceilingPose, r3.Identity(), req)
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, ceilingContact.Relation)
	require.Len(t, ceilingContact.Manifold.Points, 4)
	incomingEnd, err := r3.Translation(r3.Vec{Z: -30})
	require.NoError(t, err)
	outgoingEnd, err := r3.Translation(r3.Vec{Z: 15})
	require.NoError(t, err)
	sweepRequest := decad.SweepRequest{ContactRequest: req, TimeResolution: config.Step.TimeResolution,
		MaxPoseEvaluations: config.Step.MaxPoseEvaluations}
	still := func(duration units.Value) decad.PoseSegment {
		return decad.PoseSegment{From: r3.Identity(), To: r3.Identity(), Duration: duration}
	}
	firstSweep, err := doc.SweepPair(t.Context(), floor, box, still(units.Seconds(.4)),
		decad.PoseSegment{From: startPose, To: incomingEnd, Duration: units.Seconds(.4)}, sweepRequest)
	require.NoError(t, err)
	require.Equal(t, decad.SweepImpactBracket, firstSweep.Outcome)
	secondSweep, err := doc.SweepPair(t.Context(), box, ceiling,
		decad.PoseSegment{From: r3.Identity(), To: outgoingEnd, Duration: units.Seconds(.3)},
		still(units.Seconds(.3)), sweepRequest)
	require.NoError(t, err)
	require.Equal(t, decad.SweepImpactBracket, secondSweep.Outcome)
	require.InDelta(t, .1, firstSweep.Bracket.To.Elapsed.Value.Base(), 2e-9)
	require.InDelta(t, .22, secondSweep.Bracket.To.Elapsed.Value.Base(), 2e-9)

	report, err := w.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(.4))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Len(t, report.Events, 2)
	require.Equal(t, dynamics.BodyPair{A: floor, B: box}, report.Events[0].Pair)
	require.Equal(t, dynamics.BodyPair{A: box, B: ceiling}, report.Events[1].Pair)
	require.InDelta(t, .1, report.Events[0].Time.Base(), 2e-9)
	require.InDelta(t, .32, report.Events[1].Time.Base(), 2e-9)
	require.Equal(t, units.Seconds(0), report.Events[0].SliceStart)
	require.Equal(t, units.Seconds(.4), report.Events[0].SliceDuration)
	require.Equal(t, report.Events[0].Time, report.Events[1].SliceStart)
	require.InDelta(t, 150, report.Events[0].NormalImpulse.Base(), 1e-6)
	require.InDelta(t, 75, report.Events[1].NormalImpulse.Base(), 1e-6)
	end, ok := report.Next.Body(box)
	require.True(t, ok)
	require.InDelta(t, -25, end.LinearVelocity.Z.Base(), 1e-6)
	require.InDelta(t, 9, end.Pose.Translation().Z, 2e-6)
	require.InDelta(t, 75, report.Conservation.ContactImpulse.Value.Z.Base(), 1e-6)
	require.InDelta(t, -25, report.Conservation.Completion.LinearMomentum.Value.Z.Base(), 1e-6)
	require.InDelta(t, 312.5, report.Conservation.Completion.KineticEnergy.Value.Base(), 1e-5)
	for _, sample := range []struct{ at, z, vz float64 }{
		{.05, 5, -100}, {.15, 2.5, 50}, {.25, 7.5, 50}, {.35, 10.25, -25}, {.4, 9, -25},
	} {
		state, err := report.Trace.Sample(units.Seconds(sample.at))
		require.NoError(t, err)
		entry, ok := state.Body(box)
		require.True(t, ok)
		require.InDelta(t, sample.z, entry.Pose.Translation().Z, 2e-6)
		require.InDelta(t, sample.vz, entry.LinearVelocity.Z.Base(), 1e-6)
	}
	for i, wantVelocity := range []float64{50, -25} {
		state, err := report.Trace.Sample(report.Events[i].Time)
		require.NoError(t, err)
		entry, ok := state.Body(box)
		require.True(t, ok)
		require.InDelta(t, wantVelocity, entry.LinearVelocity.Z.Base(), 1e-6)
	}
	for _, maxEvents := range []int{1, 2} {
		limitedConfig := config
		limitedConfig.Step.MaxEvents = maxEvents
		limited, err := dynamics.NewWorld(t.Context(), doc, limitedConfig)
		require.NoError(t, err)
		limitedStart, err := limited.NewState(start.Entries())
		require.NoError(t, err)
		out, err := limited.Step(t.Context(), limitedStart,
			dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(.4))
		require.NoError(t, err)
		require.Equal(t, dynamics.Undecided, out.Status)
		require.Nil(t, out.Next)
	}
	loadedEntries := start.Entries()
	loadedEntries[1].LinearVelocity.Z = units.MillimetersPerSecond(-104)
	loaded, err := w.NewState(loadedEntries)
	require.NoError(t, err)
	gravity := zeroAcceleration()
	gravity.Z = units.MillimetersPerSecondSquared(10)
	forced, err := w.Step(t.Context(), loaded, dynamics.StepInput{Gravity: gravity}, units.Seconds(.4))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, forced.Status, "%+v", forced.Diagnostics)
	require.Len(t, forced.Events, 2)
	require.InDelta(t, -100, forced.Conservation.AfterKick.LinearMomentum.Value.Z.Base(), 1e-6)
	require.InDelta(t, 4, forced.Conservation.GravityImpulse.Value.Z.Base(), 1e-6)
	require.InDelta(t, -25, forced.Conservation.Completion.LinearMomentum.Value.Z.Base(), 1e-6)
	reversedConfig := config
	reversedConfig.Bodies = []dynamics.RigidBody{config.Bodies[2], config.Bodies[1], config.Bodies[0]}
	reversed, err := dynamics.NewWorld(t.Context(), doc, reversedConfig)
	require.NoError(t, err)
	reversedStart, err := reversed.NewState(start.Entries())
	require.NoError(t, err)
	reversedReport, err := reversed.Step(t.Context(), reversedStart,
		dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(.4))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, reversedReport.Status, "%+v", reversedReport.Diagnostics)
	require.Len(t, reversedReport.Events, 2)
	require.Equal(t, dynamics.BodyPair{A: box, B: floor}, reversedReport.Events[0].Pair)
	require.Equal(t, dynamics.BodyPair{A: ceiling, B: box}, reversedReport.Events[1].Pair)
	require.InDelta(t, 75, reversedReport.Conservation.ContactImpulse.Value.Z.Base(), 1e-6)
	replayedEnd, err := reversedReport.Trace.Sample(units.Seconds(.4))
	require.NoError(t, err)
	require.Equal(t, reversedReport.Next.Entries(), replayedEnd.Entries())
	endpoint, err := w.Step(t.Context(), start,
		dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(.1))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, endpoint.Status, "%+v", endpoint.Diagnostics)
	require.Len(t, endpoint.Events, 1)
	endpointReplay, err := endpoint.Trace.Sample(units.Seconds(.1))
	require.NoError(t, err)
	require.Equal(t, endpoint.Next.Entries(), endpointReplay.Entries())
	endpointConfig := config
	endpointConfig.Step.MaxEvents = 1
	endpointWorld, err := dynamics.NewWorld(t.Context(), doc, endpointConfig)
	require.NoError(t, err)
	endpointStart, err := endpointWorld.NewState(start.Entries())
	require.NoError(t, err)
	endpointLimited, err := endpointWorld.Step(t.Context(), endpointStart,
		dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(.1))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, endpointLimited.Status, "%+v", endpointLimited.Diagnostics)
}
