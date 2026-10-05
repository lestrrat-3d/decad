package dynamics_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/dynamics"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestThreeBodyThreeDynamicIsolatedSphereImpact(t *testing.T) {
	doc := decad.New()
	a, b, c := makeBall(t, doc), makeBall(t, doc), makeBall(t, doc)
	originalBodies := doc.Bodies()
	pa, err := r3.Translation(r3.Vec{X: -6, Y: -8})
	require.NoError(t, err)
	pb, err := r3.Translation(r3.Vec{X: 6, Y: 8})
	require.NoError(t, err)
	pc, err := r3.Translation(r3.Vec{Z: 100})
	require.NoError(t, err)
	va := dynamics.QuantityVec{X: units.MillimetersPerSecond(30),
		Y: units.MillimetersPerSecond(40), Z: units.MillimetersPerSecond(0)}
	vb := dynamics.QuantityVec{X: units.MillimetersPerSecond(-30),
		Y: units.MillimetersPerSecond(-40), Z: units.MillimetersPerSecond(0)}
	vc := dynamics.QuantityVec{X: units.MillimetersPerSecond(1),
		Y: units.MillimetersPerSecond(0), Z: units.MillimetersPerSecond(0)}
	spin := zeroAngular(t)
	request := decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
		NormalResolution: units.Radians(1e-6)}
	for _, pair := range []struct {
		name   string
		a, b   *decad.Body
		pa, pb r3.Transform
		va, vb dynamics.QuantityVec
		want   decad.SweepOutcome
	}{
		{"impact", a, b, pa, pb, va, vb, decad.SweepImpactBracket},
		{"a-clear", a, c, pa, pc, va, vc, decad.SweepClear},
		{"b-clear", b, c, pb, pc, vb, vc, decad.SweepClear},
	} {
		t.Run(pair.name, func(t *testing.T) {
			sweep, sweepErr := doc.SweepPair(t.Context(), pair.a, pair.b,
				decad.RigidDriftSegment{From: pair.pa, LinearVelocity: pair.va,
					AngularVelocity: spin, Duration: units.Seconds(.2)},
				decad.RigidDriftSegment{From: pair.pb, LinearVelocity: pair.vb,
					AngularVelocity: spin, Duration: units.Seconds(.2)},
				decad.SweepRequest{ContactRequest: request, TimeResolution: units.Seconds(1e-9),
					MaxPoseEvaluations: 128})
			require.NoError(t, sweepErr)
			require.Equal(t, pair.want, sweep.Outcome, "cause=%v", sweep.Cause)
		})
	}
	mass := exactSphereMass()
	material := dynamics.Material{Restitution: units.Scalar(.5), Friction: units.Scalar(0)}
	step := dynamics.StepConfig{Contact: request, TimeResolution: units.Seconds(1e-9),
		ContactSlop: units.Millimeters(1e-6), VelocityResidual: units.MillimetersPerSecond(1e-6),
		AngularVelocityResidual: units.RadiansPerSecond(1e-6),
		ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
		PenetrationResidual:     units.Millimeters(1e-6),
		ImpactSpeed:             units.MillimetersPerSecond(0), MaxPoseEvaluations: 128,
		MaxIterations: 8, MaxEvents: 2, MaxPairSweeps: 4096}
	config := dynamics.WorldConfig{Bodies: []dynamics.RigidBody{
		{Body: a, Role: dynamics.Dynamic, Supplied: &mass, Material: material},
		{Body: b, Role: dynamics.Dynamic, Supplied: &mass, Material: material},
		{Body: c, Role: dynamics.Dynamic, Supplied: &mass, Material: material},
	}, Step: step}
	world, err := dynamics.NewWorld(t.Context(), doc, config)
	require.NoError(t, err)
	start, err := world.NewState([]dynamics.BodyState{
		{Body: a, Pose: pa, LinearVelocity: va, AngularVelocity: spin},
		{Body: b, Pose: pb, LinearVelocity: vb, AngularVelocity: spin},
		{Body: c, Pose: pc, LinearVelocity: vc, AngularVelocity: spin},
	})
	require.NoError(t, err)
	report, err := world.Step(t.Context(), start,
		dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(.2))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Len(t, report.Events, 1)
	require.Equal(t, dynamics.BodyPair{A: a, B: b}, report.Events[0].Pair)
	require.InDelta(t, .1, report.Events[0].Time.Base(), 1e-9)
	require.InDelta(t, 75, report.Events[0].NormalImpulse.Base(), 1e-5)
	require.NotNil(t, report.Conservation)
	require.InDelta(t, 2500.5, report.Conservation.Input.KineticEnergy.Value.Base(), 1e-5)
	require.InDelta(t, 625.5, report.Conservation.Completion.KineticEnergy.Value.Base(), 1e-5)
	require.InDelta(t, 1, report.Conservation.Input.LinearMomentum.Value.X.Base(), 1e-9)
	require.InDelta(t, 1, report.Conservation.Completion.LinearMomentum.Value.X.Base(), 1e-9)
	require.InDelta(t, 100, report.Conservation.Input.AngularMomentum.Value.Y.Base(), 1e-9)
	require.InDelta(t, 100, report.Conservation.Completion.AngularMomentum.Value.Y.Base(), 1e-9)
	require.InDelta(t, 0, report.Conservation.ContactImpulse.Value.X.Base(), 1e-9)
	for _, sample := range []struct{ at, cx float64 }{{.05, .05}, {.1, .1}, {.15, .15}, {.2, .2}} {
		state, sampleErr := report.Trace.Sample(units.Seconds(sample.at))
		require.NoError(t, sampleErr)
		require.Len(t, state.Entries(), 3)
		third, ok := state.Body(c)
		require.True(t, ok)
		require.InDelta(t, sample.cx, third.Pose.Translation().X, 1e-9)
		require.InDelta(t, 100, third.Pose.Translation().Z, 1e-9)
		require.Equal(t, vc, third.LinearVelocity)
	}
	require.Equal(t, originalBodies, doc.Bodies())
	loaded, err := world.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration(),
		Loads: []dynamics.BodyLoad{{Body: c, Force: testForce(10, 0), Torque: testTorque(0)}}},
		units.Seconds(.2))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, loaded.Status, "%+v", loaded.Diagnostics)
	require.Len(t, loaded.Events, 1)
	loadedThird, ok := loaded.Next.Body(c)
	require.True(t, ok)
	require.InDelta(t, .6, loadedThird.Pose.Translation().X, 1e-9)
	require.InDelta(t, 3, loadedThird.LinearVelocity.X.Base(), 1e-9)
	require.InDelta(t, 2504.5, loaded.Conservation.AfterKick.KineticEnergy.Value.Base(), 1e-5)
	require.InDelta(t, 629.5, loaded.Conservation.Completion.KineticEnergy.Value.Base(), 1e-5)
	require.InDelta(t, 2, loaded.Conservation.LoadImpulse.Value.X.Base(), 1e-9)
	require.InDelta(t, 3, loaded.Conservation.Completion.LinearMomentum.Value.X.Base(), 1e-9)
	require.InDelta(t, 300, loaded.Conservation.Completion.AngularMomentum.Value.Y.Base(), 1e-9)
	loadedSample, err := loaded.Trace.Sample(units.Seconds(.15))
	require.NoError(t, err)
	loadedThird, ok = loadedSample.Body(c)
	require.True(t, ok)
	require.InDelta(t, .45, loadedThird.Pose.Translation().X, 1e-9)
	reordered := config
	reordered.Bodies = []dynamics.RigidBody{config.Bodies[0], config.Bodies[2], config.Bodies[1]}
	reorderedWorld, err := dynamics.NewWorld(t.Context(), doc, reordered)
	require.NoError(t, err)
	reorderedStart, err := reorderedWorld.NewState(start.Entries())
	require.NoError(t, err)
	reorderedReport, err := reorderedWorld.Step(t.Context(), reorderedStart,
		dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(.2))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, reorderedReport.Status, "%+v", reorderedReport.Diagnostics)
	require.Len(t, reorderedReport.Events, 1)
	require.Equal(t, dynamics.BodyPair{A: a, B: b}, reorderedReport.Events[0].Pair)
	reorderedSample, err := reorderedReport.Trace.Sample(units.Seconds(.15))
	require.NoError(t, err)
	reorderedThird, ok := reorderedSample.Body(c)
	require.True(t, ok)
	require.InDelta(t, .15, reorderedThird.Pose.Translation().X, 1e-9)
	clearEntries := start.Entries()
	clearEntries[1].LinearVelocity = va
	clearStart, err := world.NewState(clearEntries)
	require.NoError(t, err)
	clearReport, err := world.Step(t.Context(), clearStart,
		dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(.2))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, clearReport.Status, "%+v", clearReport.Diagnostics)
	require.Empty(t, clearReport.Events)
	require.InDelta(t, 2500.5, clearReport.Conservation.Completion.KineticEnergy.Value.Base(), 1e-5)
	clearSample, err := clearReport.Trace.Sample(units.Seconds(.1))
	require.NoError(t, err)
	clearThird, ok := clearSample.Body(c)
	require.True(t, ok)
	require.InDelta(t, .1, clearThird.Pose.Translation().X, 1e-9)
	overlappingEntries := start.Entries()
	overlappingEntries[2].Pose = pa
	overlapping, err := world.NewState(overlappingEntries)
	require.NoError(t, err)
	refused, err := world.Step(t.Context(), overlapping,
		dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(.2))
	require.NoError(t, err)
	require.Equal(t, dynamics.Undecided, refused.Status)
	require.Nil(t, refused.Next)
	config.Step.MaxEvents = 1
	limited, err := dynamics.NewWorld(t.Context(), doc, config)
	require.NoError(t, err)
	limitedStart, err := limited.NewState(start.Entries())
	require.NoError(t, err)
	undecided, err := limited.Step(t.Context(), limitedStart,
		dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(.2))
	require.NoError(t, err)
	require.Equal(t, dynamics.Undecided, undecided.Status)
	require.Nil(t, undecided.Next)
}
