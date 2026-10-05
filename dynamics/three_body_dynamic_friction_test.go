package dynamics_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/dynamics"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestThreeDynamicSimultaneousSphereFriction(t *testing.T) {
	doc := decad.New()
	a, b, c := makeBall(t, doc), makeBall(t, doc), makeBall(t, doc)
	poseB, err := r3.Translation(r3.Vec{X: 10})
	require.NoError(t, err)
	poseC, err := r3.Translation(r3.Vec{Y: 10})
	require.NoError(t, err)
	mass := exactSphereMass()
	material := dynamics.Material{Restitution: units.Scalar(.5), Friction: units.Scalar(.1)}
	cfg := sphereRestConfig()
	cfg.MaxEvents = 3
	worldConfig := dynamics.WorldConfig{Bodies: []dynamics.RigidBody{
		{Body: a, Role: dynamics.Dynamic, Supplied: &mass, Material: material},
		{Body: b, Role: dynamics.Dynamic, Supplied: &mass, Material: material},
		{Body: c, Role: dynamics.Dynamic, Supplied: &mass, Material: material},
	}, Step: cfg}
	world, err := dynamics.NewWorld(t.Context(), doc, worldConfig)
	require.NoError(t, err)
	state, err := world.NewState([]dynamics.BodyState{
		{Body: a, Pose: r3.Identity(), LinearVelocity: sphereRestVelocity(r3.Vec{X: 10, Y: 10}),
			AngularVelocity: zeroAngular(t)},
		{Body: b, Pose: poseB, LinearVelocity: sphereRestVelocity(r3.Vec{}),
			AngularVelocity: zeroAngular(t)},
		{Body: c, Pose: poseC, LinearVelocity: sphereRestVelocity(r3.Vec{}),
			AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	entries := state.Entries()
	duration := units.Seconds(.1)
	for _, pair := range []struct {
		a, b int
		want decad.ContactRelation
	}{
		{0, 1, decad.ContactTouching}, {0, 2, decad.ContactTouching},
		{1, 2, decad.ContactSeparated},
	} {
		first, second := entries[pair.a], entries[pair.b]
		contact, contactErr := doc.ContactPair(t.Context(), first.Body, second.Body,
			first.Pose, second.Pose, cfg.Contact)
		require.NoError(t, contactErr)
		require.Equal(t, pair.want, contact.Relation)
		sweep, sweepErr := doc.SweepPair(t.Context(), first.Body, second.Body,
			decad.RigidDriftSegment{From: first.Pose, LinearVelocity: first.LinearVelocity,
				AngularVelocity: first.AngularVelocity, Duration: duration},
			decad.RigidDriftSegment{From: second.Pose, LinearVelocity: second.LinearVelocity,
				AngularVelocity: second.AngularVelocity, Duration: duration},
			decad.SweepRequest{ContactRequest: cfg.Contact,
				TimeResolution: cfg.TimeResolution, MaxPoseEvaluations: cfg.MaxPoseEvaluations})
		require.NoError(t, sweepErr)
		if pair.want == decad.ContactTouching {
			require.Equal(t, decad.SweepInitiallyTouching, sweep.Outcome, "cause=%v", sweep.Cause)
		} else {
			require.Equal(t, decad.SweepClear, sweep.Outcome, "cause=%v", sweep.Cause)
		}
	}
	report, err := world.Step(t.Context(), state,
		dynamics.StepInput{Gravity: zeroAcceleration()}, duration)
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Len(t, report.Events, 2)
	for i, event := range report.Events {
		require.Equal(t, dynamics.ContactImpact, event.Kind)
		require.InDelta(t, 7.1875, event.NormalImpulse.Base(), 1e-12)
		require.Len(t, event.PointImpulses, 1)
		require.NotNil(t, event.Solver)
		if i == 0 {
			require.Equal(t, dynamics.BodyPair{A: a, B: b}, event.Pair)
			require.InDelta(t, .625, event.TangentImpulse.Y.Base(), 1e-12)
			require.Zero(t, event.TangentImpulse.X.Base())
		} else {
			require.Equal(t, dynamics.BodyPair{A: a, B: c}, event.Pair)
			require.InDelta(t, .625, event.TangentImpulse.X.Base(), 1e-12)
			require.Zero(t, event.TangentImpulse.Y.Base())
		}
	}
	endA, _ := report.Next.Body(a)
	endB, _ := report.Next.Body(b)
	endC, _ := report.Next.Body(c)
	for _, component := range []units.Value{endA.LinearVelocity.X, endA.LinearVelocity.Y} {
		require.InDelta(t, 2.1875, component.Base(), 1e-12)
	}
	require.InDelta(t, 7.1875, endB.LinearVelocity.X.Base(), 1e-12)
	require.InDelta(t, .625, endB.LinearVelocity.Y.Base(), 1e-12)
	require.InDelta(t, .625, endC.LinearVelocity.X.Base(), 1e-12)
	require.InDelta(t, 7.1875, endC.LinearVelocity.Y.Base(), 1e-12)
	require.InDelta(t, -.3125, endB.AngularVelocity.Z.Base(), 1e-12)
	require.InDelta(t, .3125, endC.AngularVelocity.Z.Base(), 1e-12)
	require.NotNil(t, report.Conservation)
	require.Less(t, report.Conservation.Completion.KineticEnergy.Value.Base(),
		report.Conservation.Input.KineticEnergy.Value.Base())
	require.InDelta(t, 10, report.Conservation.Completion.LinearMomentum.Value.X.Base(), 1e-9)
	require.InDelta(t, 10, report.Conservation.Completion.LinearMomentum.Value.Y.Base(), 1e-9)
	require.InDelta(t, 0, report.Conservation.Completion.AngularMomentum.Value.Z.Base(), 1e-9)
	require.Zero(t, report.Conservation.ContactImpulse.Value.X.Base())
	require.Zero(t, report.Conservation.ContactImpulse.Value.Y.Base())
	for _, at := range []units.Value{units.Seconds(0), units.Seconds(.025),
		units.Seconds(.05), units.Seconds(.075), duration} {
		sample, sampleErr := report.Trace.Sample(at)
		require.NoError(t, sampleErr)
		require.Len(t, sample.Entries(), 3)
		for _, pair := range []struct {
			first, second *decad.Body
			want          decad.ContactRelation
		}{
			{a, b, decad.ContactSeparated}, {a, c, decad.ContactSeparated},
			{b, c, decad.ContactSeparated},
		} {
			first, _ := sample.Body(pair.first)
			second, _ := sample.Body(pair.second)
			contact, contactErr := doc.ContactPair(t.Context(), pair.first, pair.second,
				first.Pose, second.Pose, cfg.Contact)
			require.NoError(t, contactErr)
			if at.Base() == 0 && pair.first == a {
				require.Equal(t, decad.ContactTouching, contact.Relation)
			} else {
				require.Equal(t, pair.want, contact.Relation)
			}
		}
	}
	continued, err := world.Step(t.Context(), *report.Next,
		dynamics.StepInput{Gravity: zeroAcceleration()}, duration)
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, continued.Status, "%+v", continued.Diagnostics)
	require.Empty(t, continued.Events)
	_, err = continued.Trace.Sample(units.Seconds(.05))
	require.NoError(t, err)
	for _, tc := range []struct {
		name   string
		change func(*dynamics.WorldConfig, []dynamics.BodyState)
	}{
		{"cone", func(config *dynamics.WorldConfig, _ []dynamics.BodyState) {
			for i := range config.Bodies {
				config.Bodies[i].Material.Friction = units.Scalar(.05)
			}
		}},
		{"unequal-mass", func(config *dynamics.WorldConfig, _ []dynamics.BodyState) {
			other := exactSphereMass()
			other.Mass.Value = units.Kilograms(2)
			config.Bodies[2].Supplied = &other
		}},
		{"asymmetric-speed", func(_ *dynamics.WorldConfig, entries []dynamics.BodyState) {
			entries[0].LinearVelocity.Y = units.MillimetersPerSecond(9)
		}},
		{"event-limit", func(config *dynamics.WorldConfig, _ []dynamics.BodyState) {
			config.Step.MaxEvents = 2
		}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			config := worldConfig
			config.Bodies = append([]dynamics.RigidBody(nil), worldConfig.Bodies...)
			otherEntries := append([]dynamics.BodyState(nil), entries...)
			tc.change(&config, otherEntries)
			otherWorld, worldErr := dynamics.NewWorld(t.Context(), doc, config)
			require.NoError(t, worldErr)
			otherState, stateErr := otherWorld.NewState(otherEntries)
			require.NoError(t, stateErr)
			refused, stepErr := otherWorld.Step(t.Context(), otherState,
				dynamics.StepInput{Gravity: zeroAcceleration()}, duration)
			require.NoError(t, stepErr)
			require.Equal(t, dynamics.Undecided, refused.Status)
			require.Nil(t, refused.Next)
			require.Empty(t, refused.Events)
		})
	}
	t.Run("equal-mass-scale", func(t *testing.T) {
		scaled := exactSphereMass()
		scaled.Mass.Value = units.Kilograms(2)
		scaled.Inertia.XX.Value = units.KilogramSquareMillimeters(20)
		scaled.Inertia.YY.Value = units.KilogramSquareMillimeters(20)
		scaled.Inertia.ZZ.Value = units.KilogramSquareMillimeters(20)
		config := worldConfig
		config.Bodies = append([]dynamics.RigidBody(nil), worldConfig.Bodies...)
		for i := range config.Bodies {
			config.Bodies[i].Supplied = &scaled
		}
		otherWorld, worldErr := dynamics.NewWorld(t.Context(), doc, config)
		require.NoError(t, worldErr)
		otherState, stateErr := otherWorld.NewState(entries)
		require.NoError(t, stateErr)
		resolved, stepErr := otherWorld.Step(t.Context(), otherState,
			dynamics.StepInput{Gravity: zeroAcceleration()}, duration)
		require.NoError(t, stepErr)
		require.Equal(t, dynamics.Advanced, resolved.Status, "%+v", resolved.Diagnostics)
		require.InDelta(t, 14.375, resolved.Events[0].NormalImpulse.Base(), 1e-12)
		require.InDelta(t, 1.25, resolved.Events[0].TangentImpulse.Y.Base(), 1e-12)
		require.InDelta(t, 2.8125, resolved.Events[0].PostVelocityA.X.Base(), 1e-12)
		_, sampleErr := resolved.Trace.Sample(units.Seconds(.05))
		require.NoError(t, sampleErr)
	})
}

func TestThreeDynamicRotatingClearDriftConservation(t *testing.T) {
	doc := decad.New()
	a, b, c := makeBall(t, doc), makeBall(t, doc), makeBall(t, doc)
	mass := exactSphereMass()
	material := dynamics.Material{Restitution: units.Scalar(.5), Friction: units.Scalar(.1)}
	cfg := sphereRestConfig()
	cfg.MaxEvents = 3
	world, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{Bodies: []dynamics.RigidBody{
		{Body: a, Role: dynamics.Dynamic, Supplied: &mass, Material: material},
		{Body: b, Role: dynamics.Dynamic, Supplied: &mass, Material: material},
		{Body: c, Role: dynamics.Dynamic, Supplied: &mass, Material: material},
	}, Step: cfg})
	require.NoError(t, err)
	poseA, err := r3.Translation(r3.Vec{X: 1000})
	require.NoError(t, err)
	poseB, err := r3.Translation(r3.Vec{X: 1020})
	require.NoError(t, err)
	poseC, err := r3.Translation(r3.Vec{X: 1000, Y: 20})
	require.NoError(t, err)
	spin := func(z float64) dynamics.QuantityVec {
		return dynamics.QuantityVec{X: units.RadiansPerSecond(0),
			Y: units.RadiansPerSecond(0), Z: units.RadiansPerSecond(z)}
	}
	state, err := world.NewState([]dynamics.BodyState{
		{Body: a, Pose: poseA, LinearVelocity: sphereRestVelocity(r3.Vec{X: .1, Y: .3}),
			AngularVelocity: spin(.1)},
		{Body: b, Pose: poseB, LinearVelocity: sphereRestVelocity(r3.Vec{X: .2, Y: .4}),
			AngularVelocity: spin(-.2)},
		{Body: c, Pose: poseC, LinearVelocity: sphereRestVelocity(r3.Vec{X: .3, Y: .1}),
			AngularVelocity: spin(.3)},
	})
	require.NoError(t, err)
	report, err := world.Step(t.Context(), state,
		dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(.03))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Empty(t, report.Events)
	require.NotNil(t, report.Conservation)
	require.Positive(t, report.Conservation.DriftChange.AngularMomentum.Bound.Z.Base())
	require.GreaterOrEqual(t, report.Conservation.DriftChange.AngularMomentum.Bound.Z.Base(),
		report.Conservation.AfterKick.AngularMomentum.Bound.Z.Base()+
			report.Conservation.Completion.AngularMomentum.Bound.Z.Base())
}
