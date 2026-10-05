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

func offAxisSphereFrictionFixture(t *testing.T, coefficient float64, reverse bool,
	massA decad.MassProperties) (*decad.Document, *dynamics.World, *decad.Body, *decad.Body,
	dynamics.State, dynamics.StepConfig) {
	t.Helper()
	doc := decad.New()
	a, b := makeBall(t, doc), makeBall(t, doc)
	poseB, err := r3.Translation(r3.Vec{X: 6, Y: 8})
	require.NoError(t, err)
	cfg := sphereRestConfig()
	massB := exactSphereMass()
	material := dynamics.Material{Restitution: units.Scalar(.5), Friction: units.Scalar(coefficient)}
	bodies := []dynamics.RigidBody{
		{Body: a, Role: dynamics.Dynamic, Supplied: &massA, Material: material},
		{Body: b, Role: dynamics.Dynamic, Supplied: &massB, Material: material},
	}
	states := []dynamics.BodyState{
		{Body: a, Pose: r3.Identity(), LinearVelocity: sphereRestVelocity(r3.Vec{X: 10, Y: 20}),
			AngularVelocity: zeroAngular(t)},
		{Body: b, Pose: poseB, LinearVelocity: sphereRestVelocity(r3.Vec{X: -10, Y: -20}),
			AngularVelocity: zeroAngular(t)},
	}
	if reverse {
		bodies[0], bodies[1] = bodies[1], bodies[0]
		states[0], states[1] = states[1], states[0]
	}
	world, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{Bodies: bodies, Step: cfg})
	require.NoError(t, err)
	state, err := world.NewState(states)
	require.NoError(t, err)
	return doc, world, a, b, state, cfg
}

func TestSpherePairOffAxisFrictionImpactReplaysRotatingDeparture(t *testing.T) {
	for _, tc := range []struct {
		name    string
		mu      float64
		tangent float64
		reverse bool
	}{
		{name: "sticking", mu: .5, tangent: 8.0 / 7},
		{name: "sliding", mu: .01, tangent: .33},
		{name: "reverse", mu: .5, tangent: 8.0 / 7, reverse: true},
	} {
		t.Run(tc.name, func(t *testing.T) {
			mass := exactSphereMass()
			doc, world, a, b, state, cfg := offAxisSphereFrictionFixture(t, tc.mu, tc.reverse, mass)
			initialA, _ := state.Body(a)
			initialB, _ := state.Body(b)
			contact, err := doc.ContactPair(t.Context(), a, b, initialA.Pose, initialB.Pose, cfg.Contact)
			require.NoError(t, err)
			require.Equal(t, decad.ContactTouching, contact.Relation)
			require.Len(t, contact.Manifold.Points, 1)
			require.Positive(t, contact.Manifold.Points[0].Normal.Bound.Base())
			duration := units.Seconds(.1)
			path := func(entry dynamics.BodyState) decad.RigidDriftSegment {
				return decad.RigidDriftSegment{From: entry.Pose, LinearVelocity: entry.LinearVelocity,
					AngularVelocity: entry.AngularVelocity, Duration: duration}
			}
			firstA, firstB := initialA, initialB
			if tc.reverse {
				firstA, firstB = firstB, firstA
			}
			sweep, err := doc.SweepPair(t.Context(), firstA.Body, firstB.Body,
				path(firstA), path(firstB), decad.SweepRequest{ContactRequest: cfg.Contact,
					TimeResolution: cfg.TimeResolution, MaxPoseEvaluations: cfg.MaxPoseEvaluations})
			require.NoError(t, err)
			require.Equal(t, decad.SweepInitiallyTouching, sweep.Outcome)
			report, err := world.Step(t.Context(), state, dynamics.StepInput{Gravity: zeroAcceleration()}, duration)
			require.NoError(t, err)
			require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
			require.Len(t, report.Events, 1)
			event := report.Events[0]
			require.Equal(t, dynamics.ContactImpact, event.Kind)
			require.InDelta(t, 33, event.NormalImpulse.Base(), 1e-9)
			require.InDelta(t, tc.tangent*.8, math.Abs(event.TangentImpulse.X.Base()), 1e-9)
			require.InDelta(t, tc.tangent*.6, math.Abs(event.TangentImpulse.Y.Base()), 1e-9)
			require.Len(t, event.PointImpulses, 1)
			require.NotNil(t, event.Solver)
			endA, ok := report.Next.Body(a)
			require.True(t, ok)
			endB, ok := report.Next.Body(b)
			require.True(t, ok)
			require.InDelta(t, 10-19.8+.8*tc.tangent, endA.LinearVelocity.X.Base(), 1e-9)
			require.InDelta(t, 20-26.4-.6*tc.tangent, endA.LinearVelocity.Y.Base(), 1e-9)
			require.InDelta(t, -10+19.8-.8*tc.tangent, endB.LinearVelocity.X.Base(), 1e-9)
			require.InDelta(t, -20+26.4+.6*tc.tangent, endB.LinearVelocity.Y.Base(), 1e-9)
			require.InDelta(t, -tc.tangent/2, endA.AngularVelocity.Z.Base(), 1e-9)
			require.InDelta(t, -tc.tangent/2, endB.AngularVelocity.Z.Base(), 1e-9)
			require.NotEqual(t, r3.Identity().Basis(), endA.Pose.Basis())
			for _, elapsed := range []units.Value{units.Seconds(0), units.Seconds(.05), duration} {
				sample, sampleErr := report.Trace.Sample(elapsed)
				require.NoError(t, sampleErr)
				sa, ok := sample.Body(a)
				require.True(t, ok)
				sb, ok := sample.Body(b)
				require.True(t, ok)
				pair, pairErr := doc.ContactPair(t.Context(), a, b, sa.Pose, sb.Pose, cfg.Contact)
				require.NoError(t, pairErr)
				if elapsed.Base() == 0 {
					require.Equal(t, decad.ContactTouching, pair.Relation)
				} else {
					require.Equal(t, decad.ContactSeparated, pair.Relation)
				}
			}
			require.NotNil(t, report.Conservation)
			require.Less(t, report.Conservation.Completion.KineticEnergy.Value.Base(),
				report.Conservation.Input.KineticEnergy.Value.Base())
		})
	}
}

func TestSpherePairOffAxisFrictionRefusesNoncentralMass(t *testing.T) {
	mass := exactSphereMass()
	mass.Center.Value = r3.Vec{Z: 1}
	doc, world, a, b, state, cfg := offAxisSphereFrictionFixture(t, .5, false, mass)
	initialA, ok := state.Body(a)
	require.True(t, ok)
	initialB, ok := state.Body(b)
	require.True(t, ok)
	contact, err := doc.ContactPair(t.Context(), a, b, initialA.Pose, initialB.Pose, cfg.Contact)
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, contact.Relation)
	report, err := world.Step(t.Context(), state,
		dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(.1))
	require.NoError(t, err)
	require.Equal(t, dynamics.Undecided, report.Status)
	require.Nil(t, report.Next)
	// §12: the initial impact is certified; the spinning departure that
	// follows it has no certified sweep, so the prefix keeps the impact and
	// stops at the step start.
	require.Len(t, report.Events, 1)
	require.Len(t, report.Diagnostics, 1)
	require.Equal(t, dynamics.StepPairUndecided, report.Diagnostics[0].Code)
	require.Equal(t, units.Seconds(0), report.Diagnostics[0].From)
}

func TestSpherePairOffAxisFrictionSharedVelocityKeepsPersistentTouch(t *testing.T) {
	mass := exactSphereMass()
	doc, world, a, b, initial, cfg := offAxisSphereFrictionFixture(t, .5, false, mass)
	entries := initial.Entries()
	shared := sphereRestVelocity(r3.Vec{X: 8, Y: 16})
	for i := range entries {
		entries[i].LinearVelocity = shared
	}
	state, err := world.NewState(entries)
	require.NoError(t, err)
	duration := units.Seconds(.125)
	report, err := world.Step(t.Context(), state,
		dynamics.StepInput{Gravity: zeroAcceleration()}, duration)
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Empty(t, report.Events)
	for _, elapsed := range []units.Value{units.Seconds(0), units.Seconds(.0625), duration} {
		sample, sampleErr := report.Trace.Sample(elapsed)
		require.NoError(t, sampleErr)
		sa, ok := sample.Body(a)
		require.True(t, ok)
		sb, ok := sample.Body(b)
		require.True(t, ok)
		pair, pairErr := doc.ContactPair(t.Context(), a, b, sa.Pose, sb.Pose, cfg.Contact)
		require.NoError(t, pairErr)
		require.Equal(t, decad.ContactTouching, pair.Relation)
	}
}

func TestSpherePairInteriorOffAxisFrictionReplaysBothSides(t *testing.T) {
	doc := decad.New()
	a, b := makeBall(t, doc), makeBall(t, doc)
	poseB, err := r3.Translation(r3.Vec{X: 8.5, Y: 13})
	require.NoError(t, err)
	cfg := sphereRestConfig()
	mass := exactSphereMass()
	material := dynamics.Material{Restitution: units.Scalar(.5), Friction: units.Scalar(.5)}
	world, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{Bodies: []dynamics.RigidBody{
		{Body: a, Role: dynamics.Dynamic, Supplied: &mass, Material: material},
		{Body: b, Role: dynamics.Dynamic, Supplied: &mass, Material: material}}, Step: cfg})
	require.NoError(t, err)
	state, err := world.NewState([]dynamics.BodyState{
		{Body: a, Pose: r3.Identity(), LinearVelocity: sphereRestVelocity(r3.Vec{X: 10, Y: 20}),
			AngularVelocity: zeroAngular(t)},
		{Body: b, Pose: poseB, LinearVelocity: sphereRestVelocity(r3.Vec{X: -10, Y: -20}),
			AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	duration := units.Seconds(.25)
	path := func(entry dynamics.BodyState) decad.RigidDriftSegment {
		return decad.RigidDriftSegment{From: entry.Pose, LinearVelocity: entry.LinearVelocity,
			AngularVelocity: entry.AngularVelocity, Duration: duration}
	}
	initialA, _ := state.Body(a)
	initialB, _ := state.Body(b)
	contact, err := doc.ContactPair(t.Context(), a, b, initialA.Pose, initialB.Pose, cfg.Contact)
	require.NoError(t, err)
	require.Equal(t, decad.ContactSeparated, contact.Relation)
	first, err := doc.SweepPair(t.Context(), a, b, path(initialA), path(initialB),
		decad.SweepRequest{ContactRequest: cfg.Contact, TimeResolution: cfg.TimeResolution,
			MaxPoseEvaluations: cfg.MaxPoseEvaluations})
	require.NoError(t, err)
	require.Equal(t, decad.SweepImpactBracket, first.Outcome)
	report, err := world.Step(t.Context(), state,
		dynamics.StepInput{Gravity: zeroAcceleration()}, duration)
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Len(t, report.Events, 1)
	// The event lies at its bracket's right sample, at most one
	// TimeResolution after the exact contact time 0.125 s (§5.1).
	impact := report.Events[0].Time
	require.InDelta(t, .125, impact.Base(), cfg.TimeResolution.Base())
	require.GreaterOrEqual(t, impact.Base(), .125)
	// Read at the bracket's right sample, the impulse agrees with the exact
	// 33 within ImpulseResidual (docs/multibody-dynamics-design.md §14).
	require.InDelta(t, 33, report.Events[0].NormalImpulse.Base(), cfg.ImpulseResidual.Base())
	for _, elapsed := range []units.Value{units.Seconds(.0625), impact, units.Seconds(.1875), duration} {
		sample, sampleErr := report.Trace.Sample(elapsed)
		require.NoError(t, sampleErr)
		sa, ok := sample.Body(a)
		require.True(t, ok)
		sb, ok := sample.Body(b)
		require.True(t, ok)
		pair, pairErr := doc.ContactPair(t.Context(), a, b, sa.Pose, sb.Pose, cfg.Contact)
		require.NoError(t, pairErr)
		if elapsed == impact {
			// The event's post state is pushed just apart (§6.6): the
			// spheres separate by no more than ContactSlop.
			require.Equal(t, decad.ContactSeparated, pair.Relation)
			require.LessOrEqual(t, pair.Gap.Value.Base()+pair.Gap.Bound.Base(), cfg.ContactSlop.Base())
		} else {
			require.Equal(t, decad.ContactSeparated, pair.Relation)
		}
	}
}
