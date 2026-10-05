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

func spherePairFrictionFixture(t *testing.T, coefficient, restitution, tangentAY float64, reverse bool,
	massA, massB decad.MassProperties, spinA dynamics.QuantityVec) (*decad.Document,
	*dynamics.World, *decad.Body, *decad.Body, dynamics.State, dynamics.StepConfig) {
	t.Helper()
	doc := decad.New()
	a, b := makeBall(t, doc), makeBall(t, doc)
	poseA, err := r3.Translation(r3.Vec{X: -5})
	require.NoError(t, err)
	poseB, err := r3.Translation(r3.Vec{X: 5})
	require.NoError(t, err)
	material := dynamics.Material{Restitution: units.Scalar(restitution), Friction: units.Scalar(coefficient)}
	bodies := []dynamics.RigidBody{
		{Body: a, Role: dynamics.Dynamic, Supplied: &massA, Material: material},
		{Body: b, Role: dynamics.Dynamic, Supplied: &massB, Material: material},
	}
	states := []dynamics.BodyState{
		{Body: a, Pose: poseA, LinearVelocity: sphereRestVelocity(r3.Vec{X: 50, Y: tangentAY}),
			AngularVelocity: spinA},
		{Body: b, Pose: poseB, LinearVelocity: sphereRestVelocity(r3.Vec{X: -50}),
			AngularVelocity: zeroAngular(t)},
	}
	if reverse {
		bodies[0], bodies[1] = bodies[1], bodies[0]
		states[0], states[1] = states[1], states[0]
	}
	cfg := sphereRestConfig()
	world, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{Bodies: bodies, Step: cfg})
	require.NoError(t, err)
	state, err := world.NewState(states)
	require.NoError(t, err)
	return doc, world, a, b, state, cfg
}

func TestSpherePairInitialFrictionImpactReplaysRotatingDeparture(t *testing.T) {
	for _, tc := range []struct {
		name                 string
		mu, tangent, speedAY float64
		reverse              bool
	}{
		{name: "sticking", mu: .5, tangent: 20.0 / 7, speedAY: 120.0 / 7},
		{name: "sliding", mu: .01, tangent: .75, speedAY: 19.25},
		{name: "reverse", mu: .5, tangent: 20.0 / 7, speedAY: 120.0 / 7, reverse: true},
	} {
		t.Run(tc.name, func(t *testing.T) {
			mass := exactSphereMass()
			doc, world, a, b, state, cfg := spherePairFrictionFixture(t, tc.mu, .5, 20, tc.reverse,
				mass, mass, zeroAngular(t))
			duration := units.Seconds(.1)
			initialA, _ := state.Body(a)
			initialB, _ := state.Body(b)
			contact, err := doc.ContactPair(t.Context(), a, b, initialA.Pose, initialB.Pose, cfg.Contact)
			require.NoError(t, err)
			require.Equal(t, decad.ContactTouching, contact.Relation)
			require.Len(t, contact.Manifold.Points, 1)
			path := func(entry dynamics.BodyState) decad.RigidDriftSegment {
				return decad.RigidDriftSegment{From: entry.Pose,
					LinearVelocity: entry.LinearVelocity, AngularVelocity: entry.AngularVelocity,
					Duration: duration}
			}
			firstA, firstB := initialA, initialB
			if tc.reverse {
				firstA, firstB = firstB, firstA
			}
			first, err := doc.SweepPair(t.Context(), firstA.Body, firstB.Body,
				path(firstA), path(firstB), decad.SweepRequest{ContactRequest: cfg.Contact,
					TimeResolution: cfg.TimeResolution, MaxPoseEvaluations: cfg.MaxPoseEvaluations})
			require.NoError(t, err)
			require.Equal(t, decad.SweepInitiallyTouching, first.Outcome)
			report, err := world.Step(t.Context(), state, dynamics.StepInput{Gravity: zeroAcceleration()}, duration)
			require.NoError(t, err)
			require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
			require.Len(t, report.Events, 1)
			event := report.Events[0]
			require.Equal(t, dynamics.ContactImpact, event.Kind)
			require.InDelta(t, 75, event.NormalImpulse.Base(), 1e-12)
			require.InDelta(t, tc.tangent, math.Abs(event.TangentImpulse.Y.Base()), 1e-12)
			require.Len(t, event.PointImpulses, 1)
			require.NotNil(t, event.Solver)
			endA, ok := report.Next.Body(a)
			require.True(t, ok)
			endB, ok := report.Next.Body(b)
			require.True(t, ok)
			require.InDelta(t, -25, endA.LinearVelocity.X.Base(), 1e-12)
			require.InDelta(t, 25, endB.LinearVelocity.X.Base(), 1e-12)
			require.InDelta(t, tc.speedAY, endA.LinearVelocity.Y.Base(), 1e-12)
			require.InDelta(t, tc.tangent, endB.LinearVelocity.Y.Base(), 1e-12)
			require.InDelta(t, -tc.tangent/2, endA.AngularVelocity.Z.Base(), 1e-12)
			require.InDelta(t, -tc.tangent/2, endB.AngularVelocity.Z.Base(), 1e-12)
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
			require.InDelta(t, 2700, report.Conservation.Input.KineticEnergy.Value.Base(), 1e-9)
			require.Less(t, report.Conservation.Completion.KineticEnergy.Value.Base(), 2700.0)
		})
	}
}

func TestSpherePairInitialZeroFrictionZeroRestitutionKeepsPersistentTouch(t *testing.T) {
	mass := exactSphereMass()
	doc, world, a, b, state, cfg := spherePairFrictionFixture(t, 0, 0, 0, false,
		mass, mass, zeroAngular(t))
	duration := units.Seconds(.1)
	report, err := world.Step(t.Context(), state, dynamics.StepInput{Gravity: zeroAcceleration()}, duration)
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Len(t, report.Events, 1)
	require.InDelta(t, 50, report.Events[0].NormalImpulse.Base(), 1e-12)
	for _, body := range []*decad.Body{a, b} {
		end, ok := report.Next.Body(body)
		require.True(t, ok)
		require.Zero(t, end.LinearVelocity.X.Base())
	}
	for _, elapsed := range []units.Value{units.Seconds(0), units.Seconds(.05), duration} {
		sample, sampleErr := report.Trace.Sample(elapsed)
		require.NoError(t, sampleErr)
		sa, ok := sample.Body(a)
		require.True(t, ok)
		sb, ok := sample.Body(b)
		require.True(t, ok)
		contact, pairErr := doc.ContactPair(t.Context(), a, b, sa.Pose, sb.Pose, cfg.Contact)
		require.NoError(t, pairErr)
		require.Equal(t, decad.ContactTouching, contact.Relation)
	}
	second, err := world.Step(t.Context(), *report.Next,
		dynamics.StepInput{Gravity: zeroAcceleration()}, duration)
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, second.Status, "%+v", second.Diagnostics)
	require.Empty(t, second.Events)
}

func TestSpherePairInitialZeroFrictionKeepsDensityMassResponse(t *testing.T) {
	doc := decad.New()
	a, b := makeBall(t, doc), makeBall(t, doc)
	density := units.KilogramsPerCubicMillimeter(.001)
	material := dynamics.Material{Restitution: units.Scalar(0), Friction: units.Scalar(0)}
	cfg := sphereRestConfig()
	world, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{
			{Body: a, Role: dynamics.Dynamic, Density: &density, Material: material},
			{Body: b, Role: dynamics.Dynamic, Density: &density, Material: material}},
		Step: cfg})
	require.NoError(t, err)
	poseA, err := r3.Translation(r3.Vec{X: -5})
	require.NoError(t, err)
	poseB, err := r3.Translation(r3.Vec{X: 5})
	require.NoError(t, err)
	state, err := world.NewState([]dynamics.BodyState{
		{Body: a, Pose: poseA, LinearVelocity: sphereRestVelocity(r3.Vec{X: 50}),
			AngularVelocity: zeroAngular(t)},
		{Body: b, Pose: poseB, LinearVelocity: sphereRestVelocity(r3.Vec{X: -50}),
			AngularVelocity: zeroAngular(t)}})
	require.NoError(t, err)
	report, err := world.Step(t.Context(), state, dynamics.StepInput{Gravity: zeroAcceleration()},
		units.Seconds(.1))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Len(t, report.Events, 1)
	sampled, err := report.Trace.Sample(units.Seconds(.05))
	require.NoError(t, err)
	sa, ok := sampled.Body(a)
	require.True(t, ok)
	sb, ok := sampled.Body(b)
	require.True(t, ok)
	contact, err := doc.ContactPair(t.Context(), a, b, sa.Pose, sb.Pose, cfg.Contact)
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, contact.Relation)
	material.Restitution = units.Scalar(.5)
	reboundWorld, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{
			{Body: a, Role: dynamics.Dynamic, Density: &density, Material: material},
			{Body: b, Role: dynamics.Dynamic, Density: &density, Material: material}},
		Step: cfg})
	require.NoError(t, err)
	reboundState, err := reboundWorld.NewState(state.Entries())
	require.NoError(t, err)
	rebound, err := reboundWorld.Step(t.Context(), reboundState,
		dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(.1))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, rebound.Status, "%+v", rebound.Diagnostics)
}

func TestSpherePairInitialFrictionRefusesUnsupportedResponse(t *testing.T) {
	for _, tc := range []struct {
		name string
		mu   float64
		mass decad.MassProperties
		spin dynamics.QuantityVec
	}{
		{name: "zero coefficient uses frictionless response", mu: 0,
			mass: exactSphereMass(), spin: zeroAngular(t)},
		{name: "offset center", mu: .5, mass: func() decad.MassProperties {
			mass := exactSphereMass()
			mass.Center.Value = r3.Vec{Z: 1}
			return mass
		}(), spin: zeroAngular(t)},
		{name: "anisotropic inertia", mu: .5, mass: func() decad.MassProperties {
			mass := exactSphereMass()
			mass.Inertia.ZZ.Value = units.KilogramSquareMillimeters(11)
			return mass
		}(), spin: zeroAngular(t)},
		{name: "incoming spin", mu: .5, mass: exactSphereMass(), spin: dynamics.QuantityVec{
			X: units.RadiansPerSecond(0), Y: units.RadiansPerSecond(0),
			Z: units.RadiansPerSecond(1)}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			massB := exactSphereMass()
			_, world, a, _, state, _ := spherePairFrictionFixture(t, tc.mu, .5, 20, false,
				tc.mass, massB, tc.spin)
			report, err := world.Step(t.Context(), state, dynamics.StepInput{Gravity: zeroAcceleration()},
				units.Seconds(.1))
			require.NoError(t, err)
			if tc.mu == 0 {
				require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
				require.Zero(t, report.Events[0].TangentImpulse.Y.Base())
				endA, ok := report.Next.Body(a)
				require.True(t, ok)
				require.InDelta(t, 20, endA.LinearVelocity.Y.Base(), 1e-12)
				require.Zero(t, endA.AngularVelocity.Z.Base())
				return
			}
			if tc.name == "anisotropic inertia" {
				// The island solves A's I_zz of 11 like any other inertia.
				// The normal impulse is (1 + 0.5)·100/2 = 75 on 1 kg spheres.
				// The contact sticks: 20 − Jt − 25·Jt/11 = Jt + 25·Jt/10, so
				// Jt = 220/74.5, well inside the cone 0.5·75. A turns by
				// −5·Jt/11 and B by −5·Jt/10 about z, and y momentum stays 20.
				require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
				require.Len(t, report.Events, 1)
				event := report.Events[0]
				tangent := 220 / 74.5
				require.InDelta(t, 75, event.NormalImpulse.Base(), 1e-9)
				require.InDelta(t, tangent, event.TangentImpulse.Y.Base(), 1e-9)
				endA, ok := report.Next.Body(a)
				require.True(t, ok)
				require.InDelta(t, -25, endA.LinearVelocity.X.Base(), 1e-9)
				require.InDelta(t, 20-tangent, endA.LinearVelocity.Y.Base(), 1e-9)
				require.InDelta(t, -5*tangent/11, endA.AngularVelocity.Z.Base(), 1e-9)
				require.InDelta(t, -5*tangent/10, event.PostAngularVelocityB.Z.Base(), 1e-9)
				require.InDelta(t, 25, event.PostVelocityB.X.Base(), 1e-9)
				require.InDelta(t, tangent, event.PostVelocityB.Y.Base(), 1e-9)
				return
			}
			require.Equal(t, dynamics.Undecided, report.Status, "%+v", report.Diagnostics)
			require.Nil(t, report.Next)
			if tc.name != "offset center" {
				require.Empty(t, report.Events)
				return
			}
			// §12: the initial impact is certified; the spinning departure that
			// follows it has no certified sweep, so the prefix keeps the impact
			// and stops at the step start.
			require.Len(t, report.Events, 1)
			require.Len(t, report.Diagnostics, 1)
			require.Equal(t, dynamics.StepPairUndecided, report.Diagnostics[0].Code)
			require.Equal(t, units.Seconds(0), report.Diagnostics[0].From)
		})
	}
}

func TestSpherePairInteriorFrictionImpactReplaysBothSides(t *testing.T) {
	doc := decad.New()
	a, b := makeBall(t, doc), makeBall(t, doc)
	poseA, err := r3.Translation(r3.Vec{X: -10})
	require.NoError(t, err)
	poseB, err := r3.Translation(r3.Vec{X: 10, Y: 4})
	require.NoError(t, err)
	velocityA := sphereRestVelocity(r3.Vec{X: 40})
	velocityB := sphereRestVelocity(r3.Vec{X: -40, Y: -32})
	config := sphereRestConfig()
	duration := units.Seconds(.25)
	contact, err := doc.ContactPair(t.Context(), a, b, poseA, poseB, config.Contact)
	require.NoError(t, err)
	require.Equal(t, decad.ContactSeparated, contact.Relation)
	path := func(pose r3.Transform, velocity dynamics.QuantityVec) decad.RigidDriftSegment {
		return decad.RigidDriftSegment{From: pose, LinearVelocity: velocity,
			AngularVelocity: zeroAngular(t), Duration: duration}
	}
	sweep, err := doc.SweepPair(t.Context(), a, b, path(poseA, velocityA), path(poseB, velocityB),
		decad.SweepRequest{ContactRequest: config.Contact, TimeResolution: config.TimeResolution,
			MaxPoseEvaluations: config.MaxPoseEvaluations})
	require.NoError(t, err)
	require.Equal(t, decad.SweepImpactBracket, sweep.Outcome, "cause=%v", sweep.Cause)
	require.Len(t, sweep.Event.Manifold.Points, 1)
	require.InDelta(t, 1, sweep.Event.Manifold.Points[0].Normal.Value.X, 1e-9)
	require.InDelta(t, 0, sweep.Event.Manifold.Points[0].Normal.Value.Y, 1e-9)
	mass := exactSphereMass()
	material := dynamics.Material{Restitution: units.Scalar(.5), Friction: units.Scalar(.25)}
	world, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{Bodies: []dynamics.RigidBody{
		{Body: a, Role: dynamics.Dynamic, Supplied: &mass, Material: material},
		{Body: b, Role: dynamics.Dynamic, Supplied: &mass, Material: material},
	}, Step: config})
	require.NoError(t, err)
	start, err := world.NewState([]dynamics.BodyState{
		{Body: a, Pose: poseA, LinearVelocity: velocityA, AngularVelocity: zeroAngular(t)},
		{Body: b, Pose: poseB, LinearVelocity: velocityB, AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	report, err := world.Step(t.Context(), start,
		dynamics.StepInput{Gravity: zeroAcceleration()}, duration)
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Len(t, report.Events, 1)
	event := report.Events[0]
	// The event lies at its bracket's right sample, at most one
	// TimeResolution after the exact contact time 0.125 s (§5.1).
	require.InDelta(t, .125, event.Time.Base(), config.TimeResolution.Base())
	require.GreaterOrEqual(t, event.Time.Base(), .125)
	// The impulses are read at the bracket's right sample, up to one
	// TimeResolution after the exact contact, so they agree with the exact
	// 60 and 32/7 within ImpulseResidual (docs/multibody-dynamics-design.md
	// §14).
	require.InDelta(t, 60, event.NormalImpulse.Base(), config.ImpulseResidual.Base())
	require.InDelta(t, 32.0/7, event.TangentImpulse.Y.Base(), config.ImpulseResidual.Base())
	require.Len(t, event.PointImpulses, 1)
	require.NotNil(t, event.Solver)
	for _, elapsed := range []units.Value{units.Seconds(.05), event.Time, units.Seconds(.2)} {
		sample, sampleErr := report.Trace.Sample(elapsed)
		require.NoError(t, sampleErr)
		sa, ok := sample.Body(a)
		require.True(t, ok)
		sb, ok := sample.Body(b)
		require.True(t, ok)
		pair, pairErr := doc.ContactPair(t.Context(), a, b, sa.Pose, sb.Pose, config.Contact)
		require.NoError(t, pairErr)
		if elapsed == event.Time {
			// The event's post state is pushed just apart (§6.6): the
			// spheres separate by no more than ContactSlop.
			require.Equal(t, decad.ContactSeparated, pair.Relation)
			require.LessOrEqual(t, pair.Gap.Value.Base()+pair.Gap.Bound.Base(), config.ContactSlop.Base())
		} else {
			require.Equal(t, decad.ContactSeparated, pair.Relation)
		}
	}
	endA, ok := report.Next.Body(a)
	require.True(t, ok)
	endB, ok := report.Next.Body(b)
	require.True(t, ok)
	// The velocities follow from the bracket-right impulses, so they agree
	// with the exact values within VelocityResidual and
	// AngularVelocityResidual (§14).
	velocity, spin := config.VelocityResidual.Base(), config.AngularVelocityResidual.Base()
	require.InDelta(t, -20, endA.LinearVelocity.X.Base(), velocity)
	require.InDelta(t, 20, endB.LinearVelocity.X.Base(), velocity)
	require.InDelta(t, -32.0/7, endA.LinearVelocity.Y.Base(), velocity)
	require.InDelta(t, -192.0/7, endB.LinearVelocity.Y.Base(), velocity)
	require.InDelta(t, -16.0/7, endA.AngularVelocity.Z.Base(), spin)
	require.InDelta(t, -16.0/7, endB.AngularVelocity.Z.Base(), spin)
	require.NotNil(t, report.Conservation)
}
