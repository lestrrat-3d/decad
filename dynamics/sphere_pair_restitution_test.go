package dynamics_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/dynamics"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestSpherePairInitialCardinalRestitution(t *testing.T) {
	for _, tc := range []struct {
		name        string
		reverse     bool
		densityMass bool
		restitution float64
		massRatio   float64
		incomingB   float64
	}{
		{name: "density source order", densityMass: true, restitution: .5},
		{name: "density reverse order", reverse: true, densityMass: true, restitution: .5},
		{name: "supplied source order", restitution: .5},
		{name: "supplied reverse order", reverse: true, restitution: .5},
		{name: "density resting", densityMass: true},
		{name: "unequal density resting", densityMass: true, massRatio: 2, incomingB: -25},
		{name: "unequal density resting reverse", reverse: true, densityMass: true, massRatio: 2, incomingB: -25},
	} {
		t.Run(tc.name, func(t *testing.T) {
			doc := decad.New()
			a, b := makeBall(t, doc), makeBall(t, doc)
			poseA := r3.Identity()
			poseB, err := r3.Translation(r3.Vec{X: 10})
			require.NoError(t, err)
			cfg := sphereRestConfig()
			contact, err := doc.ContactPair(t.Context(), a, b, poseA, poseB, cfg.Contact)
			require.NoError(t, err)
			require.Equal(t, decad.ContactTouching, contact.Relation)
			incomingB := -50.0
			if tc.incomingB != 0 {
				incomingB = tc.incomingB
			}
			velocityA, velocityB := sphereRestVelocity(r3.Vec{X: 50}),
				sphereRestVelocity(r3.Vec{X: incomingB})
			period := units.Seconds(.1)
			path := func(pose r3.Transform, velocity dynamics.QuantityVec) decad.RigidDriftSegment {
				return decad.RigidDriftSegment{From: pose, LinearVelocity: velocity,
					AngularVelocity: zeroAngular(t), Duration: period}
			}
			sweep, err := doc.SweepPair(t.Context(), a, b,
				path(poseA, velocityA), path(poseB, velocityB),
				decad.SweepRequest{ContactRequest: cfg.Contact, TimeResolution: cfg.TimeResolution,
					MaxPoseEvaluations: cfg.MaxPoseEvaluations})
			require.NoError(t, err)
			require.Equal(t, decad.SweepInitiallyTouching, sweep.Outcome)
			densityA := units.KilogramsPerCubicMillimeter(.001)
			densityB := densityA
			if tc.massRatio > 0 {
				densityB = units.KilogramsPerCubicMillimeter(.001 * tc.massRatio)
			}
			massA, err := a.MassProperties(t.Context(), densityA)
			require.NoError(t, err)
			massB, err := b.MassProperties(t.Context(), densityB)
			require.NoError(t, err)
			require.Positive(t, massA.Mass.Bound.Base())
			require.Positive(t, massB.Mass.Bound.Base())
			material := dynamics.Material{Restitution: units.Scalar(tc.restitution), Friction: units.Scalar(0)}
			bodies := []dynamics.RigidBody{
				{Body: a, Role: dynamics.Dynamic, Material: material},
				{Body: b, Role: dynamics.Dynamic, Material: material},
			}
			if tc.densityMass {
				bodies[0].Density, bodies[1].Density = &densityA, &densityB
			} else {
				bodies[0].Supplied, bodies[1].Supplied = &massA, &massB
			}
			states := []dynamics.BodyState{
				{Body: a, Pose: poseA, LinearVelocity: velocityA, AngularVelocity: zeroAngular(t)},
				{Body: b, Pose: poseB, LinearVelocity: velocityB, AngularVelocity: zeroAngular(t)},
			}
			if tc.reverse {
				bodies[0], bodies[1] = bodies[1], bodies[0]
				states[0], states[1] = states[1], states[0]
			}
			world, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{Bodies: bodies, Step: cfg})
			require.NoError(t, err)
			start, err := world.NewState(states)
			require.NoError(t, err)
			step, err := world.Step(t.Context(), start,
				dynamics.StepInput{Gravity: zeroAcceleration()}, period)
			require.NoError(t, err)
			require.Equal(t, dynamics.Advanced, step.Status, "%+v", step.Diagnostics)
			require.Len(t, step.Events, 1)
			require.Equal(t, units.Seconds(0), step.Events[0].Time)
			mA, mB := massA.Mass.Value.Base(), massB.Mass.Value.Base()
			expectedImpulse := (50 - incomingB) * (1 + tc.restitution) * mA * mB / (mA + mB)
			expectedA := 50 - expectedImpulse/mA
			expectedB := incomingB + expectedImpulse/mB
			require.InDelta(t, expectedImpulse, step.Events[0].NormalImpulse.Base(), 1e-5)
			for _, sampleAt := range []units.Value{units.Seconds(0), units.Seconds(.05), period} {
				sample, sampleErr := step.Trace.Sample(sampleAt)
				require.NoError(t, sampleErr)
				sa, found := sample.Body(a)
				require.True(t, found)
				sb, found := sample.Body(b)
				require.True(t, found)
				require.InDelta(t, expectedA, sa.LinearVelocity.X.Base(), 1e-6)
				require.InDelta(t, expectedB, sb.LinearVelocity.X.Base(), 1e-6)
				if tc.restitution == 0 {
					require.Equal(t, sa.LinearVelocity.X, sb.LinearVelocity.X)
				}
			}
			require.NotNil(t, step.Conservation)
			require.InDelta(t, 50*mA+incomingB*mB,
				step.Conservation.Completion.LinearMomentum.Value.X.Base(), 1e-6)
			require.InDelta(t, .5*(mA*expectedA*expectedA+mB*expectedB*expectedB),
				step.Conservation.Completion.KineticEnergy.Value.Base(), 1e-4)
			second, err := world.Step(t.Context(), *step.Next,
				dynamics.StepInput{Gravity: zeroAcceleration()}, period)
			require.NoError(t, err)
			require.Equal(t, dynamics.Advanced, second.Status, "%+v", second.Diagnostics)
			require.Empty(t, second.Events)
		})
	}
}
