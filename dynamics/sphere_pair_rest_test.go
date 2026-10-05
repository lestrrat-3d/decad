package dynamics_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/dynamics"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func sphereRestVelocity(v r3.Vec) dynamics.QuantityVec {
	return dynamics.QuantityVec{X: units.MillimetersPerSecond(v.X),
		Y: units.MillimetersPerSecond(v.Y), Z: units.MillimetersPerSecond(v.Z)}
}

func sphereRestConfig() dynamics.StepConfig {
	return dynamics.StepConfig{
		Contact: decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
			NormalResolution: units.Radians(1e-6)},
		TimeResolution: units.Seconds(1e-9), ContactSlop: units.Millimeters(1e-6),
		VelocityResidual:        units.MillimetersPerSecond(1e-6),
		AngularVelocityResidual: units.RadiansPerSecond(1e-6),
		ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
		PenetrationResidual:     units.Millimeters(1e-6),
		ImpactSpeed:             units.MillimetersPerSecond(0), MaxPoseEvaluations: 128,
		MaxIterations: 8, MaxEvents: 2, MaxPairSweeps: 4096,
	}
}

func TestSpherePairInitialZeroRestitutionRest(t *testing.T) {
	doc := decad.New()
	a, b := makeBall(t, doc), makeBall(t, doc)
	poseB, err := r3.Translation(r3.Vec{X: 6, Y: 8})
	require.NoError(t, err)
	config := sphereRestConfig()
	contact, err := doc.ContactPair(t.Context(), a, b, r3.Identity(), poseB, config.Contact)
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, contact.Relation)
	require.NotNil(t, contact.Manifold)
	require.Len(t, contact.Manifold.Points, 1)
	duration := units.Seconds(.1)
	path := func(pose r3.Transform, velocity r3.Vec) decad.RigidDriftSegment {
		return decad.RigidDriftSegment{From: pose, LinearVelocity: sphereRestVelocity(velocity),
			AngularVelocity: zeroAngular(t), Duration: duration}
	}
	initialSweep, err := doc.SweepPair(t.Context(), a, b,
		path(r3.Identity(), r3.Vec{X: 6, Y: 8}), path(poseB, r3.Vec{X: -6, Y: -8}),
		decad.SweepRequest{ContactRequest: config.Contact, TimeResolution: config.TimeResolution,
			MaxPoseEvaluations: config.MaxPoseEvaluations})
	require.NoError(t, err)
	require.Equal(t, decad.SweepInitiallyTouching, initialSweep.Outcome)
	restSweep, err := doc.SweepPair(t.Context(), a, b,
		path(r3.Identity(), r3.Vec{}), path(poseB, r3.Vec{}),
		decad.SweepRequest{ContactRequest: config.Contact, TimeResolution: config.TimeResolution,
			MaxPoseEvaluations: config.MaxPoseEvaluations, StartPolicy: decad.ContinueCertifiedTouch})
	require.NoError(t, err)
	require.Equal(t, decad.SweepPersistentTouch, restSweep.Outcome)
	mass := exactSphereMass()
	material := dynamics.Material{Restitution: units.Scalar(0), Friction: units.Scalar(0)}
	world, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{
			{Body: a, Role: dynamics.Dynamic, Supplied: &mass, Material: material},
			{Body: b, Role: dynamics.Dynamic, Supplied: &mass, Material: material},
		}, Step: config,
	})
	require.NoError(t, err)
	state, err := world.NewState([]dynamics.BodyState{
		{Body: a, Pose: r3.Identity(), LinearVelocity: sphereRestVelocity(r3.Vec{X: 6, Y: 8}),
			AngularVelocity: zeroAngular(t)},
		{Body: b, Pose: poseB, LinearVelocity: sphereRestVelocity(r3.Vec{X: -6, Y: -8}),
			AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	step, err := world.Step(t.Context(), state, dynamics.StepInput{Gravity: zeroAcceleration()}, duration)
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, step.Status, "%+v", step.Diagnostics)
	require.Len(t, step.Events, 1)
	require.InDelta(t, 10, step.Events[0].NormalImpulse.Base(), 1e-9)
	for _, body := range []*decad.Body{a, b} {
		end, ok := step.Next.Body(body)
		require.True(t, ok)
		require.Equal(t, zeroVelocity(), end.LinearVelocity)
	}
	for _, elapsed := range []units.Value{units.Seconds(0), units.Seconds(.05), duration} {
		sample, sampleErr := step.Trace.Sample(elapsed)
		require.NoError(t, sampleErr)
		sa, ok := sample.Body(a)
		require.True(t, ok)
		sb, ok := sample.Body(b)
		require.True(t, ok)
		inside, pairErr := doc.ContactPair(t.Context(), a, b, sa.Pose, sb.Pose, config.Contact)
		require.NoError(t, pairErr)
		require.Equal(t, decad.ContactTouching, inside.Relation)
	}
	second, err := world.Step(t.Context(), *step.Next, dynamics.StepInput{Gravity: zeroAcceleration()}, duration)
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, second.Status, "%+v", second.Diagnostics)
	require.Empty(t, second.Events)
}

func spherePairRestFixture(t *testing.T, velocityA, velocityB r3.Vec,
	massA, massB decad.MassProperties, reverse bool) (*decad.Document, *dynamics.World,
	*decad.Body, *decad.Body, r3.Transform, dynamics.State, dynamics.StepConfig) {
	t.Helper()
	doc := decad.New()
	a, b := makeBall(t, doc), makeBall(t, doc)
	poseB, err := r3.Translation(r3.Vec{X: 6, Y: 8})
	require.NoError(t, err)
	material := dynamics.Material{Restitution: units.Scalar(0), Friction: units.Scalar(0)}
	definitions := []dynamics.RigidBody{
		{Body: a, Role: dynamics.Dynamic, Supplied: &massA, Material: material},
		{Body: b, Role: dynamics.Dynamic, Supplied: &massB, Material: material},
	}
	entries := []dynamics.BodyState{
		{Body: a, Pose: r3.Identity(), LinearVelocity: sphereRestVelocity(velocityA),
			AngularVelocity: zeroAngular(t)},
		{Body: b, Pose: poseB, LinearVelocity: sphereRestVelocity(velocityB),
			AngularVelocity: zeroAngular(t)},
	}
	if reverse {
		definitions[0], definitions[1] = definitions[1], definitions[0]
		entries[0], entries[1] = entries[1], entries[0]
	}
	config := sphereRestConfig()
	world, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{Bodies: definitions, Step: config})
	require.NoError(t, err)
	state, err := world.NewState(entries)
	require.NoError(t, err)
	return doc, world, a, b, poseB, state, config
}

func TestSpherePairInitialZeroRestitutionMovingRest(t *testing.T) {
	mass := exactSphereMass()
	doc, world, a, b, poseB, state, config := spherePairRestFixture(t,
		r3.Vec{X: 7, Y: 8}, r3.Vec{X: -5, Y: -8}, mass, mass, false)
	duration := units.Seconds(.125)
	shared := sphereRestVelocity(r3.Vec{X: 1})
	path := func(pose r3.Transform) decad.RigidDriftSegment {
		return decad.RigidDriftSegment{From: pose, LinearVelocity: shared,
			AngularVelocity: zeroAngular(t), Duration: duration}
	}
	sweep, err := doc.SweepPair(t.Context(), a, b, path(r3.Identity()), path(poseB),
		decad.SweepRequest{ContactRequest: config.Contact, TimeResolution: config.TimeResolution,
			MaxPoseEvaluations: config.MaxPoseEvaluations, StartPolicy: decad.ContinueCertifiedTouch})
	require.NoError(t, err)
	require.Equal(t, decad.SweepPersistentTouch, sweep.Outcome, "cause=%v", sweep.Cause)
	for _, fraction := range []units.Value{units.Scalar(0), units.Scalar(.5), units.Scalar(1)} {
		manifold, manifoldErr := sweep.ContactTrack.ManifoldAt(fraction)
		require.NoError(t, manifoldErr)
		require.Len(t, manifold.Points, 1)
		require.InDelta(t, 3+.125*fraction.Base(), manifold.Points[0].OnA.Value.X, 1e-12)
	}
	step, err := world.Step(t.Context(), state, dynamics.StepInput{Gravity: zeroAcceleration()}, duration)
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, step.Status, "%+v", step.Diagnostics)
	require.Len(t, step.Events, 1)
	for _, body := range []*decad.Body{a, b} {
		end, ok := step.Next.Body(body)
		require.True(t, ok)
		require.Equal(t, shared, end.LinearVelocity)
	}
	for _, elapsed := range []units.Value{units.Seconds(0), units.Seconds(.0625), duration} {
		sampled, sampleErr := step.Trace.Sample(elapsed)
		require.NoError(t, sampleErr)
		sa, ok := sampled.Body(a)
		require.True(t, ok)
		sb, ok := sampled.Body(b)
		require.True(t, ok)
		pair, pairErr := doc.ContactPair(t.Context(), a, b, sa.Pose, sb.Pose, config.Contact)
		require.NoError(t, pairErr)
		require.Equal(t, decad.ContactTouching, pair.Relation)
	}
	second, err := world.Step(t.Context(), *step.Next, dynamics.StepInput{Gravity: zeroAcceleration()}, duration)
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, second.Status, "%+v", second.Diagnostics)
	require.Empty(t, second.Events)
}

func TestSpherePairInitialZeroRestitutionTangentialDeparture(t *testing.T) {
	mass := exactSphereMass()
	for _, reverse := range []bool{false, true} {
		t.Run(map[bool]string{false: "forward", true: "reverse"}[reverse], func(t *testing.T) {
			doc, world, a, b, _, state, config := spherePairRestFixture(t,
				r3.Vec{X: 10}, r3.Vec{}, mass, mass, reverse)
			duration := units.Seconds(.1)
			step, err := world.Step(t.Context(), state, dynamics.StepInput{Gravity: zeroAcceleration()}, duration)
			require.NoError(t, err)
			require.Equal(t, dynamics.Advanced, step.Status, "%+v", step.Diagnostics)
			require.Len(t, step.Events, 1)
			require.InDelta(t, 3, step.Events[0].NormalImpulse.Base(), 1e-9)
			if reverse {
				require.Less(t, step.Events[0].Manifold.Points[0].Normal.Value.X, 0.0)
			} else {
				require.Greater(t, step.Events[0].Manifold.Points[0].Normal.Value.X, 0.0)
			}
			endA, ok := step.Next.Body(a)
			require.True(t, ok)
			endB, ok := step.Next.Body(b)
			require.True(t, ok)
			require.InDelta(t, 8.2, endA.LinearVelocity.X.Base(), 1e-12)
			require.InDelta(t, -2.4, endA.LinearVelocity.Y.Base(), 1e-12)
			require.InDelta(t, 1.8, endB.LinearVelocity.X.Base(), 1e-12)
			require.InDelta(t, 2.4, endB.LinearVelocity.Y.Base(), 1e-12)
			for _, elapsed := range []units.Value{units.Seconds(0), units.Seconds(.05), duration} {
				sampled, sampleErr := step.Trace.Sample(elapsed)
				require.NoError(t, sampleErr)
				sa, ok := sampled.Body(a)
				require.True(t, ok)
				sb, ok := sampled.Body(b)
				require.True(t, ok)
				pair, pairErr := doc.ContactPair(t.Context(), a, b, sa.Pose, sb.Pose, config.Contact)
				require.NoError(t, pairErr)
				if elapsed.Base() == 0 {
					require.Equal(t, decad.ContactTouching, pair.Relation)
				} else {
					require.Equal(t, decad.ContactSeparated, pair.Relation)
				}
			}
		})
	}
}

func TestSpherePairInitialZeroRestitutionRefusesUnboundedSpin(t *testing.T) {
	massA, massB := exactSphereMass(), exactSphereMass()
	massA.Center.Value = r3.Vec{Z: 2}
	doc, world, a, b, _, state, _ := spherePairRestFixture(t,
		r3.Vec{X: 6, Y: 8}, r3.Vec{X: -6, Y: -8}, massA, massB, false)
	require.NotNil(t, doc)
	require.NotNil(t, a)
	require.NotNil(t, b)
	step, err := world.Step(t.Context(), state, dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(.1))
	require.NoError(t, err)
	require.Equal(t, dynamics.Undecided, step.Status)
	require.Nil(t, step.Next)
	// §12: the initial impact is certified; the spinning departure that
	// follows it has no certified sweep, so the prefix keeps the impact and
	// stops at the step start.
	require.Len(t, step.Events, 1)
	require.Len(t, step.Diagnostics, 1)
	require.Equal(t, dynamics.StepPairUndecided, step.Diagnostics[0].Code)
	require.Equal(t, units.Seconds(0), step.Diagnostics[0].From)
}

func TestSpherePairInteriorZeroRestitutionRest(t *testing.T) {
	for _, reverse := range []bool{false, true} {
		t.Run(map[bool]string{false: "forward", true: "reverse"}[reverse], func(t *testing.T) {
			doc := decad.New()
			a, b := makeBall(t, doc), makeBall(t, doc)
			poseA, err := r3.Translation(r3.Vec{X: -6, Y: -8})
			require.NoError(t, err)
			poseB, err := r3.Translation(r3.Vec{X: 6, Y: 8})
			require.NoError(t, err)
			velA, velB := sphereRestVelocity(r3.Vec{X: 30, Y: 40}),
				sphereRestVelocity(r3.Vec{X: -30, Y: -40})
			config := sphereRestConfig()
			initial, err := doc.ContactPair(t.Context(), a, b, poseA, poseB, config.Contact)
			require.NoError(t, err)
			require.Equal(t, decad.ContactSeparated, initial.Relation)
			touchA, err := r3.Translation(r3.Vec{X: -3, Y: -4})
			require.NoError(t, err)
			touchB, err := r3.Translation(r3.Vec{X: 3, Y: 4})
			require.NoError(t, err)
			contact, err := doc.ContactPair(t.Context(), a, b, touchA, touchB, config.Contact)
			require.NoError(t, err)
			require.Equal(t, decad.ContactTouching, contact.Relation)
			duration := units.Seconds(.2)
			path := func(pose r3.Transform, velocity dynamics.QuantityVec) decad.RigidDriftSegment {
				return decad.RigidDriftSegment{From: pose, LinearVelocity: velocity,
					AngularVelocity: zeroAngular(t), Duration: duration}
			}
			sweep, err := doc.SweepPair(t.Context(), a, b, path(poseA, velA), path(poseB, velB),
				decad.SweepRequest{ContactRequest: config.Contact, TimeResolution: config.TimeResolution,
					MaxPoseEvaluations: config.MaxPoseEvaluations})
			require.NoError(t, err)
			require.Equal(t, decad.SweepImpactBracket, sweep.Outcome)
			require.NotNil(t, sweep.Bracket)
			require.InDelta(t, .5, sweep.Bracket.To.Fraction.Base(), 1e-8)
			mass := exactSphereMass()
			material := dynamics.Material{Restitution: units.Scalar(0), Friction: units.Scalar(0)}
			definitions := []dynamics.RigidBody{
				{Body: a, Role: dynamics.Dynamic, Supplied: &mass, Material: material},
				{Body: b, Role: dynamics.Dynamic, Supplied: &mass, Material: material},
			}
			entries := []dynamics.BodyState{
				{Body: a, Pose: poseA, LinearVelocity: velA, AngularVelocity: zeroAngular(t)},
				{Body: b, Pose: poseB, LinearVelocity: velB, AngularVelocity: zeroAngular(t)},
			}
			if reverse {
				definitions[0], definitions[1] = definitions[1], definitions[0]
				entries[0], entries[1] = entries[1], entries[0]
			}
			world, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
				Bodies: definitions, Step: config})
			require.NoError(t, err)
			state, err := world.NewState(entries)
			require.NoError(t, err)
			step, err := world.Step(t.Context(), state,
				dynamics.StepInput{Gravity: zeroAcceleration()}, duration)
			require.NoError(t, err)
			require.Equal(t, dynamics.Advanced, step.Status, "%+v", step.Diagnostics)
			require.Len(t, step.Events, 1)
			require.InDelta(t, .1, step.Events[0].Time.Base(), 1e-8)
			require.InDelta(t, 50, step.Events[0].NormalImpulse.Base(), 1e-6)
			for _, body := range []*decad.Body{a, b} {
				end, ok := step.Next.Body(body)
				require.True(t, ok)
				require.InDelta(t, 0, end.LinearVelocity.X.Base(), 1e-12)
				require.InDelta(t, 0, end.LinearVelocity.Y.Base(), 1e-12)
			}
			for _, elapsed := range []units.Value{units.Seconds(.05), units.Seconds(.15), duration} {
				sample, sampleErr := step.Trace.Sample(elapsed)
				require.NoError(t, sampleErr)
				sa, ok := sample.Body(a)
				require.True(t, ok)
				sb, ok := sample.Body(b)
				require.True(t, ok)
				pair, pairErr := doc.ContactPair(t.Context(), a, b, sa.Pose, sb.Pose, config.Contact)
				require.NoError(t, pairErr)
				if elapsed.Base() < step.Events[0].Time.Base() {
					require.Equal(t, decad.ContactSeparated, pair.Relation)
					continue
				}
				// §6.6 corrects the resting pair into exact touch, which it
				// keeps while both spheres stand still.
				require.Equal(t, decad.ContactTouching, pair.Relation)
			}
		})
	}
}
