package dynamics_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/dynamics"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestTwoDynamicBoxStackRestsThroughTwoGravitySteps(t *testing.T) {
	doc := decad.New()
	floor := makeBox(t, doc, -20, -20, 20, 20, -10, 10)
	lower := makeBox(t, doc, -5, -5, 5, 5, 0, 10)
	upper := makeBox(t, doc, -5, -5, 5, 5, 10, 10)
	original := doc.Bodies()
	req := decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
		NormalResolution: units.Radians(1e-6)}
	for _, pair := range []struct {
		a, b   *decad.Body
		want   decad.ContactRelation
		points int
	}{{floor, lower, decad.ContactTouching, 4}, {lower, upper, decad.ContactTouching, 4},
		{floor, upper, decad.ContactSeparated, 0}} {
		contact, err := doc.ContactPair(t.Context(), pair.a, pair.b, r3.Identity(), r3.Identity(), req)
		require.NoError(t, err)
		require.Equal(t, pair.want, contact.Relation)
		if pair.points > 0 {
			require.Len(t, contact.Manifold.Points, pair.points)
		}
	}
	still := decad.PoseSegment{From: r3.Identity(), To: r3.Identity(), Duration: units.Seconds(.05)}
	fall := zeroVelocity()
	fall.Z = units.MillimetersPerSecond(-50)
	drift := decad.RigidDriftSegment{From: r3.Identity(), LinearVelocity: fall,
		AngularVelocity: zeroAngular(t), Duration: units.Seconds(.05)}
	for _, pair := range []struct {
		a, b   *decad.Body
		pa, pb decad.PairPath
		policy decad.SweepStartPolicy
		want   decad.SweepOutcome
	}{{floor, lower, still, drift, decad.StopAtInitialContact, decad.SweepInitiallyTouching},
		{lower, upper, drift, drift, decad.ContinueCertifiedTouch, decad.SweepPersistentTouch},
		{floor, upper, still, drift, decad.StopAtInitialContact, decad.SweepClear}} {
		sweep, err := doc.SweepPair(t.Context(), pair.a, pair.b, pair.pa, pair.pb,
			decad.SweepRequest{ContactRequest: req, TimeResolution: units.Seconds(1e-9),
				MaxPoseEvaluations: 128, StartPolicy: pair.policy})
		require.NoError(t, err)
		require.Equal(t, pair.want, sweep.Outcome, "cause=%v", sweep.Cause)
	}
	density := units.KilogramsPerCubicMillimeter(.001)
	material := dynamics.Material{Restitution: units.Scalar(0), Friction: units.Scalar(0)}
	step := dynamics.StepConfig{Contact: req, TimeResolution: units.Seconds(1e-9),
		ContactSlop: units.Millimeters(1e-6), VelocityResidual: units.MillimetersPerSecond(1e-6),
		AngularVelocityResidual: units.RadiansPerSecond(1e-6),
		ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
		PenetrationResidual:     units.Millimeters(1e-6), ImpactSpeed: units.MillimetersPerSecond(0),
		MaxPoseEvaluations: 128, MaxIterations: 8, MaxEvents: 3, MaxPairSweeps: 4096}
	gravity := zeroAcceleration()
	gravity.Z = units.MillimetersPerSecondSquared(-1000)
	for _, bodies := range [][]dynamics.RigidBody{
		{{Body: floor, Role: dynamics.Fixed, Material: material},
			{Body: lower, Role: dynamics.Dynamic, Density: &density, Material: material},
			{Body: upper, Role: dynamics.Dynamic, Density: &density, Material: material}},
		{{Body: upper, Role: dynamics.Dynamic, Density: &density, Material: material},
			{Body: lower, Role: dynamics.Dynamic, Density: &density, Material: material},
			{Body: floor, Role: dynamics.Fixed, Material: material}},
	} {
		world, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{Bodies: bodies, Step: step})
		require.NoError(t, err)
		state, err := world.NewState([]dynamics.BodyState{
			{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
			{Body: lower, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
			{Body: upper, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		})
		require.NoError(t, err)
		for range 2 {
			report, stepErr := world.Step(t.Context(), state,
				dynamics.StepInput{Gravity: gravity}, units.Seconds(.05))
			require.NoError(t, stepErr)
			require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
			require.Len(t, report.Events, 2)
			require.NotNil(t, report.Conservation)
			require.InDelta(t, 100, report.Conservation.ContactImpulse.Value.Z.Base(), 1e-6)
			require.InDelta(t, -100, report.Conservation.GravityImpulse.Value.Z.Base(), 1e-6)
			for _, body := range []*decad.Body{lower, upper} {
				entry, found := report.Next.Body(body)
				require.True(t, found)
				require.Equal(t, r3.Identity(), entry.Pose)
				require.Equal(t, zeroVelocity(), entry.LinearVelocity)
			}
			for _, at := range []float64{0, .025, .05} {
				sampled, sampleErr := report.Trace.Sample(units.Seconds(at))
				require.NoError(t, sampleErr)
				for _, body := range []*decad.Body{lower, upper} {
					entry, found := sampled.Body(body)
					require.True(t, found)
					require.Equal(t, r3.Identity(), entry.Pose)
					require.Equal(t, zeroVelocity(), entry.LinearVelocity)
				}
			}
			state = *report.Next
		}
		limited := step
		limited.MaxEvents = 2
		limitedWorld, err := dynamics.NewWorld(t.Context(), doc,
			dynamics.WorldConfig{Bodies: bodies, Step: limited})
		require.NoError(t, err)
		limitedState, err := limitedWorld.NewState(state.Entries())
		require.NoError(t, err)
		refused, err := limitedWorld.Step(t.Context(), limitedState,
			dynamics.StepInput{Gravity: gravity}, units.Seconds(.05))
		require.NoError(t, err)
		require.Equal(t, dynamics.Undecided, refused.Status)
		require.Nil(t, refused.Next)
		lateral := state.Entries()
		for i := range lateral {
			if lateral[i].Body == lower {
				lateral[i].LinearVelocity.X = units.MillimetersPerSecond(10)
			}
		}
		lateralState, err := world.NewState(lateral)
		require.NoError(t, err)
		refused, err = world.Step(t.Context(), lateralState,
			dynamics.StepInput{Gravity: gravity}, units.Seconds(.05))
		require.NoError(t, err)
		require.Equal(t, dynamics.Undecided, refused.Status)
		require.Nil(t, refused.Next)
		bouncingBodies := append([]dynamics.RigidBody(nil), bodies...)
		for i := range bouncingBodies {
			bouncingBodies[i].Material.Restitution = units.Scalar(.5)
		}
		bouncingWorld, err := dynamics.NewWorld(t.Context(), doc,
			dynamics.WorldConfig{Bodies: bouncingBodies, Step: step})
		require.NoError(t, err)
		bouncingState, err := bouncingWorld.NewState(state.Entries())
		require.NoError(t, err)
		refused, err = bouncingWorld.Step(t.Context(), bouncingState,
			dynamics.StepInput{Gravity: gravity}, units.Seconds(.05))
		require.NoError(t, err)
		require.Equal(t, dynamics.Undecided, refused.Status)
		require.Nil(t, refused.Next)
	}
	require.Equal(t, original, doc.Bodies())
}

func TestTwoDynamicBoxStackUsesBothMasses(t *testing.T) {
	doc := decad.New()
	floor := makeBox(t, doc, -20, -20, 20, 20, -10, 10)
	lower := makeBox(t, doc, -5, -5, 5, 5, 0, 10)
	upper := makeBox(t, doc, -5, -5, 5, 5, 10, 10)
	lowerDensity := units.KilogramsPerCubicMillimeter(.001)
	upperDensity := units.KilogramsPerCubicMillimeter(.002)
	material := dynamics.Material{Restitution: units.Scalar(0), Friction: units.Scalar(0)}
	step := dynamics.StepConfig{
		Contact: decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
			NormalResolution: units.Radians(1e-6)},
		TimeResolution: units.Seconds(1e-9), ContactSlop: units.Millimeters(1e-6),
		VelocityResidual:        units.MillimetersPerSecond(1e-6),
		AngularVelocityResidual: units.RadiansPerSecond(1e-6),
		ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
		PenetrationResidual:     units.Millimeters(1e-6), ImpactSpeed: units.MillimetersPerSecond(0),
		MaxPoseEvaluations: 128, MaxIterations: 8, MaxEvents: 3, MaxPairSweeps: 4096,
	}
	world, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{Bodies: []dynamics.RigidBody{
		{Body: floor, Role: dynamics.Fixed, Material: material},
		{Body: lower, Role: dynamics.Dynamic, Density: &lowerDensity, Material: material},
		{Body: upper, Role: dynamics.Dynamic, Density: &upperDensity, Material: material},
	}, Step: step})
	require.NoError(t, err)
	state, err := world.NewState([]dynamics.BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: lower, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: upper, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	gravity := zeroAcceleration()
	gravity.Z = units.MillimetersPerSecondSquared(-1000)
	report, err := world.Step(t.Context(), state, dynamics.StepInput{Gravity: gravity}, units.Seconds(.05))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Len(t, report.Events, 2)
	require.Equal(t, dynamics.BodyPair{A: floor, B: lower}, report.Events[0].Pair)
	require.Equal(t, dynamics.BodyPair{A: lower, B: upper}, report.Events[1].Pair)
	require.InDelta(t, 150, report.Events[0].NormalImpulse.Base(), 1e-6)
	require.InDelta(t, 100, report.Events[1].NormalImpulse.Base(), 1e-6)
	require.InDelta(t, 150, report.Conservation.ContactImpulse.Value.Z.Base(), 1e-6)
	for _, body := range []*decad.Body{lower, upper} {
		entry, found := report.Next.Body(body)
		require.True(t, found)
		require.Equal(t, zeroVelocity(), entry.LinearVelocity)
	}
	replayed, err := report.Trace.Sample(units.Seconds(.025))
	require.NoError(t, err)
	require.Len(t, replayed.Entries(), 3)
}
