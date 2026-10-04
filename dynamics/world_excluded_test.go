package dynamics_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/dynamics"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestExcludedPairCrossesAfterForceKick(t *testing.T) {
	doc := decad.New()
	a := makeBox(t, doc, 0, 0, 10, 10, 0, 10)
	b := makeBox(t, doc, 20, 0, 30, 10, 0, 10)
	density := units.KilogramsPerCubicMillimeter(.001)
	material := dynamics.Material{Restitution: units.Scalar(0), Friction: units.Scalar(0)}
	w, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{
			{Body: a, Role: dynamics.Dynamic, Density: &density, Material: material},
			{Body: b, Role: dynamics.Dynamic, Density: &density, Material: material},
		}, Excluded: []dynamics.BodyPair{{A: a, B: b}}, Step: pairMaterialStepConfig(),
	})
	require.NoError(t, err)
	start, err := w.NewState([]dynamics.BodyState{
		{Body: a, Pose: r3.Identity(), LinearVelocity: dynamics.QuantityVec{
			X: units.MillimetersPerSecond(100), Y: units.MillimetersPerSecond(0),
			Z: units.MillimetersPerSecond(0),
		}, AngularVelocity: zeroAngular(t)},
		{Body: b, Pose: r3.Identity(), LinearVelocity: dynamics.QuantityVec{
			X: units.MillimetersPerSecond(-100), Y: units.MillimetersPerSecond(0),
			Z: units.MillimetersPerSecond(0),
		}, AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	input := dynamics.StepInput{Gravity: zeroAcceleration(), Loads: []dynamics.BodyLoad{{
		Body: a, Force: testForce(10, 0), Torque: testTorque(0),
	}}}
	report, err := w.Step(t.Context(), start, input, units.Seconds(.2))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Empty(t, report.Events)
	finalA, ok := report.Next.Body(a)
	require.True(t, ok)
	finalB, ok := report.Next.Body(b)
	require.True(t, ok)
	require.InDelta(t, 20.4, finalA.Pose.Translation().X, 1e-10)
	require.InDelta(t, -20, finalB.Pose.Translation().X, 1e-10)
	require.InDelta(t, 102, finalA.LinearVelocity.X.Base(), 1e-10)
	require.InDelta(t, 2, report.Conservation.LoadImpulse.Value.X.Base(), 1e-10)
	require.Zero(t, report.Conservation.ContactImpulse.Value.X.Base())
}

func TestExcludedKinematicPairCrossesWithoutContact(t *testing.T) {
	doc := decad.New()
	a := makeBox(t, doc, 0, 0, 10, 10, 0, 10)
	b := makeBox(t, doc, 20, 0, 30, 10, 0, 10)
	density := units.KilogramsPerCubicMillimeter(.001)
	material := dynamics.Material{Restitution: units.Scalar(0), Friction: units.Scalar(0)}
	w, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{
			{Body: a, Role: dynamics.Kinematic, Material: material},
			{Body: b, Role: dynamics.Dynamic, Density: &density, Material: material},
		}, Excluded: []dynamics.BodyPair{{A: b, B: a}}, Step: pairMaterialStepConfig(),
	})
	require.NoError(t, err)
	start, err := w.NewState([]dynamics.BodyState{
		{Body: a, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: b, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	to, err := r3.Translation(r3.Vec{X: 25})
	require.NoError(t, err)
	dt := units.Seconds(.25)
	input := dynamics.StepInput{Gravity: zeroAcceleration(), Drivers: []dynamics.KinematicDriver{{
		Body: a, Path: decad.PoseSegment{From: r3.Identity(), To: to, Duration: dt},
	}}}
	report, err := w.Step(t.Context(), start, input, dt)
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Empty(t, report.Events)
	finalA, ok := report.Next.Body(a)
	require.True(t, ok)
	finalB, ok := report.Next.Body(b)
	require.True(t, ok)
	require.Equal(t, to, finalA.Pose)
	require.Equal(t, r3.Identity(), finalB.Pose)
	require.Zero(t, finalA.LinearVelocity.X.Base())
	require.Zero(t, report.Conservation.KinematicWork.Value.Base())
}

func TestExcludedPairRejectsInvalidNamesAndOverride(t *testing.T) {
	doc := decad.New()
	a := makeBox(t, doc, 0, 0, 10, 10, 0, 10)
	b := makeBox(t, doc, 20, 0, 30, 10, 0, 10)
	other := makeBox(t, doc, 40, 0, 50, 10, 0, 10)
	density := units.KilogramsPerCubicMillimeter(.001)
	material := dynamics.Material{Restitution: units.Scalar(0), Friction: units.Scalar(0)}
	base := dynamics.WorldConfig{Bodies: []dynamics.RigidBody{
		{Body: a, Role: dynamics.Fixed, Material: material},
		{Body: b, Role: dynamics.Dynamic, Density: &density, Material: material},
	}, Step: pairMaterialStepConfig()}
	for _, tc := range []struct {
		name      string
		excluded  []dynamics.BodyPair
		overrides []dynamics.PairMaterial
	}{
		{name: "nil body", excluded: []dynamics.BodyPair{{A: nil, B: b}}},
		{name: "same body", excluded: []dynamics.BodyPair{{A: a, B: a}}},
		{name: "outside world", excluded: []dynamics.BodyPair{{A: a, B: other}}},
		{name: "duplicate reverse", excluded: []dynamics.BodyPair{{A: a, B: b}, {A: b, B: a}}},
		{name: "override excluded", excluded: []dynamics.BodyPair{{A: a, B: b}},
			overrides: []dynamics.PairMaterial{{Pair: dynamics.BodyPair{A: b, B: a},
				Restitution: units.Scalar(0), Friction: units.Scalar(0)}}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			cfg := base
			cfg.Excluded, cfg.Overrides = tc.excluded, tc.overrides
			_, err := dynamics.NewWorld(t.Context(), doc, cfg)
			require.ErrorIs(t, err, dynamics.ErrInvalidInput)
		})
	}
}

func TestExcludedOverlappingPairDriftsAndReports(t *testing.T) {
	doc := decad.New()
	a := makeBox(t, doc, 0, 0, 10, 10, 0, 10)
	b := makeBox(t, doc, 0, 0, 10, 10, 0, 10)
	density := units.KilogramsPerCubicMillimeter(.001)
	material := dynamics.Material{Restitution: units.Scalar(0), Friction: overflowFriction}
	w, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{
			{Body: a, Role: dynamics.Dynamic, Density: &density, Material: material},
			{Body: b, Role: dynamics.Dynamic, Density: &density, Material: material},
		},
		Excluded: []dynamics.BodyPair{{A: b, B: a}},
		Step:     pairMaterialStepConfig(),
	})
	require.NoError(t, err)
	canonical := dynamics.BodyPair{A: a, B: b}
	require.Equal(t, []dynamics.BodyPair{canonical}, w.Excluded())

	start, err := w.NewState([]dynamics.BodyState{
		{Body: a, Pose: r3.Identity(), LinearVelocity: dynamics.QuantityVec{
			X: units.MillimetersPerSecond(100), Y: units.MillimetersPerSecond(0),
			Z: units.MillimetersPerSecond(0),
		}, AngularVelocity: zeroAngular(t)},
		{Body: b, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	dt := units.Seconds(.1)
	report, err := w.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration()}, dt)
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Empty(t, report.Events)
	require.Equal(t, []dynamics.BodyPair{canonical}, report.Excluded)
	finalA, ok := report.Next.Body(a)
	require.True(t, ok)
	require.InDelta(t, 10, finalA.Pose.Translation().X, 1e-12)
	finalB, ok := report.Next.Body(b)
	require.True(t, ok)
	require.Zero(t, finalB.Pose.Translation().X)
	traceEnd, err := report.Trace.Sample(dt)
	require.NoError(t, err)
	require.Equal(t, *report.Next, traceEnd)
	require.NotNil(t, report.Conservation)
	require.Equal(t, units.Impulse, report.Conservation.ContactImpulse.Value.X.Kind())
	require.Zero(t, report.Conservation.ContactImpulse.Value.X.Base())
	require.Equal(t, units.Torque, report.Conservation.KinematicWork.Value.Kind())
	require.Zero(t, report.Conservation.KinematicWork.Value.Base())
	view := w.Excluded()
	view[0] = dynamics.BodyPair{}
	require.Equal(t, []dynamics.BodyPair{canonical}, w.Excluded())
	report.Excluded[0] = dynamics.BodyPair{}
	require.Equal(t, []dynamics.BodyPair{canonical}, w.Excluded())
}
