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

var (
	overflowFriction = units.New(1.6578522501383104e308,
		units.Define("decad-friction-overflow", units.Dimensionless, 1.0843506317962526))
	underflowFriction = units.New(math.SmallestNonzeroFloat64,
		units.Define("decad-friction-underflow", units.Dimensionless, .5))
)

func pairMaterialStepConfig() dynamics.StepConfig {
	return dynamics.StepConfig{
		Contact: decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
			NormalResolution: units.Radians(1e-6)},
		TimeResolution: units.Seconds(1e-9), ContactSlop: units.Millimeters(1e-6),
		VelocityResidual:        units.MillimetersPerSecond(1e-6),
		AngularVelocityResidual: units.RadiansPerSecond(1e-6),
		ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
		PenetrationResidual:     units.Millimeters(1e-6),
		ImpactSpeed:             units.MillimetersPerSecond(0),
		MaxPoseEvaluations:      128, MaxIterations: 64, MaxEvents: 2, MaxPairSweeps: 4096,
	}
}

func TestPairMaterialOverrideReboundsUsingProductionGeometry(t *testing.T) {
	t.Parallel()
	for _, reverse := range []bool{false, true} {
		t.Run(map[bool]string{false: "world order", true: "reverse order"}[reverse], func(t *testing.T) {
			doc := decad.New()
			floor := makeBox(t, doc, -20, -20, 20, 20, -10, 10)
			box := makeBox(t, doc, -5, -5, 5, 5, 0, 10)
			density := units.KilogramsPerCubicMillimeter(.001)
			pair := dynamics.BodyPair{A: floor, B: box}
			if reverse {
				pair = dynamics.BodyPair{A: box, B: floor}
			}
			cfg := dynamics.WorldConfig{Bodies: []dynamics.RigidBody{
				{Body: floor, Role: dynamics.Fixed, Material: dynamics.Material{
					Restitution: units.Scalar(.2), Friction: units.Scalar(0)}},
				{Body: box, Role: dynamics.Dynamic, Density: &density, Material: dynamics.Material{
					Restitution: units.Scalar(.8), Friction: units.Scalar(0)}},
			}, Overrides: []dynamics.PairMaterial{{Pair: pair, Restitution: units.Scalar(.5),
				Friction: units.Scalar(0)}}, Step: pairMaterialStepConfig()}
			world, err := dynamics.NewWorld(t.Context(), doc, cfg)
			require.NoError(t, err)
			startPose, err := r3.Translation(r3.Vec{Z: 10})
			require.NoError(t, err)
			start, err := world.NewState([]dynamics.BodyState{
				{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
				{Body: box, Pose: startPose, LinearVelocity: dynamics.QuantityVec{
					X: units.MillimetersPerSecond(0), Y: units.MillimetersPerSecond(0),
					Z: units.MillimetersPerSecond(-100),
				}, AngularVelocity: zeroAngular(t)},
			})
			require.NoError(t, err)
			report, err := world.Step(t.Context(), start,
				dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(.2))
			require.NoError(t, err)
			require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
			require.Len(t, report.Events, 1)
			require.NotNil(t, report.Events[0].Manifold)
			require.NotEmpty(t, report.Events[0].Manifold.Points)
			require.InDelta(t, 150, report.Events[0].NormalImpulse.Base(), 1e-4)
			final, ok := report.Next.Body(box)
			require.True(t, ok)
			require.InDelta(t, 50, final.LinearVelocity.Z.Base(), 1e-6)
			require.InDelta(t, 5, final.Pose.Translation().Z, 2e-6)
		})
	}
}

func TestPairMaterialOverrideDrivesFixedFloorFriction(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	floor := makeBox(t, doc, -100, -100, 100, 100, -10, 10)
	box := makeBox(t, doc, -5, -5, 5, 5, 0, 10)
	density := units.KilogramsPerCubicMillimeter(.001)
	material := dynamics.Material{Restitution: units.Scalar(0), Friction: units.Scalar(0)}
	world, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{
			{Body: floor, Role: dynamics.Fixed, Material: material},
			{Body: box, Role: dynamics.Dynamic, Density: &density, Material: material},
		}, Overrides: []dynamics.PairMaterial{{Pair: dynamics.BodyPair{A: box, B: floor},
			Restitution: units.Scalar(0), Friction: units.Scalar(.5)}},
		Step: pairMaterialStepConfig(),
	})
	require.NoError(t, err)
	start, err := world.NewState([]dynamics.BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: box, Pose: r3.Identity(), LinearVelocity: dynamics.QuantityVec{
			X: units.MillimetersPerSecond(100), Y: units.MillimetersPerSecond(0),
			Z: units.MillimetersPerSecond(0),
		}, AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	gravity := dynamics.QuantityVec{X: units.MillimetersPerSecondSquared(0),
		Y: units.MillimetersPerSecondSquared(0), Z: units.MillimetersPerSecondSquared(-1000)}
	report, err := world.Step(t.Context(), start, dynamics.StepInput{Gravity: gravity}, units.Seconds(.1))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Len(t, report.Events, 1)
	require.Len(t, report.Events[0].Manifold.Points, 4)
	require.InDelta(t, 100, report.Events[0].NormalImpulse.Base(), 1e-6)
	require.InDelta(t, -50, report.Events[0].TangentImpulse.X.Base(), 1e-6)
	final, ok := report.Next.Body(box)
	require.True(t, ok)
	require.InDelta(t, 50, final.LinearVelocity.X.Base(), 1e-6)
	require.InDelta(t, 5, final.Pose.Translation().X, 1e-6)
}

func TestBodyFrictionMixDrivesFixedFloorStep(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name                   string
		floorMu, boxMu         float64
		wantTangent, wantSpeed float64
	}{
		{name: "quarter then one", floorMu: .25, boxMu: 1, wantTangent: -50, wantSpeed: 50},
		{name: "one then quarter", floorMu: 1, boxMu: .25, wantTangent: -50, wantSpeed: 50},
		{name: "irrational root", floorMu: .2, boxMu: .3,
			wantTangent: -100 * math.Sqrt(.2*.3), wantSpeed: 100 * (1 - math.Sqrt(.2*.3))},
		{name: "zero then positive", floorMu: 0, boxMu: .5, wantTangent: 0, wantSpeed: 100},
		{name: "positive then zero", floorMu: .5, boxMu: 0, wantTangent: 0, wantSpeed: 100},
	} {
		t.Run(tc.name, func(t *testing.T) {
			doc := decad.New()
			floor := makeBox(t, doc, -100, -100, 100, 100, -10, 10)
			box := makeBox(t, doc, -5, -5, 5, 5, 0, 10)
			density := units.KilogramsPerCubicMillimeter(.001)
			world, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
				Bodies: []dynamics.RigidBody{
					{Body: floor, Role: dynamics.Fixed, Material: dynamics.Material{
						Restitution: units.Scalar(0), Friction: units.Scalar(tc.floorMu)}},
					{Body: box, Role: dynamics.Dynamic, Density: &density, Material: dynamics.Material{
						Restitution: units.Scalar(0), Friction: units.Scalar(tc.boxMu)}},
				}, Step: pairMaterialStepConfig(),
			})
			require.NoError(t, err)
			start, err := world.NewState([]dynamics.BodyState{
				{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
				{Body: box, Pose: r3.Identity(), LinearVelocity: dynamics.QuantityVec{
					X: units.MillimetersPerSecond(100), Y: units.MillimetersPerSecond(0),
					Z: units.MillimetersPerSecond(0)}, AngularVelocity: zeroAngular(t)},
			})
			require.NoError(t, err)
			gravity := dynamics.QuantityVec{X: units.MillimetersPerSecondSquared(0),
				Y: units.MillimetersPerSecondSquared(0), Z: units.MillimetersPerSecondSquared(-1000)}
			report, err := world.Step(t.Context(), start, dynamics.StepInput{Gravity: gravity}, units.Seconds(.1))
			require.NoError(t, err)
			require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
			require.Len(t, report.Events, 1)
			require.InDelta(t, 100, report.Events[0].NormalImpulse.Base(), 1e-6)
			require.InDelta(t, tc.wantTangent, report.Events[0].TangentImpulse.X.Base(), 1e-5)
			final, ok := report.Next.Body(box)
			require.True(t, ok)
			require.InDelta(t, tc.wantSpeed, final.LinearVelocity.X.Base(), 1e-5)
			require.InDelta(t, .1*tc.wantSpeed, final.Pose.Translation().X, 1e-5)
		})
	}
}

func TestBodyFrictionMixRejectsUnrepresentableMean(t *testing.T) {
	t.Parallel()
	require.Equal(t, math.MaxFloat64, overflowFriction.Base())
	require.Zero(t, underflowFriction.Base())
	for _, tc := range []struct {
		name     string
		friction units.Value
	}{
		{name: "above maximum", friction: overflowFriction},
		{name: "below minimum positive", friction: underflowFriction},
	} {
		t.Run(tc.name, func(t *testing.T) {
			doc := decad.New()
			floor := makeBox(t, doc, -100, -100, 100, 100, -10, 10)
			box := makeBox(t, doc, -5, -5, 5, 5, 0, 10)
			density := units.KilogramsPerCubicMillimeter(.001)
			_, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
				Bodies: []dynamics.RigidBody{
					{Body: floor, Role: dynamics.Fixed, Material: dynamics.Material{
						Restitution: units.Scalar(0), Friction: tc.friction}},
					{Body: box, Role: dynamics.Dynamic, Density: &density, Material: dynamics.Material{
						Restitution: units.Scalar(0), Friction: tc.friction}},
				}, Step: pairMaterialStepConfig(),
			})
			require.ErrorIs(t, err, dynamics.ErrUnsupported)
		})
	}
}

func TestPairMaterialOverrideValidatesPairAndCoefficients(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	floor := makeBox(t, doc, -20, -20, 20, 20, -10, 10)
	box := makeBox(t, doc, -5, -5, 5, 5, 0, 10)
	other := makeBox(t, doc, 30, 30, 40, 40, 0, 10)
	density := units.KilogramsPerCubicMillimeter(.001)
	material := dynamics.Material{Restitution: units.Scalar(0), Friction: units.Scalar(0)}
	valid := dynamics.PairMaterial{Pair: dynamics.BodyPair{A: floor, B: box},
		Restitution: units.Scalar(.5), Friction: units.Scalar(.5)}
	config := dynamics.WorldConfig{Bodies: []dynamics.RigidBody{
		{Body: floor, Role: dynamics.Fixed, Material: material},
		{Body: box, Role: dynamics.Dynamic, Density: &density, Material: material},
	}, Step: pairMaterialStepConfig()}
	for _, tc := range []struct {
		name      string
		overrides []dynamics.PairMaterial
	}{
		{"unknown", []dynamics.PairMaterial{{Pair: dynamics.BodyPair{A: floor, B: other},
			Restitution: valid.Restitution, Friction: valid.Friction}}},
		{"nil body", []dynamics.PairMaterial{{Pair: dynamics.BodyPair{A: nil, B: box},
			Restitution: valid.Restitution, Friction: valid.Friction}}},
		{"same body", []dynamics.PairMaterial{{Pair: dynamics.BodyPair{A: box, B: box},
			Restitution: valid.Restitution, Friction: valid.Friction}}},
		{"duplicate reverse", []dynamics.PairMaterial{valid, {Pair: dynamics.BodyPair{A: box, B: floor},
			Restitution: valid.Restitution, Friction: valid.Friction}}},
		{"negative friction", []dynamics.PairMaterial{{Pair: valid.Pair,
			Restitution: valid.Restitution, Friction: units.Scalar(-.1)}}},
		{"excess restitution", []dynamics.PairMaterial{{Pair: valid.Pair,
			Restitution: units.Scalar(1.1), Friction: valid.Friction}}},
		{"wrong kind", []dynamics.PairMaterial{{Pair: valid.Pair,
			Restitution: units.Millimeters(1), Friction: valid.Friction}}},
		{"nonfinite", []dynamics.PairMaterial{{Pair: valid.Pair,
			Restitution: valid.Restitution, Friction: units.Scalar(math.NaN())}}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			cfg := config
			cfg.Overrides = tc.overrides
			_, err := dynamics.NewWorld(t.Context(), doc, cfg)
			require.ErrorIs(t, err, dynamics.ErrInvalidInput)
		})
	}
	config.Overrides = []dynamics.PairMaterial{valid}
	config.Excluded = []dynamics.BodyPair{{A: floor, B: box}}
	_, err := dynamics.NewWorld(t.Context(), doc, config)
	require.ErrorIs(t, err, dynamics.ErrInvalidInput)
}
