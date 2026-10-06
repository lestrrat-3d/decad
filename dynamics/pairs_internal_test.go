package dynamics

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func pairTableTestBox(t *testing.T, doc *decad.Document, x float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rectangle := s.CreateRectangle(x, 0, x+10, 10)
	s.Fix(rectangle.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{
		D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

// The pair table holds each non-excluded pair's effective material: the
// smaller restitution and the geometric mean of the friction coefficients,
// or an override's exact values. Coefficients are dyadic and every mean is
// an exact square root, so the bounding interval collapses to one value.
func TestPairTableMixesMaterialPerPair(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	var bodies [5]*decad.Body
	for i := range bodies {
		bodies[i] = pairTableTestBox(t, doc, float64(40*i))
	}
	density := units.KilogramsPerCubicMillimeter(.001)
	material := func(restitution, friction float64) Material {
		return Material{Restitution: units.Scalar(restitution), Friction: units.Scalar(friction)}
	}
	w, err := NewWorld(t.Context(), doc, WorldConfig{
		Bodies: []RigidBody{
			{Body: bodies[0], Role: Fixed, Material: material(.5, .0625)},
			{Body: bodies[1], Role: Dynamic, Density: &density, Material: material(.25, .25)},
			{Body: bodies[2], Role: Dynamic, Density: &density, Material: material(.75, 1)},
			{Body: bodies[3], Role: Fixed, Material: material(.5, .25)},
			{Body: bodies[4], Role: Kinematic, Material: material(1, 0)},
		},
		Excluded: []BodyPair{{A: bodies[4], B: bodies[2]}, {A: bodies[3], B: bodies[0]}},
		Overrides: []PairMaterial{{Pair: BodyPair{A: bodies[2], B: bodies[1]},
			Restitution: units.Scalar(.125), Friction: units.Scalar(.75)}},
		Step: pairMaterialTestStep(),
	})
	require.NoError(t, err)
	require.Len(t, w.pairs, 10)

	type want struct {
		a, b, restitution int // restitution in units of 1/8
		friction          *big.Rat
		excluded, moving  bool
	}
	r := big.NewRat
	for key, expected := range []want{
		{a: 0, b: 1, restitution: 2, friction: r(1, 8), moving: true}, // sqrt(1/16 · 1/4)
		{a: 0, b: 2, restitution: 4, friction: r(1, 4), moving: true}, // sqrt(1/16 · 1)
		{a: 0, b: 3, excluded: true},                                  // Fixed/Fixed, excluded
		{a: 0, b: 4, restitution: 4, friction: r(0, 1), moving: true}, // kinematic body is frictionless
		{a: 1, b: 2, restitution: 1, friction: r(3, 4), moving: true}, // override
		{a: 1, b: 3, restitution: 2, friction: r(1, 4), moving: true}, // sqrt(1/4 · 1/4)
		{a: 1, b: 4, restitution: 2, friction: r(0, 1), moving: true}, // kinematic body is frictionless
		{a: 2, b: 3, restitution: 4, friction: r(1, 2), moving: true}, // sqrt(1 · 1/4)
		{a: 2, b: 4, excluded: true, moving: true},                    // excluded, never mixed
		{a: 3, b: 4, restitution: 4, friction: r(0, 1), moving: true}, // fixed/kinematic still moves
	} {
		pair := w.pairs[key]
		require.Equal(t, expected.a, pair.a, "pair %d", key)
		require.Equal(t, expected.b, pair.b, "pair %d", key)
		require.Equal(t, key, canonicalPairIndex(5, pair.a, pair.b))
		require.Equal(t, key, canonicalPairIndex(5, pair.b, pair.a))
		require.Equal(t, expected.excluded, pair.excluded, "pair %d", key)
		require.Equal(t, expected.moving, pair.moving, "pair %d", key)
		if expected.excluded {
			require.Nil(t, pair.friction.lower, "an excluded pair skips material mixing")
			continue
		}
		require.Zero(t, exactBase(pair.restitution).Cmp(r(int64(expected.restitution), 8)), "pair %d", key)
		require.Zero(t, pair.friction.lower.Cmp(expected.friction), "pair %d", key)
		require.Zero(t, pair.friction.upper.Cmp(expected.friction), "pair %d", key)
	}
	for n := 2; n <= 7; n++ {
		key := 0
		for a := range n {
			for b := a + 1; b < n; b++ {
				require.Equal(t, key, canonicalPairIndex(n, a, b), "n=%d (%d,%d)", n, a, b)
				key++
			}
		}
		require.Equal(t, pairCount(n), key)
	}
}

// A three-body world's step reads its pairs straight from the table: the
// override sets the box/floor restitution, the other pair keeps the mixed
// one, and the box's mass is read once from its density. The box falls onto
// the floor at 100 mm/s and leaves at 25 mm/s, the override's restitution.
func TestThreeBodyStepReadsTheTable(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	floor := pairTableTestBox(t, doc, 0)
	box := pairTableTestBox(t, doc, 40)
	remote := pairTableTestBox(t, doc, 80)
	density := units.KilogramsPerCubicMillimeter(.001)
	material := Material{Restitution: units.Scalar(.5), Friction: units.Scalar(0)}
	w, err := NewWorld(t.Context(), doc, WorldConfig{
		Bodies: []RigidBody{
			{Body: floor, Role: Fixed, Material: material},
			{Body: box, Role: Dynamic, Density: &density, Material: material},
			{Body: remote, Role: Fixed, Material: material},
		},
		Overrides: []PairMaterial{{Pair: BodyPair{A: box, B: floor},
			Restitution: units.Scalar(.25), Friction: units.Scalar(0)}},
		Step: pairMaterialTestStep(),
	})
	require.NoError(t, err)
	require.Len(t, w.pairs, 3)
	require.Equal(t, units.Scalar(.25), w.pairs[0].restitution)
	require.Equal(t, units.Scalar(.5), w.pairs[2].restitution)
	require.NotNil(t, w.bodies[1].definition.Density, "the world keeps the caller's mass source")
	require.InDelta(t, 1, w.bodies[1].mass.Mass.Value.Base(), 1e-12)
	zeroW := QuantityVec{X: units.RadiansPerSecond(0), Y: units.RadiansPerSecond(0),
		Z: units.RadiansPerSecond(0)}
	still := QuantityVec{X: units.MillimetersPerSecond(0), Y: units.MillimetersPerSecond(0),
		Z: units.MillimetersPerSecond(0)}
	toward := still
	toward.X = units.MillimetersPerSecond(-100)
	start, err := w.NewState([]BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: still, AngularVelocity: zeroW},
		{Body: box, Pose: r3.Identity(), LinearVelocity: toward, AngularVelocity: zeroW},
		{Body: remote, Pose: r3.Identity(), LinearVelocity: still, AngularVelocity: zeroW},
	})
	require.NoError(t, err)
	gravity := QuantityVec{X: units.MillimetersPerSecondSquared(0),
		Y: units.MillimetersPerSecondSquared(0), Z: units.MillimetersPerSecondSquared(0)}
	report, err := w.Step(t.Context(), start, StepInput{Gravity: gravity}, units.Seconds(.5))
	require.NoError(t, err)
	require.Equal(t, Advanced, report.Status, "%+v", report.Diagnostics)
	require.Len(t, report.Events, 1)
	require.Equal(t, BodyPair{A: floor, B: box}, report.Events[0].Pair)
	require.InDelta(t, 25, report.Events[0].PostVelocityB.X.Base(), 1e-9)
	require.InDelta(t, 125, report.Events[0].NormalImpulse.Base(), 1e-9)
}

func pairMaterialTestStep() StepConfig {
	return StepConfig{
		Contact: decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
			NormalResolution: units.Radians(1e-6)},
		TimeResolution: units.Seconds(1e-9), ContactSlop: units.Millimeters(1e-6),
		VelocityResidual:        units.MillimetersPerSecond(1e-6),
		AngularVelocityResidual: units.RadiansPerSecond(1e-6),
		ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
		PenetrationResidual:     units.Millimeters(1e-6), ImpactSpeed: units.MillimetersPerSecond(0),
		MaxPoseEvaluations: 128, MaxIterations: 8, MaxEvents: 2, MaxPairSweeps: 4096,
	}
}
