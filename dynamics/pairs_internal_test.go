package dynamics

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
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
	require.Nil(t, w.three, "a five-body world has no three-body container")
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

// A three-body world's response pairs are two-body worlds built from the
// parent table: they share its admitted mass and its pair material.
func TestThreeBodyResponsePairsReadTheTable(t *testing.T) {
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
	require.NotNil(t, w.three)
	require.Equal(t, 1, w.three.dynamic)
	require.Equal(t, 1, w.three.dynamicCount)
	require.Nil(t, w.three.pairs[1], "the fixed/fixed pair has no response world")
	for _, key := range []int{0, 2} {
		child := w.three.pairs[key]
		require.NotNil(t, child)
		require.Len(t, child.bodies, 2)
		require.Equal(t, w.bodyPair(w.pairs[key]), child.bodyPair(child.pairs[0]))
		require.Equal(t, 0, child.pairs[0].a)
		require.Equal(t, 1, child.pairs[0].b)
		require.Equal(t, w.pairs[key].restitution, child.pairs[0].restitution)
		dynamicSide := 1 - key/2 // the box is b in pair (0,1) and a in pair (1,2)
		require.Nil(t, child.bodies[dynamicSide].definition.Density)
		require.NotNil(t, child.bodies[dynamicSide].definition.Supplied)
		require.Equal(t, w.bodies[1].mass, *child.bodies[dynamicSide].definition.Supplied)
		require.Equal(t, w.bodies[1].mass, child.bodies[dynamicSide].mass)
	}
	require.Equal(t, units.Scalar(.25), w.three.pairs[0].pairs[0].restitution)
	require.Equal(t, units.Scalar(.5), w.three.pairs[2].pairs[0].restitution)
	require.NotNil(t, w.bodies[1].definition.Density, "the parent keeps the caller's mass source")
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
		MaxPoseEvaluations: 128, MaxIterations: 8, MaxEvents: 2,
	}
}
