package decad

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// This file tests ONE rule across bounds.go's outward-rounding primitives and
// every chorded-loft bound built on them: a bound may publish 0 only when the
// quantity it bounds is genuinely ABSENT, never because float64's own
// arithmetic flushed a positive result to +0.
//
// float64 rounds a positive product or quotient to +0 as soon as that result
// falls below half the smallest subnormal, and upRound cannot undo it — 0 is
// not a representable neighbour away from a positive number, it IS the rounded
// answer. The consequence is not a slightly loose bound but an UNSOUND one:
// the published interval [value−bound, value+bound] collapses to a point that
// excludes the truth, and exactnessOf reads the same 0 as the CLAIM that the
// value is exactly representable, so a flushed cell publishes an Exact
// measurement for a body whose error is merely tiny.
//
// math.SmallestNonzeroFloat64 is the correct replacement rather than a fudge:
// rounding to +0 PROVES the exact result sits at or below half the smallest
// subnormal, so the smallest subnormal encloses it — and stays FINITE, which a
// +Inf refusal would not, and which the consumers that gate on a positive
// bound (chordedBoundaryVolumeAllow's own wallAreaUpper > 0 branch) need.
//
// No leg below pins a float literal a platform's own FMA contraction could
// move: each asserts the SIGN of the published bound and the exactness class
// it lands in, and the primitive legs pin only the smallest subnormal itself,
// which a single multiply or divide reaches identically on every target.

// TestOutwardRoundingNeverPublishesAFlushedZero pins the rule productUpper and
// divUpper carry for every bound in this package: two operands the helper has
// itself PROVEN positive can never answer 0.
//
// An operand at or below zero is an ABSENT term, not a flushed one. It is the
// only way a product or quotient legitimately vanishes, and the only zero
// exactnessOf may read as a claim of exactness, so it must keep answering an
// honest 0 — widening it would convert every genuinely exact reading in the
// package into Approximate.
func TestOutwardRoundingNeverPublishesAFlushedZero(t *testing.T) {
	const tiny = math.SmallestNonzeroFloat64

	t.Run("product", func(t *testing.T) {
		for _, tc := range []struct {
			name string
			a, b float64
		}{
			{"both factors far below the subnormal floor", 1e-200, 1e-200},
			{"a subnormal factor scaled down", 1e-320, 1e-10},
			{"the smallest subnormal halved", tiny, 0.5},
		} {
			t.Run(tc.name, func(t *testing.T) {
				require.Equal(t, 0.0, tc.a*tc.b, `the fixture must actually exercise a flush`)
				require.Equal(t, tiny, productUpper(tc.a, tc.b))
				require.Equal(t, Approximate, exactnessOf(productUpper(tc.a, tc.b)))
			})
		}
	})

	t.Run("quotient", func(t *testing.T) {
		for _, tc := range []struct {
			name     string
			num, den float64
		}{
			{"a subnormal numerator over a large denominator", 1e-320, 1e10},
			{"the smallest subnormal thirded", tiny, 3},
		} {
			t.Run(tc.name, func(t *testing.T) {
				require.Equal(t, 0.0, tc.num/tc.den, `the fixture must actually exercise a flush`)
				require.Equal(t, tiny, divUpper(tc.num, tc.den))
				require.Equal(t, Approximate, exactnessOf(divUpper(tc.num, tc.den)))
			})
		}
	})

	t.Run("an absent term still publishes an honest zero", func(t *testing.T) {
		require.Equal(t, 0.0, productUpper(0, 1e300))
		require.Equal(t, 0.0, productUpper(1e300, 0))
		require.Equal(t, 0.0, productUpper(-1, 5))
		require.Equal(t, 0.0, divUpper(0, 2))
		require.Equal(t, 0.0, divUpper(-1, 2))
		require.Equal(t, Exact, exactnessOf(productUpper(0, 1e300)))
	})

	t.Run("an ordinary reading still rounds outward", func(t *testing.T) {
		require.GreaterOrEqual(t, productUpper(3, 5), 15.0)
		require.GreaterOrEqual(t, productUpper(0.1, 0.1), 0.01)
		require.GreaterOrEqual(t, divUpper(15, 5), 3.0)
		require.GreaterOrEqual(t, divUpper(1, 3), 1.0/3.0)
	})

	t.Run("a divisor that states no scale refuses", func(t *testing.T) {
		require.True(t, math.IsInf(divUpper(1, 0), 1))
		require.True(t, math.IsInf(divUpper(1, -2), 1))
		require.True(t, math.IsInf(divUpper(1, math.Inf(1)), 1))
		require.True(t, math.IsInf(divUpper(math.NaN(), 2), 1))
		require.True(t, math.IsInf(divUpper(1, math.NaN()), 1))
		// A numerator that already refuses stays a refusal rather than
		// becoming a finite quotient.
		require.True(t, math.IsInf(divUpper(math.Inf(1), 3), 1))
	})
}

// TestChordedBoundsNeverPublishAFlushedZero drives every chorded-loft bound
// this package publishes at subnormal geometry, where each helper's own
// certified operands are POSITIVE but their float product or quotient
// underflows.
//
// Each fixture states a claim the helper admits — no refusal arm fires — and
// bounds a quantity that is genuinely positive, so the published 0 each leg
// used to return excluded the very displacement it was there to enclose.
func TestChordedBoundsNeverPublishAFlushedZero(t *testing.T) {
	const s = math.SmallestNonzeroFloat64

	// A degenerate-in-magnitude but non-degenerate-in-shape cell: the four
	// corners are distinct, so every certified span is positive.
	var (
		vLo = r3.Vec{X: 0, Y: 0, Z: 0}
		vHi = r3.Vec{X: s, Y: 0, Z: 0}
		wLo = r3.Vec{X: 0, Y: s, Z: 0}
		wHi = r3.Vec{X: s, Y: s, Z: 0}
		// The same cell sheared so the exact twist vector
		// T = vLo − vHi − wLo + wHi is (s,0,0), not the zero vector.
		twHi = r3.Vec{X: 2 * s, Y: s, Z: 0}
	)

	t.Run("cell chord-to-curve area", func(t *testing.T) {
		// The reviewer's own reproduction. Both certified spans and both eA
		// and eB are positive, so the ruled patch this bounds carries
		// positive area — a zero here claims the wall cell's whole
		// chord-to-curve homotopy sweeps no area at all.
		got := cellChordCurveAreaUpper(vLo, vHi, wLo, wHi, 4*s, 4*s, 0)
		require.Greater(t, got, 0.0,
			`a cell with four distinct corners bounds a positive area`)
		require.Equal(t, Approximate, exactnessOf(got))
	})

	t.Run("cell twist volume", func(t *testing.T) {
		// The exact T is nonzero, so the ruled patch and the built triangle
		// pair are genuinely different surfaces with a positive volume
		// between them.
		require.Greater(t, cellTwistQuarterUpper(vLo, vHi, wLo, twHi), 0.0,
			`the fixture must carry a certified nonzero twist`)
		got := cellTwistVolumeAllow(vLo, vHi, wLo, twHi)
		require.Greater(t, got, 0.0,
			`a twisted cell bounds a positive ruled-to-triangle volume`)
		require.Equal(t, Approximate, exactnessOf(got))
	})

	t.Run("cap area volume", func(t *testing.T) {
		// Both operands pass the helper's own positivity gate, so |h|·|ΔArea|/3
		// is positive and a zero would republish a moved cap as an unmoved one.
		got := capAreaVolumeAllow(s, s)
		require.Greater(t, got, 0.0,
			`a positive plane offset over a positive area gap bounds a positive volume`)
		require.Equal(t, Approximate, exactnessOf(got))
	})

	t.Run("cap area volume whose product survives but whose third flushes", func(t *testing.T) {
		// The product itself is representable; only the division by 3
		// underflows, so this leg falsifies the DIVIDE independently of the
		// multiply the leg above covers.
		require.Greater(t, productUpper(s, 0.1), 0.0,
			`the fixture must carry a positive numerator into the divide`)
		require.Equal(t, 0.0, productUpper(s, 0.1)/3,
			`the fixture must actually exercise a flush at the divide`)
		got := capAreaVolumeAllow(s, 0.1)
		require.Greater(t, got, 0.0, `the third of a positive volume is positive`)
		require.Equal(t, Approximate, exactnessOf(got))
	})

	t.Run("wall chord-to-curve leg", func(t *testing.T) {
		// matchedDelta and wallAreaUpper both pass the helper's own > 0
		// branch gate, so the wall leg matchedDelta·wallAreaUpper is positive
		// and the composed total cannot be zero.
		got := chordedBoundaryVolumeAllow(s, s, 0, 0, 0)
		require.Greater(t, got, 0.0,
			`a positive displacement over a positive wall area bounds a positive volume`)
		require.Equal(t, Approximate, exactnessOf(got))
	})

	t.Run("seam correction", func(t *testing.T) {
		got := chordedBoundarySeamAllow(s, s, s)
		require.Greater(t, got, 0.0,
			`three positive operands bound a positive line-integral residue`)
		require.Equal(t, Approximate, exactnessOf(got))
	})

	t.Run("seam correction whose product survives but whose third flushes", func(t *testing.T) {
		// As with the cap: the numerator is representable and only the
		// division by 3 underflows.
		require.Equal(t, 0.0, productUpper(s, productUpper(0.1, 1))/3,
			`the fixture must actually exercise a flush at the divide`)
		got := chordedBoundarySeamAllow(s, 0.1, 1)
		require.Greater(t, got, 0.0, `the third of a positive residue is positive`)
		require.Equal(t, Approximate, exactnessOf(got))
	})

	t.Run("moment", func(t *testing.T) {
		// The volume leg is positive, and the widened radius is positive
		// through matchedDelta alone, so the first moment this bounds is
		// positive whatever the held coordinate envelope.
		got := chordedBoundaryMomentAllow(s, s, 0, 0, 0, 0, 0)
		require.Greater(t, got, 0.0,
			`a positive displaced volume at a positive radius bounds a positive moment`)
		require.Equal(t, Approximate, exactnessOf(got))
	})

	t.Run("a genuinely absent term still publishes an honest zero", func(t *testing.T) {
		// Every zero below comes from a term that is ABSENT, not flushed, and
		// each must survive the change: these are the only readings in the
		// family exactnessOf may keep calling Exact.
		require.Equal(t, 0.0, cellChordCurveAreaUpper(vLo, vLo, vLo, vLo, 0, 0, 0))
		require.Equal(t, 0.0, cellTwistVolumeAllow(vLo, vHi, wLo, wHi))
		require.Equal(t, 0.0, capAreaVolumeAllow(0, 1))
		require.Equal(t, 0.0, capAreaVolumeAllow(1, 0))
		require.Equal(t, 0.0, chordedBoundaryVolumeAllow(0, 1, 0, 0, 0))
		require.Equal(t, 0.0, chordedBoundarySeamAllow(0, 1, 1))
		require.Equal(t, 0.0, chordedBoundarySeamAllow(1, 0, 1))
		require.Equal(t, 0.0, chordedBoundaryMomentAllow(0, 1, 0, 0, 0, 0, 0))
	})
}
