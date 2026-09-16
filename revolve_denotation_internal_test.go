package decad

import (
	"math"
	"math/rand"
	"testing"

	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file proves the reflex-sweep box bound (docs/evaluator-design.md §6):
// sweepExtremeBounds' default arm certifies both extremes are the amplitude
// once the sweep is proven at least a half turn wide, through
// sweepDenotation.halfTurnExcessFor. Every assertion is a RELATION — a sign,
// a containment, or a ratio ceiling — never a bound literal, since the bound
// differs between amd64 and arm64 through FMA.

// probeEnd resolves v to its held radian float64 and its own angleDenotation,
// the same pair resolveAngularExtent hands the evaluator.
func probeEnd(t *testing.T, v units.Value) (float64, angleDenotation) {
	t.Helper()
	held, err := v.In(units.Radian)
	require.NoError(t, err)
	return held, angleDenotationFromValue(v)
}

// TestSweepDenotationHalfTurnExcess is design §9 test 1: halfTurnExcessFor's
// sign certifies "at least a half turn" exactly at a degree-stated half turn
// (the turn factor is exactly 1/2, so the excess is the point [0,0]),
// strictly under it just below, strictly over it just above, and reproduces
// today's held-float fallback wherever the denotation cannot state one.
func TestSweepDenotationHalfTurnExcess(t *testing.T) {
	deg := func(d float64) (float64, angleDenotation) { return probeEnd(t, units.Degrees(d)) }

	t.Run("exactly 180deg certifies exactly", func(t *testing.T) {
		phi1, d1 := deg(180)
		ex, ok := sweepDenotation{phi0: zeroAngleDenotation(), phi1: d1}.halfTurnExcessFor(0, phi1)
		require.True(t, ok)
		require.Zero(t, ex.lo.Sign())
		require.Zero(t, ex.hi.Sign())
	})

	t.Run("179.99deg is strictly under", func(t *testing.T) {
		phi1, d1 := deg(179.99)
		ex, ok := sweepDenotation{phi0: zeroAngleDenotation(), phi1: d1}.halfTurnExcessFor(0, phi1)
		require.True(t, ok)
		require.Negative(t, ex.hi.Sign())
	})

	t.Run("180.01deg is strictly over", func(t *testing.T) {
		phi1, d1 := deg(180.01)
		ex, ok := sweepDenotation{phi0: zeroAngleDenotation(), phi1: d1}.halfTurnExcessFor(0, phi1)
		require.True(t, ok)
		require.Positive(t, ex.lo.Sign())
	})

	t.Run("radian-stated pi is certified under a half turn", func(t *testing.T) {
		// fl(math.Pi) sits 1.2e-16 BELOW true pi, so a radian-stated sweep of
		// that width is certified strictly under a half turn.
		r, dr := probeEnd(t, units.Radians(math.Pi))
		ex, ok := sweepDenotation{phi0: zeroAngleDenotation(), phi1: dr}.halfTurnExcessFor(0, r)
		require.True(t, ok)
		require.Negative(t, ex.hi.Sign())
	})

	t.Run("two-sided 90/90 certifies exactly", func(t *testing.T) {
		phi0, d0neg := deg(90)
		phi1, d1 := deg(90)
		sd := sweepDenotation{phi0: d0neg.neg(), phi1: d1}
		ex, ok := sd.halfTurnExcessFor(-phi0, phi1)
		require.True(t, ok)
		require.Zero(t, ex.lo.Sign())
		require.Zero(t, ex.hi.Sign())
	})

	t.Run("mixed 2rad/-100deg is sign-definite positive", func(t *testing.T) {
		phi1, d1 := probeEnd(t, units.Radians(2))
		phi0, d0 := deg(100)
		sd := sweepDenotation{phi0: d0.neg(), phi1: d1}
		ex, ok := sd.halfTurnExcessFor(-phi0, phi1)
		require.True(t, ok)
		require.Positive(t, ex.lo.Sign())
	})

	t.Run("empty denotation falls back to the held floats", func(t *testing.T) {
		ex, ok := sweepDenotation{}.halfTurnExcessFor(0, 3.2)
		require.True(t, ok)
		require.Positive(t, ex.lo.Sign())

		ex, ok = sweepDenotation{}.halfTurnExcessFor(0, 3.1)
		require.True(t, ok)
		require.Negative(t, ex.hi.Sign())
	})

	t.Run("a NaN held float with no denotation answers not ok", func(t *testing.T) {
		_, ok := sweepDenotation{}.halfTurnExcessFor(0, math.NaN())
		require.False(t, ok)
	})
}

// bruteExtremes is an independent, non-certified oracle for m(φ) = c0 cos φ +
// c1 sin φ's own min/max over [phi0, phi1]: a dense grid plus the exact
// interior critical angles. It never shares code with sweepExtremeBounds.
func bruteExtremes(c0, c1, phi0, phi1 float64) (float64, float64) {
	m := func(phi float64) float64 { return c0*math.Cos(phi) + c1*math.Sin(phi) }
	lo, hi := math.Inf(1), math.Inf(-1)
	const n = 200000
	for i := 0; i <= n; i++ {
		v := m(phi0 + (phi1-phi0)*float64(i)/n)
		lo, hi = math.Min(lo, v), math.Max(hi, v)
	}
	star := math.Atan2(c1, c0)
	for k := -3.0; k <= 3; k++ {
		cand := star + k*math.Pi
		if cand >= phi0 && cand <= phi1 {
			v := m(cand)
			lo, hi = math.Min(lo, v), math.Max(hi, v)
		}
	}
	return lo, hi
}

// TestSweepExtremeBoundsReflexArm is design §9 test 2: a randomized probe
// over direction coefficients (a quarter of them with c1 = 0, an exactly
// critical start) and sweeps stated Along in degrees, Along in radians,
// two-sided in degrees, and mixed radian/degree, widths spanning (0, 2π).
// Every case must stay SOUND — the brute-force extreme lies within the
// published held value plus its bound, plus slack for the oracle's own grid
// and float error — and every sweep of width at least a half turn must be
// TIGHT: bound <= 1e-9 * amplitude.
func TestSweepExtremeBoundsReflexArm(t *testing.T) {
	rng := rand.New(rand.NewSource(7))
	reflex, tight := 0, 0
	for range 3000 {
		c0, c1 := rng.Float64()*2-1, rng.Float64()*2-1
		if rng.Intn(4) == 0 {
			c1 = 0 // an exactly critical endpoint at phi = 0
		}
		amp := math.Hypot(c0, c1)

		var phi0, phi1 float64
		var den sweepDenotation
		switch rng.Intn(4) {
		case 0: // degrees, Along
			d := 1 + rng.Float64()*358
			phi0, den.phi0 = 0, zeroAngleDenotation()
			phi1, den.phi1 = probeEnd(t, units.Degrees(d))
		case 1: // radians, Along
			r := 0.01 + rng.Float64()*6.27
			phi0, den.phi0 = 0, zeroAngleDenotation()
			phi1, den.phi1 = probeEnd(t, units.Radians(r))
		case 2: // two-sided degrees, both ends off zero
			a := 1 + rng.Float64()*179
			b := 1 + rng.Float64()*179
			var neg angleDenotation
			phi0, neg = probeEnd(t, units.Degrees(b))
			phi0, den.phi0 = -phi0, neg.neg()
			phi1, den.phi1 = probeEnd(t, units.Degrees(a))
		default: // mixed: radians one side, degrees the other
			a := 0.01 + rng.Float64()*3.1
			b := 1 + rng.Float64()*179
			var neg angleDenotation
			phi0, neg = probeEnd(t, units.Degrees(b))
			phi0, den.phi0 = -phi0, neg.neg()
			phi1, den.phi1 = probeEnd(t, units.Radians(a))
		}

		heldLo, heldHi := sweepExtremes(c0, c1, phi0, phi1, false)
		loB, hiB := sweepExtremeBounds(c0, c1, phi0, phi1, den, heldLo, heldHi, false)
		trueLo, trueHi := bruteExtremes(c0, c1, phi0, phi1)

		require.LessOrEqual(t, math.Abs(trueLo-heldLo), loB+1e-9*amp+1e-12,
			"min unsound c=(%g,%g) [%g,%g]", c0, c1, phi0, phi1)
		require.LessOrEqual(t, math.Abs(trueHi-heldHi), hiB+1e-9*amp+1e-12,
			"max unsound c=(%g,%g) [%g,%g]", c0, c1, phi0, phi1)

		if phi1-phi0 >= math.Pi {
			reflex++
			require.LessOrEqual(t, loB, 1e-9*amp,
				"reflex min loose c=(%g,%g) [%g,%g] width=%g", c0, c1, phi0, phi1, phi1-phi0)
			require.LessOrEqual(t, hiB, 1e-9*amp,
				"reflex max loose c=(%g,%g) [%g,%g] width=%g", c0, c1, phi0, phi1, phi1-phi0)
			tight++
		}
	}
	t.Logf("reflex cases: %d, tight: %d", reflex, tight)
	require.Greater(t, reflex, 500, "the seeded run must actually exercise the reflex regime")
	require.Equal(t, reflex, tight, "every reflex case the run produced must be tight")
}
