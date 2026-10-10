package apitest_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file holds a recorded region's mass readings to the region sketch
// arranged, at a CUT junction: a bound where two fragments meet at a crossing.
// Each fragment records its entity at its own recorded parameter, and neither
// parameter is the exact crossing, so the two denoted ends differ by the cut
// parameters' own error — about ulp(t)·|entity|, which a long entity makes
// large. The region is the one the segments bound with that gap closed by a
// chord, and that chord lies within a sliver of the exact crossing geometry
// far below every bound here, so each reading is checked against the exact
// crossing-based closed form.
//
// Shown-to-fail: without the junction charge (momentinput's
// chargeLoopJunctions) both fixtures read outside their bounds. The
// line/circle caps read 1.5e-13 mm² off the exact cap area under bounds of
// 6e-16 to 1.1e-14, and the square reads 2.7e-10 mm² off 100 under a 6e-15
// bound, its centroid 1.4e-11 mm off (5, 5) under 3e-16. Dropping only the
// exact closing chord (ExactChord) keeps the square Approximate and still
// enclosing; it is the Exact leg below that goes red.

const junctionPrec = 256

func jf(x float64) *big.Float { return new(big.Float).SetPrec(junctionPrec).SetFloat64(x) }

// junctionAtan is atan(y) for |y| ≤ 1, halved twice by
// atan(y) = 2·atan(y / (1 + √(1 + y²))) before its Taylor series.
func junctionAtan(y *big.Float) *big.Float {
	one := jf(1)
	for range 2 {
		s := new(big.Float).SetPrec(junctionPrec).Mul(y, y)
		s.Add(s, one).Sqrt(s).Add(s, one)
		y = new(big.Float).SetPrec(junctionPrec).Quo(y, s)
	}
	sum := jf(0)
	term := new(big.Float).SetPrec(junctionPrec).Set(y)
	y2 := new(big.Float).SetPrec(junctionPrec).Mul(y, y)
	tiny := new(big.Float).SetPrec(junctionPrec).SetMantExp(one, -junctionPrec-8)
	for n := int64(0); new(big.Float).Abs(term).Cmp(tiny) > 0; n++ {
		piece := new(big.Float).SetPrec(junctionPrec).Quo(term, jf(float64(2*n+1)))
		if n%2 == 0 {
			sum.Add(sum, piece)
		} else {
			sum.Sub(sum, piece)
		}
		term.Mul(term, y2)
	}
	return sum.Mul(sum, jf(4))
}

// junctionPi is Machin's 16·atan(1/5) − 4·atan(1/239).
func junctionPi() *big.Float {
	a := junctionAtan(new(big.Float).SetPrec(junctionPrec).Quo(jf(1), jf(5)))
	b := junctionAtan(new(big.Float).SetPrec(junctionPrec).Quo(jf(1), jf(239)))
	a.Mul(a, jf(16))
	b.Mul(b, jf(4))
	return a.Sub(a, b)
}

// requireJunctionEncloses checks that a published value lies within its bound
// of the exact truth.
func requireJunctionEncloses(t *testing.T, what string, value, bound float64, truth *big.Float) {
	t.Helper()
	diff, _ := new(big.Float).SetPrec(junctionPrec).Sub(jf(value), truth).Float64()
	require.LessOrEqualf(t, math.Abs(diff), bound, "%s: published %.17g is %.3e from the exact value, outside its bound %.3e", what, value, diff, bound)
}

// TestProfileRecordChargesCutJunction cuts a radius-7.3 circle centred at
// (0.1, 0.2) with a 137000 mm line along v = 0.3, so each cap is bounded by a
// line fragment and a circle fragment meeting at two irrational crossings.
// The cap beyond the chord at distance d = 0.1 from the centre has the exact
// area R²·acos(d/R) − d·√(R² − d²), the other πR² minus that.
func TestProfileRecordChargesCutJunction(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	s.CreateLine(s.CreatePoint(-37000, 0.3), s.CreatePoint(100000, 0.3))
	s.CreateCircle(s.CreatePoint(0.1, 0.2), 7.3)

	r := jf(7.3)
	d := new(big.Float).SetPrec(junctionPrec).Sub(jf(0.3), jf(0.2))
	r2 := new(big.Float).SetPrec(junctionPrec).Mul(r, r)
	half := new(big.Float).SetPrec(junctionPrec).Mul(d, d)
	half.Sub(r2, half).Sqrt(half)
	// acos(d/R) = 2·atan(√(R² − d²) / (R + d)), and that ratio is below 1.
	angle := junctionAtan(new(big.Float).SetPrec(junctionPrec).Quo(half, new(big.Float).SetPrec(junctionPrec).Add(r, d)))
	angle.Mul(angle, jf(2))
	beyond := new(big.Float).SetPrec(junctionPrec).Mul(r2, angle)
	beyond.Sub(beyond, new(big.Float).SetPrec(junctionPrec).Mul(d, half))
	disk := new(big.Float).SetPrec(junctionPrec).Mul(junctionPi(), r2)
	near := new(big.Float).SetPrec(junctionPrec).Sub(disk, beyond)

	profiles := s.Profiles()
	require.Len(t, profiles, 2)
	doc := decad.New()
	for _, profile := range profiles {
		cut := 0
		for _, edge := range profile.Outer {
			if edge.Partial {
				require.True(t, edge.TExact, `an analytic line/circle cut is certified`)
				cut++
			}
		}
		require.GreaterOrEqual(t, cut, 2, `each cap holds a line fragment and a circle fragment`)

		record, _, err := momentinput.RecordProfile(s, profile)
		require.NoError(t, err)
		area, err := record.Area()
		require.NoError(t, err)
		value, err := area.Value.In(units.SquareMillimeter)
		require.NoError(t, err)
		bound, err := area.Bound.In(units.SquareMillimeter)
		require.NoError(t, err)
		require.Equal(t, measurement.Approximate, area.Exactness)
		truth := near
		if b, _ := beyond.Float64(); math.Abs(value-b) < 1 {
			truth = beyond
		}
		requireJunctionEncloses(t, `record area`, value, bound, truth)
		require.Less(t, bound, 1e-9*value, `the junction charge stays near the rounding level`)

		body, err := doc.Extrude(s, profile, decad.Distance{D: units.Millimeters(1), Dir: decad.Along})
		require.NoError(t, err)
		volume, err := body.Volume()
		require.NoError(t, err)
		vv, err := volume.Value.In(units.CubicMillimeter)
		require.NoError(t, err)
		vb, err := volume.Bound.In(units.CubicMillimeter)
		require.NoError(t, err)
		requireJunctionEncloses(t, `extruded volume`, vv, vb, truth)
	}
}

// TestProfileRecordClosesExactLineJunction draws the 10 mm square whose right
// side is a line from (10, −10⁶) cut at the base corner, so the fragment's
// recorded start sits about 1e-10 mm from (10, 0) along that side. The
// segments bound exactly the square once that gap is closed, so the area is
// exactly 100 and the centroid exactly (5, 5). Every end is an exact rational,
// so the closing chord is integrated exactly and the area stays Exact.
func TestProfileRecordClosesExactLineJunction(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	origin := s.CreatePoint(0, 0)
	base := s.CreatePoint(10, 0)
	top := s.CreatePoint(10, 10)
	left := s.CreatePoint(0, 10)
	s.CreateLine(origin, base)
	s.CreateLine(s.CreatePoint(10, -1e6), top)
	s.CreateLine(top, left)
	s.CreateLine(left, origin)

	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	cut := 0
	for _, edge := range profiles[0].Outer {
		if edge.Partial {
			require.True(t, edge.TExact)
			require.NotZero(t, edge.TStart)
			cut++
		}
	}
	require.Equal(t, 1, cut, `only the long side is cut, at the base corner`)

	record, _, err := momentinput.RecordProfile(s, profiles[0])
	require.NoError(t, err)
	area, err := record.Area()
	require.NoError(t, err)
	value, err := area.Value.In(units.SquareMillimeter)
	require.NoError(t, err)
	bound, err := area.Bound.In(units.SquareMillimeter)
	require.NoError(t, err)
	requireJunctionEncloses(t, `square area`, value, bound, jf(100))
	require.Equal(t, measurement.Exact, area.Exactness, `an exact closing chord keeps a line-only region exact`)

	centroid, err := record.Centroid()
	require.NoError(t, err)
	cb, err := centroid.Bound.In(units.Millimeter)
	require.NoError(t, err)
	requireJunctionEncloses(t, `centroid u`, centroid.Value.X, cb, jf(5))
	requireJunctionEncloses(t, `centroid v`, centroid.Value.Y, cb, jf(5))
}
