package apitest_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/stretchr/testify/require"
)

// This file pins docs/clearance-design.md §5's closed-form readings: every
// cell reads its distance as a proven enclosure of the truth, never as a
// single float its own products, square roots and differences rounded. Each
// fixture is built from exact integer (or float-exact) coordinates and
// unplaced, axis-aligned bodies, so no body carries a displacement that could
// widen the row, and each truth is a + s·√q for rationals a and q, compared
// exactly by squaring.

// requireGapEnclosesSurd requires the row's interval [v − b, v + b] to hold
// a + s·√q, s = ±1, decided exactly over the rationals.
func requireGapEnclosesSurd(t *testing.T, gap decad.Measurement, a *big.Rat, s int, q *big.Rat) {
	t.Helper()
	v, b := ratOf(gap.Value.Mag()), ratOf(gap.Bound.Mag())
	lo, hi := new(big.Rat).Sub(v, b), new(big.Rat).Add(v, b)
	// a + s·√q ∈ [lo, hi] is √q ∈ [x, y].
	x, y := new(big.Rat).Sub(lo, a), new(big.Rat).Sub(hi, a)
	if s < 0 {
		x, y = new(big.Rat).Sub(a, hi), new(big.Rat).Sub(a, lo)
	}
	atLeastX := x.Sign() <= 0 || new(big.Rat).Mul(x, x).Cmp(q) <= 0
	atMostY := y.Sign() >= 0 && new(big.Rat).Mul(y, y).Cmp(q) >= 0
	require.Truef(t, atLeastX && atMostY,
		"the gap %.17g (%v) with bound %g excludes %v %+d·√%v", gap.Value.Mag(), gap.Exactness, gap.Bound.Mag(), a, s, q)
}

// TestClearanceCellReadingsContainTruth places each closed-form cell family
// where its float reading rounds, and requires the published row to hold
// the exact gap.
//
// Shown to fail: before the cells read their distances through
// internal/clearance's Dist enclosures, every subtest published its float
// reading as Exact with a zero bound, off the truth by about one rounding:
//
//	vertex × vertex √3:                   1.7320508075688772, 1.0e-16 off
//	vertex × slanted plane √2:            1.4142135623730949, 1.3e-16 off
//	edge × slanted plane √2:              1.4142135623730949, 1.3e-16 off
//	edge × cylinder (3.5·√2 − 3):         1.9497474683058327, 5.3e-18 off
//	vertex × circular edge √5:            2.2360679774997894, 3.4e-16 off
//	cylinder × cylinder (√10 − 2):        1.1622776601683791, 2.5e-16 off
//	cylinder × cylinder (1 − 0.3 − 0.6):  0.099999999999999978, 5.6e-17 off
func TestClearanceCellReadingsContainTruth(t *testing.T) {
	t.Parallel()
	zero := new(big.Rat)
	t.Run("vertex×vertex", func(t *testing.T) {
		t.Parallel()
		// The prism's corner (10, 0, 10) against the box's corner
		// (11, −1, 11): √3 apart.
		doc := decad.New()
		polyPrismBody(t, doc, [][2]float64{{0, 0}, {10, 0}, {0, 10}}, 10)
		boxBodyAtZ(t, doc, 11, -2, 12, -1, 11, 1)
		requireGapEnclosesSurd(t, clearanceRow(t, doc), zero, 1, big.NewRat(3, 1))
	})
	t.Run("vertex×slanted plane", func(t *testing.T) {
		t.Parallel()
		// The wall x + y = 10 against the corner (6, 6) of a triangle prism
		// pointing at it: √2 apart.
		doc := decad.New()
		polyPrismBody(t, doc, [][2]float64{{0, 0}, {10, 0}, {0, 10}}, 10)
		polyPrismBody(t, doc, [][2]float64{{6, 6}, {9, 7}, {7, 9}}, 5)
		requireGapEnclosesSurd(t, clearanceRow(t, doc), zero, 1, big.NewRat(2, 1))
	})
	t.Run("edge×slanted plane", func(t *testing.T) {
		t.Parallel()
		// The same wall against a box's vertical edge at (6, 6): √2 apart.
		doc := decad.New()
		polyPrismBody(t, doc, [][2]float64{{0, 0}, {10, 0}, {0, 10}}, 10)
		boxBodyAtZ(t, doc, 6, 6, 7, 7, 2, 3)
		requireGapEnclosesSurd(t, clearanceRow(t, doc), zero, 1, big.NewRat(2, 1))
	})
	t.Run("edge×cylinder", func(t *testing.T) {
		t.Parallel()
		// A disc of radius 3 against a box's vertical edge at (3.5, 3.5):
		// 3.5·√2 − 3 = √24.5 − 3.
		doc := decad.New()
		diskBody(t, doc, 0, 0, 3)
		boxBodyAtZ(t, doc, 3.5, 3.5, 4.5, 4.5, 2, 3)
		requireGapEnclosesSurd(t, clearanceRow(t, doc), big.NewRat(-3, 1), 1, big.NewRat(49, 2))
	})
	t.Run("vertex×circular edge", func(t *testing.T) {
		t.Parallel()
		// A disc of radius 3 over z ∈ [0, 20] against the corner (3, 4, 21)
		// of a box above and outside its rim: ρ = 5, so the corner lies
		// √(1² + (5 − 3)²) = √5 from the rim circle.
		doc := decad.New()
		diskBody(t, doc, 0, 0, 3)
		boxBodyAtZ(t, doc, 3, 4, 4, 5, 21, 3)
		requireGapEnclosesSurd(t, clearanceRow(t, doc), zero, 1, big.NewRat(5, 1))
	})
	t.Run("cylinder×cylinder root", func(t *testing.T) {
		t.Parallel()
		// Two unit rods with parallel axes √10 apart, overlapping axially.
		doc := decad.New()
		diskBody(t, doc, 0, 0, 1)
		rodBody(t, doc, 3, 1, 1, 5)
		requireGapEnclosesSurd(t, clearanceRow(t, doc), big.NewRat(-2, 1), 1, big.NewRat(10, 1))
	})
	t.Run("cylinder×cylinder radii", func(t *testing.T) {
		t.Parallel()
		// Rods of radii 0.3 and 0.6 on axes 1 apart: the gap is exactly
		// 1 − fl(0.3) − fl(0.6), which no float holds.
		doc := decad.New()
		diskBody(t, doc, 0, 0, 0.3)
		diskBody(t, doc, 1, 0, 0.6)
		truth := new(big.Rat).Sub(new(big.Rat).Sub(big.NewRat(1, 1), ratOf(0.3)), ratOf(0.6))
		requireGapEnclosesSurd(t, clearanceRow(t, doc), truth, 1, zero)
	})
}

// TestClearanceCellReadingsStayExact pins the other half of §5's reading: a
// cell whose exact value a float holds keeps an Exact row with a zero bound.
// The prism's corner (10, 0, 10) lies 13 from the box's corner
// (13, −4, 22), and a disc of radius fl(0.1) lies fl(0.3) − fl(0.1) from the
// box face x = fl(0.3), a difference the float holds exactly.
func TestClearanceCellReadingsStayExact(t *testing.T) {
	t.Parallel()
	t.Run("vertex×vertex", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		polyPrismBody(t, doc, [][2]float64{{0, 0}, {10, 0}, {0, 10}}, 10)
		boxBodyAtZ(t, doc, 13, -5, 14, -4, 22, 1)
		gap := clearanceRow(t, doc)
		require.Equal(t, decad.Exact, gap.Exactness)
		require.Equal(t, 13.0, gap.Value.Mag())
		require.Zero(t, gap.Bound.Mag())
	})
	t.Run("plane×cylinder", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		diskBody(t, doc, 0, 0, 0.1)
		boxBodyAtZ(t, doc, 0.3, -1, 1.3, 1, 2, 3)
		gap := clearanceRow(t, doc)
		require.Equal(t, decad.Exact, gap.Exactness)
		truth := new(big.Rat).Sub(ratOf(0.3), ratOf(0.1))
		require.Zero(t, ratOf(gap.Value.Mag()).Cmp(truth), "the gap %.17g is not fl(0.3) − fl(0.1)", gap.Value.Mag())
		require.Zero(t, gap.Bound.Mag())
	})
}
