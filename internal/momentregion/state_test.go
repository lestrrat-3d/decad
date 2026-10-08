package momentregion_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentline"
	"github.com/lestrrat-3d/decad/internal/momentregion"
	"github.com/lestrrat-3d/decad/internal/polynomial"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/stretchr/testify/require"
)

// regionSums is the storage a momentregion.State writes through.
type regionSums struct {
	coord     float64
	values    [6]float64
	bounds    [6]float64
	exact     freeform.ExactMoments
	exactDead bool
	third     [4]proofbound.RatInterval
	thirdDead bool
}

func (r *regionSums) state() momentregion.State {
	s := momentregion.State{CoordUpper: &r.coord, Exact: &r.exact, ExactDead: &r.exactDead, Third: &r.third, ThirdDead: &r.thirdDead}
	for i := range s.Fields {
		s.Fields[i] = momentregion.Field{Value: &r.values[i], Bound: &r.bounds[i]}
	}
	return s
}

// TestChargeJunctionCoversTheClosingChord holds ChargeJunction's widening,
// read with no exact chord, to the exact integral of the chord it stands for:
// every held field's bound and every third-order interval must cover that
// chord's own contribution, computed over rationals in the same boundary forms
// the segment kinds use (momentline.ExactChordMoments about the anchor,
// freeform.PolyThirdMoments about the origin). The chords run in several
// directions at several distances from the anchor, so no orientation of the
// gap escapes the charge.
//
// Shown-to-fail: dropping the held-field charge turns every field leg red,
// and dropping the third-order widening turns every third-order leg red.
func TestChargeJunctionCoversTheClosingChord(t *testing.T) {
	t.Parallel()
	anchor := sectionrecord.Point2{U: 3.25, V: -1.5}
	chords := [][2]sectionrecord.Point2{
		{{U: 10, V: 0}, {U: 10, V: 1e-9}},
		{{U: -7.5, V: 4}, {U: -7.5 + 3e-10, V: 4 - 2e-10}},
		{{U: 1000, V: 0.3}, {U: 1000 - 5e-8, V: 0.3}},
		{{U: 0.1, V: -250}, {U: 0.1 + 1e-6, V: -250 + 1e-6}},
	}
	for _, c := range chords {
		p, q := c[0], c[1]
		gap := proofbound.RatSqrtUp(ratSquareSum(sub(q.U, p.U), sub(q.V, p.V)))
		reach := math.Max(
			proofbound.RatSqrtUp(ratSquareSum(sub(p.U, anchor.U), sub(p.V, anchor.V))),
			proofbound.RatSqrtUp(ratSquareSum(sub(q.U, anchor.U), sub(q.V, anchor.V))),
		)
		originReach := math.Max(
			proofbound.RatSqrtUp(ratSquareSum(rat(p.U), rat(p.V))),
			proofbound.RatSqrtUp(ratSquareSum(rat(q.U), rat(q.V))),
		)

		var sums regionSums
		sums.third = [4]proofbound.RatInterval{
			proofbound.PointInterval(new(big.Rat)), proofbound.PointInterval(new(big.Rat)),
			proofbound.PointInterval(new(big.Rat)), proofbound.PointInterval(new(big.Rat)),
		}
		sums.state().ChargeJunction(gap, reach, originReach, nil, anchor, freeform.MomentThirdOrder)
		require.True(t, sums.exactDead, `a junction with no exact chord retires the rational sum`)

		exact := momentline.ExactChordMoments(sub(p.U, anchor.U), sub(p.V, anchor.V), sub(q.U, anchor.U), sub(q.V, anchor.V), freeform.MomentSecondOrder)
		for i, term := range exact.Fields() {
			magnitude := new(big.Rat).Abs(term)
			require.LessOrEqualf(t, magnitude.Cmp(rat(sums.bounds[i])), 0,
				"field %d: the chord contributes %s, beyond the charged bound %g", i, magnitude.FloatString(30), sums.bounds[i])
		}
		third := freeform.PolyThirdMoments(
			polynomial.RatPoly{rat(p.U), sub(q.U, p.U)},
			polynomial.RatPoly{rat(p.V), sub(q.V, p.V)},
		)
		for i, term := range third {
			require.Falsef(t, sums.thirdDead, `a finite widening keeps the third-order enclosure`)
			require.LessOrEqualf(t, sums.third[i].Lo.Cmp(term), 0, "third-order term %d below its widened enclosure", i)
			require.GreaterOrEqualf(t, sums.third[i].Hi.Cmp(term), 0, "third-order term %d above its widened enclosure", i)
		}
	}
}

// TestChargeJunctionAddsAnExactChord holds the exact arm: with both ends
// exact, the rational sum stays alive and gains exactly the chord's integral.
func TestChargeJunctionAddsAnExactChord(t *testing.T) {
	t.Parallel()
	anchor := sectionrecord.Point2{U: 1, V: 2}
	from := freeform.RatPoint{U: big.NewRat(10, 1), V: big.NewRat(1, 3)}
	to := freeform.RatPoint{U: big.NewRat(10, 1), V: big.NewRat(2, 3)}
	var sums regionSums
	sums.exact = momentregion.NewExactMoments()
	sums.state().ChargeJunction(1, 20, 20, &momentregion.ExactChord{From: from, To: to}, anchor, freeform.MomentSecondOrder)
	require.False(t, sums.exactDead)

	want := momentline.ExactChordMoments(
		new(big.Rat).Sub(from.U, rat(anchor.U)), new(big.Rat).Sub(from.V, rat(anchor.V)),
		new(big.Rat).Sub(to.U, rat(anchor.U)), new(big.Rat).Sub(to.V, rat(anchor.V)),
		freeform.MomentSecondOrder,
	)
	got := sums.exact.Fields()
	for i, term := range want.Fields() {
		require.Zerof(t, got[i].Cmp(term), "exact field %d", i)
	}
}

func rat(x float64) *big.Rat { return proofarith.FloatRat(x) }

func sub(a, b float64) *big.Rat { return new(big.Rat).Sub(rat(a), rat(b)) }

func ratSquareSum(a, b *big.Rat) *big.Rat {
	return new(big.Rat).Add(new(big.Rat).Mul(a, a), new(big.Rat).Mul(b, b))
}
