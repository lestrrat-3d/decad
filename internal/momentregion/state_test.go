package momentregion_test

import (
	"math"
	"math/big"
	"math/rand/v2"
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

// TestAddExactMatchesRatAdd folds 300 random exact contributions into a State
// and into six plain big.Rat sums that use Rat.Add, and requires the same
// numerator and denominator in every field after each fold. The terms mix zero,
// integers, float lifts (power-of-two denominators), fractions over odd
// denominators and huge numerators over huge denominators, of either sign.
func TestAddExactMatchesRatAdd(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(21, 23))
	term := func() *big.Rat {
		sign := int64(1 - 2*rng.IntN(2))
		switch rng.IntN(6) {
		case 0:
			return new(big.Rat)
		case 1:
			return new(big.Rat).SetInt64(sign * rng.Int64N(40))
		case 2:
			return proofarith.FloatRat(rng.NormFloat64())
		case 3:
			return proofarith.FloatRat(math.Ldexp(rng.NormFloat64(), rng.IntN(200)-100))
		case 4:
			return new(big.Rat).SetFrac(big.NewInt(sign*rng.Int64N(1000)), big.NewInt(2*rng.Int64N(500)+1))
		}
		num := new(big.Int).Lsh(big.NewInt(rng.Int64N(1<<40)+1), uint(rng.IntN(200)))
		den := new(big.Int).Lsh(big.NewInt(rng.Int64N(1<<40)*2+1), uint(rng.IntN(200)))
		return new(big.Rat).SetFrac(num.Mul(num, big.NewInt(sign)), den)
	}
	var sums regionSums
	sums.exact = momentregion.NewExactMoments()
	state := sums.state()
	var want [6]*big.Rat
	for i := range want {
		want[i] = new(big.Rat)
	}
	for step := range 300 {
		exact := momentregion.NewExactMoments()
		terms := [6]*big.Rat{term(), term(), term(), term(), term(), term()}
		exact.Area, exact.Mu, exact.Mv, exact.Muu, exact.Muv, exact.Mvv = terms[0], terms[1], terms[2], terms[3], terms[4], terms[5]
		state.AddExact(exact)
		got := sums.exact.Fields()
		for i := range want {
			want[i].Add(want[i], terms[i])
			require.True(t, got[i].Num().Cmp(want[i].Num()) == 0 && got[i].Denom().Cmp(want[i].Denom()) == 0,
				"step %d field %d: got %v, want %v", step, i, got[i], want[i])
		}
	}
}

// TestChargeJunctionAddsTheExactThirdOrderChord starts the third-order sum at
// a known point and requires ChargeJunction with an exact chord to add
// freeform.PolyThirdMoments of the chord's two coordinate polynomials, each
// with the chord's difference taken through big.Rat.Sub, as a point.
func TestChargeJunctionAddsTheExactThirdOrderChord(t *testing.T) {
	t.Parallel()
	anchor := sectionrecord.Point2{U: 1, V: 2}
	chord := &momentregion.ExactChord{
		From: freeform.RatPoint{U: big.NewRat(7, 3), V: big.NewRat(-1, 5)},
		To:   freeform.RatPoint{U: big.NewRat(11, 2), V: big.NewRat(9, 4)},
	}
	var sums regionSums
	sums.exact = momentregion.NewExactMoments()
	var start [4]proofbound.RatInterval
	for i := range start {
		start[i] = proofbound.PointInterval(big.NewRat(int64(i+1), 7))
	}
	sums.third = start
	sums.state().ChargeJunction(1, 20, 20, chord, anchor, freeform.MomentThirdOrder)
	require.False(t, sums.thirdDead)

	exact := freeform.PolyThirdMoments(
		polynomial.RatPoly{chord.From.U, new(big.Rat).Sub(chord.To.U, chord.From.U)},
		polynomial.RatPoly{chord.From.V, new(big.Rat).Sub(chord.To.V, chord.From.V)},
	)
	for i, value := range exact {
		want := new(big.Rat).Add(start[i].Lo, value)
		require.Zerof(t, sums.third[i].Lo.Cmp(want), "third-order term %d lower end", i)
		require.Zerof(t, sums.third[i].Hi.Cmp(want), "third-order term %d upper end", i)
	}
}
