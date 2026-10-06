package proof_test

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/stretchr/testify/require"
)

// sharedDraw holds one random rational in both forms: as a big.Rat and as
// the SRat the same arithmetic produced over the shared denominator.
type sharedDraw struct {
	rat    *big.Rat
	shared proof.SRat
}

// sharedInputs draws rationals of every kind the certificate reads: zeros,
// float64 values of many scales, wide dyadics whose numerators leave the
// inline mantissa, and fractions with odd factors in their denominators.
func sharedInputs(r *rand.Rand, n int) []*big.Rat {
	odd := []int64{3, 5, 7, 9, 15, 21}
	out := make([]*big.Rat, 0, n)
	for range n {
		var x *big.Rat
		switch r.IntN(6) {
		case 0:
			x = new(big.Rat)
		case 1:
			x = big.NewRat(int64(r.IntN(41)-20), 1)
		case 2:
			x = new(big.Rat).SetFloat64(math.Ldexp(r.NormFloat64(), r.IntN(120)-60))
		case 3:
			// A sum across exponents: a numerator wider than the inline 127 bits.
			x = new(big.Rat).SetFloat64(r.NormFloat64())
			x.Add(x, new(big.Rat).SetFloat64(math.Ldexp(r.NormFloat64(), -100-r.IntN(60))))
		default:
			x = new(big.Rat).SetFloat64(math.Ldexp(r.NormFloat64(), r.IntN(40)-20))
			x.Quo(x, big.NewRat(odd[r.IntN(len(odd))], 1))
		}
		out = append(out, x)
	}
	return out
}

// newShared lifts inputs over a SharedDenom wide enough for every one of
// them, through Widen, and then forms products of up to three of them, so
// that values sit over several powers of the denominator.
func newShared(t *testing.T, r *rand.Rand, inputs []*big.Rat) (*proof.SharedDenom, []sharedDraw) {
	t.Helper()
	s := proof.NewSharedDenom(nil)
	for _, x := range inputs {
		s.Lift(x)
	}
	s, _ = s.Widen()
	draws := make([]sharedDraw, 0, 2*len(inputs))
	for _, x := range inputs {
		draws = append(draws, sharedDraw{rat: x, shared: s.Lift(x)})
	}
	_, widen := s.Widen()
	require.False(t, widen, "every input lifts over the widened denominator")
	for range inputs {
		a, b := draws[r.IntN(len(inputs))], draws[r.IntN(len(inputs))]
		product := sharedDraw{rat: new(big.Rat).Mul(a.rat, b.rat), shared: s.Mul(a.shared, b.shared)}
		if r.IntN(2) == 0 {
			c := draws[r.IntN(len(inputs))]
			product = sharedDraw{rat: product.rat.Mul(product.rat, c.rat), shared: s.Mul(product.shared, c.shared)}
		}
		draws = append(draws, product)
	}
	return s, draws
}

func requireSameRatTerms(t *testing.T, want, got *big.Rat, msgAndArgs ...any) {
	t.Helper()
	require.Zero(t, want.Cmp(got), append([]any{"want %s, got %s", want.RatString(), got.RatString()}, msgAndArgs...)...)
	require.Zero(t, want.Num().Cmp(got.Num()), msgAndArgs...)
	require.Zero(t, want.Denom().Cmp(got.Denom()), msgAndArgs...)
}

func requireSameSharedInterval(t *testing.T, s *proof.SharedDenom, want proof.RatInterval, got proof.SInterval,
	msgAndArgs ...any) {
	t.Helper()
	requireSameRatTerms(t, want.Lo, s.Rat(got.Lo), msgAndArgs...)
	requireSameRatTerms(t, want.Hi, s.Rat(got.Hi), msgAndArgs...)
}

func TestSharedDenomScalarsMatchRat(t *testing.T) {
	t.Parallel()
	r := rand.New(rand.NewPCG(31, 37))
	s, draws := newShared(t, r, sharedInputs(r, 400))
	for i := range 4000 {
		a, b := draws[r.IntN(len(draws))], draws[r.IntN(len(draws))]
		requireSameRatTerms(t, a.rat, s.Rat(a.shared), "draw %d", i)
		requireSameRatTerms(t, new(big.Rat).Add(a.rat, b.rat), s.Rat(s.Add(a.shared, b.shared)), "add %d", i)
		requireSameRatTerms(t, new(big.Rat).Sub(a.rat, b.rat), s.Rat(s.Sub(a.shared, b.shared)), "sub %d", i)
		requireSameRatTerms(t, new(big.Rat).Mul(a.rat, b.rat), s.Rat(s.Mul(a.shared, b.shared)), "mul %d", i)
		requireSameRatTerms(t, new(big.Rat).Neg(a.rat), s.Rat(proof.SNeg(a.shared)), "neg %d", i)
		requireSameRatTerms(t, new(big.Rat).Abs(a.rat), s.Rat(proof.SAbs(a.shared)), "abs %d", i)
		requireSameRatTerms(t, new(big.Rat).Quo(a.rat, big.NewRat(8, 1)), s.Rat(proof.SShift(a.shared, -3)), "shift %d", i)
		require.Equal(t, a.rat.Cmp(b.rat), s.Cmp(a.shared, b.shared), "cmp %d", i)
		require.Equal(t, a.rat.Sign(), a.shared.Sign(), "sign %d", i)
		wantF, wantExact := a.rat.Float64()
		gotF, gotExact := s.Float64(a.shared)
		require.Equal(t, math.Float64bits(wantF), math.Float64bits(gotF), "float %d", i)
		require.Equal(t, wantExact, gotExact, "float exactness %d", i)
	}
	for _, f := range []float64{0, 1, -0.75, 0x1p-1074, math.MaxFloat64} {
		x, ok := proof.SFloat(f)
		require.True(t, ok)
		requireSameRatTerms(t, new(big.Rat).SetFloat64(f), s.Rat(x), "float %g", f)
	}
	_, ok := proof.SFloat(math.Inf(1))
	require.False(t, ok)
	requireSameRatTerms(t, big.NewRat(-7, 1), s.Rat(proof.SInt(-7)))
}

// TestSharedDenomIntervalsMatchRatInterval holds every interval operation to
// its RatInterval twin, endpoint for endpoint and in lowest terms, over
// intervals of every sign pattern.
func TestSharedDenomIntervalsMatchRatInterval(t *testing.T) {
	t.Parallel()
	r := rand.New(rand.NewPCG(41, 43))
	s, draws := newShared(t, r, sharedInputs(r, 400))
	interval := func() (proof.RatInterval, proof.SInterval) {
		a, b := draws[r.IntN(len(draws))], draws[r.IntN(len(draws))]
		if r.IntN(5) == 0 {
			b = a
		}
		if a.rat.Cmp(b.rat) > 0 {
			a, b = b, a
		}
		return proof.Interval(a.rat, b.rat), proof.SInterval{Lo: a.shared, Hi: b.shared}
	}
	vector := func() ([3]proof.RatInterval, proof.SIVec3) {
		var old [3]proof.RatInterval
		var shared proof.SIVec3
		for axis := range 3 {
			old[axis], shared[axis] = interval()
		}
		return old, shared
	}
	for i := range 3000 {
		oldA, a := interval()
		oldB, b := interval()
		scale := draws[r.IntN(len(draws))]
		requireSameSharedInterval(t, s, proof.AddInterval(oldA, oldB), s.AddI(a, b), "add %d", i)
		requireSameSharedInterval(t, s, proof.SubInterval(oldA, oldB), s.SubI(a, b), "sub %d", i)
		requireSameSharedInterval(t, s, proof.NegInterval(oldA), proof.SNegI(a), "neg %d", i)
		requireSameSharedInterval(t, s, proof.ScaleInterval(oldA, scale.rat), s.ScaleI(a, scale.shared), "scale %d", i)
		requireSameSharedInterval(t, s, proof.MulInterval(oldA, oldB), s.MulI(a, b), "mul %d", i)
		requireSameSharedInterval(t, s, proof.PointInterval(scale.rat), proof.SPoint(scale.shared), "point %d", i)
		magnitude := new(big.Rat).Abs(oldA.Lo)
		if hi := new(big.Rat).Abs(oldA.Hi); hi.Cmp(magnitude) > 0 {
			magnitude = hi
		}
		requireSameRatTerms(t, magnitude, s.Rat(s.Magnitude(a)), "magnitude %d", i)

		oldU, u := vector()
		oldV, v := vector()
		var oldPoint [3]proof.RatInterval
		var point [3]proof.SRat
		for axis := range 3 {
			x := draws[r.IntN(len(draws))]
			oldPoint[axis], point[axis] = proof.PointInterval(x.rat), x.shared
		}
		requireSameSharedInterval(t, s, proof.DotInterval3(oldU, oldV), s.DotI3(u, v), "dot %d", i)
		cross, pointCross := s.CrossI3(u, v), s.PointCrossI3(point, v)
		wantCross, wantPointCross := proof.CrossInterval3(oldU, oldV), proof.CrossInterval3(oldPoint, oldV)
		var wantAdd, wantSub, wantScale [3]proof.RatInterval
		for axis := range 3 {
			wantAdd[axis] = proof.AddInterval(oldU[axis], oldV[axis])
			wantSub[axis] = proof.SubInterval(oldU[axis], oldV[axis])
			wantScale[axis] = proof.ScaleInterval(oldU[axis], scale.rat)
		}
		add, sub, scaled := s.AddI3(u, v), s.SubI3(u, v), s.ScaleI3(u, scale.shared)
		pointVector := proof.SPoint3(point)
		for axis := range 3 {
			requireSameSharedInterval(t, s, wantCross[axis], cross[axis], "cross %d axis %d", i, axis)
			requireSameSharedInterval(t, s, wantPointCross[axis], pointCross[axis], "point cross %d axis %d", i, axis)
			requireSameSharedInterval(t, s, wantAdd[axis], add[axis], "add3 %d axis %d", i, axis)
			requireSameSharedInterval(t, s, wantSub[axis], sub[axis], "sub3 %d axis %d", i, axis)
			requireSameSharedInterval(t, s, wantScale[axis], scaled[axis], "scale3 %d axis %d", i, axis)
			requireSameSharedInterval(t, s, oldPoint[axis], pointVector[axis], "point3 %d axis %d", i, axis)
		}
	}
}

// TestSharedDenomWiden reads a fraction whose odd denominator part the
// shared denominator lacks as zero and records it; the widened denominator
// covers it, and every lift over a covering denominator is exact.
func TestSharedDenomWiden(t *testing.T) {
	t.Parallel()
	s := proof.NewSharedDenom(nil)
	requireSameRatTerms(t, big.NewRat(1, 2), s.Rat(s.Lift(big.NewRat(1, 2))))
	_, widen := s.Widen()
	require.False(t, widen, "a dyadic needs no odd denominator")

	require.Zero(t, s.Lift(big.NewRat(2, 3)).Sign(), "2/3 has no numerator over 1")
	require.Zero(t, s.Lift(big.NewRat(-1, 20)).Sign(), "−1/20 has no numerator over 1")
	wide, widen := s.Widen()
	require.True(t, widen)
	for _, x := range []*big.Rat{big.NewRat(2, 3), big.NewRat(-1, 20), big.NewRat(7, 15), big.NewRat(5, 8)} {
		requireSameRatTerms(t, x, wide.Rat(wide.Lift(x)))
	}
	_, widen = wide.Widen()
	require.False(t, widen, "15 covers 3, 5 and 15")
	require.Zero(t, wide.Lift(big.NewRat(1, 7)).Sign(), "7 does not divide 15")
	wider, widen := wide.Widen()
	require.True(t, widen)
	for _, x := range []*big.Rat{big.NewRat(1, 7), big.NewRat(2, 3), big.NewRat(4, 105)} {
		requireSameRatTerms(t, x, wider.Rat(wider.Lift(x)))
	}
}
