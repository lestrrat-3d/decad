package loftface

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/stretchr/testify/require"
)

// randRatOperand draws one operand of every shape the loftface helpers see:
// a dyadic float, a fraction with a mixed denominator, zero or a small
// integer.
func randRatOperand(rng *rand.Rand) *big.Rat {
	switch pick := rng.IntN(10); {
	case pick == 0:
		return new(big.Rat)
	case pick < 4:
		v := math.Ldexp(rng.Float64()+0.5, rng.IntN(121)-60)
		if rng.IntN(2) == 0 {
			v = -v
		}
		return ratF(v)
	case pick < 6:
		return big.NewRat(int64(rng.IntN(41)-20), 1)
	default:
		q := new(big.Int).Lsh(big.NewInt(1), uint(rng.IntN(65)))
		q.Mul(q, new(big.Int).Exp(big.NewInt(3), big.NewInt(int64(rng.IntN(21))), nil))
		q.Mul(q, new(big.Int).Exp(big.NewInt(5), big.NewInt(int64(rng.IntN(21))), nil))
		q.Mul(q, big.NewInt(int64(2*rng.IntN(1<<19)+1)))
		p := big.NewInt(rng.Int64N(1<<62) - 1<<61)
		return new(big.Rat).SetFrac(p, q)
	}
}

func requireRatEqual(t *testing.T, want, got *big.Rat) {
	t.Helper()
	require.Zero(t, want.Num().Cmp(got.Num()), "numerator: want %s, got %s", want, got)
	require.Zero(t, want.Denom().Cmp(got.Denom()), "denominator: want %s, got %s", want, got)
}

func TestRatHelpersMatchBigRat(t *testing.T) {
	rng := rand.New(rand.NewPCG(41, 43))
	for range 3000 {
		a, b := randRatOperand(rng), randRatOperand(rng)
		a0, b0 := new(big.Rat).Set(a), new(big.Rat).Set(b)

		requireRatEqual(t, new(big.Rat).Add(a, b), ratAdd(a, b))
		requireRatEqual(t, new(big.Rat).Sub(a, b), ratSub(a, b))
		requireRatEqual(t, new(big.Rat).Mul(a, b), ratMul(a, b))
		requireRatEqual(t, new(big.Rat).Mul(a, a), ratMul(a, a))
		if b.Sign() != 0 {
			requireRatEqual(t, new(big.Rat).Quo(a, b), ratDiv(a, b))
		}
		require.Zero(t, a.Cmp(a0))
		require.Zero(t, b.Cmp(b0))
	}
}

// legacyCapBernsteinRows is CapBernsteinRows over raw big.Rat operations.
func legacyCapBernsteinRows(m Map, span freeform.BezierSpan) ([]*big.Rat, [3][]*big.Rat) {
	add := func(a, b *big.Rat) *big.Rat { return new(big.Rat).Add(a, b) }
	mul := func(a, b *big.Rat) *big.Rat { return new(big.Rat).Mul(a, b) }
	g := make([]*big.Rat, len(span))
	var h [3][]*big.Rat
	for axis := range h {
		h[axis] = make([]*big.Rat, len(span))
	}
	for i, p := range span {
		g[i] = add(add(m.G0, mul(m.GU, p.U)), mul(m.GV, p.V))
		for axis := range h {
			h[axis][i] = add(mul(m.Constant[axis], g[i]),
				add(add(m.Q0[axis], mul(m.QU[axis], p.U)), mul(m.QV[axis], p.V)))
		}
	}
	return g, h
}

func TestCapBernsteinRowsMatchLegacy(t *testing.T) {
	rng := rand.New(rand.NewPCG(41, 44))
	vec := func() RatVec {
		return RatVec{randRatOperand(rng), randRatOperand(rng), randRatOperand(rng)}
	}
	for range 200 {
		m := Map{
			Constant: vec(), Q0: vec(), QU: vec(), QV: vec(),
			G0: randRatOperand(rng), GU: randRatOperand(rng), GV: randRatOperand(rng),
		}
		span := make(freeform.BezierSpan, 2+rng.IntN(5))
		for i := range span {
			span[i] = freeform.RatPoint{U: randRatOperand(rng), V: randRatOperand(rng)}
		}
		wantG, wantH := legacyCapBernsteinRows(m, span)
		gotG, gotH := CapBernsteinRows(m, span)
		require.Len(t, gotG, len(wantG))
		for i := range wantG {
			requireRatEqual(t, wantG[i], gotG[i])
			for axis := range wantH {
				requireRatEqual(t, wantH[axis][i], gotH[axis][i])
			}
		}
	}
}
