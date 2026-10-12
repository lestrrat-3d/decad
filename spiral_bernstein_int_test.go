package decad

import (
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/stretchr/testify/require"
)

// The *Ref functions are the big.Rat formulations the integer primitives
// replaced. The differential tests compare against them value for value.

// Products of Bernstein polynomials retain exact rational coefficients.
// The largest degree used here is 23, whose binomial weights fit int64.
func spiralMulRef(a, b []*big.Rat) []*big.Rat {
	n, m := len(a)-1, len(b)-1
	out := make([]*big.Rat, n+m+1)
	for k := range out {
		out[k] = lfZero()
		for i := max(0, k-m); i <= min(n, k); i++ {
			j := k - i
			weight := big.NewRat(spiralBinom(n, i)*spiralBinom(m, j), spiralBinom(n+m, k))
			out[k] = lfAdd(out[k], lfMul(weight, lfMul(a[i], b[j])))
		}
	}
	return out
}

func spiralBernValueRef(coeff []*big.Rat, parameter *big.Rat) *big.Rat {
	work := append([]*big.Rat(nil), coeff...)
	for width := len(work); width > 1; width-- {
		for i := 0; i+1 < width; i++ {
			work[i] = lfAdd(lfMul(lfSub(lfOne(), parameter), work[i]),
				lfMul(parameter, work[i+1]))
		}
	}
	return work[0]
}

func spiralIsolateRootNRef(coeff []*big.Rat, steps int) spiralRootCrossing {
	lo, hi := lfZero(), lfOne()
	for range steps {
		mid := lfMul(lfAdd(lo, hi), big.NewRat(1, 2))
		if spiralBernValueRef(coeff, mid).Sign() < 0 {
			lo = mid
		} else {
			hi = mid
		}
	}
	return spiralRootCrossing{span: 0, lo: lo, hi: hi}
}

func spiralBernSplitRef(a []*big.Rat, at *big.Rat) ([]*big.Rat, []*big.Rat) {
	n := len(a) - 1
	left, right := make([]*big.Rat, n+1), make([]*big.Rat, n+1)
	work := append([]*big.Rat(nil), a...)
	for level := 0; level <= n; level++ {
		left[level], right[n-level] = work[0], work[n-level]
		for i := range n - level {
			work[i] = lfAdd(lfMul(lfSub(lfOne(), at), work[i]), lfMul(at, work[i+1]))
		}
	}
	return left, right
}

func spiralRootPolynomialRef(first, second loftFaceMap, span freeform.BezierSpan,
	m, b *big.Rat) [3][]*big.Rat {
	g0, h0 := spiralCapBern(first, span)
	g1, h1 := spiralCapBern(second, span)
	d := spiralMulRef(g0, g1)
	var a, z [3][]*big.Rat
	for axis := range a {
		a[axis] = spiralMulRef(h0[axis], g1)
		z[axis] = spiralMulRef(h1[axis], g0)
	}
	al := spiralAdd(spiralScale(a[0], m), spiralScale(d, b))
	zl := spiralAdd(spiralScale(z[0], m), spiralScale(d, b))
	dy, dz, dl := spiralSub(z[1], a[1]), spiralSub(z[2], a[2]), spiralSub(zl, al)
	q0 := spiralSub(spiralAdd(spiralMulRef(a[1], a[1]), spiralMulRef(a[2], a[2])), spiralMulRef(al, al))
	q1 := spiralScale(spiralSub(spiralAdd(spiralMulRef(a[1], dy), spiralMulRef(a[2], dz)),
		spiralMulRef(al, dl)), big.NewRat(2, 1))
	q2 := spiralSub(spiralAdd(spiralMulRef(dy, dy), spiralMulRef(dz, dz)), spiralMulRef(dl, dl))
	return [3][]*big.Rat{q0, spiralAdd(q0, spiralScale(q1, big.NewRat(1, 2))),
		spiralAdd(spiralAdd(q0, q1), q2)}
}

func spiralRootXOrderRef(first, second loftFaceMap, span freeform.BezierSpan,
	q [3][]*big.Rat) [3][]*big.Rat {
	g0, h0 := spiralCapBern(first, span)
	g1, h1 := spiralCapBern(second, span)
	d := spiralMulRef(g0, g1)
	a, z := spiralMulRef(h0[0], g1), spiralMulRef(h1[0], g0)
	b := spiralSub(z, a)
	bd := spiralMulRef(b, d)
	u0 := spiralSub(spiralMulRef(spiralDeriv(a), d), spiralMulRef(a, spiralDeriv(d)))
	u1 := spiralSub(spiralMulRef(spiralDeriv(b), d), spiralMulRef(b, spiralDeriv(d)))
	q0 := q[0]
	q1 := spiralScale(spiralSub(q[1], q[0]), big.NewRat(2, 1))
	q2 := spiralAdd(spiralSub(q[2], spiralScale(q[1], big.NewRat(2, 1))), q[0])
	j0 := spiralSub(spiralMulRef(bd, spiralDeriv(q0)), spiralMulRef(u0, q1))
	j1 := spiralSub(spiralMulRef(bd, spiralDeriv(q1)),
		spiralAdd(spiralScale(spiralMulRef(u0, q2), big.NewRat(2, 1)), spiralMulRef(u1, q1)))
	j2 := spiralSub(spiralMulRef(bd, spiralDeriv(q2)),
		spiralScale(spiralMulRef(u1, q2), big.NewRat(2, 1)))
	return [3][]*big.Rat{j0, spiralAdd(j0, spiralScale(j1, big.NewRat(1, 2))),
		spiralAdd(spiralAdd(j0, j1), j2)}
}

func randomRatRow(rng *rand.Rand, n int) []*big.Rat {
	out := make([]*big.Rat, n)
	for i := range out {
		// Float-derived rationals have 2^-k denominators; mix in odd ones too.
		num := big.NewInt(rng.Int64N(1<<40) - 1<<39)
		num.Mul(num, big.NewInt(rng.Int64N(1<<30)+1))
		den := new(big.Int).Lsh(big.NewInt(1), uint(rng.IntN(60)))
		if rng.IntN(3) == 0 {
			den.Mul(den, big.NewInt(rng.Int64N(1000)+1))
		}
		out[i] = new(big.Rat).SetFrac(num, den)
	}
	return out
}

func randomParameter(rng *rand.Rand) *big.Rat {
	switch rng.IntN(4) {
	case 0:
		return big.NewRat(rng.Int64N(1<<20), 1<<20)
	case 1:
		return new(big.Rat).SetFrac(big.NewInt(rng.Int64N(1<<62)), new(big.Int).Lsh(big.NewInt(1), 64))
	case 2:
		return big.NewRat(rng.Int64N(997), 997)
	default:
		return big.NewRat(rng.Int64N(3)-1, 1)
	}
}

func requireSameRows(t *testing.T, want, got []*big.Rat) {
	t.Helper()
	require.Len(t, got, len(want))
	for i := range want {
		require.Equal(t, 0, want[i].Cmp(got[i]), "index %d", i)
		require.Equal(t, want[i].Num().String(), got[i].Num().String())
		require.Equal(t, want[i].Denom().String(), got[i].Denom().String())
	}
}

func TestSpiralBernsteinIntegerMatchesReference(t *testing.T) {
	rng := rand.New(rand.NewPCG(7, 11))
	for round := range 72 {
		n := rng.IntN(13) + 1
		a := randomRatRow(rng, n+1)
		m := rng.IntN(13) + 1
		b := randomRatRow(rng, m+1)
		s := randomParameter(rng)
		requireSameRows(t, spiralMulRef(a, b), spiralMul(a, b))
		product := spiralMul(a, b)
		pl, pr := spiralBernSplitRef(product, s)
		gl2, gr2 := spiralBernSplit(product, s)
		requireSameRows(t, pl, gl2)
		requireSameRows(t, pr, gr2)
		requireSameRows(t, []*big.Rat{spiralBernValueRef(product, s)}, []*big.Rat{spiralBernValue(product, s)})
		pRoot, gRoot := spiralIsolateRootNRef(product, 20), spiralIsolateRootN(product, 20)
		requireSameRows(t, []*big.Rat{pRoot.lo, pRoot.hi}, []*big.Rat{gRoot.lo, gRoot.hi})
		want, got := spiralBernValueRef(a, s), spiralBernValue(a, s)
		requireSameRows(t, []*big.Rat{want}, []*big.Rat{got})
		require.Equal(t, want.Sign(), spiralBernSign(a, s))
		wl, wr := spiralBernSplitRef(a, s)
		gl, gr := spiralBernSplit(a, s)
		requireSameRows(t, wl, gl)
		requireSameRows(t, wr, gr)
		steps := []int{0, 1, 5, 64, 80, 128}[round%6]
		wantRoot, gotRoot := spiralIsolateRootNRef(a, steps), spiralIsolateRootN(a, steps)
		requireSameRows(t, []*big.Rat{wantRoot.lo, wantRoot.hi}, []*big.Rat{gotRoot.lo, gotRoot.hi})
	}
}

func TestSpiralRootRowsMatchReference(t *testing.T) {
	rng := rand.New(rand.NewPCG(13, 17))
	for range 12 {
		first, second := randomGateMap(rng), randomGateMap(rng)
		span := make(freeform.BezierSpan, 4)
		for i := range span {
			span[i] = freeform.RatPoint{U: lfF(rng.Float64()*4 - 2), V: lfF(rng.Float64()*4 - 2)}
		}
		m, b := lfF(rng.Float64()*2-1), lfF(rng.Float64()*2-1)
		want := spiralRootPolynomialRef(first, second, span, m, b)
		got := spiralRootPolynomial(first, second, span, m, b)
		for i := range want {
			requireSameRows(t, want[i], got[i])
		}
		wantX := spiralRootXOrderRef(first, second, span, want)
		gotX := spiralRootXOrder(first, second, span, got)
		for i := range wantX {
			requireSameRows(t, wantX[i], gotX[i])
		}
	}
}
