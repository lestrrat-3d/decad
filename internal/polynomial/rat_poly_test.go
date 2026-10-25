package polynomial

import (
	"math/big"
	"math/rand/v2"
	"slices"
	"testing"
)

// The reference deliberately uses math/big's full normalization, independently
// of the exact-arithmetic helpers the production convolution uses.
func referenceRatProduct(a, b RatPoly) RatPoly {
	for len(a) > 0 && a[len(a)-1].Sign() == 0 {
		a = a[:len(a)-1]
	}
	for len(b) > 0 && b[len(b)-1].Sign() == 0 {
		b = b[:len(b)-1]
	}
	if len(a) == 0 || len(b) == 0 {
		return RatPoly{}
	}
	out := make(RatPoly, len(a)+len(b)-1)
	for i := range out {
		out[i] = new(big.Rat)
	}
	var product big.Rat
	for i, ai := range a {
		for j, bj := range b {
			product.Mul(ai, bj)
			out[i+j].Add(out[i+j], &product)
		}
	}
	return out
}

func TestRatPolynomialProductMatchesFullNormalization(t *testing.T) {
	rng := rand.New(rand.NewPCG(101, 107))
	row := func() RatPoly {
		out := make(RatPoly, rng.IntN(10))
		for i := range out {
			out[i] = randCoeff(rng)
			if rng.IntN(4) == 0 {
				out[i].SetInt64(0)
			}
		}
		return out
	}
	for sample := range 300 {
		a, b := row(), row()
		if sample%3 == 0 {
			b = a
		}
		before := make([]string, 0, len(a)+len(b))
		for _, input := range []RatPoly{a, b} {
			for _, coefficient := range input {
				before = append(before, coefficient.RatString())
			}
		}
		want, got := referenceRatProduct(a, b), RpMul(a, b)
		if len(got) != len(want) {
			t.Fatalf("sample %d: got %d coefficients, want %d", sample, len(got), len(want))
		}
		for i := range want {
			if got[i].Num().Cmp(want[i].Num()) != 0 || got[i].Denom().Cmp(want[i].Denom()) != 0 {
				t.Fatalf("sample %d: coefficient %d differs from full normalization", sample, i)
			}
			// Results own their storage, even for a square and constant factors.
			got[i].SetInt64(91)
		}
		index := 0
		for _, input := range []RatPoly{a, b} {
			for i, coefficient := range input {
				if coefficient.RatString() != before[index] {
					t.Fatalf("sample %d: input coefficient %d changed", sample, i)
				}
				index++
			}
		}
	}
}

func referenceRatEval(a RatPoly, x *big.Rat) *big.Rat {
	out := new(big.Rat)
	for _, coefficient := range slices.Backward(a) {
		out.Mul(out, x)
		out.Add(out, coefficient)
	}
	return out
}

func referenceRatRemainder(a, b RatPoly) RatPoly {
	rem := make(RatPoly, len(a))
	for i := range a {
		rem[i] = new(big.Rat).Set(a[i])
	}
	for len(b) > 0 && b[len(b)-1].Sign() == 0 {
		b = b[:len(b)-1]
	}
	if len(b) == 0 {
		return rem
	}
	var term big.Rat
	for len(rem) >= len(b) {
		for len(rem) > 0 && rem[len(rem)-1].Sign() == 0 {
			rem = rem[:len(rem)-1]
		}
		if len(rem) < len(b) {
			break
		}
		factor := new(big.Rat).Quo(rem[len(rem)-1], b[len(b)-1])
		shift := len(rem) - len(b)
		for j := range b {
			term.Mul(factor, b[j])
			rem[shift+j].Sub(rem[shift+j], &term)
		}
		rem = rem[:len(rem)-1]
	}
	for len(rem) > 0 && rem[len(rem)-1].Sign() == 0 {
		rem = rem[:len(rem)-1]
	}
	return rem
}

func TestRatPolynomialOperationsKeepNormalizedCoefficients(t *testing.T) {
	rng := rand.New(rand.NewPCG(109, 127))
	for sample := range 100 {
		a, b := RatPoly(randRow(rng)), RatPoly(randRow(rng))
		x := randCoeff(rng)
		check := func(op string, want, got RatPoly) {
			t.Helper()
			if len(want) != len(got) {
				t.Fatalf("sample %d %s: got %d coefficients, want %d", sample, op, len(got), len(want))
			}
			for i := range want {
				if want[i].Num().Cmp(got[i].Num()) != 0 || want[i].Denom().Cmp(got[i].Denom()) != 0 {
					t.Fatalf("sample %d %s: coefficient %d differs", sample, op, i)
				}
			}
		}
		check("evaluate", RatPoly{referenceRatEval(a, x)}, RatPoly{RpEval(a, x)})
		wantAdd := make(RatPoly, max(len(a), len(b)))
		wantScale := make(RatPoly, len(a))
		wantDeriv := make(RatPoly, len(a)-1)
		for i := range wantAdd {
			wantAdd[i] = new(big.Rat)
			if i < len(a) {
				wantAdd[i].Add(wantAdd[i], a[i])
			}
			if i < len(b) {
				wantAdd[i].Add(wantAdd[i], b[i])
			}
		}
		for i := range a {
			wantScale[i] = new(big.Rat).Mul(a[i], x)
			if i > 0 {
				wantDeriv[i-1] = new(big.Rat).Mul(a[i], big.NewRat(int64(i), 1))
			}
		}
		check("add", wantAdd, RpAdd(a, b))
		check("scale", wantScale, RpScale(a, x))
		check("derive", wantDeriv, RpDeriv(a))
		check("remainder", referenceRatRemainder(a, b), RpRem(a, b))
		check("quotient", RpTrim(a), RpQuo(referenceRatProduct(a, b), b))
	}
}
