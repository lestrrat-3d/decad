package proof

import (
	"math/big"
	"math/rand"
	"testing"
)

func TestReplayReducedFractionMatches(t *testing.T) {
	r := rand.New(rand.NewSource(217))
	for i := range 250 {
		n := new(big.Int).Rand(r, new(big.Int).Lsh(big.NewInt(1), uint(1+i%1000)))
		d := new(big.Int).Rand(r, new(big.Int).Lsh(big.NewInt(1), uint(1+(i*7)%1000)))
		if i%2 == 0 {
			n.Neg(n)
		}
		if d.Sign() == 0 {
			d.SetInt64(1)
		}
		want := new(big.Rat).SetFrac(n, d)
		got, err := ReplayReducedFraction(t.Context(), n, d, new(keyTestBudget))
		if err != nil || got.Num().Cmp(want.Num()) != 0 || got.Denom().Cmp(want.Denom()) != 0 {
			t.Fatalf("case %d: %v", i, err)
		}
	}
}

func TestReplayRatArithmeticMatches(t *testing.T) {
	r := rand.New(rand.NewSource(918))
	for i := range 100 {
		makeRat := func() *big.Rat {
			n := new(big.Int).Rand(r, new(big.Int).Lsh(big.NewInt(1), uint(1+i%400)))
			d := new(big.Int).Rand(r, new(big.Int).Lsh(big.NewInt(1), uint(1+(i*7)%400)))
			if i%2 == 0 {
				n.Neg(n)
			}
			if d.Sign() == 0 {
				d.SetInt64(1)
			}
			return new(big.Rat).SetFrac(n, d)
		}
		a, b := makeRat(), makeRat()
		for _, op := range []byte{'+', '-', '*', '/'} {
			if op == '/' && b.Sign() == 0 {
				continue
			}
			var want big.Rat
			switch op {
			case '+':
				want.Add(a, b)
			case '-':
				want.Sub(a, b)
			case '*':
				want.Mul(a, b)
			case '/':
				want.Quo(a, b)
			}
			got, err := ReplayRatArithmetic(t.Context(), a, b, op, new(keyTestBudget))
			if err != nil || got.Num().Cmp(want.Num()) != 0 || got.Denom().Cmp(want.Denom()) != 0 {
				t.Fatalf("case %d op %c: %v", i, op, err)
			}
		}
		cmp, err := ReplayRatCompare(t.Context(), a, b, new(keyTestBudget))
		if err != nil || cmp != a.Cmp(b) {
			t.Fatalf("compare %d: %v", i, err)
		}
	}
}
