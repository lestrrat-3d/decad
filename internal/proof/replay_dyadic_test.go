package proof

import (
	"context"
	"math"
	"math/rand"
	"testing"
)

func TestReplayDyadicArithmetic(t *testing.T) {
	rng := rand.New(rand.NewSource(44))
	for i := range 200 {
		a := DyOfFinite(math.Ldexp(rng.Float64()-.5, rng.Intn(120)-60))
		b := DyOfFinite(math.Ldexp(rng.Float64()-.5, rng.Intn(120)-60))
		for _, op := range []byte{'+', '-', '*'} {
			got, err := ReplayDyadicArithmetic(context.Background(), a, b, op, &keyTestBudget{})
			if err != nil {
				t.Fatal(err)
			}
			var want Dyadic
			switch op {
			case '+':
				want = DyAdd(a, b)
			case '-':
				want = DySubScalar(a, b)
			case '*':
				want = DyMul(a, b)
			}
			if DyCmp(got, want) != 0 {
				t.Fatalf("case %d operation %c differs", i, op)
			}
		}
	}
}
