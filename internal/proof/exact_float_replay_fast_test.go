package proof

import (
	"math"
	"math/big"
	"math/rand"
	"testing"
)

func TestReplayFractionFloat64Wide(t *testing.T) {
	rng := rand.New(rand.NewSource(19))
	for i := range 2000 {
		n := new(big.Int).SetUint64(rng.Uint64() | 1)
		d := new(big.Int).SetUint64(rng.Uint64() | 1)
		n.Lsh(n, uint(rng.Intn(1800)))
		d.Lsh(d, uint(rng.Intn(1800)))
		n.Add(n, new(big.Int).SetUint64(rng.Uint64()))
		d.Add(d, new(big.Int).SetUint64(rng.Uint64()|1))
		if i%2 == 0 {
			n.Neg(n)
		}
		want, _ := new(big.Rat).SetFrac(n, d).Float64()
		got, err := ReplayFractionFloat64(t.Context(), n, d, &keyTestBudget{})
		if err != nil {
			t.Fatalf("case %d: %v", i, err)
		}
		if math.Float64bits(got) != math.Float64bits(want) {
			t.Fatalf("case %d: got %x want %x", i, math.Float64bits(got), math.Float64bits(want))
		}
	}
}

func TestReplayFractionFloat64Ties(t *testing.T) {
	for _, f := range []float64{0, math.SmallestNonzeroFloat64, math.Float64frombits(0x000fffffffffffff), 1, math.Nextafter(1, 2), math.MaxFloat64 / 2} {
		next := math.Nextafter(f, math.Inf(1))
		mid := new(big.Rat).Add(new(big.Rat).SetFloat64(f), new(big.Rat).SetFloat64(next))
		mid.Quo(mid, big.NewRat(2, 1))
		for _, sign := range []int64{-1, 1} {
			q := new(big.Rat).Mul(mid, big.NewRat(sign, 1))
			want, _ := q.Float64()
			got, err := ReplayFractionFloat64(t.Context(), q.Num(), q.Denom(), &keyTestBudget{})
			if err != nil || math.Float64bits(got) != math.Float64bits(want) {
				t.Fatalf("tie at %g sign %d: got %x want %x: %v", f, sign, math.Float64bits(got), math.Float64bits(want), err)
			}
		}
	}
}
