package proof

import (
	"math"
	"math/big"
	"math/rand"
	"testing"
)

func TestReplayXptVecMatches(t *testing.T) {
	r := rand.New(rand.NewSource(91))
	for i := range 300 {
		width := uint(1 + i%1500)
		den := new(big.Int).Rand(r, new(big.Int).Lsh(big.NewInt(1), width))
		if den.Sign() == 0 {
			den.SetInt64(1)
		}
		vals := [3]*big.Int{}
		for j := range vals {
			vals[j] = new(big.Int).Rand(r, new(big.Int).Lsh(big.NewInt(1), width))
			if (i+j)%2 == 0 {
				vals[j].Neg(vals[j])
			}
		}
		p := Xpt{X: vals[0], Y: vals[1], Z: vals[2], W: den}
		want := p.Vec()
		got, err := ReplayXptVec(t.Context(), p, new(keyTestBudget))
		if err != nil {
			t.Fatalf("case %d: %v", i, err)
		}
		if math.Float64bits(got.X) != math.Float64bits(want.X) || math.Float64bits(got.Y) != math.Float64bits(want.Y) || math.Float64bits(got.Z) != math.Float64bits(want.Z) {
			t.Fatalf("case %d differs", i)
		}
	}
}
