package proof

import (
	"math/big"
	"math/rand"
	"testing"
)

func TestReplayProductDifferenceMatches(t *testing.T) {
	r := rand.New(rand.NewSource(77))
	for i := range 400 {
		v := [4]*big.Int{}
		for j := range v {
			v[j] = new(big.Int).Rand(r, new(big.Int).Lsh(big.NewInt(1), uint(1+(i*11+j*37)%1200)))
			if (i+j)%3 == 0 {
				v[j].Neg(v[j])
			}
		}
		want := new(big.Int).Sub(new(big.Int).Mul(v[0], v[1]), new(big.Int).Mul(v[2], v[3]))
		got, err := ReplayProductDifference(t.Context(), v[0], v[1], v[2], v[3], new(keyTestBudget))
		if err != nil || got.Cmp(want) != 0 {
			t.Fatalf("case %d: %v", i, err)
		}
	}
}

func TestReplayXlerpMatches(t *testing.T) {
	r := rand.New(rand.NewSource(104))
	for i := range 100 {
		point := func() Xpt {
			v := [4]*big.Int{}
			for j := range v {
				v[j] = new(big.Int).Rand(r, new(big.Int).Lsh(big.NewInt(1), uint(1+(i*3+j*17)%300)))
				if j < 3 && (i+j)%2 == 0 {
					v[j].Neg(v[j])
				}
			}
			if v[3].Sign() == 0 {
				v[3].SetInt64(1)
			}
			return Xpt{X: v[0], Y: v[1], Z: v[2], W: v[3]}
		}
		a, b := point(), point()
		n := big.NewInt(int64(i - 50))
		d := big.NewInt(int64(i + 1))
		want := Xlerp(a, b, n, d)
		got, err := ReplayXlerp(t.Context(), a, b, n, d, new(keyTestBudget))
		if err != nil || want.X.Cmp(got.X) != 0 || want.Y.Cmp(got.Y) != 0 || want.Z.Cmp(got.Z) != 0 || want.W.Cmp(got.W) != 0 {
			t.Fatalf("case %d: %v", i, err)
		}
	}
}
