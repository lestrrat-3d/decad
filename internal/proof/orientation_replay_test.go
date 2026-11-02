package proof

import (
	"errors"
	"math/big"
	"math/rand"
	"testing"
)

func TestReplayXdotDiffSignMatches(t *testing.T) {
	r := rand.New(rand.NewSource(87))
	for i := range 300 {
		point := func() Xpt {
			v := [4]*big.Int{}
			for j := range v {
				v[j] = new(big.Int).Rand(r, new(big.Int).Lsh(big.NewInt(1), uint(1+(i*13+j*31)%900)))
				if j < 3 && (i+j)%2 == 0 {
					v[j].Neg(v[j])
				}
			}
			if v[3].Sign() == 0 {
				v[3].SetInt64(1)
			}
			return Xpt{X: v[0], Y: v[1], Z: v[2], W: v[3]}
		}
		n, p, q := point(), point(), point()
		got, err := ReplayXdotDiffSign(t.Context(), n, p, q, new(keyTestBudget))
		if err != nil || got != XdotDiffSign(n, p, q) {
			t.Fatalf("case %d: %v", i, err)
		}
	}
}
func TestReplayXdotDiffSignRefusesEachCharge(t *testing.T) {
	p := Xpt{X: big.NewInt(31), Y: big.NewInt(-43), Z: big.NewInt(7), W: big.NewInt(11)}
	b := new(keyTestBudget)
	if _, err := ReplayXdotDiffSign(t.Context(), p, p, p, b); err != nil {
		t.Fatal(err)
	}
	for i := uint64(1); i <= b.fail; i++ {
		if _, err := ReplayXdotDiffSign(t.Context(), p, p, p, &keyTestBudget{failAt: i}); !errors.Is(err, errKeyRefused) {
			t.Fatalf("charge %d: %v", i, err)
		}
	}
}
