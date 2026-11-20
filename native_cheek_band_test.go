package decad

import (
	"math/big"
	"testing"
)

func TestCutBandCapRegistration(t *testing.T) {
	held := big.NewRat(1, 1)
	for _, sign := range []int64{-1, 1} {
		step := big.NewRat(sign, 1<<55)
		exact := new(big.Rat).Add(held, step)
		gap, ok := roundedCapGap(exact, held)
		if !ok || gap.Cmp(new(big.Rat).Abs(step)) != 0 {
			t.Fatalf("incorrect cap gap for rounding direction %d", sign)
		}
	}
	if _, ok := roundedCapGap(big.NewRat(5, 4), held); ok {
		t.Fatal("accepted unrelated held level")
	}
}
