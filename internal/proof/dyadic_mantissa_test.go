package proof_test

import (
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
)

func TestDyadicMantIntoMatchesNormalizedInteger(t *testing.T) {
	rng := rand.New(rand.NewPCG(173, 179))
	var destination big.Int
	for sample := range 300 {
		width := []int{0, 1, 31, 32, 33, 63, 64, 65, 100, 126, 127, 128, 200, 400}[sample%14]
		words := make([]byte, (width+7)/8)
		for i := range words {
			words[i] = byte(rng.UintN(256))
		}
		mantissa := new(big.Int).SetBytes(words)
		if sample%2 == 0 {
			mantissa.Neg(mantissa)
		}
		expected := new(big.Int).Set(mantissa)
		if expected.Sign() != 0 {
			expected.Rsh(expected, expected.TrailingZeroBits())
		}
		d := dyadicFromParts(t, mantissa, rng.IntN(2401)-1200)
		before := d.Rat()
		if result := d.MantInto(&destination); result != &destination || destination.Cmp(expected) != 0 {
			t.Fatalf("sample %d: mantissa differs from the normalized input integer", sample)
		}
		destination.Add(&destination, big.NewInt(1))
		if d.Rat().Cmp(before) != 0 {
			t.Fatalf("sample %d: writing the destination changed the source", sample)
		}
	}
}

func TestDyadicMantIntoReusesDestination(t *testing.T) {
	wide := new(big.Int).Sub(new(big.Int).Lsh(big.NewInt(1), 127), big.NewInt(1))
	for _, d := range []proof.Dyadic{proof.MustDyOf(0.1), proof.MustDyOf(-0.1),
		dyadicFromParts(t, wide, -30), proof.DyZero()} {
		var destination big.Int
		d.MantInto(&destination)
		allocs := testing.AllocsPerRun(100, func() { d.MantInto(&destination) })
		if allocs != 0 {
			t.Fatalf("reusing an inline mantissa destination allocated %g times per call", allocs)
		}
	}
}
