package meshbool

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/stretchr/testify/require"
)

// twoCallRimBound is the rim bound read straight from sin2, a fixed copy of
// RimBound's body: SinLowerBound runs inside it, apart from any sine the
// caller computes for itself.
func twoCallRimBound(deltaA, deltaB float64, sin2 *SinSquare) float64 {
	d := proofbound.AbsSumUpper(deltaA, deltaB)
	if d <= 0 {
		return 0
	}
	if sin2 == nil {
		return math.Inf(1)
	}
	return proofbound.DivUpper(d, SinLowerBound(sin2))
}

func TestRimBoundFromSinMatchesTwoCall(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(7, 11))
	big1 := new(big.Int).Lsh(big.NewInt(1), 200)
	sins := []*SinSquare{
		nil,
		{Num: big.NewInt(0), Den: big.NewInt(1)},
		{Num: big.NewInt(1), Den: big.NewInt(1)},
		{Num: big.NewInt(1), Den: big.NewInt(4)},
		{Num: big.NewInt(3), Den: big.NewInt(4)},
		{Num: big.NewInt(1), Den: new(big.Int).Set(big1)},
		{Num: new(big.Int).Set(big1), Den: new(big.Int).Add(big1, big.NewInt(1))},
	}
	for range 200 {
		num := big.NewInt(rng.Int64N(1<<40) + 1)
		den := new(big.Int).Add(num, big.NewInt(rng.Int64N(1<<40)))
		sins = append(sins, &SinSquare{Num: num, Den: den})
	}
	deltas := [][2]float64{{0, 0}, {1e-3, 0}, {1e-6, 2e-6}, {0.25, 0.5}, {1e300, 1e300}, {5e-324, 0}}
	for _, s2 := range sins {
		sin := 0.0
		if s2 != nil {
			sin = SinLowerBound(s2)
		}
		for _, d := range deltas {
			want := twoCallRimBound(d[0], d[1], s2)
			got := rimBoundFromSin(d[0], d[1], sin, s2 != nil)
			require.Equal(t, math.Float64bits(want), math.Float64bits(got))
			require.Equal(t, math.Float64bits(want), math.Float64bits(RimBound(d[0], d[1], s2)))
		}
	}
}

func TestRimBoundFromSinWrongSineDiffers(t *testing.T) {
	t.Parallel()
	s2 := &SinSquare{Num: big.NewInt(1), Den: big.NewInt(4)}
	wrong := math.Nextafter(SinLowerBound(s2), 1)
	want := twoCallRimBound(0.25, 0.5, s2)
	require.NotEqual(t, math.Float64bits(want), math.Float64bits(rimBoundFromSin(0.25, 0.5, wrong, true)))
}
