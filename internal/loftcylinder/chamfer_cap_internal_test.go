package loftcylinder

import (
	"math/big"
	"math/rand"
	"testing"

	"github.com/stretchr/testify/require"
)

func chamferSegmentGapAboveReference(a, b, c, d XY, bound2 *big.Rat) bool {
	if EdgesMeet(a, b, c, d) {
		return false
	}
	for _, gap2 := range [4]*big.Rat{
		chamferPointSegmentGap2(a, c, d),
		chamferPointSegmentGap2(b, c, d),
		chamferPointSegmentGap2(c, a, b),
		chamferPointSegmentGap2(d, a, b),
	} {
		if gap2 == nil || gap2.Cmp(bound2) <= 0 {
			return false
		}
	}
	return true
}

func TestLoftCylinderChamferSegmentGapAboveMatchesExact(t *testing.T) {
	point := func(x, y int64) XY {
		return XY{X: big.NewRat(x, 1), Y: big.NewRat(y, 1)}
	}
	tiny := new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), 100))
	near := XY{X: new(big.Rat).Add(big.NewRat(1, 1), tiny), Y: big.NewRat(0, 1)}
	for _, tc := range []struct {
		name       string
		a, b, c, d XY
		bound2     *big.Rat
		want       bool
	}{
		{"separated axis", point(0, 0), point(0, 1), point(3, 0), point(3, 1), big.NewRat(1, 1), true},
		{"equal bound", point(0, 0), point(0, 1), point(1, 0), point(1, 1), big.NewRat(1, 1), false},
		{"rational near miss", point(0, 0), point(0, 1), near,
			XY{X: near.X, Y: big.NewRat(1, 1)}, big.NewRat(1, 1), true},
		{"crossing", point(0, 0), point(2, 2), point(0, 2), point(2, 0), big.NewRat(1, 1), false},
		{"degenerate", point(0, 0), point(0, 0), point(3, 0), point(3, 1), big.NewRat(1, 1), false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			require.Equal(t, tc.want, chamferSegmentGapAboveReference(tc.a, tc.b, tc.c, tc.d, tc.bound2))
			require.Equal(t, tc.want, chamferSegmentGapAbove(tc.a, tc.b, tc.c, tc.d, tc.bound2))
		})
	}
	rng := rand.New(rand.NewSource(38))
	randomPoint := func() XY {
		return XY{
			X: big.NewRat(int64(rng.Intn(41)-20), int64(rng.Intn(5)+1)),
			Y: big.NewRat(int64(rng.Intn(41)-20), int64(rng.Intn(5)+1)),
		}
	}
	for i := range 200 {
		a, b, c, d := randomPoint(), randomPoint(), randomPoint(), randomPoint()
		bound2 := big.NewRat(int64(rng.Intn(17)), 4)
		require.Equal(t, chamferSegmentGapAboveReference(a, b, c, d, bound2),
			chamferSegmentGapAbove(a, b, c, d, bound2), "random segment pair %d", i)
	}
}
