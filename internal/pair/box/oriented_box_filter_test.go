package box

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/stretchr/testify/require"
)

func TestOrientedBoxesOutsideBandUsesStrictExactGap(t *testing.T) {
	unitBox := func(start *big.Rat) OrientedBox {
		var result OrientedBox
		for axis := range 3 {
			result.Edge[axis][axis] = proof.DyInt(2)
		}
		for index := range result.Corner {
			for axis := range 3 {
				origin := new(big.Rat)
				if axis == 0 {
					origin.Set(start)
				}
				coordinate := new(big.Rat).Add(origin,
					big.NewRat(2*int64((index>>axis)&1), 1))
				result.Corner[index][axis], _ = proof.DyOfRat(coordinate)
			}
		}
		return result
	}
	a := unitBox(new(big.Rat))
	etaSum := big.NewRat(1, 2)
	for _, test := range []struct {
		name  string
		start *big.Rat
		want  bool
	}{
		{name: "gap exceeds allowance", start: big.NewRat(11, 4), want: true},
		{name: "gap equals allowance", start: big.NewRat(5, 2), want: true},
		{name: "gap is below allowance", start: big.NewRat(9, 4), want: false},
	} {
		t.Run(test.name, func(t *testing.T) {
			b := unitBox(test.start)
			require.Equal(t, test.want, orientedBoxesOutsideBand(a, b, etaSum))
			require.Equal(t, test.want, orientedBoxesOutsideBand(b, a, etaSum))
		})
	}

	t.Run("zero allowance", func(t *testing.T) {
		separated := unitBox(big.NewRat(3, 1))
		touching := unitBox(big.NewRat(2, 1))
		overlapping := unitBox(big.NewRat(1, 1))
		zero := new(big.Rat)
		require.True(t, orientedBoxesOutsideBand(a, separated, zero))
		require.True(t, orientedBoxesOutsideBand(separated, a, zero))
		require.True(t, orientedBoxesOutsideBand(a, touching, zero))
		require.True(t, orientedBoxesOutsideBand(touching, a, zero))
		require.False(t, orientedBoxesOutsideBand(a, overlapping, zero))
	})
}
