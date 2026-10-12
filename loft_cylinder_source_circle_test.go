package decad

import (
	"math/big"
	"testing"

	"github.com/stretchr/testify/require"
)

func TestLoftCylinderSourceTimeSliceMatchesPoint(t *testing.T) {
	t.Parallel()
	rat := func(n, d int64) *big.Rat { return big.NewRat(n, d) }
	connector := loftCylinderSourceConnector{
		a: [2]loftCylinderXY{
			{x: rat(1, 3), y: rat(2, 5)},
			{x: rat(7, 11), y: rat(3, 7)},
		},
		d: [2]loftCylinderXY{
			{x: rat(5, 4), y: rat(9, 8)},
			{x: rat(13, 10), y: rat(11, 9)},
		},
		f: [2]loftCylinderXY{
			{x: rat(15, 8), y: rat(7, 4)},
			{x: rat(17, 9), y: rat(19, 10)},
		},
	}
	for _, height := range []*big.Rat{rat(0, 1), rat(1, 7), rat(3, 4), rat(1, 1)} {
		slice := connector.timeSlice(height)
		for _, along := range []*big.Rat{rat(0, 1), rat(1, 3), rat(1, 1), rat(5, 4), rat(2, 1)} {
			want := connector.point(along, height)
			got := slice.point(along)
			require.Zero(t, got.x.Cmp(want.x))
			require.Zero(t, got.y.Cmp(want.y))
		}
	}
}
