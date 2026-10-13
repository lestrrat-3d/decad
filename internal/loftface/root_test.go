package loftface_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/stretchr/testify/require"
)

func rat(n, d int64) *big.Rat { return big.NewRat(n, d) }

// ratVecXY builds the exact rational vector (x, y, 0) from integers.
func ratVecXY(x, y int64) loftface.RatVec {
	return loftface.RatVec{rat(x, 1), rat(y, 1), rat(0, 1)}
}

// shiftMap is the cap map X = u + k, Y = v, Z = 0 with constant denominator 1.
func shiftMap(k int64) loftface.Map {
	return loftface.Map{
		Constant: ratVecXY(k, 0),
		Q0:       ratVecXY(0, 0),
		QU:       ratVecXY(1, 0),
		QV:       ratVecXY(0, 1),
		G0:       rat(1, 1),
		GU:       rat(0, 1),
		GV:       rat(0, 1),
	}
}

// constantXMap is the cap map with constant X = x and no source dependence.
func constantXMap(x int64) loftface.Map {
	return loftface.Map{
		Constant: ratVecXY(x, 0),
		Q0:       ratVecXY(0, 0),
		QU:       ratVecXY(0, 0),
		QV:       ratVecXY(0, 0),
		G0:       rat(1, 1),
		GU:       rat(0, 1),
		GV:       rat(0, 1),
	}
}

// cubic is a cubic Bezier span whose control points are (us[i], 0).
func cubic(us ...int64) freeform.BezierSpan {
	span := make(freeform.BezierSpan, len(us))
	for i, u := range us {
		span[i] = freeform.RatPoint{U: rat(u, 1), V: rat(0, 1)}
	}
	return span
}

func requireInterval(t *testing.T, lo, hi *big.Rat, got loftface.Interval) {
	t.Helper()
	require.Zero(t, lo.Cmp(got.Lo), "Lo is %s, want %s", got.Lo, lo)
	require.Zero(t, hi.Cmp(got.Hi), "Hi is %s, want %s", got.Hi, hi)
}

func TestCapXRangeAndBand(t *testing.T) {
	decreasing := cubic(3, 2, 1, 0)
	crossing := loftface.RootCrossing{Lo: rat(1, 4), Hi: rat(1, 2)}

	// The cap X is 3 - 3s on the span, so the crossing [1/4, 1/2] maps to
	// X in [3/2, 9/4].
	first := shiftMap(0)
	requireInterval(t, rat(3, 2), rat(9, 4), loftface.CapXRange(first, decreasing, crossing))

	// The second cap is shifted by one; band time 1/4 blends 3/4 of the first
	// range with 1/4 of the second: 3/4*3/2 + 1/4*5/2 and 3/4*9/4 + 1/4*13/4.
	band := loftface.BandXRange(first, shiftMap(1), decreasing, crossing, rat(1, 4))
	requireInterval(t, rat(7, 4), rat(5, 2), band)

	require.True(t, loftface.CapXDecreases(first, decreasing))
	require.False(t, loftface.CapXDecreases(first, cubic(0, 1, 2, 3)))
	// A zero derivative coefficient is not strictly decreasing.
	require.False(t, loftface.CapXDecreases(first, cubic(3, 3, 0, 0)))
}

func TestConstantXBandAndClassifyRows(t *testing.T) {
	maps := []loftface.Map{constantXMap(0), constantXMap(1), constantXMap(2)}
	box := [4]*big.Rat{rat(0, 1), rat(1, 1), rat(0, 1), rat(1, 1)}

	band, time, ok := loftface.ConstantXBand(maps, box, rat(5, 4))
	require.True(t, ok)
	require.Equal(t, 1, band)
	require.Zero(t, rat(1, 4).Cmp(time), "time is %s", time)

	// X equal to a cap X is not strictly inside any band.
	_, _, ok = loftface.ConstantXBand(maps, box, rat(1, 1))
	require.False(t, ok)

	rows := [3][]*big.Rat{{rat(1, 1), rat(2, 1)}, {rat(1, 1), rat(1, 1)}, {rat(3, 1), rat(1, 2)}}
	side, got := loftface.ClassifyRows(rows)
	require.Equal(t, loftface.SideOutside, side)
	require.Zero(t, rat(1, 2).Cmp(got[2][1]))

	side, got = loftface.ClassifyRows(loftface.NegateRows(rows))
	require.Equal(t, loftface.SideInside, side)
	for i := range rows {
		for j := range rows[i] {
			require.Zero(t, rows[i][j].Cmp(got[i][j]), "entry (%d, %d) is %s", i, j, got[i][j])
		}
	}
	require.Zero(t, rat(1, 2).Cmp(got[2][1]))
}
