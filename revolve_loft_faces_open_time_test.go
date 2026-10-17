package decad

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/polynomial"
	"github.com/lestrrat-3d/decad/internal/revolveloftfaces"
	"github.com/stretchr/testify/require"
)

func TestRevolveLoftFacesOpenTimeExcludesExactPointRoot(t *testing.T) {
	// Q(s,t)=s+t-1/2 vanishes at (s=0,t=1/2). The strict side starts
	// immediately after that exact time, but includes no root point.
	q := [3][]*big.Rat{
		{big.NewRat(-1, 2), big.NewRat(1, 2)},
		{big.NewRat(0, 1), big.NewRat(1, 1)},
		{big.NewRat(1, 2), big.NewRat(3, 2)},
	}
	require.Equal(t, loftface.SideOutside,
		polynomial.BernsteinRectOpenSide(q, big.NewRat(1, 2), lfOne(), true, false))
	require.Zero(t,
		polynomial.BernsteinRectOpenSide(q, big.NewRat(1, 2), lfOne(), false, false))
	// On the left, Q has a moving source root that reaches s=0 at the
	// excluded time endpoint. The strict-side shortcut must refuse it.
	require.Zero(t,
		polynomial.BernsteinRectOpenSide(q, lfZero(), big.NewRat(1, 2), false, true))
}

func TestRevolveLoftFacesOpenMovingCrossingRefuses(t *testing.T) {
	mapAt := func(x, y *big.Rat) loftFaceMap {
		zero, one := lfZero(), lfOne()
		return loftFaceMap{Constant: loftFaceRatVec{zero, zero, zero},
			Q0: loftFaceRatVec{x, y, zero},
			QU: loftFaceRatVec{one, one, zero},
			QV: loftFaceRatVec{zero, zero, zero},
			G0: one, GU: zero, GV: zero}
	}
	first := mapAt(lfZero(), big.NewRat(1, 2))
	last := mapAt(lfOne(), big.NewRat(3, 2))
	span := freeform.BezierSpan{
		{U: lfZero(), V: lfZero()}, {U: lfOne(), V: lfZero()},
	}
	section := revolveSectionSpan{Start: planePoint{U: 0, V: 1},
		End: planePoint{U: 2, V: 1}}
	m, b := revolveloftfaces.RevolveLoftFacesMeridian(section)
	q := loftface.MeridianQRows(first, last, span, m, b)
	j := loftface.MeridianJRows(first, last, span, q)
	remaining := revolveloftfaces.RevolveLoftFacesMaxBandCells
	_, ok, _ := revolveloftfaces.RevolveLoftFacesOpenTimeSlab(t.Context(), q, j,
		first, last, span, section, 0, 0, 0,
		lfZero(), big.NewRat(1, 2), false, true, &remaining)
	require.False(t, ok)
}
