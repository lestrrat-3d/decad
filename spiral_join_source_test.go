package decad

import (
	"context"
	"math/big"
	"testing"

	"github.com/stretchr/testify/require"
)

func TestSpiralRootTubePropagatesCancellation(t *testing.T) {
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	_, ok, err := spiralRootTube(ctx, nil, 0, big.NewRat(0, 1), big.NewRat(1, 1))
	require.False(t, ok)
	require.ErrorIs(t, err, context.Canceled)
}

func TestSpiralCapPlaneMustMissTheWholeRootCircle(t *testing.T) {
	box := [4]*big.Rat{big.NewRat(-1, 1), big.NewRat(1, 1),
		big.NewRat(-1, 1), big.NewRat(1, 1)}
	corners := lfBoxCorners(box)
	cap := loftFaceMap{
		constant: loftFaceRatVec{lfZero(), lfZero(), lfZero()},
		q0:       loftFaceRatVec{lfZero(), big.NewRat(1, 1), lfZero()},
		qu:       loftFaceRatVec{lfOne(), lfZero(), lfZero()},
		qv:       loftFaceRatVec{lfZero(), lfZero(), lfOne()},
		g0:       lfOne(), gu: lfZero(), gv: lfZero(),
	}
	m, b, x := lfZero(), lfOne(), lfZero()
	require.False(t, spiralCapPlaneAvoidsRootCircle(cap, corners, m, b, x),
		"a tangent cap can meet the circle")
	cap.q0[1] = lfAdd(cap.q0[1], big.NewRat(1, 1<<50))
	require.True(t, spiralCapPlaneAvoidsRootCircle(cap, corners, m, b, x),
		"an exact sub-femtometre gap still separates the full cap")
	cap.q0[1] = lfZero()
	require.False(t, spiralCapPlaneAvoidsRootCircle(cap, corners, m, b, x),
		"a cap crossing the circle must refuse")
	cap.qu = loftFaceRatVec{lfZero(), lfZero(), lfZero()}
	require.False(t, spiralCapPlaneAvoidsRootCircle(cap, corners, m, b, x),
		"a collapsed cap has no supporting plane")
}

func TestSpiralToolCapMustStayOnCertifiedSideOfBlankMeridian(t *testing.T) {
	toe := Point2{U: 2, V: 8}
	outer := Point2{U: 3, V: 7}
	cone := &pointConeLimit{apex: big.NewRat(9999999999999999, 1000000000000000),
		slope: big.NewRat(1, 1)}
	gap, ok := spiralToolOuterGap(cone, toe, outer, 1)
	require.True(t, ok)
	require.Equal(t, big.NewRat(1, 1000000000000000), gap)

	// A tiny side reversal must refuse the source-specific join.
	cone.apex = big.NewRat(10000000000000001, 1000000000000000)
	_, ok = spiralToolOuterGap(cone, toe, outer, 1)
	require.False(t, ok)

	// A different slope may keep the side, but its full finite-meridian
	// separation must be returned for the later cap-strip allowance gate.
	cone.apex, cone.slope = big.NewRat(10, 1), big.NewRat(2, 1)
	gap, ok = spiralToolOuterGap(cone, toe, outer, 1)
	require.True(t, ok)
	require.Equal(t, big.NewRat(8, 1), gap)
}

func TestSpiralFiniteConeSideRequiresWholeGeneratorAndFiniteTool(t *testing.T) {
	cone := &pointConeLimit{apex: big.NewRat(10, 1), far: big.NewRat(-1, 1),
		slope: big.NewRat(1, 1)}
	first, last := Point2{U: 2, V: 9}, Point2{U: 3, V: 8}
	require.True(t, spiralFiniteConeSide(cone, first, last, 1))
	last.V = 6
	require.False(t, spiralFiniteConeSide(cone, first, last, 1),
		"opposite endpoint signs cannot certify a complete face")
	last.V = 8
	cone.far = big.NewRat(0, 1)
	require.False(t, spiralFiniteConeSide(cone, first, last, 1),
		"the admitted tool must have a finite far plane before the source face")
	cone.far = big.NewRat(-1, 1)
	last.U = 10
	require.False(t, spiralFiniteConeSide(cone, first, last, 1),
		"the admitted generator must end before the cone apex")
}
