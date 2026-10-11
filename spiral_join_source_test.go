package decad

import (
	"math/big"
	"testing"

	"github.com/stretchr/testify/require"
)

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
