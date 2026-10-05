package decad

import (
	"math"
	"math/big"
	"testing"

	"github.com/stretchr/testify/require"
)

func TestFacetedAreaBoundCompositionRoundsOutward(t *testing.T) {
	t.Parallel()
	// Each edge's stored length plus its proven bound is a rational number.
	// Ten short edges disappear in the old float perimeter accumulation.
	short := math.Ldexp(1, -54)
	lengths := []float64{1, 1}
	for range 10 {
		lengths = append(lengths, short)
	}
	face := &Face{loops: []*Loop{{}}}
	exactPerimeter := new(big.Rat)
	oldPerimeter := 0.0
	for _, length := range lengths {
		bound := chainLengthBound(1, 0, length)
		face.loops[0].coedges = append(face.loops[0].coedges, coedge{edge: &Edge{
			length: length, lengthBound: bound,
		}})
		exactPerimeter.Add(exactPerimeter, new(big.Rat).SetFloat64(length))
		exactPerimeter.Add(exactPerimeter, new(big.Rat).SetFloat64(bound))
		oldPerimeter += length + bound
	}

	perimeter, err := facetedFacePerimeterUpper(face, newWorkBudget(t.Context()))
	require.NoError(t, err)
	require.GreaterOrEqual(t, new(big.Rat).SetFloat64(perimeter).Cmp(exactPerimeter), 0)

	meshBound := 0.25
	areaSlack := sumSlop(12, short)
	areaBound := facetedAreaBound(meshBound, perimeter, areaSlack, 0)
	required := new(big.Rat).Mul(new(big.Rat).SetFloat64(meshBound), exactPerimeter)
	required.Add(required, new(big.Rat).SetFloat64(areaSlack))
	oldBound := upRound(meshBound*oldPerimeter + areaSlack)
	require.Less(t, new(big.Rat).SetFloat64(oldBound).Cmp(required), 0,
		"the former perimeter and final rounding understate the required area allowance")
	require.GreaterOrEqual(t, new(big.Rat).SetFloat64(areaBound).Cmp(required), 0)

	// Body.Area counts each face's perimeter, including short later faces.
	bodyPerimeter := absSumUpper(perimeter, short)
	exactBodyPerimeter := new(big.Rat).Add(exactPerimeter, new(big.Rat).SetFloat64(short))
	bodyRequired := new(big.Rat).Mul(new(big.Rat).SetFloat64(meshBound), exactBodyPerimeter)
	bodyRequired.Add(bodyRequired, new(big.Rat).SetFloat64(areaSlack))
	bodyBound := facetedAreaBound(meshBound, bodyPerimeter, areaSlack, 0)
	require.GreaterOrEqual(t, new(big.Rat).SetFloat64(bodyBound).Cmp(bodyRequired), 0)

	// The exact zero remains exact, and every finite term stays finite.
	require.Zero(t, facetedAreaBound(0, 0, 0, 0))
	require.Zero(t, upRound(math.SmallestNonzeroFloat64*math.SmallestNonzeroFloat64),
		"the former final rounding cannot recover a positive product that underflows")
	require.Positive(t, facetedAreaBound(math.SmallestNonzeroFloat64, math.SmallestNonzeroFloat64, 0, 0))
	require.False(t, math.IsInf(areaBound, 0))
}
