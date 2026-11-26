package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/stretchr/testify/require"
)

func TestCoaxialPocketAfterFacetedCuts(t *testing.T) {
	t.Parallel()
	ctx := t.Context()
	doc, _, part := crossCylinderBore(t, 5, .6, 4, 2)
	// Three disjoint cuts put the final pocket one literal above the six-literal regroup limit.
	for _, center := range [][2]float64{{0, -3}, {2, -2}, {2, 2}} {
		tool := circlePrismAtZ(t, doc, center[0], center[1], .3, -.5, 5)
		var err error
		part, err = decad.Cut(part, tool).Do(ctx)
		require.NoError(t, err)
	}
	hole := circlePrismAtZ(t, doc, 0, 3, .6, -.5, 12)
	drilled, err := decad.Cut(part, hole).Do(ctx)
	require.NoError(t, err)
	before, err := drilled.Volume()
	require.NoError(t, err)
	pocket := circlePrismAtZ(t, doc, 0, 3, 1, -.5, 1.25)
	result, err := decad.Cut(drilled, pocket).Do(ctx)
	require.NoError(t, err)
	require.Empty(t, result.OperationDiagnostics())
	after, err := result.Volume()
	require.NoError(t, err)
	wantRemoved := math.Pi * (1 - .6*.6) * .75
	uncertainty := before.Bound.Base() + after.Bound.Base()
	require.Less(t, uncertainty, wantRemoved)
	require.InDelta(t, wantRemoved, before.Value.Base()-after.Value.Base(), uncertainty)
}
