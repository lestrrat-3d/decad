package facetproof_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestSectorSeamPlacesSeamInLargestGap(t *testing.T) {
	seam, err := facetproof.SectorSeam(r3.Vec{Y: 1}, r3.Vec{Z: 1}, []r3.Transform{r3.Identity()})
	require.NoError(t, err)
	require.Greater(t, seam, math.Pi/2)
	require.Less(t, seam, 2*math.Pi)
	require.InDelta(t, 5*math.Pi/4, seam, 1e-12)

	_, err = facetproof.SectorSeam(r3.Vec{Y: 1}, r3.Vec{Z: 1}, nil)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorContains(t, err, "sector source has no root contacts")
}

func TestSectorAngularGap(t *testing.T) {
	require.Equal(t, 1.0, facetproof.SectorAngularGap([2]float64{0, 1}, [2]float64{2, 3}))
	require.Equal(t, 1.0, facetproof.SectorAngularGap([2]float64{2, 3}, [2]float64{0, 1}))
	require.True(t, math.IsInf(facetproof.SectorAngularGap([2]float64{0, 2}, [2]float64{1, 3}), -1))
}

func TestAngleAt(t *testing.T) {
	require.InDelta(t, 3*math.Pi/2, facetproof.AngleAt(r3.Vec{Z: -1}, 0), 1e-12)
	got := facetproof.AngleAt(r3.Vec{Y: 1}, -10)
	require.GreaterOrEqual(t, got, -10.0)
	require.Less(t, got, -10+2*math.Pi)
}
