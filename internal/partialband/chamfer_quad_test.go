package partialband_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/partialband"
	"github.com/stretchr/testify/require"
)

func TestChamferQuadExact(t *testing.T) {
	e := brepgeom.Embed{Axis: [3]int{0, 1, 2}, Sign: [3]float64{1, 1, 1}}
	points := [4][3]float64{{0, 0, 1}, {4, 0, 1}, {4, 0.5, 0}, {0, 0.5, 0}}
	require.NoError(t, partialband.ChamferQuadExact(points, 0.5, 1, e))

	err := partialband.ChamferQuadExact(points, 0.25, 1, e)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorContains(t, err, "a partial chamfer's cap line is not the exact requested offset")

	err = partialband.ChamferQuadExact(points, 0.5, 2, e)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorContains(t, err, "a partial chamfer's side level is not the exact requested setback")

	bent := points
	bent[3] = [3]float64{0, 0.5, 0.5}
	err = partialband.ChamferQuadExact(bent, 0.5, 1, e)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorContains(t, err, "a partial chamfer patch is not one exact plane")

	swapped := points
	swapped[2], swapped[3] = points[3], points[2]
	err = partialband.ChamferQuadExact(swapped, 0.5, 1, e)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorContains(t, err, "a partial chamfer patch has inconsistent triangle winding")
}

func TestChamferQuadMass(t *testing.T) {
	square := [4][3]float64{{0, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 1, 1}}
	area, vol3, moments, err := partialband.ChamferQuadMass(square)
	require.NoError(t, err)
	require.Zero(t, vol3.Cmp(big.NewRat(1, 1)))
	require.Zero(t, moments[0].Sign())
	require.Zero(t, moments[1].Sign())
	require.Zero(t, moments[2].Cmp(big.NewRat(1, 2)))
	one := big.NewRat(1, 1)
	require.LessOrEqual(t, area.Lo.Cmp(one), 0)
	require.GreaterOrEqual(t, area.Hi.Cmp(one), 0)
	width := new(big.Rat).Sub(area.Hi, area.Lo)
	require.Equal(t, -1, width.Cmp(big.NewRat(1, 1_000_000_000)))
}
