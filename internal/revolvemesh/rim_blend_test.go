package revolvemesh_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/stretchr/testify/require"
)

func TestRimStationsAndBounds(t *testing.T) {
	source := revolvemesh.RimSource{XLo: 0, XHi: 4, Radius: 2, Setback: 0.5}
	stations, err := revolvemesh.RimStations(source)
	require.NoError(t, err)
	require.Len(t, stations, 641)
	require.Equal(t, 0.0, stations[0].Z)
	require.Equal(t, 3.5, stations[0].X)
	require.Equal(t, 0.5, stations[128].Z)
	require.Equal(t, 4.0, stations[128].X)
	require.Equal(t, 2.0, stations[640].Z)
	require.Equal(t, 0.0, stations[640].Y)

	// Every blend station z < 1/2 keeps the fillet circle
	// (x-7/2)^2 = z(1-z) between the rounded-down x and its exact upper x.
	for i, s := range stations[:128] {
		z := new(big.Rat).SetFloat64(s.Z)
		want := new(big.Rat).Mul(z, new(big.Rat).Sub(big.NewRat(1, 1), z))
		lo := new(big.Rat).Sub(new(big.Rat).SetFloat64(s.X), big.NewRat(7, 2))
		hi := new(big.Rat).Sub(s.XUpper, big.NewRat(7, 2))
		require.LessOrEqual(t, lo.Mul(lo, lo).Cmp(want), 0, "station %d", i)
		require.LessOrEqual(t, want.Cmp(hi.Mul(hi, hi)), 0, "station %d", i)
	}
	require.Equal(t, 0.125, stations[64].Z)

	bound, missing, err := revolvemesh.RimBounds(source, stations)
	require.NoError(t, err)
	require.GreaterOrEqual(t, bound, 0.0)
	require.Less(t, bound, 0.1)
	require.GreaterOrEqual(t, missing, 0.0)

	two := []revolvemesh.RimStation{stations[0], stations[1]}
	two[1].XUpper = new(big.Rat).Sub(new(big.Rat).SetFloat64(two[0].X), big.NewRat(1, 1))
	_, _, err = revolvemesh.RimBounds(source, two)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorContains(t, err, "cap rim intervals are not monotone")
}

func TestRimStationsChamferAndRefusal(t *testing.T) {
	source := revolvemesh.RimSource{XLo: 0, XHi: 4, Radius: 2, Setback: 0.5, Kind: revolvemesh.RimChamfer}
	stations, err := revolvemesh.RimStations(source)
	require.NoError(t, err)
	require.Equal(t, 3.625, stations[64].X)
	require.Zero(t, stations[64].XUpper.Cmp(big.NewRat(29, 8)))

	source.Setback = 3
	_, err = revolvemesh.RimStations(source)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorContains(t, err, "the cap rim has no certified blend station")
}
