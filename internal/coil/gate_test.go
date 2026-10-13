package coil_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/coil"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestMagnitude(t *testing.T) {
	m, err := coil.Magnitude(units.Millimeters(2), units.Length, units.Millimeter, "the coil pitch")
	require.NoError(t, err)
	require.Equal(t, 2.0, m)

	_, err = coil.Magnitude(units.Scalar(3), units.Length, units.Millimeter, "the coil pitch")
	require.ErrorIs(t, err, decaderr.ErrUnitKind)

	_, err = coil.Magnitude(units.Millimeters(math.NaN()), units.Length, units.Millimeter, "the coil pitch")
	require.ErrorIs(t, err, decaderr.ErrNotFinite)

	_, err = coil.Magnitude(units.Millimeters(0), units.Length, units.Millimeter, "the coil pitch")
	require.ErrorIs(t, err, decaderr.ErrDegenerate)
	require.ErrorContains(t, err, "a zero pitch is a Revolve")

	_, err = coil.Magnitude(units.Scalar(0), units.Dimensionless, units.One, "the coil turn count")
	require.ErrorIs(t, err, decaderr.ErrDegenerate)
	require.NotContains(t, err.Error(), "a zero pitch is a Revolve")
}

func TestPreflight(t *testing.T) {
	const perTurn, maxStations = 256, 1 << 15
	// One turn of a four-vertex profile: 256 stations, 2*256*4 + 2*(4-2) = 2052 triangles.
	require.NoError(t, coil.Preflight(1, 4, 0, perTurn, maxStations, 1<<20))

	err := coil.Preflight(1, 4, 0, perTurn, maxStations, 2051)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorContains(t, err, "the coil's 2052 triangles exceed the 2051-triangle ceiling")
	require.NoError(t, coil.Preflight(1, 4, 0, perTurn, maxStations, 2052))

	// 128 turns is exactly 32768 stations.
	require.NoError(t, coil.Preflight(128, 4, 0, perTurn, maxStations, 1<<40))

	err = coil.Preflight(128.5, 4, 0, perTurn, maxStations, 1<<40)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorContains(t, err, "needs more than the 32768-station ceiling at 256 stations per turn")
}
