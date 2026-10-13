package massmoment_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestXAxisPlacementCharge(t *testing.T) {
	shift, err := r3.Translation(r3.Vec{X: 1})
	require.NoError(t, err)
	aboutZ, err := r3.Rotation(r3.Vec{Z: 1}, units.Degrees(30))
	require.NoError(t, err)

	require.True(t, massmoment.XAxisRotation(r3.Identity()))
	require.False(t, massmoment.XAxisRotation(shift))
	require.False(t, massmoment.XAxisRotation(aboutZ))

	charge, err := massmoment.XAxisPlacementCharge(r3.Identity())
	require.NoError(t, err)
	require.Equal(t, massmoment.MapCharge{}, charge)

	_, err = massmoment.XAxisPlacementCharge(aboutZ)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorContains(t, err, "sector placement is not an axis rotation")

	placed := massmoment.PlacedBound(2, massmoment.MapCharge{Stretch: 0.5})
	require.GreaterOrEqual(t, placed, 3.0)
	require.InDelta(t, 3.0, placed, 1e-12)
}
