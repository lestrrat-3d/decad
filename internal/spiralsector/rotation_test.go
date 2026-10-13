package spiralsector_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/spiralsector"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestNearAxisRotation(t *testing.T) {
	t.Parallel()
	aboutX, err := r3.Rotation(r3.Vec{X: 1}, units.Degrees(30))
	require.NoError(t, err)
	aboutZ, err := r3.Rotation(r3.Vec{Z: 1}, units.Degrees(30))
	require.NoError(t, err)
	shift, err := r3.Translation(r3.Vec{X: 1})
	require.NoError(t, err)

	require.True(t, spiralsector.NearAxisRotation(aboutX))
	require.False(t, spiralsector.NearAxisRotation(r3.Identity()))
	require.False(t, spiralsector.NearAxisRotation(aboutZ))
	require.False(t, spiralsector.NearAxisRotation(shift))
}
