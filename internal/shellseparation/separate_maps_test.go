package shellseparation_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/shellseparation"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func intMap(m [3][3]int64) [3][3]*big.Rat {
	var out [3][3]*big.Rat
	for i := range m {
		for j := range m[i] {
			out[i][j] = big.NewRat(m[i][j], 1)
		}
	}
	return out
}

// liedSet describes three shells whose second and third overlap each other
// while both clear the first. The caller's linked claim is false on purpose:
// only reference-gap transfer, never a direct slab check, can accept the
// overlapping pair, so the result shows whether transfer ran.
type liedSet struct {
	shells []shellseparation.Shell
	moves  []r3.Transform
}

func newLiedSet(t *testing.T, axis, near, far r3.Vec, second, third units.Value) liedSet {
	t.Helper()
	first := shellAt(near)
	other := shellAt(far)
	m1, err := r3.Rotation(axis, second)
	require.NoError(t, err)
	m2, err := r3.Rotation(axis, third)
	require.NoError(t, err)
	return liedSet{
		shells: []shellseparation.Shell{first, other, other},
		moves:  []r3.Transform{r3.Identity(), m1, m2},
	}
}

func TestSeparateTransfersThroughExactMaps(t *testing.T) {
	// Quarter and half turns about X. The float moves are deliberately wrong
	// for the pair (1, 2), so only the exact maps can compose to the third.
	set := newLiedSet(t, r3.Vec{X: 1}, r3.Vec{Y: 10}, r3.Vec{Y: -10}, units.Degrees(90), units.Degrees(0))
	maps := [][3][3]*big.Rat{
		{},
		intMap([3][3]int64{{1, 0, 0}, {0, 0, -1}, {0, 1, 0}}),
		intMap([3][3]int64{{1, 0, 0}, {0, -1, 0}, {0, 0, -1}}),
	}
	sep := func(maps [][3][3]*big.Rat, linked bool) bool {
		return shellseparation.Separate(t.Context(), set.shells, set.moves, maps,
			shellseparation.SourceShell, shellseparation.AxisX, linked)
	}
	require.True(t, sep(maps, true), "exact maps compose, so transfer accepts the pair")
	require.False(t, sep(nil, true), "the wrong float moves do not compose")
	require.False(t, sep(maps, false), "without a linked claim the direct slab check decides")
}

func TestSeparateRefusesMalformedMaps(t *testing.T) {
	set := newLiedSet(t, r3.Vec{X: 1}, r3.Vec{Y: 10}, r3.Vec{Y: -10}, units.Degrees(90), units.Degrees(180))
	good := intMap([3][3]int64{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}})
	partial := good
	partial[2][2] = nil
	for name, maps := range map[string][][3][3]*big.Rat{
		"short":   {{}, good},
		"partial": {{}, good, partial},
	} {
		t.Run(name, func(t *testing.T) {
			require.False(t, shellseparation.Separate(t.Context(), set.shells, set.moves, maps,
				shellseparation.SourceShell, shellseparation.AxisX, true))
		})
	}
}

func TestSeparateNeverTransfersExactMapsThroughOtherAxes(t *testing.T) {
	// Quarter and half turns about Y, with the longitudinal axis Y. Float moves
	// canonicalize into the local frame and transfer correctly. Exact maps are
	// in the world frame, so any exact entry switches transfer off there. The
	// maps below pass the X-rotation structure test and compose exactly, so
	// only the axis rule can refuse them.
	set := newLiedSet(t, r3.Vec{Y: 1}, r3.Vec{Z: 10}, r3.Vec{Z: -10}, units.Degrees(90), units.Degrees(180))
	exact := [][3][3]*big.Rat{
		{},
		intMap([3][3]int64{{1, 0, 0}, {0, 0, -1}, {0, 1, 0}}),
		intMap([3][3]int64{{1, 0, 0}, {0, -1, 0}, {0, 0, -1}}),
	}
	sep := func(maps [][3][3]*big.Rat) bool {
		return shellseparation.Separate(t.Context(), set.shells, set.moves, maps,
			shellseparation.SourceShell, shellseparation.AxisY, true)
	}
	require.True(t, sep(nil), "float moves transfer in the canonical frame")
	require.True(t, sep(make([][3][3]*big.Rat, 3)), "all-nil entries are the same as nil maps")
	require.False(t, sep(exact), "an exact entry disables transfer off the X axis")
}
