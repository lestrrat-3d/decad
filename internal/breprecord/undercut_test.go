package breprecord_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/stretchr/testify/require"
)

func TestListVerdict(t *testing.T) {
	t.Parallel()

	a, b := new(topo.Face), new(topo.Face)
	var faces []*topo.Face
	undecided := false

	require.True(t, breprecord.ListVerdict(&faces, &undecided, a, survey2d.PullOpposes, true))
	require.Equal(t, []*topo.Face{a}, faces)
	require.False(t, undecided)

	require.True(t, breprecord.ListVerdict(&faces, &undecided, b, survey2d.PullUndecided, true))
	require.Equal(t, []*topo.Face{a}, faces, "an undecided face is not listed")
	require.True(t, undecided)

	other := survey2d.PullVerdict(-1)
	for _, v := range []survey2d.PullVerdict{survey2d.PullClear, other} {
		require.True(t, breprecord.ListVerdict(&faces, &undecided, b, v, true))
	}
	require.Equal(t, []*topo.Face{a}, faces, "a clear face is not listed")

	// A failed enclosure is a total refusal that changes nothing.
	faces, undecided = nil, false
	require.False(t, breprecord.ListVerdict(&faces, &undecided, a, survey2d.PullOpposes, false))
	require.False(t, breprecord.ListVerdict(&faces, &undecided, a, survey2d.PullUndecided, false))
	require.Empty(t, faces)
	require.False(t, undecided)
}
