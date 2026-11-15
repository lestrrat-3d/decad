package boolshift_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/boolshift"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// A displacement stated with no record is relative to the tag alone, so a
// translated piece keeps it; the retained plane's slab reads it.
func TestMovedGroupKeepsADisplacementStatedWithNoRecord(t *testing.T) {
	t.Parallel()
	frame, err := r3.NewFrame(r3.NewVec(0, 0, 4), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	group := facetedrecord.Group{
		Surface: surfacegeom.Plane{Frame: frame}, Retained: true,
		AxialDelta: 0.0137, HasAxialDelta: true,
	}
	verts := []r3.Vec{r3.NewVec(0, 0, 4), r3.NewVec(1, 1, 4)}
	moved := boolshift.MovedGroup(group, 0, true,
		map[int]boolshift.PieceShift{0: {Move: r3.NewVec(0, 0, 0.5)}}, verts, 1)
	require.NotNil(t, moved.Surface)
	require.True(t, moved.HasAxialDelta)
	require.Equal(t, 0.0137, moved.AxialDelta)
}
