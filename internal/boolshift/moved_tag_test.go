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

// An oblique wall's extent (topo.Face.ExtentDelta) describes the tag where it stands, so a translated piece drops it
// and keeps the departed tag; a piece no shift names keeps the group whole.
func TestMovedGroupDropsExtentDelta(t *testing.T) {
	t.Parallel()
	frame, err := r3.NewFrame(r3.NewVec(0, 0, 4), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	group := facetedrecord.Group{
		Surface: surfacegeom.Plane{Frame: frame}, Retained: true,
		NormalBound: 3e-14, ExtentDelta: 5e-15,
	}
	verts := []r3.Vec{r3.NewVec(0, 0, 4), r3.NewVec(1, 1, 4)}

	moved := boolshift.MovedGroup(group, 0, true,
		map[int]boolshift.PieceShift{0: {Move: r3.NewVec(0, 0, 0.5)}}, verts, 1)
	require.NotNil(t, moved.Surface, "the departed tag stays")
	require.Zero(t, moved.ExtentDelta)
	require.GreaterOrEqual(t, moved.NormalBound, 3e-14)

	unmoved := boolshift.MovedGroup(group, 0, true, map[int]boolshift.PieceShift{}, verts, 1)
	require.Equal(t, 5e-15, unmoved.ExtentDelta)
	require.Equal(t, 3e-14, unmoved.NormalBound)
}
