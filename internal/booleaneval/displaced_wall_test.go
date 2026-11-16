package booleaneval_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/booleaneval"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// Rule DW (agent-docs/carrier-record-design.md §8.7) declines the retained tag
// of a displaced wall, Plane and Cylinder alike, and keeps every other tag.
func TestDisplacedWall(t *testing.T) {
	t.Parallel()
	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	wall, other := &topo.Face{}, &topo.Face{}
	walls := map[*topo.Face]struct{}{wall: {}}
	for _, g := range []facetedrecord.Group{
		{Surface: surfacegeom.Plane{Frame: frame}, Retained: true, AxialDelta: 1, HasAxialDelta: true, NormalBound: 0.5},
		{Surface: surfacegeom.Cylinder{}, Retained: true},
	} {
		got := booleaneval.DisplacedWall(g, wall, walls)
		require.Nil(t, got.Surface)
		require.False(t, got.Retained)
		require.False(t, got.HasAxialDelta)
		require.Zero(t, got.NormalBound)
		require.Equal(t, g, booleaneval.DisplacedWall(g, other, walls), "a face outside the set keeps its tag")
		require.Equal(t, g, booleaneval.DisplacedWall(g, wall, nil))
	}
}
