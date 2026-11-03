package topo_test

import (
	"maps"
	"testing"

	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// TestPlanarRecordFollowsItsMesh holds the record to the mesh it was proven
// for: a plain shallow copy keeps it, and a copy that moves the vertex array,
// widens a bound, or swaps the face-bound map drops it.
func TestPlanarRecordFollowsItsMesh(t *testing.T) {
	t.Parallel()
	face := &topo.Face{}
	m := &topo.Mesh{
		Vertices:  []r3.Vec{{}, {X: 1}, {Y: 1}},
		Triangles: [][3]int{{0, 1, 2}},
		Source:    []*topo.Face{face},
		FaceBound: map[*topo.Face]float64{face: 0.1},
		Bound:     0.1,
	}
	m.SetPlanarRecord(&topo.PlanarRecord{Own: []float64{0.01}})
	require.NotNil(t, m.Planar())

	same := *m
	require.NotNil(t, same.Planar())

	moved := *m
	moved.Vertices = append([]r3.Vec(nil), m.Vertices...)
	require.Nil(t, moved.Planar())

	widened := *m
	widened.Bound = 0.2
	require.Nil(t, widened.Planar())

	coord := *m
	coord.CoordBound = 0.05
	require.Nil(t, coord.Planar())

	faces := *m
	faces.FaceBound = maps.Clone(m.FaceBound)
	require.Nil(t, faces.Planar())

	m.SetPlanarRecord(&topo.PlanarRecord{Own: []float64{0.01, 0.02}})
	require.Nil(t, m.Planar(), "a record of the wrong length is not kept")
}
