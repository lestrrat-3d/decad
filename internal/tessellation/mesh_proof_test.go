package tessellation_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestBrepPlanarDisplacement(t *testing.T) {
	walks := [][]survey2d.SegmentWalk{
		{{Length: 1.25, LengthBound: 0.01}, {Length: 0, LengthBound: 0.02}},
		{{Length: 2.5, LengthBound: 0.03}},
	}
	perimeter := 0.0
	for _, loop := range walks {
		for _, w := range loop {
			perimeter = proofbound.AbsSumUpper(perimeter, w.Length, w.LengthBound)
		}
	}
	want := proofbound.SectionDisplacementArea(0.125, 3, perimeter)
	require.Equal(t, want, tessellation.BrepPlanarDisplacement(0.125, walks))

	invalid := [][]survey2d.SegmentWalk{{{Length: math.NaN()}}}
	zero := tessellation.BrepPlanarDisplacement(math.Copysign(0, -1), invalid)
	require.Equal(t, 0.0, zero)
	require.False(t, math.Signbit(zero))
}

func TestPatchEdgeSense(t *testing.T) {
	triangles := [][3]int{{0, 1, 2}, {2, 1, 3}, {4, 5, 6}}

	require.Equal(t, 1, tessellation.PatchEdgeSense(triangles, []int{0}, 0, 1))
	require.Equal(t, -1, tessellation.PatchEdgeSense(triangles, []int{0}, 1, 0))
	require.Equal(t, 1, tessellation.PatchEdgeSense(triangles, []int{0}, 2, 0), "wrap-around edge")
	require.Equal(t, -1, tessellation.PatchEdgeSense(triangles, []int{0, 1}, 2, 1))
	require.Equal(t, 0, tessellation.PatchEdgeSense(triangles, []int{0}, 0, 3))
	require.Equal(t, 0, tessellation.PatchEdgeSense(triangles, []int{2}, 0, 1), "only the named facets count")
	require.Equal(t, 0, tessellation.PatchEdgeSense(triangles, nil, 0, 1))
}

// TestPlanarFacetRecord holds the record to its faces: a square cap whose
// right edge is the chord of a curved wall lists that edge as its chord,
// with the trim term padded as its reach; each cap facet's own bound is its
// corners' store plus the section and axial terms, and the wall's facet
// carries +Inf.
func TestPlanarFacetRecord(t *testing.T) {
	t.Parallel()
	capFace, wall := &topo.Face{}, &topo.Face{}
	vertices := []r3.Vec{{}, {X: 1}, {X: 1, Y: 1}, {Y: 1}, {X: 2, Y: 0.5}}
	triangles := [][3]int{{0, 1, 2}, {0, 2, 3}, {2, 1, 4}}
	source := []*topo.Face{capFace, capFace, wall}
	planar := func(f *topo.Face) bool { return f == capFace }
	store := []float64{1e-6, 2e-6, 0, 3e-6, 0}
	rec := tessellation.PlanarFacetRecord(vertices, triangles, source, planar,
		map[*topo.Face]float64{capFace: 0.01}, map[*topo.Face]float64{capFace: 1e-5}, store, 1e-4)
	require.Equal(t, [][2]r3.Vec{{{X: 1}, {X: 1, Y: 1}}}, rec.Chords[capFace])
	require.Greater(t, rec.Reach[capFace], 0.01)
	require.Less(t, rec.Reach[capFace], 0.0101)
	require.InDelta(t, 2e-6+1e-4+1e-5, rec.Own[0], 1e-15)
	require.InDelta(t, 3e-6+1e-4+1e-5, rec.Own[1], 1e-15)
	require.True(t, math.IsInf(rec.Own[2], 1))
}
