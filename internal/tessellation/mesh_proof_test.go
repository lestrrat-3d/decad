package tessellation_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/tessellation"
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
