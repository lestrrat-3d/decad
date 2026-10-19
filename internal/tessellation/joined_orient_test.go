package tessellation_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestOrientJoinedMesh(t *testing.T) {
	verts := []r3.Vec{{}, {X: 1}, {Y: 1}, {Z: 1}}
	outward := func() [][3]int {
		return [][3]int{{0, 2, 1}, {0, 1, 3}, {1, 2, 3}, {0, 3, 2}}
	}

	t.Run("one flipped triangle", func(t *testing.T) {
		tris := outward()
		tris[1] = [3]int{0, 3, 1}
		require.NoError(t, tessellation.OrientJoinedMesh(verts, tris))
		require.Equal(t, outward(), tris)
		require.Equal(t, []r3.Vec{{}, {X: 1}, {Y: 1}, {Z: 1}}, verts)
	})
	t.Run("whole mesh inward", func(t *testing.T) {
		tris := outward()
		for i := range tris {
			tris[i][1], tris[i][2] = tris[i][2], tris[i][1]
		}
		require.NoError(t, tessellation.OrientJoinedMesh(verts, tris))
		require.Equal(t, outward(), tris)
	})

	for _, tc := range []struct {
		name  string
		verts []r3.Vec
		tris  [][3]int
		text  string
	}{
		{"collapsed triangle", verts, [][3]int{{0, 0, 1}}, "joined mesh has a collapsed triangle"},
		{"unpaired edges", verts, outward()[:3], "joined seam does not pair every mesh edge"},
		{
			"second component",
			append(append([]r3.Vec(nil), verts...), verts...),
			append(outward(), [][3]int{{4, 6, 5}, {4, 5, 7}, {5, 6, 7}, {4, 7, 6}}...),
			"joined mesh has another component",
		},
		{"empty", verts, nil, "joined mesh is empty"},
	} {
		t.Run(tc.name, func(t *testing.T) {
			err := tessellation.OrientJoinedMesh(tc.verts, tc.tris)
			require.ErrorIs(t, err, decaderr.ErrUnsupported)
			require.ErrorContains(t, err, tc.text)
		})
	}
}
