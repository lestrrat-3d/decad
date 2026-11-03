package meshbool

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// TestParityObliqueRays queries the octahedron |x| + |y| + |z| ≤ 1 at points
// on its symmetry axes. Every axis ray from the centre runs through a vertex,
// so all six are ambiguous; an oblique ray crosses one face and places the
// centre inside, both through the subset scan and the all-facets scan. A
// point beyond a vertex on an axis is outside, and a vertex itself, where
// every ray meets the boundary, stays a genuine failure.
//
// Shown to fail: without the oblique fallback the centre is refused with
// "every parity ray was ambiguous".
func TestParityObliqueRays(t *testing.T) {
	t.Parallel()
	verts := []r3.Vec{{X: 1}, {X: -1}, {Y: 1}, {Y: -1}, {Z: 1}, {Z: -1}}
	var tris [][3]int
	for _, x := range []int{0, 1} {
		for _, y := range []int{2, 3} {
			for _, z := range []int{4, 5} {
				tri := [3]int{x, y, z}
				// Orient each face outward: (y−x)×(z−x) along the face's octant.
				a, b, c := verts[x], verts[y], verts[z]
				n := b.Sub(a).Cross(c.Sub(a))
				if n.Dot(a.Add(b).Add(c)) < 0 {
					tri[1], tri[2] = tri[2], tri[1]
				}
				tris = append(tris, tri)
			}
		}
	}
	all := make([]int, len(tris))
	for i := range all {
		all[i] = i
	}
	at := func(x, y, z int64) proof.Xpt {
		return ratXpt(big.NewRat(x, 1), big.NewRat(y, 1), big.NewRat(z, 1))
	}
	for _, tc := range []struct {
		name   string
		p      proof.Xpt
		inside bool
	}{
		{"centre", at(0, 0, 0), true},
		{"beyond a vertex", at(0, 0, 2), false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			inside, onBoundary, err := MeshParityPreparedContext(t.Context(), tc.p, NewParityMesh(verts, tris), all)
			require.NoError(t, err)
			require.False(t, onBoundary)
			require.Equal(t, tc.inside, inside)
			inside, onBoundary, err = MeshParityAllContext(t.Context(), tc.p, NewParityMesh(verts, tris))
			require.NoError(t, err)
			require.False(t, onBoundary)
			require.Equal(t, tc.inside, inside)
		})
	}
	t.Run("a vertex", func(t *testing.T) {
		t.Parallel()
		_, _, err := MeshParityAllContext(t.Context(), at(1, 0, 0), NewParityMesh(verts, tris))
		require.ErrorContains(t, err, "every parity ray was ambiguous")
	})
}
