package decad

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/lattice"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// BenchmarkMeshPlaneGroups groups the lattice meshes Infill converts in
// TestInfillTakesEachLatticeKind: the 10 x 10 x 12 block widened by a quarter cell, cell 10, eight samples a cell.
func BenchmarkMeshPlaneGroups(b *testing.B) {
	lo, hi := r3.NewVec(2.5, 2.5, -2.5), r3.NewVec(17.5, 17.5, 14.5)
	for _, c := range []struct {
		name  string
		kind  lattice.Kind
		level float64
	}{
		{"gyroid", lattice.KindGyroid, 0.6},
		{"schwarzp", lattice.KindSchwarzP, 1},
		{"diamond", lattice.KindDiamond, 0.5},
		{"neovius", lattice.KindNeovius, 4},
		{"iwp", lattice.KindIWP, 2},
	} {
		verts, tris, err := lattice.Surface(b.Context(), c.kind, lo, hi, 10, c.level, 8)
		require.NoError(b, err)
		b.Run(c.name, func(b *testing.B) {
			b.ReportAllocs()
			for b.Loop() {
				meshPlaneGroups(1, verts, tris)
			}
		})
	}
}
