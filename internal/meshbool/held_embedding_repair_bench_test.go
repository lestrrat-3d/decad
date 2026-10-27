package meshbool

import (
	"math/rand/v2"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/stretchr/testify/require"
)

// heldTubeWithFolds is heldRevolvedTube moved 20 along x, every vertex its own
// exact point and moved but none movable, beside stitchLayered's sliver
// stacks. Every facet is a query, so the first repair pass scans the whole
// tube, and the slivers fold so the repair has to run.
func heldTubeWithFolds(segments, layers int, seed uint64) HeldRounding {
	tube := heldRevolvedTube(segments, false)
	slivers := stitchLayered(rand.New(rand.NewPCG(seed, 3)), layers)
	h := HeldRounding{}
	for _, v := range tube.Verts {
		v.X += 20
		h.Verts = append(h.Verts, v)
		h.Exact = append(h.Exact, proof.XptOf(v))
		h.Moved = append(h.Moved, true)
		h.Movable = append(h.Movable, false)
	}
	h.Tris = append(h.Tris, tube.Tris...)
	base := len(h.Verts)
	h.Verts = append(h.Verts, slivers.Verts...)
	h.Exact = append(h.Exact, slivers.Exact...)
	h.Moved = append(h.Moved, slivers.Moved...)
	h.Movable = append(h.Movable, slivers.Movable...)
	for _, t := range slivers.Tris {
		h.Tris = append(h.Tris, [3]int{t[0] + base, t[1] + base, t[2] + base})
	}
	return h
}

// BenchmarkHeldEmbeddingRepairScan runs the whole repair over a tube whose
// every facet is a query beside sliver stacks that fold, so the first scan
// fails and offenders scans every facet before the repair moves anything.
func BenchmarkHeldEmbeddingRepairScan(b *testing.B) {
	h := heldTubeWithFolds(1160, 24, 5)
	var want []int
	for b.Loop() {
		c := h
		c.Verts = slices.Clone(h.Verts)
		_, failing, err := RepairHeldEmbedding(b.Context(), c)
		require.NoError(b, err)
		if want == nil {
			want = append([]int{}, failing...)
		}
		require.Equal(b, want, failing)
	}
}

// BenchmarkHeldEmbeddingRepairMoves runs the whole repair over stacked
// slivers that all overlap, so most of the time goes to moving vertices:
// improve's trials and the passes over the facets they touch.
func BenchmarkHeldEmbeddingRepairMoves(b *testing.B) {
	h := stitchLayered(rand.New(rand.NewPCG(9, 1)), 120)
	var want []int
	for b.Loop() {
		c := h
		c.Verts = slices.Clone(h.Verts)
		_, failing, err := RepairHeldEmbedding(b.Context(), c)
		require.NoError(b, err)
		if want == nil {
			want = append([]int{}, failing...)
		}
		require.Equal(b, want, failing)
	}
}
