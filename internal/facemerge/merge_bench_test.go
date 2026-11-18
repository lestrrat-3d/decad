package facemerge_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/facemerge"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// plateInput is a 64×64×1 plate whose top cells alternate between two operands, one source group per cell, so
// the whole top is one component. Operand A's groups come first; group 0 is the bottom and sides.
func plateInput(tb testing.TB, n int) facemerge.Input {
	tb.Helper()
	id := map[[2]int]int{}
	next := 1
	for pass := range 2 {
		for i := range n {
			for j := range n {
				if ((i+j)%2 == 0) == (pass == 0) {
					id[[2]int{i, j}] = next
					next++
				}
			}
		}
		if pass == 0 {
			id[[2]int{-1, -1}] = next
		}
	}
	v, f := r3.NewVec, float64(n)
	m := newMeshBuilder()
	m.gridFace(v(0, 0, 1), v(1, 0, 0), v(0, 1, 0), n, n, func(i, j int) int { return id[[2]int{i, j}] }, nil)
	m.gridFace(v(0, 0, 0), v(0, 1, 0), v(1, 0, 0), n, n, one(0), nil)
	m.gridFace(v(0, 0, 0), v(1, 0, 0), v(0, 0, 1), n, 1, one(0), nil)
	m.gridFace(v(0, f, 0), v(0, 0, 1), v(1, 0, 0), 1, n, one(0), nil)
	m.gridFace(v(0, 0, 0), v(0, 0, 1), v(0, 1, 0), 1, n, one(0), nil)
	m.gridFace(v(f, 0, 0), v(0, 1, 0), v(0, 0, 1), n, 1, one(0), nil)
	audit, err := facetproof.AuditFacetedMesh(tb.Context(), m.verts, m.tris)
	require.NoError(tb, err)
	return facemerge.Input{
		Verts: m.verts, VertexBound: make([]float64, len(m.verts)), Tris: m.tris, Src: m.src,
		Groups: plainGroups(next), Adj: audit.Adj, OperandAGroups: id[[2]int{-1, -1}],
	}
}

func BenchmarkMerge(b *testing.B) {
	in := plateInput(b, 64)
	_, groups, changed, err := facemerge.Merge(b.Context(), in)
	require.NoError(b, err)
	require.True(b, changed)
	require.Len(b, groups, len(in.Groups)+1)
	b.ReportAllocs()
	for b.Loop() {
		if _, _, _, err := facemerge.Merge(b.Context(), in); err != nil {
			b.Fatal(err)
		}
	}
}
