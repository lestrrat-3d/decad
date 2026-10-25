package meshbool

import (
	"context"
	"errors"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestPlaneSignSharesCoplanarFacetsWithOppositeWinding(t *testing.T) {
	verts := []r3.Vec{
		r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0),
		r3.NewVec(1, 1, 0), r3.NewVec(0, 0, 1),
	}
	h := HeldRounding{
		Verts: verts,
		Tris:  [][3]int{{0, 1, 2}, {3, 1, 2}},
		Exact: make([]proof.Xpt, len(verts)),
	}
	for i, v := range verts {
		h.Exact[i] = proof.XptOf(v)
	}
	for _, first := range []int{0, 1} {
		e := newHeldEmbedding(h, proofbound.NewWorkBudget(t.Context()), true)
		e.sideSigns = make(map[[2]int]int)
		require.Equal(t, 1-2*first, e.planeSign(first, 4))
		require.Equal(t, 2*first-1, e.planeSign(1-first, 4))
		require.Len(t, e.sideSigns, 1)
		require.Equal(t, 0, e.planeSign(first, 0))
		require.Equal(t, 0, e.planeSign(1-first, 0))
		require.Len(t, e.sideSigns, 2)
	}
}

// The rounded facets are separate, but all 4,096 of them need checking.
// This enters parallelClean and compares its answer with the serial repair
// path on the same real rounding relationship.
func TestParallelHeldEmbeddingMatchesSerialAndPollsCancellation(t *testing.T) {
	const facets = 4_096
	shift := r3.NewVec(0.1, 0, 0)
	h := HeldRounding{
		Verts:   make([]r3.Vec, 3*facets),
		Tris:    make([][3]int, facets),
		Exact:   make([]proof.Xpt, 3*facets),
		Moved:   make([]bool, 3*facets),
		Movable: make([]bool, 3*facets),
	}
	for i := range facets {
		x := float64(4 * i)
		unplaced := [3]r3.Vec{
			r3.NewVec(x, 0, 0), r3.NewVec(x+1, 0, 0), r3.NewVec(x, 1, 0),
		}
		h.Tris[i] = [3]int{3 * i, 3*i + 1, 3*i + 2}
		for k, p := range unplaced {
			j := 3*i + k
			h.Verts[j] = p.Add(shift)
			h.Exact[j] = ExactRigidImage(r3.Identity().Basis(), shift, p)
			h.Moved[j] = CoordDistance(h.Exact[j], h.Verts[j]).Sign() != 0
			h.Movable[j] = h.Moved[j]
		}
		if !h.Moved[3*i] && !h.Moved[3*i+1] && !h.Moved[3*i+2] {
			t.Fatalf("facet %d has no rounded vertex", i)
		}
	}
	serial := slices.Clone(h.Verts)
	parallel := slices.Clone(h.Verts)
	h.Verts = serial
	nSerial, err := enforceHeldEmbedding(t.Context(), h, false)
	if err != nil {
		t.Fatalf("serial embedding: %v", err)
	}
	h.Verts = parallel
	nParallel, err := enforceHeldEmbedding(t.Context(), h, true)
	if err != nil || nParallel != nSerial || !slices.Equal(parallel, serial) {
		t.Fatalf("parallel embedding = (%d, %v), serial = (%d, nil)", nParallel, err, nSerial)
	}

	e := newHeldEmbedding(h, proofbound.NewWorkBudget(t.Context()), true)
	if err := e.buildGrid(); err != nil {
		t.Fatal(err)
	}
	query := make([]int, facets)
	for i := range query {
		query[i] = i
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	_, err = e.parallelClean(ctx, query)
	if !errors.Is(err, context.Canceled) {
		t.Fatalf("parallel scan after cancellation = %v, want context.Canceled", err)
	}
}

func TestHeldEmbeddingScanPollsAcrossChunks(t *testing.T) {
	h := HeldRounding{
		Verts: []r3.Vec{{}, {X: 1}, {Y: 1}},
		Tris:  [][3]int{{0, 1, 2}},
	}
	e := newHeldEmbedding(h, proofbound.NewWorkBudget(t.Context()), true)
	ctx, cancel := context.WithCancel(t.Context())
	defer cancel()
	e.pollErr, e.pollInterval, e.pollRemaining = ctx.Err, 4, 4
	tree := newHeldBoxTree(e)
	// One node and its one facet consume half an interval. A new chunk
	// must use the remaining half rather than restart the countdown.
	clean, err := e.cleanPairs([]int{0}, []bool{true}, &tree)
	require.NoError(t, err)
	require.True(t, clean)
	require.Equal(t, 2, e.pollRemaining)
	cancel()
	clean, err = e.cleanPairs([]int{0}, []bool{true}, &tree)
	require.ErrorIs(t, err, context.Canceled)
	require.False(t, clean)
	require.Equal(t, 4, e.pollRemaining)
}
