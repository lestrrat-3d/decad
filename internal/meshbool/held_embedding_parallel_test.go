package meshbool

import (
	"context"
	"errors"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

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
