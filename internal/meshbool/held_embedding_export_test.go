package meshbool

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// EnforceHeldEmbeddingMemo runs the embedding check with the box cache and
// the pair memo on or off, so the tests can compare the two paths.
func EnforceHeldEmbeddingMemo(ctx context.Context, h HeldRounding, memo bool) (int, error) {
	return enforceHeldEmbedding(ctx, h, memo)
}

// HeldPairsAsymmetric lists the facet pairs of h, lower index first, whose
// pairEmbedded answer depends on which facet is asked first. The pair memo
// keys on the unordered pair, which is sound only while this list is empty.
func HeldPairsAsymmetric(ctx context.Context, h HeldRounding) [][2]int {
	e := newHeldEmbedding(h, proofbound.NewWorkBudget(ctx), false)
	var out [][2]int
	for i := range h.Tris {
		for j := i + 1; j < len(h.Tris); j++ {
			if e.pairEmbedded(i, j) != e.pairEmbedded(j, i) {
				out = append(out, [2]int{i, j})
			}
		}
	}
	return out
}
