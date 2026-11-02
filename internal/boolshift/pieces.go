package boolshift

import (
	"context"
	"slices"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// NewPieces copies piece labels for a mesh.
func NewPieces(vertices, facets []int, count int) Pieces {
	return Pieces{ofVertex: slices.Clone(vertices), ofFacet: slices.Clone(facets), count: count}
}
func (p Pieces) Count() int       { return p.count }
func (p Pieces) VertexCount() int { return len(p.ofVertex) }
func (p Pieces) FacetCount() int  { return len(p.ofFacet) }
func (p Pieces) Vertex(i int) int { return p.ofVertex[i] }
func (p Pieces) Facet(i int) int  { return p.ofFacet[i] }
func (p Pieces) Clone() Pieces    { return NewPieces(p.ofVertex, p.ofFacet, p.count) }
func (p Pieces) EqualLabels(other Pieces) bool {
	return slices.Equal(p.ofVertex, other.ofVertex) && slices.Equal(p.ofFacet, other.ofFacet)
}
func (p Pieces) EqualSnapshot(other Pieces) bool {
	return (p.ofVertex == nil) == (other.ofVertex == nil) && (p.ofFacet == nil) == (other.ofFacet == nil) && p.EqualLabels(other)
}

// Snapshot clones both label arrays after charging the supplied replay budget.
func (p Pieces) Snapshot(ctx context.Context, budget *proofbound.ReplayBudget, tooLarge error) (Pieces, error) {
	for _, src := range [][]int{p.ofVertex, p.ofFacet} {
		if src == nil {
			continue
		}
		n := uint64(len(src))
		if n > proofbound.ReplayTrialRecords {
			return Pieces{}, tooLarge
		}
		if err := budget.SpendWork(ctx, n); err != nil {
			return Pieces{}, err
		}
		if err := budget.ReserveRecords(ctx, n); err != nil {
			return Pieces{}, err
		}
		if err := budget.ReserveArray(ctx, n, uint64(unsafe.Sizeof(int(0)))); err != nil {
			return Pieces{}, err
		}
	}
	return p.Clone(), nil
}
