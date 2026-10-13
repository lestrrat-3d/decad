// Package spiralsector proves rigid copies of one spiral-bevel tooth disjoint
// before their combined Union and maps a copy's stitched result back to the
// first tooth's frame. See docs/faceted-vertex-bounds-design.md §3.7.
package spiralsector

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

// Shell is one tooth's held mesh as the sector proofs read it: vertices,
// facets, the per-vertex bound β and the largest facet bound.
type Shell struct {
	Verts       []r3.Vec
	Tris        [][3]int
	VertexBound []float64
	MeshBound   float64
}

func ratZero() *big.Rat { return new(big.Rat) }

func ratOne() *big.Rat { return big.NewRat(1, 1) }

func ratF(v float64) *big.Rat { return proof.FloatRat(v) }

func ratAdd(a, b *big.Rat) *big.Rat { return new(big.Rat).Add(a, b) }

func ratSub(a, b *big.Rat) *big.Rat { return new(big.Rat).Sub(a, b) }

func ratMul(a, b *big.Rat) *big.Rat { return new(big.Rat).Mul(a, b) }

func ratAbs(a *big.Rat) *big.Rat { return new(big.Rat).Abs(a) }
