package prismpattern

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/units"
)

// MeshSeparated reports whether all held section vertices strictly separate
// after charging both verified source-to-mesh bounds. A bounded region lies in the convex hull of its
// boundary, so this one plane separates the complete source solids.
func MeshSeparated(a, b facetedrecord.PrismPatternPart) bool {
	pa := a.Mesh.Vertices[0].Add(a.Mesh.Vertices[2*a.RootStart])
	pb := b.Mesh.Vertices[0].Add(b.Mesh.Vertices[2*b.RootStart])
	nx := new(big.Rat).Sub(proofarith.FloatRat(pa.X), proofarith.FloatRat(pb.X))
	ny := new(big.Rat).Sub(proofarith.FloatRat(pa.Y), proofarith.FloatRat(pb.Y))
	normSq := new(big.Rat).Add(new(big.Rat).Mul(nx, nx), new(big.Rat).Mul(ny, ny))
	if normSq.Sign() == 0 {
		return false
	}
	var minA, maxB *big.Rat
	for _, v := range a.Mesh.Vertices {
		dot := new(big.Rat).Add(new(big.Rat).Mul(nx, proofarith.FloatRat(v.X)),
			new(big.Rat).Mul(ny, proofarith.FloatRat(v.Y)))
		if minA == nil || dot.Cmp(minA) < 0 {
			minA = dot
		}
	}
	for _, v := range b.Mesh.Vertices {
		dot := new(big.Rat).Add(new(big.Rat).Mul(nx, proofarith.FloatRat(v.X)),
			new(big.Rat).Mul(ny, proofarith.FloatRat(v.Y)))
		if maxB == nil || dot.Cmp(maxB) > 0 {
			maxB = dot
		}
	}
	margin := proofbound.ProductUpper(proofbound.RatSqrtUp(normSq),
		proofbound.AbsSumUpper(units.Millimeters(a.Mesh.Bound).Base(), units.Millimeters(b.Mesh.Bound).Base()))
	return minA != nil && maxB != nil &&
		new(big.Rat).Sub(minA, maxB).Cmp(proofarith.FloatRat(margin)) > 0
}
