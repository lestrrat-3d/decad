package loftmesh

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/capcontour"
	"github.com/lestrrat-3d/decad/internal/polynomial"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
)

// EdgeLength bounds a straight loft edge's held length. StraightEdgeBound
// charges the square root's committed error against the exact squared length;
// a zero-delta build needs no other allowance. A positive delta, whether from
// placement or a computed station (loft design §12), adds ChainLengthBound's
// one-chord allowance for both endpoints through AbsSumUpper.
func EdgeLength(a, b r3.Vec, delta float64) (float64, float64) {
	held := a.Sub(b).Len()
	sq, sqOK := proofarith.DySquaredDistance3(a.X, a.Y, a.Z, b.X, b.Y, b.Z)
	bound := capcontour.StraightEdgeBound(held, sq, sqOK)
	if delta > 0 {
		bound = proofbound.AbsSumUpper(bound, proofbound.ChainLengthBound(1, delta, held))
	}
	return held, bound
}

// JunctionApex returns the other incident triangle's vertex outside the shared edge.
func JunctionApex(tri [3]int, a, b int) int {
	for _, v := range tri {
		if v != a && v != b {
			return v
		}
	}
	return tri[0]
}

// JunctionConvex reads the exact orientation across a shared loft edge.
// The primary triangle's outward-wound vertices are A, B, C and the other's
// apex is D. OrientSign(A, B, C, D) < 0 marks a convex rung or diagonal;
// zero is a decided flat edge under loft design §5.
func JunctionConvex(verts []r3.Vec, primary, other [3]int, a, b int) bool {
	apex := JunctionApex(other, a, b)
	return proofarith.OrientSign(verts[primary[0]], verts[primary[1]], verts[primary[2]], verts[apex]) < 0
}

// CapTriangleAreaAllow sums the displacement allowance over a cap's own
// triangulation. It widens that cap's exact rational polygon area in the same
// way the loft mass accumulator widens its wall triangle areas.
func CapTriangleAreaAllow(verts []r3.Vec, tris [][3]int, delta float64) float64 {
	total := 0.0
	for _, t := range tris {
		total = proofbound.UpRound(total + proofbound.PerturbedTriangleAreaAllow(verts[t[0]], verts[t[1]], verts[t[2]], delta))
	}
	return total
}

// CapPolygonAreaRat sums the exact rational shoelace area of the plane-local
// points used to build the cap triangles. These are the same points passed to
// triangulation, so the cap's published area follows its built boundary. An
// untrimmed line endpoint matches the record's area integral; a trimmed line
// endpoint instead comes from the walk's float lerp, while the record integral
// uses rational lerp. The cap follows its built point in that case.
//
// The outer loop walks counterclockwise and holes walk clockwise, so their
// signed areas subtract. MustRatOf takes each coordinate exactly from its
// float64 value. Assembly already checks every lifted point is finite before
// this reading. The calculation needs no segment-kind assumption: circular
// and free-form stations enter as further points in the same loops.
func CapPolygonAreaRat(pts []sectionrecord.Point2, loopIdx [][]int) *big.Rat {
	sum := new(big.Rat)
	for _, idx := range loopIdx {
		n := len(idx)
		for j := range n {
			p, q := pts[idx[j]], pts[idx[(j+1)%n]]
			term := new(big.Rat).Mul(polynomial.MustRatOf(p.U), polynomial.MustRatOf(q.V))
			term.Sub(term, new(big.Rat).Mul(polynomial.MustRatOf(q.U), polynomial.MustRatOf(p.V)))
			sum.Add(sum, term)
		}
	}
	return sum.Quo(sum, big.NewRat(2, 1))
}
