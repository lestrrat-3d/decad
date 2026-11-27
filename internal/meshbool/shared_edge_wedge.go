package meshbool

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// SharedEdgeCertifier vouches for one shared edge of a pair of operands. A caller
// that holds source records for both operands implements it; the mesh pass asks
// it before it accepts a shared edge (agent-docs/coplanar-contact-design.md §22).
type SharedEdgeCertifier interface {
	// CertifySharedEdge reports whether edge k of facet i of the first operand
	// and edge l of facet j of the second are both held on one true straight
	// edge that the two true operands share, and whether each of the four faces
	// beside the shared edge meets it in a plane that the held facet's plane
	// matches within sharedEdgeMargin of a radian. The two edges need not have
	// the same extent.
	CertifySharedEdge(ctx context.Context, i, k, j, l int) (bool, error)
}

// sharedEdgeMargin is the least sine of the angle every pair of the four
// half-planes at a shared edge must hold exactly. A certifier vouches that each
// held half-plane lies within a small fraction of it from the true one, so a
// verdict the exact held angles give is the verdict of the true angles.
var sharedEdgeMargin = new(big.Rat).SetFrac64(1, 1000)

// sharedEdgeKey names one shared edge by the undirected mesh edge of each operand.
type sharedEdgeKey struct{ a, b [2]int }

// sharedEdgeVerdict is how many of the first operand's two facets beside the
// shared edge lie inside the second operand's wedge there, and the converse.
type sharedEdgeVerdict struct{ aInside, bInside int }

// kept is how many of the four facets a Boolean keeps at the edge, given where
// each operand's kept facets lie (BooleanKeep).
func (v sharedEdgeVerdict) kept(wantAInside, wantBInside bool) int {
	n := 0
	if wantAInside {
		n += v.aInside
	} else {
		n += 2 - v.aInside
	}
	if wantBInside {
		n += v.bInside
	} else {
		n += 2 - v.bInside
	}
	return n
}

// edgeKeyOf is the undirected mesh edge k of facet f.
func edgeKeyOf(m *BoolMesh, f, k int) [2]int {
	u, v := m.Tris[f][k], m.Tris[f][(k+1)%3]
	return [2]int{min(u, v), max(u, v)}
}

// wedgeFacet is one facet beside the edge, read exactly in the plane
// perpendicular to the edge: the direction of the facet's half-plane away from
// the edge, and its outward normal.
type wedgeFacet struct {
	dir, normal [3]*big.Rat
}

// sharedEdgeFacet reads facet f, whose edge k lies on the line through origin
// with direction d, as its half-plane direction and outward normal.
func sharedEdgeFacet(m *BoolMesh, f, k int, origin, d [3]*big.Rat) (wedgeFacet, bool) {
	tri := m.Tris[f]
	v0, v1, v2 := ratVec(m.Verts[tri[0]]), ratVec(m.Verts[tri[1]]), ratVec(m.Verts[tri[2]])
	normal := ratCross(ratSub(v1, v0), ratSub(v2, v0))
	apex := ratSub(ratVec(m.Verts[tri[(k+2)%3]]), origin)
	along := new(big.Rat).Quo(ratDot(apex, d), ratDot(d, d))
	var dir [3]*big.Rat
	for c := range dir {
		dir[c] = new(big.Rat).Sub(apex[c], new(big.Rat).Mul(along, d[c]))
	}
	if ratIsZero(dir) || ratIsZero(normal) || ratDot(dir, normal).Sign() != 0 || ratDot(d, normal).Sign() != 0 {
		return wedgeFacet{}, false
	}
	return wedgeFacet{dir: dir, normal: normal}, true
}

// clearOf reports whether sign s of the product of u and w, whose squared
// lengths are uu and ww, is at least the margin in sine from zero.
func clearOf(s, uu, ww *big.Rat) bool {
	lhs := new(big.Rat).Mul(s, s)
	rhs := new(big.Rat).Mul(uu, ww)
	rhs.Mul(rhs, sharedEdgeMargin)
	rhs.Mul(rhs, sharedEdgeMargin)
	return s.Sign() != 0 && lhs.Cmp(rhs) >= 0
}

// wedgeOf is the solid wedge two facets beside an edge bound: it reads whether
// the wedge is convex, and refuses (false) a flat, folded or near-flat one.
func wedgeOf(f1, f2 wedgeFacet) (convex, ok bool) {
	c12, c21 := ratDot(f2.dir, f1.normal), ratDot(f1.dir, f2.normal)
	if c12.Sign() == 0 || c12.Sign() != c21.Sign() {
		return false, false
	}
	if !clearOf(c12, ratDot(f2.dir, f2.dir), ratDot(f1.normal, f1.normal)) ||
		!clearOf(c21, ratDot(f1.dir, f1.dir), ratDot(f2.normal, f2.normal)) {
		return false, false
	}
	return c12.Sign() < 0, true
}

// insideWedge reports whether facet q's half-plane lies strictly inside the
// wedge of w1 and w2, and false in ok when q lies within the margin of either
// bounding plane.
func insideWedge(q, w1, w2 wedgeFacet, convex bool) (inside, ok bool) {
	s1, s2 := ratDot(q.dir, w1.normal), ratDot(q.dir, w2.normal)
	qq := ratDot(q.dir, q.dir)
	if !clearOf(s1, qq, ratDot(w1.normal, w1.normal)) || !clearOf(s2, qq, ratDot(w2.normal, w2.normal)) {
		return false, false
	}
	if convex {
		return s1.Sign() < 0 && s2.Sign() < 0, true
	}
	return s1.Sign() < 0 || s2.Sign() < 0, true
}

// sharedEdgeWedge reads the local arrangement at a shared edge: edge k of
// facet i of ma and edge l of facet j of mb lie on one line, and each
// operand's two facets there bound a wedge of its solid. It returns how many
// of each operand's two facets lie inside the other's wedge. It refuses when
// any two of the four half-planes come within the margin of each other's
// plane, so a coplanar or tangent arrangement never passes.
func sharedEdgeWedge(ma, mb *BoolMesh, i, k, j, l int) (sharedEdgeVerdict, bool) {
	twinA, okA := ma.TwinFacet(i, k)
	twinB, okB := mb.TwinFacet(j, l)
	if !okA || !okB {
		return sharedEdgeVerdict{}, false
	}
	u, v := ma.Verts[ma.Tris[i][k]], ma.Verts[ma.Tris[i][(k+1)%3]]
	if !proofbound.FiniteVec(u) || !proofbound.FiniteVec(v) {
		return sharedEdgeVerdict{}, false
	}
	origin := ratVec(u)
	d := ratSub(ratVec(v), origin)
	if ratIsZero(d) {
		return sharedEdgeVerdict{}, false
	}
	// The second operand's edge must lie on the same line.
	for _, e := range [2]int{l, (l + 1) % 3} {
		rel := ratSub(ratVec(mb.Verts[mb.Tris[j][e]]), origin)
		if !ratIsZero(ratCross(d, rel)) {
			return sharedEdgeVerdict{}, false
		}
	}
	// A facet's own edge index on the twin: the edge it shares with the facet.
	twinEdge := func(m *BoolMesh, f, g int) (int, bool) {
		for e := range 3 {
			if t, ok := m.TwinFacet(f, e); ok && t == g {
				return e, true
			}
		}
		return 0, false
	}
	eA, okA := twinEdge(ma, twinA, i)
	eB, okB := twinEdge(mb, twinB, j)
	if !okA || !okB {
		return sharedEdgeVerdict{}, false
	}
	var fa, fb [2]wedgeFacet
	var ok [4]bool
	fa[0], ok[0] = sharedEdgeFacet(ma, i, k, origin, d)
	fa[1], ok[1] = sharedEdgeFacet(ma, twinA, eA, origin, d)
	fb[0], ok[2] = sharedEdgeFacet(mb, j, l, origin, d)
	fb[1], ok[3] = sharedEdgeFacet(mb, twinB, eB, origin, d)
	if !ok[0] || !ok[1] || !ok[2] || !ok[3] {
		return sharedEdgeVerdict{}, false
	}
	convexA, okA := wedgeOf(fa[0], fa[1])
	convexB, okB := wedgeOf(fb[0], fb[1])
	if !okA || !okB {
		return sharedEdgeVerdict{}, false
	}
	var verdict sharedEdgeVerdict
	for _, q := range fa {
		in, ok := insideWedge(q, fb[0], fb[1], convexB)
		if !ok {
			return sharedEdgeVerdict{}, false
		}
		if in {
			verdict.aInside++
		}
	}
	for _, q := range fb {
		in, ok := insideWedge(q, fa[0], fa[1], convexA)
		if !ok {
			return sharedEdgeVerdict{}, false
		}
		if in {
			verdict.bInside++
		}
	}
	return verdict, true
}

// refuseNonManifoldSharedEdges refuses a Boolean whose result would hold a
// shared edge with other than two or no kept facets: the result there would be
// a non-manifold edge, which stitching must never be handed.
func refuseNonManifoldSharedEdges(edges map[sharedEdgeKey]sharedEdgeVerdict, wantAInside, wantBInside bool) error {
	for _, verdict := range edges {
		if n := verdict.kept(wantAInside, wantBInside); n != 0 && n != 2 {
			return ExpectedBoolean(BooleanExpectedContact, &TouchContactError{What: touchGraze})
		}
	}
	return nil
}
