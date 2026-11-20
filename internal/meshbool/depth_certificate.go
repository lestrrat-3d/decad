package meshbool

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// The witness depth certificate (agent-docs/evaluator-design.md §9, "The
// witness depth scan stops once a candidate is settled as no deeper than b").
// CertifiedInteriorDepth reads the float minimum F of depthFacetDistance over
// a box-pruned scan of the other mesh's facets and returns
//
//	fl(fl(fl(F·c) − R) − E),  c = 1 − 1e-12,  R = PointRoundBound,
//	E = ProvenUpRound(depthErrCoef·M + depthErrFloor),
//
// where M is the largest coordinate magnitude of the rounded query point pf
// and of the mesh's vertices. It returns 0 when F <= 0, F is not finite, or
// M exceeds depthReachLimit (or is not finite). Rounding to nearest is
// monotone and R, E >= 0, so the value is at most fl(F·c), which the scan's
// early stops rely on.
//
// Soundness: the value never exceeds the exact distance d(p) from the exact
// point p to the mesh's closed facets. Write u = 2^-53 and let d* be the exact
// distance from pf, so d(p) >= d* − |p − pf| >= d* − R. Below, every
// coordinate the scan reads is at most M <= 2^500 in magnitude, so nothing
// overflows (the largest intermediate, an edge cross product dotted with the
// normal, stays below 2^1010). Underflow is charged at the end.
//
//   - The scan bound. Let i be a facet whose exact distance from pf is d*.
//     When the scan computes facet i, F <= its computed distance D_i. When it
//     skips facet i, by its own box or by a depth block containing that box,
//     the box distance it read is at least the running minimum, so F is at
//     most that reading. PointBoxDist takes nonnegative per-axis gaps with one
//     rounded subtraction each, then three squares, two sums and a square
//     root, so it reads at most (1+4u) times the exact box distance, which is
//     at most d* because the box contains the facet. So it is enough to show
//     D_i <= (1+12u)·d* + A with A = 132u·M.
//   - The normal. depthNormalOf scales the exact normal of the facet's float
//     corners by a power of two so its largest component lies in (1, 4), then
//     rounds each component to nearest. With N that scaled exact normal and n
//     its rounding, |n − N| <= u'|N| with u' = u + 2^-1073 (a subnormal
//     component errs by at most 2^-1075 < 2^-1075·|N|), and n·n >= 1.
//   - The foot reading (pointTriFoot returns true). Its value is within
//     (1+12u) of the distance from pf to the facet's PLANE plus 5.1u·|w| +
//     1.75u·M, with w = pf − t0 and |w| <= 2√3·M: the dot product n·w errs by
//     at most 5.01u·|N||w|, the division by n·n, the scaling by n, the
//     subtraction from pf and the final length add relative errors on the
//     plane distance and u per coordinate of the foot. The plane distance is
//     at most d*, so this reading is at most (1+12u)·d* + 20u·M whether or not
//     the foot truly lies in the facet.
//   - An edge reading. pointSegDist3 clamps its parameter into [0, 1], so its
//     point lies on the closed edge up to rounding. Its parameter times the
//     edge length errs by at most 11.1u·|w| (w = pf − q0, |w| <= 2√3·M), the
//     scaled edge by 2.1u times its length (<= 2√3·M), and the point's
//     coordinates by u·M each. So the reading is at most (1+5u) times the exact
//     distance to the closed edge plus 48u·M.
//   - The edge fallback. When the exact nearest point of the facet lies on its
//     boundary, d* is the least exact edge distance and the edge bound above
//     gives D_i <= (1+5u)·d* + 48u·M. Otherwise the exact foot f lies inside
//     the facet and d* is the plane distance, yet some edge test of
//     PointInTriangle read the computed foot as outside. That test's exact
//     value at f is |e||N| times σ, the in-plane distance from f to the
//     edge's line, and its computed value errs by at most |e||N| times
//     8.9u·|f − t_j| + 1.01·|computed foot − f|. The computed foot lies within
//     14.3u·|w| + 1.74u·M of f, so σ <= 83u·M. Inside a convex polygon the
//     distance to the boundary is the least distance to an edge line, so some
//     closed edge lies within d* + σ of pf, and D_i <= (1+5u)·d* + 132u·M.
//   - Underflow. A subnormal product or quotient errs by at most 2^-1075. The
//     scaled normal keeps n·n >= 1. An edge shorter than 2^-510 leaves every
//     point of the facet within that length of another edge, and an edge test
//     or squared edge length that underflows does so only within 2^-510 of an
//     edge or for an edge that short. PointBoxDist squares its gaps directly
//     and loses at most 2^-536 when they underflow. Len squares only
//     components in [2^-511, 2^511], whose squares stay normal, and passes
//     any other vector to math.Hypot, which scales before it squares, so it
//     loses at most 2^-1070 beyond its relative error. So underflow adds at
//     most 2^-505 to A, which depthErrFloor covers.
//
// So F <= (1+12u)·(d* + A + 2^-505) with A + 2^-505 <= E. When the value is
// positive, fl(fl(F·c) − R) − E rounds to at most fl(F·c)·(1+3u) − R − E <=
// (1−1e-12)(1+17u)·(d* + E) − R − E <= d* − R <= d(p), because 17u is far
// below 1e-12. Relative errors of the scan (the box readings, the square
// roots, the products by c) are what the factor c absorbs; the absolute ones
// are E's.
const (
	// depthErrCoef is 256u, about twice the 132u the argument above needs.
	depthErrCoef = 0x1p-45
	// depthErrFloor covers every underflow loss of the scan.
	depthErrFloor = 0x1p-500
	// depthReachLimit is the largest coordinate magnitude the argument above
	// covers without overflow. A query or mesh past it certifies no depth.
	depthReachLimit = 0x1p500
)

// depthCertificate is CertifiedInteriorDepth's value for a scan minimum best
// from the rounded point pf of p.
func depthCertificate(best float64, p proof.Xpt, pf r3.Vec, other *BoolMesh) float64 {
	if best <= 0 || proofbound.IsNonFinite(best) {
		return 0
	}
	m := max(math.Abs(pf.X), math.Abs(pf.Y), math.Abs(pf.Z), other.depthCoordReach())
	if !(m <= depthReachLimit) {
		return 0
	}
	e := depthErrBound(m)
	return depthCertFrom(best, PointRoundBound(p, pf), e)
}

// depthErrBound is depthCertificate's E for the coordinate magnitude m. Each
// step rounds on its own, so no fused multiply-add can change the value from
// one architecture to the next.
func depthErrBound(m float64) float64 {
	return proofbound.ProvenUpRound(float64(float64(depthErrCoef*m) + depthErrFloor))
}

// depthCertFrom is fl(fl(fl(best·c) − r) − e), depthCertificate's value for a
// positive finite scan minimum best, with r = PointRoundBound and e =
// depthErrBound. It is non-decreasing in best, because each step rounds a
// non-decreasing function of the step before it.
func depthCertFrom(best, r, e float64) float64 {
	return float64(float64(best*(1-1e-12))-r) - e
}

// depthFacetDistance is the depth scan's distance from p to the facet t with
// depth normal n (BoolMesh.depthNormal): the perpendicular foot along n when
// PointInTriangle places it inside t, otherwise the nearest closed edge.
//
// It stays out of line so depthBeyond's hint reading and its scan run the same
// compiled code (agent-docs/evaluator-design.md §9).
//
//go:noinline
func depthFacetDistance(p r3.Vec, t [3]r3.Vec, n r3.Vec) float64 {
	if d, ok := pointTriFoot(p, t, n); ok {
		return d
	}
	best := posInf
	for i := range 3 {
		best = minFloat64(best, pointSegDist3(p, t[i], t[(i+1)%3]))
	}
	return best
}

// depthNormalOf scales the exact normal n by a power of two that puts its
// largest component in (1, 4), then rounds each component to the nearest
// float64. A zero normal stays zero.
func depthNormalOf(n proof.Xpt) r3.Vec {
	if max(n.X.BitLen(), n.Y.BitLen(), n.Z.BitLen()) == 0 {
		return r3.Vec{}
	}
	shift := depthNormalShift(n)
	if shift >= 0 {
		k := uint(shift)
		return proof.Xpt{X: new(big.Int).Lsh(n.X, k), Y: new(big.Int).Lsh(n.Y, k), Z: new(big.Int).Lsh(n.Z, k), W: n.W}.Vec()
	}
	return proof.Xpt{X: n.X, Y: n.Y, Z: n.Z, W: new(big.Int).Lsh(n.W, uint(-shift))}.Vec()
}

// depthNormalShift is the power of two depthNormalOf scales the nonzero exact
// normal n by. The largest component lies in (2^(top−w−1), 2^(top−w+1)), top
// the largest component bit length and w the weight's, so 2^shift times it
// lies in (1, 4).
func depthNormalShift(n proof.Xpt) int {
	return n.W.BitLen() - max(n.X.BitLen(), n.Y.BitLen(), n.Z.BitLen()) + 1
}

// setDepthNormals fills the depth scan's per-facet normals from Norms and the
// largest vertex coordinate magnitude from Verts. Every preparation calls it
// once Norms is filled, and a caller that changes Verts, Tris or Norms
// afterwards must call it again. It charges no budget work: the budgeted
// preparations already charged each exact normal it rounds, and they reserve
// the depthNorms array.
func (bm *BoolMesh) setDepthNormals() {
	bm.depthNorms = make([]r3.Vec, len(bm.Tris))
	for i := range bm.Tris {
		bm.depthNorms[i] = depthNormalOf(bm.Norms[i])
	}
	bm.depthReach = vertexReach(bm.Verts)
}

// depthNormal is facet i's depth normal: depthNormalOf of the exact normal of
// its float corners. A mesh without prepared depth normals computes it from
// the corners on each call and stores nothing, so concurrent scans stay safe.
func (bm *BoolMesh) depthNormal(i int) r3.Vec {
	if len(bm.depthNorms) == len(bm.Tris) {
		return bm.depthNorms[i]
	}
	t := TriCorners(bm, i)
	return depthNormalOf(proof.XtriangleNormal(proof.XptOf(t[0]), proof.XptOf(t[1]), proof.XptOf(t[2])))
}

// depthCoordReach is the largest coordinate magnitude of the mesh's vertices,
// read from setDepthNormals when it ran and computed afresh otherwise.
func (bm *BoolMesh) depthCoordReach() float64 {
	if len(bm.depthNorms) == len(bm.Tris) {
		return bm.depthReach
	}
	return vertexReach(bm.Verts)
}

// vertexReach is the largest coordinate magnitude over verts, NaN when one is
// NaN.
func vertexReach(verts []r3.Vec) float64 {
	m := 0.0
	for _, v := range verts {
		m = max(m, math.Abs(v.X), math.Abs(v.Y), math.Abs(v.Z))
	}
	return m
}
