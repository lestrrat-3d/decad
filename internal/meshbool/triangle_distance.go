package meshbool

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// TriTriDistanceExceeds reports whether the closed float triangles ta and tb
// provably stand farther apart than slack. It answers true only when both of
// these hold, for every input, including NaN, infinite and out-of-range ones:
//
//   - The float reading is far: TriTriDistance(ta, tb) > slack.
//   - A proof shows that the exact distance between the two triangles exceeds
//     slack: a gap along an axis, float error charged, or
//     triDistanceExactAbove's exact distance. The axes are triGapAbove's
//     seventeen and the offset between the float closest points of the
//     nearest edge pair, which triTriDistanceEdgeOffset returns with the
//     float reading.
//
// The float reading alone decides only "near". Its candidates carry an
// absolute float error of about eps*m (m the largest coordinate magnitude),
// far more for two edges near parallel, where SegSegDist3's a*c - b*b
// cancels, and for a sliver facet, whose float normal is tilted. The
// relative nudge TriTriDistance applies does not cover that error, so the
// float reading can exceed slack while the exact distance does not.
//
// The answer never turns from false to true as slack grows. The float
// reading, the axis thresholds and the exact comparison are each monotone in
// slack, so a pair far at one slack is far at every smaller one.
//
// The float reading skips the distance candidates that a separating axis
// along a facet normal proves exceed slack, and answers from the rest exactly
// as TriTriDistance's minimum would. Such an axis is one of triGapAbove's, so
// it also proves the exact distance. A pair that neither normal separates
// takes TriTriDistance in full, and then needs one of the other sixteen axes
// or the exact distance. Any direction n separates the triangles by at least
// the gap of their projections over |n|, so an axis needs no exactness of its
// own. The nearest-edge offset is the separating direction of a pair whose
// nearest points lie on an edge of each, and a facet normal is that of a pair
// whose nearest point lies inside a facet. So among pairs inside the axes'
// range, the exact distance runs for one whose distance lies within the axis
// threshold's margin of slack, or whose float closest points are poorly
// placed (two edges near parallel). Corners past 2^200 in magnitude, and a
// negative slack, skip the axes and go straight to triDistanceExactAbove.
//
// TriTriDistance(ta, tb) > slack holds exactly when every one of its fifteen
// candidates d satisfies d > 0 and fl(d*(1-1e-12)) > slack: the minimum is
// one of them (NaN wins the fold, and x -> fl(x*c) is monotone for x > 0),
// and a candidate failing the test makes the result 0, or a value no larger
// than its own, that is not above slack.
//
// The skip applies only to finite slack >= 0 and corners whose coordinates
// are finite with magnitude at most m <= 2^200, so no step of TriTriDistance
// overflows or produces a NaN. Let n be the facet normal PointTriDistance
// projects along (triNormal). The interval of the corners' projections onto
// n is separated by a gap G from the other facet's: then every point of one
// facet lies at least G/|n| from every point of the other, and from the
// other's corner along n. axisGapExceeds proves G/|n| >= thr with
// thr = slack + (slack+m)*2^-26 + 2^-199, charging each projection's float
// error at 2^-49*|n|_1*m (above 4*gamma3*|n|_1*m) and the remaining few
// roundings to its 2^-40 margins.
//
// Every candidate is the float length of the difference of two computed
// points. A segment candidate's points lie within 9*eps*m of points of the
// two closed segments (s and t are clamped into [0, 1]), and a foot
// candidate's length is within 18*eps of the point's distance along n to the
// facet's first corner, less 16*eps*m (eps = 2^-53, nn >= 2^-300 keeps the
// underflow terms below 2^-900). So each skipped candidate is at least
// D*(1-2^-48) - 2^-47*m with D >= thr, which exceeds slack*(1+2^-27) + 2^-200,
// and its fl(d*(1-1e-12)) exceeds slack. An axis along A's normal bounds the
// edge candidates and B's corners' foot candidates on A; B's normal bounds
// the edge candidates and A's corners' foot candidates on B. The foot
// candidates no axis bounds are computed by PointTriDistance's own foot code
// (pointTriFoot): an edge fallback it takes is bounded by either axis.
// Without a normal axis the float reading is TriTriDistance's own.
func TriTriDistanceExceeds(ta, tb [3]r3.Vec, slack float64) bool {
	m, ok := triPairScale(ta, tb)
	if !ok || !(slack >= 0 && slack <= math.MaxFloat64) {
		return TriTriDistance(ta, tb) > slack && triDistanceExactAbove(ta, tb, slack)
	}
	thr := slack + (slack+m)*0x1p-26 + 0x1p-199
	na, nb := triNormal(ta), triNormal(tb)
	clearA := axisGapExceeds(na, ta, tb, m, thr)
	clearB := axisGapExceeds(nb, ta, tb, m, thr)
	if !clearA && !clearB {
		d, offset := triTriDistanceEdgeOffset(ta, tb)
		return d > slack && (axisGapExceeds(offset, ta, tb, m, thr) ||
			edgeAxesGapExceeds(ta, tb, na, nb, m, thr) || triDistanceExactAbove(ta, tb, slack))
	}
	for i := range 3 {
		if !clearB {
			if d, foot := pointTriFoot(ta[i], tb, nb); foot && !distanceCandidateExceeds(d, slack) {
				return false
			}
		}
		if !clearA {
			if d, foot := pointTriFoot(tb[i], ta, na); foot && !distanceCandidateExceeds(d, slack) {
				return false
			}
		}
	}
	return true
}

// edgeAxesGapExceeds reports whether one of the fifteen axes triGapAbove
// tries after the facet normals na and nb proves a gap over thr: each edge
// crossed with its own facet's normal, and the nine edge-edge cross products.
// m and thr are as axisGapExceeds takes them.
func edgeAxesGapExceeds(ta, tb [3]r3.Vec, na, nb r3.Vec, m, thr float64) bool {
	for i := range 3 {
		ea, eb := ta[(i+1)%3].Sub(ta[i]), tb[(i+1)%3].Sub(tb[i])
		if axisGapExceeds(ea.Cross(na), ta, tb, m, thr) || axisGapExceeds(eb.Cross(nb), ta, tb, m, thr) {
			return true
		}
		for j := range 3 {
			ej := tb[(j+1)%3].Sub(tb[j])
			if axisGapExceeds(ea.Cross(ej), ta, tb, m, thr) {
				return true
			}
		}
	}
	return false
}

// triDistanceExactAbove reports whether the exact distance between the closed
// float triangles ta and tb exceeds slack. A non-finite corner, a NaN slack
// and an infinite one answer false. A slack below zero is below every
// distance and answers true. Otherwise disjointTriDistance2Above decides on
// the corners' exact rationals: at slack zero it answers whether the
// triangles are disjoint. It refuses a degenerate triangle.
func triDistanceExactAbove(ta, tb [3]r3.Vec, slack float64) bool {
	for _, t := range [2]*[3]r3.Vec{&ta, &tb} {
		for _, v := range t {
			if proofbound.IsNonFinite(v.X) || proofbound.IsNonFinite(v.Y) || proofbound.IsNonFinite(v.Z) {
				return false
			}
		}
	}
	if slack < 0 {
		return true
	}
	if !(slack <= math.MaxFloat64) {
		return false
	}
	bound2 := proof.FloatRat(slack)
	bound2.Mul(bound2, bound2)
	a := [3]proof.Xpt{proof.XptOf(ta[0]), proof.XptOf(ta[1]), proof.XptOf(ta[2])}
	b := [3]proof.Xpt{proof.XptOf(tb[0]), proof.XptOf(tb[1]), proof.XptOf(tb[2])}
	return disjointTriDistance2Above(a, b, bound2)
}

// distanceCandidateExceeds reports whether the TriTriDistance candidate d,
// were it the minimum, would make TriTriDistance exceed slack.
func distanceCandidateExceeds(d, slack float64) bool {
	return d > 0 && d*(1-1e-12) > slack
}

// triPairScale is the largest coordinate magnitude of the six corners, and
// whether every coordinate is finite with magnitude at most 2^200.
func triPairScale(ta, tb [3]r3.Vec) (float64, bool) {
	m := 0.0
	for _, t := range [2]*[3]r3.Vec{&ta, &tb} {
		for _, v := range t {
			for _, c := range [3]float64{v.X, v.Y, v.Z} {
				if c < 0 {
					c = -c
				}
				if !(c <= 0x1p200) {
					return 0, false
				}
				if c > m {
					m = c
				}
			}
		}
	}
	return m, true
}

// axisGapExceeds reports whether the corners of ta and tb project onto n in
// intervals whose gap, divided by |n|, provably exceeds thr. m bounds every
// coordinate magnitude, as triPairScale returns it. It refuses an axis with
// n·n below 2^-300.
func axisGapExceeds(n r3.Vec, ta, tb [3]r3.Vec, m, thr float64) bool {
	if !(n.Dot(n) >= 0x1p-300) {
		return false
	}
	loA, hiA := n.Dot(ta[0]), n.Dot(ta[0])
	loB, hiB := n.Dot(tb[0]), n.Dot(tb[0])
	for i := 1; i < 3; i++ {
		if p := n.Dot(ta[i]); p < loA {
			loA = p
		} else if p > hiA {
			hiA = p
		}
		if p := n.Dot(tb[i]); p < loB {
			loB = p
		} else if p > hiB {
			hiB = p
		}
	}
	gap := loB - hiA
	if g := loA - hiB; g > gap {
		gap = g
	}
	l1 := absFloat(n.X) + absFloat(n.Y) + absFloat(n.Z)
	return gap-l1*m*0x1p-49 > n.Len()*thr*(1+0x1p-40)
}

// absFloat is |x| by comparison: the race build instruments math.Abs.
func absFloat(x float64) float64 {
	if x < 0 {
		return -x
	}
	return x
}

// DisjointTriDistanceAbove proves that two closed, nondegenerate triangles
// have Euclidean distance strictly greater than bound. It refuses a pair that
// touches or intersects, an invalid bound, or a degenerate triangle. The
// exact separating-axis pass proves disjointness before the distance pass;
// the vertex-face and edge-edge candidates are complete only for disjoint
// triangles. All comparisons use rational coordinates and the exact dyadic
// value of bound.
func DisjointTriDistanceAbove(a, b [3]proof.Xpt, bound float64) bool {
	if bound <= 0 || math.IsNaN(bound) || math.IsInf(bound, 0) {
		return false
	}
	bound2 := proof.FloatRat(bound)
	bound2.Mul(bound2, bound2)
	return disjointTriDistance2Above(a, b, bound2)
}

// disjointTriDistance2Above is DisjointTriDistanceAbove against the exact
// squared bound bound2 >= 0. At bound2 zero it proves the closed triangles
// disjoint: an axis gap is then above the bound, and a pair no axis separates
// is refused.
func disjointTriDistance2Above(a, b [3]proof.Xpt, bound2 *big.Rat) bool {
	var ae, be [3]proof.Xpt
	for i := range 3 {
		ae[i] = proof.Xsub(a[(i+1)%3], a[i])
		be[i] = proof.Xsub(b[(i+1)%3], b[i])
	}
	na := proof.Xcross(ae[0], ae[1])
	nb := proof.Xcross(be[0], be[1])
	if proof.XdotSign(na, na) == 0 || proof.XdotSign(nb, nb) == 0 {
		return false
	}
	separated := false
	for _, axis := range [2]proof.Xpt{na, nb} {
		gap, above := exactTriangleAxisGap(a, b, axis, bound2)
		if above {
			return true
		}
		separated = separated || gap
	}
	for i := range 3 {
		for j := range 3 {
			gap, above := exactTriangleAxisGap(a, b, proof.Xcross(ae[i], be[j]), bound2)
			if above {
				return true
			}
			separated = separated || gap
		}
		for _, axis := range [2]proof.Xpt{proof.Xcross(ae[i], na), proof.Xcross(be[i], nb)} {
			gap, above := exactTriangleAxisGap(a, b, axis, bound2)
			if above {
				return true
			}
			separated = separated || gap
		}
	}
	if !separated {
		return false
	}

	for i := range 3 {
		if d2, interior := exactPointFaceDistance2(a[i], b, nb); interior && d2.Cmp(bound2) <= 0 {
			return false
		}
		if d2, interior := exactPointFaceDistance2(b[i], a, na); interior && d2.Cmp(bound2) <= 0 {
			return false
		}
		for j := range 3 {
			if exactPointSegmentDistance2(a[i], b[j], b[(j+1)%3]).Cmp(bound2) <= 0 ||
				exactPointSegmentDistance2(b[i], a[j], a[(j+1)%3]).Cmp(bound2) <= 0 {
				return false
			}
		}
	}
	for i := range 3 {
		for j := range 3 {
			if d2, interior := exactEdgeEdgeDistance2(a[i], ae[i], b[j], be[j]); interior && d2.Cmp(bound2) <= 0 {
				return false
			}
		}
	}
	return true
}

// exactTriangleAxisGap returns whether the projected intervals have a strict
// gap and whether that gap alone proves distance greater than bound.
func exactTriangleAxisGap(a, b [3]proof.Xpt, axis proof.Xpt, bound2 *big.Rat) (bool, bool) {
	norm2 := proof.XdotRat(axis, axis)
	if norm2.Sign() == 0 {
		return false, false
	}
	minA, maxA := proof.XdotRat(axis, a[0]), proof.XdotRat(axis, a[0])
	minB, maxB := proof.XdotRat(axis, b[0]), proof.XdotRat(axis, b[0])
	for i := 1; i < 3; i++ {
		p := proof.XdotRat(axis, a[i])
		if p.Cmp(minA) < 0 {
			minA = p
		}
		if p.Cmp(maxA) > 0 {
			maxA = p
		}
		p = proof.XdotRat(axis, b[i])
		if p.Cmp(minB) < 0 {
			minB = p
		}
		if p.Cmp(maxB) > 0 {
			maxB = p
		}
	}
	gap := new(big.Rat).Sub(minA, maxB)
	if reverse := new(big.Rat).Sub(minB, maxA); reverse.Cmp(gap) > 0 {
		gap = reverse
	}
	if gap.Sign() <= 0 {
		return false, false
	}
	gap2 := new(big.Rat).Mul(gap, gap)
	return true, gap2.Cmp(new(big.Rat).Mul(bound2, norm2)) > 0
}

// exactPointFaceDistance2 returns the squared normal distance only when the
// point's orthogonal projection lies in the closed triangle.
func exactPointFaceDistance2(p proof.Xpt, tri [3]proof.Xpt, normal proof.Xpt) (*big.Rat, bool) {
	u := proof.Xsub(tri[1], tri[0])
	v := proof.Xsub(tri[2], tri[0])
	w := proof.Xsub(p, tri[0])
	uu, uv, vv := proof.XdotRat(u, u), proof.XdotRat(u, v), proof.XdotRat(v, v)
	wu, wv := proof.XdotRat(w, u), proof.XdotRat(w, v)
	det := new(big.Rat).Sub(new(big.Rat).Mul(uu, vv), new(big.Rat).Mul(uv, uv))
	if det.Sign() <= 0 {
		return nil, false
	}
	s := new(big.Rat).Sub(new(big.Rat).Mul(wu, vv), new(big.Rat).Mul(wv, uv))
	t := new(big.Rat).Sub(new(big.Rat).Mul(wv, uu), new(big.Rat).Mul(wu, uv))
	if s.Sign() < 0 || t.Sign() < 0 || new(big.Rat).Add(s, t).Cmp(det) > 0 {
		return nil, false
	}
	wn := proof.XdotRat(w, normal)
	return new(big.Rat).Quo(new(big.Rat).Mul(wn, wn), proof.XdotRat(normal, normal)), true
}

// exactPointSegmentDistance2 covers both endpoints and the open segment.
func exactPointSegmentDistance2(p, a, b proof.Xpt) *big.Rat {
	u := proof.Xsub(b, a)
	w := proof.Xsub(p, a)
	uu, wu := proof.XdotRat(u, u), proof.XdotRat(w, u)
	if uu.Sign() == 0 || wu.Sign() <= 0 {
		return proof.XdotRat(w, w)
	}
	if wu.Cmp(uu) >= 0 {
		v := proof.Xsub(p, b)
		return proof.XdotRat(v, v)
	}
	return new(big.Rat).Sub(proof.XdotRat(w, w),
		new(big.Rat).Quo(new(big.Rat).Mul(wu, wu), uu))
}

// exactEdgeEdgeDistance2 returns the squared distance only when both closest
// points of the supporting lines lie in their closed segments. Endpoint
// cases, including parallel segments, are checked by point-segment distance.
func exactEdgeEdgeDistance2(a proof.Xpt, u proof.Xpt, b proof.Xpt, v proof.Xpt) (*big.Rat, bool) {
	w := proof.Xsub(a, b)
	uu, uv, vv := proof.XdotRat(u, u), proof.XdotRat(u, v), proof.XdotRat(v, v)
	wu, wv := proof.XdotRat(w, u), proof.XdotRat(w, v)
	det := new(big.Rat).Sub(new(big.Rat).Mul(uu, vv), new(big.Rat).Mul(uv, uv))
	if det.Sign() <= 0 {
		return nil, false
	}
	s := new(big.Rat).Sub(new(big.Rat).Mul(uv, wv), new(big.Rat).Mul(vv, wu))
	t := new(big.Rat).Sub(new(big.Rat).Mul(uu, wv), new(big.Rat).Mul(uv, wu))
	if s.Sign() < 0 || s.Cmp(det) > 0 || t.Sign() < 0 || t.Cmp(det) > 0 {
		return nil, false
	}
	s.Quo(s, det)
	t.Quo(t, det)
	d2 := proof.XdotRat(w, w)
	d2.Add(d2, new(big.Rat).Mul(wu, s))
	d2.Sub(d2, new(big.Rat).Mul(wv, t))
	return d2, true
}
