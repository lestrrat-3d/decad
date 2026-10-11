package meshbool

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proof"
)

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
