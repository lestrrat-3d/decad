package planar

import (
	"github.com/lestrrat-3d/decad/internal/proof"
)

// rayLadder is the fixed, deterministic sequence of parity-cast directions.
var rayLadder = [][3]int64{
	{1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {-1, 0, 0}, {0, -1, 0}, {0, 0, -1},
	{3, 5, 7}, {-7, 3, 5}, {5, -7, 3}, {3, 5, -7}, {-5, -3, 7}, {7, -5, -3},
	{-3, 7, -5}, {11, 13, 17}, {-13, 17, 11}, {17, -11, 13},
}

// castResult is a parity cast's outcome for one point.
type castResult int

const (
	castAmbiguous castResult = iota
	castInside
	castOutside
	castOnBoundary
)

// cast classifies p against the closed surface of solid by counting the
// facets a ray crosses strictly inside. A ray that meets a facet's boundary
// ahead of p moves to the next direction; p on a facet is reported as such.
func (k *planarKernel) cast(p proof.DyV3, solid *planarPrep) (castResult, error) {
	boxed := solid.triBox != nil
	var pb floatBox
	if boxed {
		pb = pointFloatBox(p)
	}
	for _, dir := range rayLadder {
		result, err := k.castAlong(p, pb, boxed, dir, solid)
		if err != nil {
			return castAmbiguous, err
		}
		if result != castAmbiguous {
			return result, nil
		}
	}
	return castAmbiguous, nil
}

// castAlong is one ray of cast, ambiguous when the ray meets a facet's
// boundary. When boxed, pb is p's outward float box and solid carries its
// float boxes: an axis ray then skips a triangle the float pre-test proves it
// passes beside (planar_prune.go). An axis ray also skips a facet whose exact
// plane is parallel to the ray and does not contain p.
func (k *planarKernel) castAlong(p proof.DyV3, pb floatBox, boxed bool, dir [3]int64, solid *planarPrep) (castResult, error) {
	d := proof.DyV3{proof.DyInt(dir[0]), proof.DyInt(dir[1]), proof.DyInt(dir[2])}
	axis := rayAxis(dir)
	crossings := 0
	for t, tri := range solid.s.Tris {
		if err := k.poll(); err != nil {
			return castAmbiguous, err
		}
		if boxed && axis >= 0 && solid.normal[t][axis].Sign() != 0 && floatApartOff(pb, solid.triBox[t], axis) {
			continue
		}
		ahead := 0
		aheadReady := false
		var rel [3]proof.DyV3
		if axis >= 0 && solid.normal[t][axis].Sign() == 0 {
			rel[0] = proof.DvSub(solid.s.Verts[tri[0]], p)
			ahead = proof.DvDot(solid.normal[t], rel[0]).Sign()
			aheadReady = true
			if ahead != 0 {
				continue
			}
		}
		for i := range 3 {
			if i == 0 && aheadReady {
				continue
			}
			rel[i] = proof.DvSub(solid.s.Verts[tri[i]], p)
		}
		var signs [3]int
		for i := range 3 {
			signs[i] = proof.DvDot(d, proof.DvCross(rel[i], rel[(i+1)%3])).Sign()
		}
		if hasSign(signs, 1) && hasSign(signs, -1) {
			continue
		}
		normal := solid.normal[t]
		along := proof.DvDot(normal, d).Sign()
		if !aheadReady {
			ahead = proof.DvDot(normal, rel[0]).Sign()
		}
		if along == 0 {
			if ahead != 0 {
				continue
			}
			if pointInFacetBoxed(solid, t, dyPoint(p), pb, boxed) {
				return castOnBoundary, nil
			}
			return castAmbiguous, nil
		}
		if ahead == 0 {
			return castOnBoundary, nil
		}
		if ahead != along {
			continue
		}
		if signs[0] == 0 || signs[1] == 0 || signs[2] == 0 {
			return castAmbiguous, nil
		}
		crossings++
	}
	if crossings%2 == 1 {
		return castInside, nil
	}
	return castOutside, nil
}

// rayAxis returns the coordinate axis a ray direction runs along, or -1 when
// it has more than one nonzero component.
func rayAxis(dir [3]int64) int {
	axis := -1
	for i, c := range dir {
		if c == 0 {
			continue
		}
		if axis >= 0 {
			return -1
		}
		axis = i
	}
	return axis
}

func hasSign(signs [3]int, sign int) bool {
	return signs[0] == sign || signs[1] == sign || signs[2] == sign
}

// inBox reports whether x/w lies in the closed box [lo, hi]: lo·w ≤ x ≤ hi·w
// on every axis for w > 0, the reverse for w < 0. A unit w compares the
// coordinates directly. A zero w makes no claim and reports true.
func (h hpoint) inBox(lo, hi [3]proof.Dyadic) bool {
	sign := h.w.Sign()
	if sign == 0 {
		return true
	}
	unit := proof.DyCmp(h.w, proof.DyInt(1)) == 0
	for axis := range 3 {
		low, high := lo[axis], hi[axis]
		if !unit {
			low, high = proof.DyMul(low, h.w), proof.DyMul(high, h.w)
			if sign < 0 {
				low, high = high, low
			}
		}
		if proof.DyCmp(h.x[axis], low) < 0 || proof.DyCmp(h.x[axis], high) > 0 {
			return false
		}
	}
	return true
}

// pointInFacet reports whether x lies in the closed triangle t. The closed
// triangle lies in its box, so a point outside the box, decided by
// comparisons, is outside the triangle. The box decides only for a triangle
// with a nonzero normal, the one the plane and edge tests below bound: on a
// zero normal every test reads zero and accepts any point.
//
// xb, when boxed, is x's outward float box (hpoint.floatBox), and p carries
// its float boxes: a float box apart from the triangle's proves x outside the
// exact box (planar_prune.go). Otherwise, for positive weight, the exact plane
// and edge tests imply containment in the triangle and its box. With negative
// weight the three edge expressions sum to a negative value for a nonzero
// normal, so they cannot all pass. At zero weight inBox already accepts.
func pointInFacet(p *planarPrep, t int, x hpoint) bool {
	return pointInFacetBoxed(p, t, x, floatBox{}, false)
}

func pointInFacetBoxed(p *planarPrep, t int, x hpoint, xb floatBox, boxed bool) bool {
	return pointInFacetBoxedEdges(p, t, x, xb, boxed, nil)
}

// pointInFacetBoxedEdges reports whether x lies in triangle t and, when
// edges is non-nil, returns each directed edge plane's exact sign. A zero sign
// identifies an edge containing x when the triangle has a nonzero normal.
func pointInFacetBoxedEdges(p *planarPrep, t int, x hpoint, xb floatBox, boxed bool, edges *[3]int) bool {
	if !proof.DvIsZero(p.normal[t]) && ((boxed && floatApart(xb, p.triBox[t])) || (!boxed && !x.inBox(p.triLo[t], p.triHi[t]))) {
		return false
	}
	tri := p.s.Tris[t]
	if p.planeSign(t, x) != 0 {
		return false
	}
	planes, boxes := p.sidePlanes(t)
	weightSign := x.w.Sign()
	for i := range 3 {
		sign, decided := 0, false
		if boxed && weightSign != 0 {
			// The float test uses x/w; the exact homogeneous sign includes w.
			sign, decided = floatDotSign(xb, p.vertBox[tri[i]], boxes[i])
			if weightSign < 0 {
				sign = -sign
			}
		}
		if !decided {
			sign = proof.DvDot(x.from(p.s.Verts[tri[i]]), planes[i]).Sign()
		}
		if edges != nil {
			edges[i] = sign
		}
		if sign < 0 {
			return false
		}
	}
	return true
}

// planeSign evaluates normal·(x.x - x.w*v0), reusing the exact normal·v0.
func (p *planarPrep) planeSign(t int, x hpoint) int {
	if p.offsets == nil {
		p.offsets = make([]facetOffset, len(p.s.Tris))
	}
	offset := &p.offsets[t]
	if !offset.set {
		offset.d = proof.DvDot(p.normal[t], p.s.Verts[p.s.Tris[t][0]])
		offset.set = true
	}
	return proof.DySubScalar(proof.DvDot(p.normal[t], x.x), proof.DyMul(x.w, offset.d)).Sign()
}

// shellsInside casts one witness vertex per shell of from against to. A shell
// whose every vertex lies on to's boundary needs no cast: it meets the
// contact set, which localSeparation covers. decided is false when every
// witness of some shell was ambiguous on every ray.
func (k *planarKernel) shellsInside(from, to *planarPrep) (bool, bool, error) {
	done := make([]bool, from.shells)
	pending := make([]bool, from.shells)
	for v, point := range from.s.Verts {
		shell := from.shellOf[v]
		if done[shell] {
			continue
		}
		result, err := k.cast(point, to)
		if err != nil {
			return false, false, err
		}
		switch result {
		case castInside:
			return true, true, nil
		case castOutside:
			done[shell], pending[shell] = true, false
		case castAmbiguous:
			pending[shell] = true
		case castOnBoundary:
		}
	}
	for _, p := range pending {
		if p {
			return false, false, nil
		}
	}
	return false, true, nil
}
