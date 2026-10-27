package revolvemesh

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// This file keeps the float pre-test's straightforward form: every product of
// two intervals takes all four end products and math.Min/math.Max, every
// outward step calls proof.NextUp/NextDown, and every projection is rebuilt
// per pair. The differential tests compare the production pre-test with it
// bit for bit.

func legacyRevIvAdd(a, b RevIv) RevIv {
	return RevIv{proof.NextDown(a.Lo + b.Lo), proof.NextUp(a.Hi + b.Hi)}
}

func legacyRevIvSub(a, b RevIv) RevIv {
	return RevIv{proof.NextDown(a.Lo - b.Hi), proof.NextUp(a.Hi - b.Lo)}
}

func legacyRevIvMul(a, b RevIv) RevIv {
	p0, p1 := float64(a.Lo*b.Lo), float64(a.Lo*b.Hi)
	p2, p3 := float64(a.Hi*b.Lo), float64(a.Hi*b.Hi)
	return RevIv{proof.NextDown(math.Min(math.Min(p0, p1), math.Min(p2, p3))),
		proof.NextUp(math.Max(math.Max(p0, p1), math.Max(p2, p3)))}
}

func legacyRevIvDot(a, b RevIvVec) RevIv {
	return legacyRevIvAdd(legacyRevIvAdd(legacyRevIvMul(a[0], b[0]), legacyRevIvMul(a[1], b[1])), legacyRevIvMul(a[2], b[2]))
}

func legacyRevIvCross(a, b RevIvVec) RevIvVec {
	return RevIvVec{
		legacyRevIvSub(legacyRevIvMul(a[1], b[2]), legacyRevIvMul(a[2], b[1])),
		legacyRevIvSub(legacyRevIvMul(a[2], b[0]), legacyRevIvMul(a[0], b[2])),
		legacyRevIvSub(legacyRevIvMul(a[0], b[1]), legacyRevIvMul(a[1], b[0])),
	}
}

func legacyRevIvProject(p [3]r3.Vec, g RevIvVec) (float64, float64) {
	lo, hi := math.Inf(1), math.Inf(-1)
	for _, q := range p {
		d := legacyRevIvDot(RevIvPoint(q), g)
		if !d.Finite() {
			return math.NaN(), math.NaN()
		}
		lo = math.Min(lo, d.Lo)
		hi = math.Max(hi, d.Hi)
	}
	return lo, hi
}

func legacyRevolveSeparatedFloat(a, b RevolveAuditTri, delta float64) bool {
	margin := proofbound.ProductUpper(2, delta)
	if proofbound.IsNonFinite(margin) {
		return false
	}
	ea := [3]RevIvVec{a.Fu, a.Fv, a.Fw}
	eb := [3]RevIvVec{b.Fu, b.Fv, b.Fw}
	la := [3]float64{a.Lu, a.Lv, a.Lw}
	lb := [3]float64{b.Lu, b.Lv, b.Lw}
	lnA := proofbound.ProductUpper(a.Lu, a.Lv)
	lnB := proofbound.ProductUpper(b.Lu, b.Lv)
	for gi := range 17 {
		var g RevIvVec
		var bound float64
		switch {
		case gi == 0:
			g, bound = a.Fn, lnA
		case gi == 1:
			g, bound = b.Fn, lnB
		case gi < 11:
			x, y := (gi-2)/3, (gi-2)%3
			g, bound = legacyRevIvCross(ea[x], eb[y]), proofbound.ProductUpper(la[x], lb[y])
		case gi < 14:
			x := gi - 11
			g, bound = legacyRevIvCross(a.Fn, ea[x]), proofbound.ProductUpper(lnA, la[x])
		default:
			y := gi - 14
			g, bound = legacyRevIvCross(b.Fn, eb[y]), proofbound.ProductUpper(lnB, lb[y])
		}
		cheap := proofbound.ProductUpper(margin, bound)
		if proofbound.IsNonFinite(cheap) {
			continue
		}
		aLo, aHi := legacyRevIvProject(a.Fp, g)
		bLo, bHi := legacyRevIvProject(b.Fp, g)
		gap := math.Max(proof.NextDown(bLo-aHi), proof.NextDown(aLo-bHi))
		if !proofbound.IsNonFinite(gap) && gap > cheap {
			return true
		}
	}
	return false
}
