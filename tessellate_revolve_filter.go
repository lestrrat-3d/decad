package decad

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// This file is the float pre-test in front of the revolve facet-contact
// audit's exact predicates (tessellate_revolve_proof.go). The audit reads
// every sign and every separating-axis gap over Dyadic arithmetic, and most of
// those readings are far from their thresholds. Each one is first enclosed
// here in a float interval rounded outward at every step, so the interval
// holds the exact value of the same expression over the same stored floats.
// Where the interval settles the reading, the exact answer is the one the
// interval states; where it does not, the exact arithmetic runs unchanged. The
// pre-test only ever skips work: it never decides a reading the exact
// arithmetic would decide differently, so the audit's verdict, and the
// refusal it names, are the same with or without it.

// revIv is a closed float interval holding one exact real.
type revIv struct{ lo, hi float64 }

// revIvVec is a vector of intervals, one per coordinate.
type revIvVec [3]revIv

func revDown(x float64) float64 { return math.Nextafter(x, math.Inf(-1)) }
func revUp(x float64) float64   { return math.Nextafter(x, math.Inf(1)) }

// revIvDiff encloses a − b for two exact floats.
func revIvDiff(a, b float64) revIv {
	d := a - b
	return revIv{revDown(d), revUp(d)}
}

// revIvPointDiff encloses p − q componentwise for two stored points.
func revIvPointDiff(p, q r3.Vec) revIvVec {
	return revIvVec{revIvDiff(p.X, q.X), revIvDiff(p.Y, q.Y), revIvDiff(p.Z, q.Z)}
}

// revIvPoint is a stored point as an interval of zero width.
func revIvPoint(p r3.Vec) revIvVec {
	return revIvVec{{p.X, p.X}, {p.Y, p.Y}, {p.Z, p.Z}}
}

func revIvAdd(a, b revIv) revIv { return revIv{revDown(a.lo + b.lo), revUp(a.hi + b.hi)} }
func revIvSub(a, b revIv) revIv { return revIv{revDown(a.lo - b.hi), revUp(a.hi - b.lo)} }

// revIvMul encloses a × b. Each product is converted explicitly so the
// compiler cannot fuse it into a later addition; a NaN end leaves the interval
// NaN, which settles nothing below.
func revIvMul(a, b revIv) revIv {
	p0, p1 := float64(a.lo*b.lo), float64(a.lo*b.hi)
	p2, p3 := float64(a.hi*b.lo), float64(a.hi*b.hi)
	return revIv{revDown(math.Min(math.Min(p0, p1), math.Min(p2, p3))), revUp(math.Max(math.Max(p0, p1), math.Max(p2, p3)))}
}

func revIvDot(a, b revIvVec) revIv {
	return revIvAdd(revIvAdd(revIvMul(a[0], b[0]), revIvMul(a[1], b[1])), revIvMul(a[2], b[2]))
}

func revIvSubVec(a, b revIvVec) revIvVec {
	return revIvVec{revIvSub(a[0], b[0]), revIvSub(a[1], b[1]), revIvSub(a[2], b[2])}
}

func revIvCross(a, b revIvVec) revIvVec {
	return revIvVec{
		revIvSub(revIvMul(a[1], b[2]), revIvMul(a[2], b[1])),
		revIvSub(revIvMul(a[2], b[0]), revIvMul(a[0], b[2])),
		revIvSub(revIvMul(a[0], b[1]), revIvMul(a[1], b[0])),
	}
}

// finite reports whether both ends are finite numbers.
func (x revIv) finite() bool {
	return !proofbound.IsNonFinite(x.lo) && !proofbound.IsNonFinite(x.hi)
}

// revIvSide settles revolveSepAxis.sideOf's exact reading from an interval h
// holding the exact dot product, against the same allowance the exact reading
// compares |h| with. It reports the side and true when the interval decides
// the reading — a side of 0 meaning the exact reading refuses — and false when
// only the exact arithmetic can.
func revIvSide(h revIv, allow float64) (int, bool) {
	if !h.finite() || allow < 0 || proofbound.IsNonFinite(allow) {
		return 0, false
	}
	switch {
	case h.lo > allow:
		return 1, true
	case h.hi < -allow:
		return -1, true
	case h.lo >= -allow && h.hi <= allow:
		return 0, true
	}
	return 0, false
}

// revolveSeparatedFloat is revolveSeparated's pre-test: it walks the same
// seventeen candidate axes, enclosing each in floats, and reports true when one
// axis's enclosed gap clears the same cheap length bound the exact walk tries
// first. The exact gap is at least the enclosed one, so the exact walk would
// accept that axis too. False means only that the floats did not settle it.
func revolveSeparatedFloat(a, b revolveAuditTri, delta float64) bool {
	margin := proofbound.ProductUpper(2, delta)
	if proofbound.IsNonFinite(margin) {
		return false
	}
	ea := [3]revIvVec{a.fu, a.fv, a.fw}
	eb := [3]revIvVec{b.fu, b.fv, b.fw}
	la := [3]float64{a.lu, a.lv, a.lw}
	lb := [3]float64{b.lu, b.lv, b.lw}
	lnA := proofbound.ProductUpper(a.lu, a.lv)
	lnB := proofbound.ProductUpper(b.lu, b.lv)
	for gi := range 17 {
		var g revIvVec
		var bound float64
		switch {
		case gi == 0:
			g, bound = a.fn, lnA
		case gi == 1:
			g, bound = b.fn, lnB
		case gi < 11:
			x, y := (gi-2)/3, (gi-2)%3
			g, bound = revIvCross(ea[x], eb[y]), proofbound.ProductUpper(la[x], lb[y])
		case gi < 14:
			x := gi - 11
			g, bound = revIvCross(a.fn, ea[x]), proofbound.ProductUpper(lnA, la[x])
		default:
			y := gi - 14
			g, bound = revIvCross(b.fn, eb[y]), proofbound.ProductUpper(lnB, lb[y])
		}
		cheap := proofbound.ProductUpper(margin, bound)
		if proofbound.IsNonFinite(cheap) {
			continue
		}
		aLo, aHi := revIvProject(a.fp, g)
		bLo, bHi := revIvProject(b.fp, g)
		gap := math.Max(revDown(bLo-aHi), revDown(aLo-bHi))
		if !proofbound.IsNonFinite(gap) && gap > cheap {
			return true
		}
	}
	return false
}

// revIvProject is a lower bound on the least and an upper bound on the
// greatest exact projection of a triangle's corners onto g.
func revIvProject(p [3]r3.Vec, g revIvVec) (float64, float64) {
	lo, hi := math.Inf(1), math.Inf(-1)
	for _, q := range p {
		d := revIvDot(revIvPoint(q), g)
		if !d.finite() {
			return math.NaN(), math.NaN()
		}
		lo = math.Min(lo, d.lo)
		hi = math.Max(hi, d.hi)
	}
	return lo, hi
}
