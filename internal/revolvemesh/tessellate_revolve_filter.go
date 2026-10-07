package revolvemesh

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

// RevIv is a closed float interval holding one exact real.
type RevIv struct{ Lo, Hi float64 }

// RevIvVec is a vector of intervals, one per coordinate.
type RevIvVec [3]RevIv

func RevDown(x float64) float64 { return math.Nextafter(x, math.Inf(-1)) }
func RevUp(x float64) float64   { return math.Nextafter(x, math.Inf(1)) }

// RevIvDiff encloses a − b for two exact floats.
func RevIvDiff(a, b float64) RevIv {
	d := a - b
	return RevIv{RevDown(d), RevUp(d)}
}

// RevIvPointDiff encloses p − q componentwise for two stored points.
func RevIvPointDiff(p, q r3.Vec) RevIvVec {
	return RevIvVec{RevIvDiff(p.X, q.X), RevIvDiff(p.Y, q.Y), RevIvDiff(p.Z, q.Z)}
}

// RevIvPoint is a stored point as an interval of zero width.
func RevIvPoint(p r3.Vec) RevIvVec {
	return RevIvVec{{p.X, p.X}, {p.Y, p.Y}, {p.Z, p.Z}}
}

func RevIvAdd(a, b RevIv) RevIv { return RevIv{RevDown(a.Lo + b.Lo), RevUp(a.Hi + b.Hi)} }
func RevIvSub(a, b RevIv) RevIv { return RevIv{RevDown(a.Lo - b.Hi), RevUp(a.Hi - b.Lo)} }

// RevIvMul encloses a × b. Each product is converted explicitly so the
// compiler cannot fuse it into a later addition; a NaN end leaves the interval
// NaN, which settles nothing below.
func RevIvMul(a, b RevIv) RevIv {
	p0, p1 := float64(a.Lo*b.Lo), float64(a.Lo*b.Hi)
	p2, p3 := float64(a.Hi*b.Lo), float64(a.Hi*b.Hi)
	return RevIv{RevDown(math.Min(math.Min(p0, p1), math.Min(p2, p3))), RevUp(math.Max(math.Max(p0, p1), math.Max(p2, p3)))}
}

func RevIvDot(a, b RevIvVec) RevIv {
	return RevIvAdd(RevIvAdd(RevIvMul(a[0], b[0]), RevIvMul(a[1], b[1])), RevIvMul(a[2], b[2]))
}

func RevIvSubVec(a, b RevIvVec) RevIvVec {
	return RevIvVec{RevIvSub(a[0], b[0]), RevIvSub(a[1], b[1]), RevIvSub(a[2], b[2])}
}

func RevIvCross(a, b RevIvVec) RevIvVec {
	return RevIvVec{
		RevIvSub(RevIvMul(a[1], b[2]), RevIvMul(a[2], b[1])),
		RevIvSub(RevIvMul(a[2], b[0]), RevIvMul(a[0], b[2])),
		RevIvSub(RevIvMul(a[0], b[1]), RevIvMul(a[1], b[0])),
	}
}

// finite reports whether both ends are finite numbers.
func (x RevIv) Finite() bool {
	return !proofbound.IsNonFinite(x.Lo) && !proofbound.IsNonFinite(x.Hi)
}

// RevIvSide settles RevolveSepAxis.sideOf's exact reading from an interval h
// holding the exact dot product, against the same allowance the exact reading
// compares |h| with. It reports the side and true when the interval decides
// the reading — a side of 0 meaning the exact reading refuses — and false when
// only the exact arithmetic can.
func RevIvSide(h RevIv, allow float64) (int, bool) {
	if !h.Finite() || allow < 0 || proofbound.IsNonFinite(allow) {
		return 0, false
	}
	switch {
	case h.Lo > allow:
		return 1, true
	case h.Hi < -allow:
		return -1, true
	case h.Lo >= -allow && h.Hi <= allow:
		return 0, true
	}
	return 0, false
}

// RevolveSeparatedFloat is RevolveSeparated's pre-test: it walks the same
// seventeen candidate axes, enclosing each in floats, and reports true when one
// axis's enclosed gap clears the same cheap length bound the exact walk tries
// first. The exact gap is at least the enclosed one, so the exact walk would
// accept that axis too. False means only that the floats did not settle it.
func RevolveSeparatedFloat(a, b RevolveAuditTri, delta float64) bool {
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
			g, bound = RevIvCross(ea[x], eb[y]), proofbound.ProductUpper(la[x], lb[y])
		case gi < 14:
			x := gi - 11
			g, bound = RevIvCross(a.Fn, ea[x]), proofbound.ProductUpper(lnA, la[x])
		default:
			y := gi - 14
			g, bound = RevIvCross(b.Fn, eb[y]), proofbound.ProductUpper(lnB, lb[y])
		}
		cheap := proofbound.ProductUpper(margin, bound)
		if proofbound.IsNonFinite(cheap) {
			continue
		}
		aLo, aHi := RevIvProject(a.Fp, g)
		bLo, bHi := RevIvProject(b.Fp, g)
		gap := math.Max(RevDown(bLo-aHi), RevDown(aLo-bHi))
		if !proofbound.IsNonFinite(gap) && gap > cheap {
			return true
		}
	}
	return false
}

// RevIvProject is a lower bound on the least and an upper bound on the
// greatest exact projection of a triangle's corners onto g.
func RevIvProject(p [3]r3.Vec, g RevIvVec) (float64, float64) {
	lo, hi := math.Inf(1), math.Inf(-1)
	for _, q := range p {
		d := RevIvDot(RevIvPoint(q), g)
		if !d.Finite() {
			return math.NaN(), math.NaN()
		}
		lo = math.Min(lo, d.Lo)
		hi = math.Max(hi, d.Hi)
	}
	return lo, hi
}
