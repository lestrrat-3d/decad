package revolvemesh

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// This file is the float pre-test in front of the revolve facet-contact
// audit's exact predicates (revolve_contact_audit.go). The audit reads
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

// RevDown is proof.NextDown(x), bit for bit, and RevUp is proof.NextUp(x).
// Each steps an x of magnitude at least 2^-969 away from zero itself, small
// enough to inline: that covers almost every end the pre-test rounds. Every
// other x, NaN included, calls proof.
//
// One product and one sum give the neighbour with no tie to test. Write
// x = m·2^E with m in [1, 2), ulp = 2^(E−52) and c = (1 + 2^-52)·2^-53. The
// product x·c is normal because x ≥ 2^-969, and it rounds to r = q·ulp with q
// strictly between 1/2 and 3/2:
//
//   - at m = 1 the product (1 + 2^-52)·2^(E−53) is exact, so q = (1 + 2^-52)/2;
//   - at m ≥ 1 + 2^-52, q ≥ (1 + 2^-52)²(1 − 2^-53)/2, which exceeds 1/2;
//   - at m ≤ 2 − 2^-52, q ≤ (2 − 2^-52)(1 + 2^-52)(1 + 2^-53)/2, below 3/2.
//
// So x + r lies strictly between x + ulp/2 and x + 3ulp/2 and rounds to
// x + ulp, also at the top of a binade, where the spacing above x + ulp
// doubles. At x = MaxFloat64 the sum passes the MaxFloat64 + ulp/2 overflow
// threshold and rounds to +Inf, which is NextUp's answer there, and an
// infinite x returns itself, as NextUp's does. A negative x mirrors the
// argument with every sign flipped. The float64() conversion forbids fusing
// the product into the sum.
func RevDown(x float64) float64 {
	if x <= -0x1p-969 {
		return x + float64(x*0x1.0000000000001p-53)
	}
	return proof.NextDown(x)
}

func RevUp(x float64) float64 {
	if x >= 0x1p-969 {
		return x + float64(x*0x1.0000000000001p-53)
	}
	return proof.NextUp(x)
}

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
//
// The ends are math.Min and math.Max of the four products, taken with the
// builtin min and max whenever no product is NaN. On non-NaN operands the two
// agree bit for bit: both order −0 below +0 and treat infinities as ordinary
// values. They part only on a NaN, where math.Min(−Inf, NaN) is −Inf and the
// builtin answers NaN; the builtin min is NaN exactly when some product is,
// and that case takes math.Min and math.Max themselves.
func RevIvMul(a, b RevIv) RevIv {
	p0, p1 := float64(a.Lo*b.Lo), float64(a.Lo*b.Hi)
	p2, p3 := float64(a.Hi*b.Lo), float64(a.Hi*b.Hi)
	lo, hi := min(p0, p1, p2, p3), max(p0, p1, p2, p3)
	if lo != lo {
		lo, hi = math.Min(math.Min(p0, p1), math.Min(p2, p3)), math.Max(math.Max(p0, p1), math.Max(p2, p3))
	}
	return RevIv{RevDown(lo), RevUp(hi)}
}

// revIvScale is RevIvMul({q, q}, g), bit for bit, for one exact float q. With
// a zero-width a the four products are q·g.Lo and q·g.Hi, each twice, and
// math.Min(m, m) is m for every m — NaN, −0 and −Inf included — so the ends are
// math.Min and math.Max of the two distinct products, taken as RevIvMul takes
// them.
func revIvScale(q float64, g RevIv) RevIv {
	p0, p1 := float64(q*g.Lo), float64(q*g.Hi)
	lo, hi := min(p0, p1), max(p0, p1)
	if lo != lo {
		lo, hi = math.Min(p0, p1), math.Max(p0, p1)
	}
	return RevIv{RevDown(lo), RevUp(hi)}
}

func RevIvDot(a, b RevIvVec) RevIv {
	return RevIvAdd(RevIvAdd(RevIvMul(a[0], b[0]), RevIvMul(a[1], b[1])), RevIvMul(a[2], b[2]))
}

// revIvPointDot is RevIvDot(RevIvPoint(p), g), bit for bit: the three
// revIvScale products summed by RevIvAdd in the same order. It writes those
// calls out in one body because the pre-test runs it for every corner of every
// projection, and under the race detector each call costs more than its
// arithmetic. When any product is NaN it makes the calls themselves.
func revIvPointDot(p r3.Vec, g RevIvVec) RevIv {
	x0, x1 := float64(p.X*g[0].Lo), float64(p.X*g[0].Hi)
	y0, y1 := float64(p.Y*g[1].Lo), float64(p.Y*g[1].Hi)
	z0, z1 := float64(p.Z*g[2].Lo), float64(p.Z*g[2].Hi)
	xl, yl, zl := min(x0, x1), min(y0, y1), min(z0, z1)
	if xl != xl || yl != yl || zl != zl {
		return RevIvAdd(RevIvAdd(revIvScale(p.X, g[0]), revIvScale(p.Y, g[1])), revIvScale(p.Z, g[2]))
	}
	xh, yh, zh := max(x0, x1), max(y0, y1), max(z0, z1)
	return RevIv{
		RevDown(RevDown(RevDown(xl)+RevDown(yl)) + RevDown(zl)),
		RevUp(RevUp(RevUp(xh)+RevUp(yh)) + RevUp(zh)),
	}
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
		// A triangle's projection onto its own normal enclosure depends on
		// that triangle alone, so NewRevolveAuditTri holds it.
		var aLo, aHi, bLo, bHi float64
		switch gi {
		case 0:
			aLo, aHi = a.FnProj[0], a.FnProj[1]
			bLo, bHi = RevIvProject(b.Fp, g)
		case 1:
			aLo, aHi = RevIvProject(a.Fp, g)
			bLo, bHi = b.FnProj[0], b.FnProj[1]
		default:
			aLo, aHi = RevIvProject(a.Fp, g)
			bLo, bHi = RevIvProject(b.Fp, g)
		}
		gap := math.Max(RevDown(bLo-aHi), RevDown(aLo-bHi))
		if !proofbound.IsNonFinite(gap) && gap > cheap {
			return true
		}
	}
	return false
}

// RevIvProject is a lower bound on the least and an upper bound on the
// greatest exact projection of a triangle's corners onto g. Every end it folds
// is finite, so the builtin min and max give math.Min's and math.Max's bits.
func RevIvProject(p [3]r3.Vec, g RevIvVec) (float64, float64) {
	lo, hi := math.Inf(1), math.Inf(-1)
	for _, q := range p {
		d := revIvPointDot(q, g)
		if !d.Finite() {
			return math.NaN(), math.NaN()
		}
		lo = min(lo, d.Lo)
		hi = max(hi, d.Hi)
	}
	return lo, hi
}
