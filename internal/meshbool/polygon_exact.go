package meshbool

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/polynomial"
	"github.com/lestrrat-3d/decad/internal/proof"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// Xp2 is an exact 2D point (a plane projection of an xpt). The cached float
// coordinates only accelerate conservative filters; the rational coordinates
// remain the source of truth whenever a filter cannot decide. Callers must not
// mutate either coordinate or its float cache after construction.
// FloatFinite means both float coordinates are usable by the sign filter.
type Xp2 struct {
	U, V        *big.Rat
	Fu, Fv      float64
	FloatFinite bool
	// Only the constructors set this; a struct literal has no proven float cache.
	floatFromExact bool
	Hu, Hv, Hw     *big.Int
}

func NewXP2(u, v *big.Rat) Xp2 {
	fu, _ := proof.RatFloat64(u)
	fv, _ := proof.RatFloat64(v)
	return Xp2{
		U:              u,
		V:              v,
		Fu:             fu,
		Fv:             fv,
		floatFromExact: true,
		FloatFinite:    xp2FloatSafe(u, fu) && xp2FloatSafe(v, fv),
	}
}

// newXP2FromFloats is NewXP2(MustRatOf(fu), MustRatOf(fv)) without the
// GCD and the float round trip. A finite float is an integer mantissa times a
// power of two, so proof.FloatRat builds the same lowest-terms rational that
// big.Rat.SetFloat64 builds by normalising. The rational is exactly the
// float, so big.Rat.Float64 would return the input float (+0 for a zero of
// either sign); the cache is set to that float directly. It panics, with
// MustRatOf's message, on a NaN or an infinity.
func newXP2FromFloats(fu, fv float64) Xp2 {
	u, v := liftProjectionFloat(fu), liftProjectionFloat(fv)
	return Xp2{
		U:              u,
		V:              v,
		Fu:             cacheFloat(fu),
		Fv:             cacheFloat(fv),
		floatFromExact: true,
		FloatFinite:    xp2FloatSafe(u, cacheFloat(fu)) && xp2FloatSafe(v, cacheFloat(fv)),
	}
}

// liftProjectionFloat lifts a nonzero finite float with proof.FloatRat. Zero
// and non-finite input take MustRatOf. The zero guard exists for the Rat
// representation: FloatRat(0) is an uninitialised Rat whose Denom returns a
// fresh detached 1 on every call, while SetFloat64(0) stores the denominator,
// so Denom returns the same pointer each time. A non-finite float panics as
// before.
func liftProjectionFloat(f float64) *big.Rat {
	if f == 0 || math.IsNaN(f) || math.IsInf(f, 0) {
		return polynomial.MustRatOf(f)
	}
	return proof.FloatRat(f)
}

// cacheFloat is the float big.Rat.Float64 returns for the rational f lifts
// to: f itself, except that -0 lifts to the rational 0 and comes back as +0.
func cacheFloat(f float64) float64 {
	if f == 0 {
		return 0
	}
	return f
}

// NewXP2FromXpt keeps the projected point's homogeneous numerators alongside
// its rational coordinates. The rational values remain the public exact 2D
// representation used by polygon construction, while Cross2x can use the
// homogeneous form to avoid normalising four intermediate differences.
func NewXP2FromXpt(p proof.Xpt, u, v int) Xp2 {
	return newXP2FromXptInto(p, u, v, new(big.Rat), new(big.Rat))
}

// newXP2FromXptInto fills distinct caller-owned rationals. The caller must
// keep both unchanged until every reader of the returned projection finishes.
func newXP2FromXptInto(p proof.Xpt, u, v int, ur, vr *big.Rat) Xp2 {
	proof.RatFrac(ur, XIntCoordOf(p, u), p.W)
	proof.RatFrac(vr, XIntCoordOf(p, v), p.W)
	fu, _ := proof.RatFloat64(ur)
	fv, _ := proof.RatFloat64(vr)
	return Xp2{
		U:              ur,
		V:              vr,
		Fu:             fu,
		Fv:             fv,
		floatFromExact: true,
		Hu:             XIntCoordOf(p, u),
		Hv:             XIntCoordOf(p, v),
		Hw:             p.W,
		FloatFinite:    xp2FloatSafe(ur, fu) && xp2FloatSafe(vr, fv),
	}
}

// xp2FloatSafe admits exact zero or a normal rounded coordinate in
// [2^-300, 2^250]. Within that interval, exact-to-float conversion has
// relative error at most 2^-53; a rounded zero from a nonzero rational or
// a subnormal need not have such a relative bound.
func xp2FloatSafe(exact *big.Rat, rounded float64) bool {
	if exact.Sign() == 0 {
		return rounded == 0
	}
	m := math.Abs(rounded)
	return m >= 0x1p-300 && m <= 0x1p250
}

// Cross2x is the exact value of (b − a) × (c − a): positive when a, b, c turn
// counter-clockwise.
//
// The exact result remains available for callers that need more than its sign.
func Cross2x(a, b, c Xp2) *big.Rat {
	if a.Hu != nil && b.Hu != nil && c.Hu != nil {
		den := new(big.Int).Mul(new(big.Int).Mul(a.Hw, a.Hw), new(big.Int).Mul(b.Hw, c.Hw))
		return new(big.Rat).SetFrac(cross2xHomNumerator(a, b, c), den)
	}
	bu := proof.SubRat(new(big.Rat), b.U, a.U)
	bv := proof.SubRat(new(big.Rat), b.V, a.V)
	cu := proof.SubRat(new(big.Rat), c.U, a.U)
	cv := proof.SubRat(new(big.Rat), c.V, a.V)
	return proof.SubRat(new(big.Rat), proof.MulRat(new(big.Rat), bu, cv), proof.MulRat(new(big.Rat), bv, cu))
}

func cross2xHomNumerator(a, b, c Xp2) *big.Int {
	baU := new(big.Int).Sub(new(big.Int).Mul(b.Hu, a.Hw), new(big.Int).Mul(a.Hu, b.Hw))
	baV := new(big.Int).Sub(new(big.Int).Mul(b.Hv, a.Hw), new(big.Int).Mul(a.Hv, b.Hw))
	caU := new(big.Int).Sub(new(big.Int).Mul(c.Hu, a.Hw), new(big.Int).Mul(a.Hu, c.Hw))
	caV := new(big.Int).Sub(new(big.Int).Mul(c.Hv, a.Hw), new(big.Int).Mul(a.Hv, c.Hw))
	return new(big.Int).Sub(new(big.Int).Mul(baU, caV), new(big.Int).Mul(baV, caU))
}

// ratDiffNumInto leaves the difference's positive denominator implicit. The
// scratch integer is reused across the four differences in one orientation.
func ratDiffNumInto(dst, scratch *big.Int, x, y *big.Rat) {
	dst.Mul(x.Num(), y.Denom())
	scratch.Mul(y.Num(), x.Denom())
	dst.Sub(dst, scratch)
}

func cross2xRatSign(a, b, c Xp2) int {
	var bu, bv, cu, cv, scratch big.Int
	ratDiffNumInto(&bu, &scratch, b.U, a.U)
	ratDiffNumInto(&bv, &scratch, b.V, a.V)
	ratDiffNumInto(&cu, &scratch, c.U, a.U)
	ratDiffNumInto(&cv, &scratch, c.V, a.V)
	// Both terms are scaled to the same positive product of all six
	// coordinate denominators. Their difference has Cross2x's exact sign.
	var left, right big.Int
	left.Mul(&bu, &cv)
	left.Mul(&left, b.V.Denom())
	left.Mul(&left, c.U.Denom())
	right.Mul(&bv, &cu)
	right.Mul(&right, b.U.Denom())
	right.Mul(&right, c.V.Denom())
	return left.Sub(&left, &right).Sign()
}

// ClipFrac is an exact fraction kept UNNORMALISED: no common factor is ever
// divided out of num and den, so building one costs no GCD. Two of them
// compare through CmpClipFrac, which cross-multiplies instead of normalising.
// The value is exact — nothing here rounds — it simply is not in lowest terms.
//
// Every ClipFrac this package builds carries den > 0, which is what makes the
// cross-multiplied comparison keep its direction.
//
// Neither field is ever used as an arithmetic destination: den commonly
// ALIASES an Xp2's own hw, and a projection cache shares one Xp2 across every
// query of its mesh, so mutating either field would corrupt the cache.
type ClipFrac struct{ Num, Den *big.Int }

// CmpClipFrac compares x against y, both with positive denominators: x is
// below y exactly when x.num·y.den is below y.num·x.den.
func CmpClipFrac(x, y ClipFrac) int {
	left := new(big.Int).Mul(x.Num, y.Den)
	right := new(big.Int).Mul(y.Num, x.Den)
	return left.Cmp(right)
}

// EdgeCross2Fracs is Cross2x(e0, e1, a) and Cross2x(e0, e1, b) as unnormalised
// fractions, both scaled by ONE factor they share.
//
// Its caller (SegTriOverlap2) reads only the ratio −fa/(fb − fa), and that
// ratio does not move when fa and fb are both multiplied by the same nonzero
// constant. So the homogeneous form's e0.hw²·e1.hw — the part of Cross2x's
// denominator that does not depend on the third point — is dropped rather than
// carried, which keeps both integers small. Only the third point's own weight
// survives, and it is the returned denominator.
//
// The homogeneous route needs all four points to carry homogeneous
// coordinates, because the shared factor only cancels when both fractions are
// really scaled by it. When any point lacks them, both fractions come from
// Cross2x's rational value, whose numerator and denominator are already the
// pair (and whose denominator big.Rat keeps positive).
//
// The returned integers are read-only; see ClipFrac.
func EdgeCross2Fracs(e0, e1, a, b Xp2) (ClipFrac, ClipFrac) {
	if e0.Hu == nil || e1.Hu == nil || a.Hu == nil || b.Hu == nil {
		fa, fb := Cross2x(e0, e1, a), Cross2x(e0, e1, b)
		return ClipFrac{Num: new(big.Int).Set(fa.Num()), Den: new(big.Int).Set(fa.Denom())},
			ClipFrac{Num: new(big.Int).Set(fb.Num()), Den: new(big.Int).Set(fb.Denom())}
	}
	// The edge's own homogeneous difference, computed once for both points.
	var baU, baV, scratch big.Int
	baU.Mul(e1.Hu, e0.Hw)
	scratch.Mul(e0.Hu, e1.Hw)
	baU.Sub(&baU, &scratch)
	baV.Mul(e1.Hv, e0.Hw)
	scratch.Mul(e0.Hv, e1.Hw)
	baV.Sub(&baV, &scratch)
	cross := func(p Xp2) ClipFrac {
		// An endpoint's exact determinant is zero, including when its
		// weight is zero or negative. Keep its denominator unchanged.
		// Incomplete inputs still take the arithmetic path and panic there.
		if homComplete(e0) && homComplete(e1) && homComplete(p) &&
			(homSharedPoint(p, e0) || homSharedPoint(p, e1)) {
			return ClipFrac{Num: new(big.Int), Den: p.Hw}
		}
		return HomCross2Frac(e0, &baU, &baV, p)
	}
	return cross(a), cross(b)
}

// HomCross2Frac finishes EdgeCross2Fracs for one point: the cross product's
// homogeneous numerator over that point's own weight, the edge's shared factor
// already dropped.
func HomCross2Frac(e0 Xp2, baU, baV *big.Int, p Xp2) ClipFrac {
	var caU, caV, scratch big.Int
	caU.Mul(p.Hu, e0.Hw)
	scratch.Mul(e0.Hu, p.Hw)
	caU.Sub(&caU, &scratch)
	caV.Mul(p.Hv, e0.Hw)
	scratch.Mul(e0.Hv, p.Hw)
	caV.Sub(&caV, &scratch)
	scratch.Mul(baV, &caU)
	num := new(big.Int).Mul(baU, &caV)
	num.Sub(num, &scratch)
	return ClipFrac{Num: num, Den: p.Hw}
}

// Cross2xSign uses a conservative float filter for the common case and keeps
// the exact rational predicate for values close enough to zero that rounding
// could change the answer. The scale uses the original coordinates, not only
// their float differences, so cancellation during subtraction is covered too.
// With all cached coordinates admitted, nonzero rounded coordinates lie on
// the 2^-352 grid. Their differences and products lie on the 2^-352 and
// 2^-704 grids; every nonzero result is normal. No arithmetic step can
// lose a nonzero value to underflow, and all intermediates stay at most 2^503.
// Each expanded determinant term charges two coordinate conversions, two
// difference roundings, a product rounding and the final subtraction. With
// u=2^-53, gamma6=6u/(1-6u) bounds the error by gamma6 times the exact
// coordinate-product sum. That sum is at most the sum formed from rounded
// coordinates divided by (1-u)^2; evaluating scale rounds at most four times.
// Thus the error is below gamma6/(1-u)^6 times the computed scale, less
// than 7e-16 times it. A positive scale is at least 2^-600, so the 1e-12
// threshold stays normal and dominates that bound, including its rounding.
//
// Once the filter abstains, two of the three points holding the same
// coordinate pointers return 0 without big.Int work: (b-a) x (c-a) is zero
// whenever a==b, a==c or b==c as exact points, and the exact paths below give
// that same 0 for such inputs. The shortcut applies only when every field the
// selected path reads (Hu, Hv, Hw, or U and V) is set on all three points; a
// point missing one still reaches the path that panics on it.
func Cross2xSign(a, b, c Xp2) int {
	au, av, bu, bv, cu, cv := a.Fu, a.Fv, b.Fu, b.Fv, c.Fu, c.Fv
	if a.FloatFinite && b.FloatFinite && c.FloatFinite {
		det := (bu-au)*(cv-av) - (bv-av)*(cu-au)
		scale := (proof.Abs(bu)+proof.Abs(au))*(proof.Abs(cv)+proof.Abs(av)) +
			(proof.Abs(bv)+proof.Abs(av))*(proof.Abs(cu)+proof.Abs(au))
		err := 1e-12 * scale
		if det == det && det <= math.MaxFloat64 && det >= -math.MaxFloat64 && err <= math.MaxFloat64 && err > 0 {
			if det > err {
				return 1
			}
			if det < -err {
				return -1
			}
		}
	}
	if a.Hu != nil && b.Hu != nil && c.Hu != nil {
		// Every field the exact path reads must be set on all three points,
		// so a malformed point still panics there instead of reading as 0.
		if homComplete(a) && homComplete(b) && homComplete(c) &&
			(homSharedPoint(a, b) || homSharedPoint(a, c) || homSharedPoint(b, c)) {
			return 0
		}
		return cross2xHomNumerator(a, b, c).Sign()
	}
	// Two points that hold the same coordinate pointers are one exact point.
	if ratComplete(a) && ratComplete(b) && ratComplete(c) &&
		(ratSharedPoint(a, b) || ratSharedPoint(a, c) || ratSharedPoint(b, c)) {
		return 0
	}
	// A shared exact coordinate removes one product from the determinant.
	if au == bu && proof.CmpRat(a.U, b.U) == 0 {
		return -proof.CmpRat(b.V, a.V) * proof.CmpRat(c.U, a.U)
	}
	if av == bv && proof.CmpRat(a.V, b.V) == 0 {
		return proof.CmpRat(b.U, a.U) * proof.CmpRat(c.V, a.V)
	}
	if au == cu && proof.CmpRat(a.U, c.U) == 0 {
		return proof.CmpRat(b.U, a.U) * proof.CmpRat(c.V, a.V)
	}
	if av == cv && proof.CmpRat(a.V, c.V) == 0 {
		return -proof.CmpRat(b.V, a.V) * proof.CmpRat(c.U, a.U)
	}
	return cross2xRatSign(a, b, c)
}

// homSharedPoint reports whether p and q hold the same three homogeneous
// integer pointers.
func homSharedPoint(p, q Xp2) bool {
	return p.Hu == q.Hu && p.Hv == q.Hv && p.Hw == q.Hw
}

// homComplete reports whether every homogeneous field is set, which is what
// cross2xHomNumerator reads from each point.
func homComplete(p Xp2) bool { return p.Hu != nil && p.Hv != nil && p.Hw != nil }

// ratComplete reports whether both rational coordinates are set, which is what
// the rational paths of Cross2xSign read from each point.
func ratComplete(p Xp2) bool { return p.U != nil && p.V != nil }

// ratSharedPoint is homSharedPoint for the rational coordinates U and V.
func ratSharedPoint(p, q Xp2) bool { return p.U == q.U && p.V == q.V }

// PointInTriX reports whether p lies inside or on the closed triangle a, b,
// c, whichever way it is wound — the exact analog of pointInTri.
func PointInTriX(p, a, b, c Xp2) bool {
	d1 := Cross2xSign(a, b, p)
	d2 := Cross2xSign(b, c, p)
	d3 := Cross2xSign(c, a, p)
	hasNeg := d1 < 0 || d2 < 0 || d3 < 0
	hasPos := d1 > 0 || d2 > 0 || d3 > 0
	return !hasNeg || !hasPos
}

// OnSegment2 reports whether p lies on the closed segment (a, b) — collinear
// and within the endpoints. interior additionally excludes the endpoints.
func OnSegment2(a, b, p Xp2) (bool, bool) {
	if Cross2xSign(a, b, p) != 0 {
		return false, false
	}
	// Collinear: order along the first nonconstant coordinate.
	cached := a.floatFromExact && b.floatFromExact && p.floatFromExact
	uOrder := compareProjectedCoordinate(a.U, b.U, a.Fu, b.Fu, cached)
	var lo, hi, x *big.Rat
	var flo, fhi, fx float64
	if uOrder != 0 {
		lo, hi, x = a.U, b.U, p.U
		flo, fhi, fx = a.Fu, b.Fu, p.Fu
	} else {
		vOrder := compareProjectedCoordinate(a.V, b.V, a.Fv, b.Fv, cached)
		if vOrder == 0 {
			// A zero-length segment holds only its own point.
			eq := compareProjectedCoordinate(p.U, a.U, p.Fu, a.Fu, cached) == 0 &&
				compareProjectedCoordinate(p.V, a.V, p.Fv, a.Fv, cached) == 0
			return eq, false
		}
		lo, hi, x = a.V, b.V, p.V
		flo, fhi, fx = a.Fv, b.Fv, p.Fv
		uOrder = vOrder
	}
	if uOrder > 0 {
		lo, hi = hi, lo
		flo, fhi = fhi, flo
	}
	lower := compareProjectedCoordinate(x, lo, fx, flo, cached)
	upper := compareProjectedCoordinate(x, hi, fx, fhi, cached)
	if lower < 0 || upper > 0 {
		return false, false
	}
	return true, lower > 0 && upper < 0
}

// compareProjectedCoordinate uses the monotonicity of exact-to-float rounding:
// unequal rounded values have the same order as their exact coordinates.
// Equal roundings, NaNs and coordinates without constructor caches compare
// exactly. cached must certify both rounded values came from a and b.
func compareProjectedCoordinate(a, b *big.Rat, fa, fb float64, cached bool) int {
	if cached {
		if fa < fb {
			return -1
		}
		if fa > fb {
			return 1
		}
	}
	return proof.CmpRat(a, b)
}

// PointInPoly2 reports whether p lies strictly inside the simple polygon —
// exact parity along a +u ray with the half-open rule, so a crossing at a
// shared vertex counts exactly once. A p on the boundary reports onBoundary.
func PointInPoly2(budget *proofbound.WorkBudget, poly []Xp2, p Xp2) (bool, bool, error) {
	n := len(poly)
	for i := range n {
		if err := budget.Step(); err != nil {
			return false, false, err
		}
		on, _ := OnSegment2(poly[i], poly[(i+1)%n], p)
		if on {
			return false, true, nil
		}
	}
	inside := false
	for i := range n {
		if err := budget.Step(); err != nil {
			return false, false, err
		}
		a, b := poly[i], poly[(i+1)%n]
		belowA := compareProjectedCoordinate(a.V, p.V, a.Fv, p.Fv, a.floatFromExact && p.floatFromExact) <= 0
		if belowA == (compareProjectedCoordinate(b.V, p.V, b.Fv, p.Fv, b.floatFromExact && p.floatFromExact) <= 0) {
			continue
		}
		// An upward edge crosses right of p when its orientation with p is
		// positive; a downward edge does so when the orientation is negative.
		// Cross2xSign gives that exact comparison without dividing rationals.
		side := Cross2xSign(a, b, p)
		if (belowA && side > 0) || (!belowA && side < 0) {
			inside = !inside
		}
	}
	return inside, false, nil
}

// polyArea2FloatSign accepts only a sign separated from zero by a proven
// rounding allowance. All admitted coordinates are exact zero or normal
// floats in [2^-300, 2^250], so products and nonzero sums cannot underflow.
// For at most 1024 edges, coordinate conversion, products, differences and
// summation contribute less than (n+6)*2^-52 times the product magnitude.
// The magnitude's own rounding changes it by less than 2^-40; the larger
// 1e-12*(n+1) allowance covers both, including its final rounding.
func polyArea2FloatSign(poly []Xp2) int {
	n := len(poly)
	if n == 0 || n > 1024 {
		return 0
	}
	var sum, scale float64
	for i, a := range poly {
		b := poly[(i+1)%n]
		if !a.floatFromExact || !b.floatFromExact || !a.FloatFinite || !b.FloatFinite {
			return 0
		}
		left, right := a.Fu*b.Fv, b.Fu*a.Fv
		sum += left - right
		scale += math.Abs(left) + math.Abs(right)
	}
	err := 1e-12 * (float64(n) + 1) * scale
	if math.IsNaN(sum) || math.IsInf(sum, 0) || math.IsNaN(err) || math.IsInf(err, 0) || err <= 0 {
		return 0
	}
	if sum > err {
		return 1
	}
	if sum < -err {
		return -1
	}
	return 0
}

// PolyArea2Sign is the exact sign of twice the polygon's signed area.
// Reduce after every edge so the integer accumulator cannot grow with the
// number of edges beyond the exact area's denominator.
func PolyArea2Sign(budget *proofbound.WorkBudget, poly []Xp2) (int, error) {
	if sign := polyArea2FloatSign(poly); sign != 0 {
		if _, err := budget.StepN(uint64(len(poly))); err != nil {
			return 0, err
		}
		return sign, nil
	}
	var num, den big.Int
	den.SetInt64(1)
	var leftDen, rightDen, termDen, left, right, next, part, gcd big.Int
	n := len(poly)
	for i := range n {
		if err := budget.Step(); err != nil {
			return 0, err
		}
		a, b := poly[i], poly[(i+1)%n]
		leftDen.Mul(a.U.Denom(), b.V.Denom())
		rightDen.Mul(b.U.Denom(), a.V.Denom())
		termDen.Mul(&leftDen, &rightDen)
		left.Mul(a.U.Num(), b.V.Num())
		left.Mul(&left, &rightDen)
		right.Mul(b.U.Num(), a.V.Num())
		right.Mul(&right, &leftDen)
		left.Sub(&left, &right)
		next.Mul(&num, &termDen)
		next.Add(&next, part.Mul(&den, &left))
		den.Mul(&den, &termDen)
		num.Set(&next)
		gcd.GCD(nil, nil, &num, &den)
		if gcd.BitLen() > 1 {
			num.Quo(&num, &gcd)
			den.Quo(&den, &gcd)
		}
	}
	return num.Sign(), nil
}

// EarClipX triangulates a weakly-simple counter-clockwise polygon (given as
// indices into pts) by exact ear clipping. Unlike the float cap triangulator
// it NEVER drops a collinear vertex — every input vertex appears in the
// output, which is what keeps a conforming subdivision conforming — so only
// strictly convex, unblocked ears are clipped. A stall means the polygon is
// not weakly simple, which is an internal error, never a wrong mesh.
func EarClipX(budget *proofbound.WorkBudget, pts []Xp2, poly []int) ([][3]int, error) {
	idx := make([]int, len(poly))
	for i, vi := range poly {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		idx[i] = vi
	}
	tris := make([][3]int, 0, len(idx))
	for len(idx) > 3 {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		n := len(idx)
		clipped := false
		for i := range n {
			if err := budget.Step(); err != nil {
				return nil, err
			}
			ia, ib, ic := idx[(i-1+n)%n], idx[i], idx[(i+1)%n]
			if Cross2xSign(pts[ia], pts[ib], pts[ic]) <= 0 {
				continue
			}
			blocked, err := EarBlockedX(budget, pts, idx, i)
			if err != nil {
				return nil, err
			}
			if blocked {
				continue
			}
			tris = append(tris, [3]int{ia, ib, ic})
			for j := i; j+1 < len(idx); j++ {
				if err := budget.Step(); err != nil {
					return nil, err
				}
				idx[j] = idx[j+1]
			}
			idx = idx[:len(idx)-1]
			clipped = true
			break
		}
		if !clipped {
			return nil, fmt.Errorf(`%w: exact ear clipping stalled on a boolean subdivision polygon`, decaderr.ErrBooleanFailed)
		}
	}
	if Cross2xSign(pts[idx[0]], pts[idx[1]], pts[idx[2]]) > 0 {
		tris = append(tris, [3]int{idx[0], idx[1], idx[2]})
	} else if Cross2xSign(pts[idx[0]], pts[idx[1]], pts[idx[2]]) < 0 {
		return nil, fmt.Errorf(`%w: a boolean subdivision polygon closed clockwise`, decaderr.ErrBooleanFailed)
	}
	return tris, nil
}

func finiteRoundedCache(p Xp2) bool {
	return p.floatFromExact && p.FloatFinite &&
		!math.IsNaN(p.Fu) && !math.IsInf(p.Fu, 0) &&
		!math.IsNaN(p.Fv) && !math.IsInf(p.Fv, 0)
}

// EarBlockedX reports whether another polygon vertex lies inside the closed
// candidate ear — the exact analog of earBlocked, except that every OTHER
// vertex can block (collinear duplicates included), which is the conservative
// direction.
func EarBlockedX(budget *proofbound.WorkBudget, pts []Xp2, idx []int, i int) (bool, error) {
	n := len(idx)
	ip, in := (i-1+n)%n, (i+1)%n
	a, b, c := pts[idx[ip]], pts[idx[i]], pts[idx[in]]
	minU := math.Min(a.Fu, math.Min(b.Fu, c.Fu))
	maxU := math.Max(a.Fu, math.Max(b.Fu, c.Fu))
	minV := math.Min(a.Fv, math.Min(b.Fv, c.Fv))
	maxV := math.Max(a.Fv, math.Max(b.Fv, c.Fv))
	finiteBox := a.FloatFinite && b.FloatFinite && c.FloatFinite
	finiteA, finiteC := finiteRoundedCache(a), finiteRoundedCache(c)
	for j := range n {
		if err := budget.Step(); err != nil {
			return false, err
		}
		if j == ip || j == i || j == in {
			continue
		}
		p := pts[idx[j]]
		// Float conversion is monotone. A strict float separation therefore
		// proves exact separation, while equal rounded coordinates fall
		// through to the exact predicate.
		if finiteBox && p.FloatFinite &&
			(p.Fu < minU || p.Fu > maxU || p.Fv < minV || p.Fv > maxV) {
			continue
		}
		finiteP := finiteRoundedCache(p)
		// Unequal finite roundings disprove exact endpoint identity. Equal
		// roundings, absent caches, and nonfinite values still compare exactly.
		if (!finiteP || !finiteA || (p.Fu == a.Fu && p.Fv == a.Fv)) &&
			p.U.Cmp(a.U) == 0 && p.V.Cmp(a.V) == 0 {
			continue
		}
		if (!finiteP || !finiteC || (p.Fu == c.Fu && p.Fv == c.Fv)) &&
			p.U.Cmp(c.U) == 0 && p.V.Cmp(c.V) == 0 {
			continue
		}
		if PointInTriX(p, a, b, c) {
			return true, nil
		}
	}
	return false, nil
}
