package meshbool

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proof"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// Xp2 is an exact 2D point (a plane projection of an xpt). The cached float
// coordinates only accelerate the conservative sign filter; the rational
// coordinates remain the source of truth whenever the filter cannot decide.
type Xp2 struct {
	U, V        *big.Rat
	Fu, Fv      float64
	FloatFinite bool
	Hu, Hv, Hw  *big.Int
}

func NewXP2(u, v *big.Rat) Xp2 {
	fu, _ := u.Float64()
	fv, _ := v.Float64()
	return Xp2{
		U:  u,
		V:  v,
		Fu: fu,
		Fv: fv,
		FloatFinite: !math.IsNaN(fu) && !math.IsInf(fu, 0) &&
			!math.IsNaN(fv) && !math.IsInf(fv, 0),
	}
}

// NewXP2FromXpt keeps the projected point's homogeneous numerators alongside
// its rational coordinates. The rational values remain the public exact 2D
// representation used by polygon construction, while Cross2x can use the
// homogeneous form to avoid normalising four intermediate differences.
func NewXP2FromXpt(p proof.Xpt, u, v int) Xp2 {
	ur, vr := RatCoordOf(p, u), RatCoordOf(p, v)
	fu, _ := ur.Float64()
	fv, _ := vr.Float64()
	return Xp2{
		U:  ur,
		V:  vr,
		Fu: fu,
		Fv: fv,
		Hu: XIntCoordOf(p, u),
		Hv: XIntCoordOf(p, v),
		Hw: p.W,
		FloatFinite: !math.IsNaN(fu) && !math.IsInf(fu, 0) &&
			!math.IsNaN(fv) && !math.IsInf(fv, 0),
	}
}

// key2 is the exact 2D identity.
func (p Xp2) Key2() string { return p.U.RatString() + "|" + p.V.RatString() }

// Cross2x is the exact value of (b − a) × (c − a): positive when a, b, c turn
// counter-clockwise.
//
// The exact result remains available for callers that need more than its sign.
func Cross2x(a, b, c Xp2) *big.Rat {
	if a.Hu != nil && b.Hu != nil && c.Hu != nil {
		baU := new(big.Int).Sub(new(big.Int).Mul(b.Hu, a.Hw), new(big.Int).Mul(a.Hu, b.Hw))
		baV := new(big.Int).Sub(new(big.Int).Mul(b.Hv, a.Hw), new(big.Int).Mul(a.Hv, b.Hw))
		caU := new(big.Int).Sub(new(big.Int).Mul(c.Hu, a.Hw), new(big.Int).Mul(a.Hu, c.Hw))
		caV := new(big.Int).Sub(new(big.Int).Mul(c.Hv, a.Hw), new(big.Int).Mul(a.Hv, c.Hw))
		num := new(big.Int).Sub(new(big.Int).Mul(baU, caV), new(big.Int).Mul(baV, caU))
		den := new(big.Int).Mul(new(big.Int).Mul(a.Hw, a.Hw), new(big.Int).Mul(b.Hw, c.Hw))
		return new(big.Rat).SetFrac(num, den)
	}
	bu := new(big.Rat).Sub(b.U, a.U)
	bv := new(big.Rat).Sub(b.V, a.V)
	cu := new(big.Rat).Sub(c.U, a.U)
	cv := new(big.Rat).Sub(c.V, a.V)
	return new(big.Rat).Sub(new(big.Rat).Mul(bu, cv), new(big.Rat).Mul(bv, cu))
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
		return ClipFrac{Num: fa.Num(), Den: fa.Denom()}, ClipFrac{Num: fb.Num(), Den: fb.Denom()}
	}
	// The edge's own homogeneous difference, computed once for both points.
	baU := new(big.Int).Sub(new(big.Int).Mul(e1.Hu, e0.Hw), new(big.Int).Mul(e0.Hu, e1.Hw))
	baV := new(big.Int).Sub(new(big.Int).Mul(e1.Hv, e0.Hw), new(big.Int).Mul(e0.Hv, e1.Hw))
	return HomCross2Frac(e0, baU, baV, a), HomCross2Frac(e0, baU, baV, b)
}

// HomCross2Frac finishes EdgeCross2Fracs for one point: the cross product's
// homogeneous numerator over that point's own weight, the edge's shared factor
// already dropped.
func HomCross2Frac(e0 Xp2, baU, baV *big.Int, p Xp2) ClipFrac {
	caU := new(big.Int).Sub(new(big.Int).Mul(p.Hu, e0.Hw), new(big.Int).Mul(e0.Hu, p.Hw))
	caV := new(big.Int).Sub(new(big.Int).Mul(p.Hv, e0.Hw), new(big.Int).Mul(e0.Hv, p.Hw))
	num := new(big.Int).Sub(new(big.Int).Mul(baU, caV), new(big.Int).Mul(baV, caU))
	return ClipFrac{Num: num, Den: p.Hw}
}

// Cross2xSign uses a conservative float filter for the common case and keeps
// the exact rational predicate for values close enough to zero that rounding
// could change the answer. The scale uses the original coordinates, not only
// their float differences, so cancellation during subtraction is covered too.
func Cross2xSign(a, b, c Xp2) int {
	au, av, bu, bv, cu, cv := a.Fu, a.Fv, b.Fu, b.Fv, c.Fu, c.Fv
	if a.FloatFinite && b.FloatFinite && c.FloatFinite {
		det := (bu-au)*(cv-av) - (bv-av)*(cu-au)
		scale := (math.Abs(bu)+math.Abs(au))*(math.Abs(cv)+math.Abs(av)) +
			(math.Abs(bv)+math.Abs(av))*(math.Abs(cu)+math.Abs(au))
		err := 1e-12 * scale
		if !math.IsNaN(det) && !math.IsInf(det, 0) && !math.IsInf(err, 0) && err > 0 {
			if det > err {
				return 1
			}
			if det < -err {
				return -1
			}
		}
	}
	return Cross2x(a, b, c).Sign()
}

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
	// Collinear: order along the dominant axis of the segment.
	du := new(big.Rat).Sub(b.U, a.U)
	dv := new(big.Rat).Sub(b.V, a.V)
	var lo, hi, x *big.Rat
	if du.Sign() != 0 {
		lo, hi, x = a.U, b.U, p.U
	} else if dv.Sign() != 0 {
		lo, hi, x = a.V, b.V, p.V
	} else {
		// A zero-length segment holds only its own point.
		eq := p.U.Cmp(a.U) == 0 && p.V.Cmp(a.V) == 0
		return eq, false
	}
	if lo.Cmp(hi) > 0 {
		lo, hi = hi, lo
	}
	if x.Cmp(lo) < 0 || x.Cmp(hi) > 0 {
		return false, false
	}
	interior := x.Cmp(lo) > 0 && x.Cmp(hi) < 0
	return true, interior
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
		belowA := a.V.Cmp(p.V) <= 0
		if belowA == (b.V.Cmp(p.V) <= 0) {
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

// PolyArea2Sign is the exact sign of twice the polygon's signed area.
// Reduce after every edge so the integer accumulator cannot grow with the
// number of edges beyond the exact area's denominator.
func PolyArea2Sign(budget *proofbound.WorkBudget, poly []Xp2) (int, error) {
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
		if p.U.Cmp(a.U) == 0 && p.V.Cmp(a.V) == 0 {
			continue
		}
		if p.U.Cmp(c.U) == 0 && p.V.Cmp(c.V) == 0 {
			continue
		}
		if PointInTriX(p, a, b, c) {
			return true, nil
		}
	}
	return false, nil
}
