package motionbound

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

// PoseDeviation is docs/motion-check-design.md §5.1's η: a proven upper bound
// on how far any point of a mover sits between where the kernel measured it —
// under the float composed placement C = P0 then PoseAt — and where the ideal
// motion T* puts it, T* composed onto the mover's own placement P0. For a
// record point p with |p| ≤ r0 the two images are B(C)·p + t(C) and
// R·(B(P0)·p + t(P0) − c) + c + s, so they differ by at most
// ‖B(C) − R·B(P0)‖_F·r0 + |t(C) − (R·(t(P0) − c) + c + s)|. Every float is
// read exactly off Basis()/Translation(), and both norms are rational
// enclosures rooted by proofbound.RatSqrtUp, so η covers every rounding the pose
// committed: math.Sincos, Rodrigues' formula, the pivot offset, Then, and
// the rounded π/180 of a degree-stated angle.
//
// A linear part matching the ideal one exactly contributes nothing whatever
// r0 is — the true radius is finite even where no reader states it — so a
// pure translation stays chargeable on a payload with no record radius.
// Every other unreadable term answers +Inf, a refusal rather than a bound.
//
// The second result is the linear term's own factor, an upper bound on
// ‖B(C) − R·B(P0)‖_F, which PathAreaUpper reads to bound how far the straight
// path between the two images stretches the mover's surface.
func PoseDeviation(composed, placement r3.Transform, ideal IdealPose, r0 float64) (float64, float64) {
	bc, bp := composed.Basis(), placement.Basis()
	colsC := [3]r3.Vec{bc.EX, bc.EY, bc.EZ}
	colsP := [3]r3.Vec{bp.EX, bp.EY, bp.EZ}
	var linear []proofbound.RatInterval
	for j := range 3 {
		c, okC := RatVecOf(colsC[j])
		p, okP := RatVecOf(colsP[j])
		if !okC || !okP {
			return math.Inf(1), math.Inf(1)
		}
		image := ideal.Rot.Apply(PointVec(p))
		diff := IvVecSub(PointVec(c), image)
		linear = append(linear, diff[:]...)
	}
	tc, okC := RatVecOf(composed.Translation())
	tp, okP := RatVecOf(placement.Translation())
	if !okC || !okP {
		return math.Inf(1), math.Inf(1)
	}
	idealT := IvVecAdd(IvVecAdd(ideal.Rot.Apply(IvVecSub(PointVec(tp), ideal.Pivot)), ideal.Pivot), ideal.Shift)
	dt := IvVecSub(PointVec(tc), idealT)
	transUp := proofbound.RatSqrtUp(MagnitudeSquaredUpper(dt[:]...))
	linSq := MagnitudeSquaredUpper(linear...)
	if linSq.Sign() == 0 {
		return transUp, 0
	}
	linUp := proofbound.RatSqrtUp(linSq)
	return proofbound.AbsSumUpper(proofbound.ProductUpper(linUp, r0), transUp), linUp
}

// BasisSigmaLower is a proven lower bound on the smallest singular value of a
// placement's linear part B. r3 holds B orthonormal only to rounding, so
// BᵀB = I + E with E read exactly off the float columns; every eigenvalue of
// BᵀB is then at least 1 − ‖E‖_F, and the bound is that value's root,
// rounded down. An exactly orthonormal basis answers exactly 1; a defect too
// large to bound answers 0, which PathAreaUpper reads as a refusal.
func BasisSigmaLower(t r3.Transform) float64 {
	e, ok := BasisDefectUpper(t)
	switch {
	case !ok:
		return 0
	case e.Sign() == 0:
		return 1
	case e.Cmp(big.NewRat(1, 1)) >= 0:
		return 0
	}
	return proofbound.RatSqrtDown(new(big.Rat).Sub(big.NewRat(1, 1), e))
}

// BasisSigmaUpper is BasisSigmaLower's mirror: a proven upper bound on the
// largest singular value of a transform's linear part B. Every eigenvalue of
// BᵀB = I + E is at most 1 + ‖E‖_F, and the bound is that value's root,
// rounded up. An exactly orthonormal basis answers exactly 1; a defect that
// cannot be read answers +Inf, a refusal rather than a bound.
func BasisSigmaUpper(t r3.Transform) float64 {
	e, ok := BasisDefectUpper(t)
	switch {
	case !ok:
		return math.Inf(1)
	case e.Sign() == 0:
		return 1
	}
	return proofbound.RatSqrtUp(new(big.Rat).Add(big.NewRat(1, 1), e))
}

// BasisDefectUpper is a proven upper bound on ‖BᵀB − I‖_F for a transform's
// linear part B, read exactly off its float columns and rooted upward; it is
// exactly zero for an exactly orthonormal basis. ok is false when a column is
// not finite or the root overflows.
func BasisDefectUpper(t r3.Transform) (*big.Rat, bool) {
	b := t.Basis()
	var cols [3]RatVec
	for j, v := range []r3.Vec{b.EX, b.EY, b.EZ} {
		c, ok := RatVecOf(v)
		if !ok {
			return nil, false
		}
		cols[j] = c
	}
	var defect []proofbound.RatInterval
	for i := range 3 {
		for j := range 3 {
			e := proofbound.RatAdd(proofbound.RatMul(cols[i][0], cols[j][0]), proofbound.RatMul(cols[i][1], cols[j][1]), proofbound.RatMul(cols[i][2], cols[j][2]))
			if i == j {
				e.Sub(e, big.NewRat(1, 1))
			}
			defect = append(defect, proofbound.PointInterval(e))
		}
	}
	sq := MagnitudeSquaredUpper(defect...)
	if sq.Sign() == 0 {
		return new(big.Rat), true
	}
	e := proofarith.FloatRat(proofbound.RatSqrtUp(sq))
	return e, e != nil
}

// PathAreaUpper bounds the mover's surface area at every point of the straight
// path between the float pose's image and the ideal pose's — the area
// proofbound.SweptVolumeAllow's own contract asks for along the WHOLE path. Relative to
// the mover at rest, a point of that path is M_t·x + c with
// M_t = R + (1 − t)·(B(C) − R·B(P0))·B(P0)⁻¹, so
// ‖M_t‖₂ ≤ ‖R‖₂ + linear/sigma, and an area scales by at most ‖M_t‖₂². area
// is the rest area's upper bound (Area().Value + Bound), linear
// PoseDeviation's second result, sigma BasisSigmaLower of the rest placement.
//
// base is a proven upper bound on ‖R‖₂, the stretch base of
// docs/motion-check-design.md §5.1: exactly 1 for a Revolute and a Prismatic,
// whose ideal rotation is exactly orthogonal, and BasisSigmaUpper of the
// between's From — at s = 1 the larger of From's and To's — for a Between,
// whose ideal linear part R(s·θ, n)·B(From) is orthogonal only as far as
// B(From) is. An exact linear part scales area by base² alone, and base 1
// leaves it unscaled.
func PathAreaUpper(area, linear, sigma, base float64) float64 {
	if linear == 0 {
		if base == 1 {
			return area
		}
		return proofbound.ProductUpper(area, proofbound.ProductUpper(base, base))
	}
	stretch := proofbound.AbsSumUpper(base, proofbound.DivUpper(linear, sigma))
	return proofbound.ProductUpper(area, proofbound.ProductUpper(stretch, stretch))
}

// ExceedsResolution reports whether the interval between two exact parameters
// is wider than the resolution. Parts stated in the same terms — both whole
// turns, or both base units — compare exactly, which is what lets a dyadic
// step equal to the resolution stop on it. Mixed parts compare the interval's
// smallest possible width, π at its lower enclosure, against the resolution's
// largest, π at its upper, so the floor never stops refinement early by
// rounding.
func ExceedsResolution(p, q, res MotionParam) bool {
	dTurn := new(big.Rat).Sub(q.Turn, p.Turn)
	dBase := new(big.Rat).Sub(q.Base, p.Base)
	switch {
	case dBase.Sign() == 0 && res.Base.Sign() == 0:
		return new(big.Rat).Abs(dTurn).Cmp(res.Turn) > 0
	case dTurn.Sign() == 0 && res.Turn.Sign() == 0:
		return new(big.Rat).Abs(dBase).Cmp(res.Base) > 0
	}
	twoPi := proofbound.TwoPiInterval()
	width := proofbound.IntervalAdd(proofbound.IntervalScale(twoPi, dTurn), proofbound.PointInterval(dBase))
	lower := new(big.Rat)
	switch {
	case width.Lo.Sign() > 0:
		lower = width.Lo
	case width.Hi.Sign() < 0:
		lower = new(big.Rat).Neg(width.Hi)
	}
	upper := new(big.Rat).Mul(twoPi.Hi, res.Turn)
	upper.Add(upper, res.Base)
	return lower.Cmp(upper) > 0
}

// MoverTravel is τ of docs/motion-check-design.md §5.2 as an exact rational:
// a proven upper bound on how far any point of the mover travels while the
// parameter runs from p to q. A prismatic moves every point by exactly the
// displacement change along a unit direction; a revolute moves a point at
// distance ρ from the axis along an arc of length ρ·|Δθ|, which bounds its
// chord, and ρ ≤ rho. A between rotates a point at distance ρ from the screw
// axis through an arc of length ρ·|Δs|·θ and slides it |Δs|·|d| along the
// axis; the sum bounds the resultant, so τ = |Δs|·(rho·θ + |d|), with θ and
// d the exact rationals of the floats r3 read and no π entering. No rounding
// is committed: rho is already an upper bound, and a span over an angle is
// taken at π's upper enclosure.
func MoverTravel(mf MotionFrame, rho float64, p, q MotionParam) *big.Rat {
	span := p.SpanUpper(q)
	if mf.Kind == MotionPrismatic {
		return span
	}
	r := proofarith.FloatRat(rho)
	if r == nil {
		return nil
	}
	if mf.Kind == MotionRevolute {
		return span.Mul(span, r)
	}
	zero := MotionParam{Turn: new(big.Rat), Base: new(big.Rat)}
	rate := r.Mul(r, zero.SpanUpper(mf.Theta))
	rate.Add(rate, new(big.Rat).Abs(mf.Slide))
	return span.Mul(span, rate)
}
