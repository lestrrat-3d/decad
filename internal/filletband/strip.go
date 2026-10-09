package filletband

import (
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// Heights holds the height integrals of the circular height map
// t(h) = r − √(r² − h²) over h ∈ [0, r] (docs/loop-fillet-design.md §5.2):
// J_k = ∫ t(h)^k dh and H_k = ∫ h·t(h)^k dh.
type Heights struct {
	J1, J2, J3, H1, H2 Interval
}

func pi() Interval { return proofbound.Interval(proofbound.PiLower, proofbound.PiUpper) }

// HeightsOf encloses J₁ = r²(1 − π/4), J₂ = r³(5/3 − π/2), J₃ = r⁴(3 − 15π/16),
// H₁ = r³/6 and H₂ = r⁴/12 for every radius in r, a positive interval: the
// radius as stated, widened by its conversion rounding.
func HeightsOf(r Interval) (Heights, error) {
	if r.Lo.Sign() <= 0 {
		return Heights{}, fmt.Errorf(`%w: a loop fillet's radius is not positive`, decaderr.ErrDegenerate)
	}
	p := pi()
	r2 := proofbound.IntervalMul(r, r)
	r3 := proofbound.IntervalMul(r2, r)
	r4 := proofbound.IntervalMul(r3, r)
	poly := func(c0 *big.Rat, num, den int64) Interval {
		return proofbound.IntervalSub(point(c0), scale(p, num, den))
	}
	return Heights{
		J1: proofbound.IntervalMul(r2, poly(big.NewRat(1, 1), 1, 4)),
		J2: proofbound.IntervalMul(r3, poly(big.NewRat(5, 3), 1, 2)),
		J3: proofbound.IntervalMul(r4, poly(big.NewRat(3, 1), 15, 16)),
		H1: scale(r3, 1, 6),
		H2: scale(r4, 1, 12),
	}, nil
}

// Strip is the wedge the band removes or fills (§5.3), in F's frame: Volume
// is V_strip and Moment its first moment along u, v and F's normal.
type Strip struct {
	Volume Interval
	Moment [3]Interval
}

// StripOf integrates the strip's sections over the band's height: the
// section at height h from the side level toward F is S(t(h)), so
// V_strip = a₁J₁ + a₂J₂, its in-plane moment m₁J₁ + m₂J₂ + m₃J₃, and its
// axial moment sideZ·V_strip − m·(a₁H₁ + a₂H₂), m the band's material sign
// (the side level lies at F's level plus m·r).
func StripOf(c Coefficients, h Heights, sideZ Interval, m int64) Strip {
	mul := proofbound.IntervalMul
	add := proofbound.IntervalAdd
	volume := add(mul(c.A1, h.J1), mul(c.A2, h.J2))
	var out Strip
	out.Volume = volume
	for i := range 2 {
		out.Moment[i] = add(add(mul(c.M1[i], h.J1), mul(c.M2[i], h.J2)), mul(c.M3[i], h.J3))
	}
	axial := add(mul(c.A1, h.H1), mul(c.A2, h.H2))
	out.Moment[2] = proofbound.IntervalSub(mul(sideZ, volume), scale(axial, m, 1))
	return out
}

// PatchArea is §5.3's area of one patch over every radius in r: a quarter of
// the pipe swept along the level loop, whose length at offset t is the
// piece's share of a₁ + 2a₂t.
//
//	LF1:    r·(ℓ·π/2 − (κ₀ + κ₁)·r·(π/2 − 1))
//	LF2/3:  β·r·(R·π/2 ∓ r·(π/2 − 1))
//	LF6:    ψ·r²·(π/2 − 1)
func PatchArea(p Piece, r Interval) Interval {
	mul := proofbound.IntervalMul
	halfPi := proofbound.HalfPiInterval()
	q := proofbound.IntervalSub(halfPi, pointInt(1))
	rq := mul(r, q)
	switch p.Kind {
	case Cylinder:
		ksum := proofbound.IntervalAdd(p.Kappa0, p.Kappa1)
		return mul(r, proofbound.IntervalSub(mul(p.Length, halfPi), mul(ksum, rq)))
	case InnerTorus:
		return mul(mul(p.Sweep, r), proofbound.IntervalSub(mul(p.Radius, halfPi), rq))
	case OuterTorus:
		return mul(mul(p.Sweep, r), proofbound.IntervalAdd(mul(p.Radius, halfPi), rq))
	default:
		return mul(mul(p.Sweep, mul(r, r)), q)
	}
}

// QuarterEllipseLength encloses the length of an LF4 edge, the quarter of
// the ellipse with semi-axes a = r/sin(θ/2) = r·√(1 + κ²) and b = r:
// r·∫₀^{π/2} √(1 + κ²·sin²φ) dφ. The integrand rises over the range, so its
// left and right Riemann sums over 64 equal sub-ranges are a lower and an
// upper bound, each evaluated over the enclosures of its own ends (§2).
func QuarterEllipseLength(r, kappa Interval) (Interval, error) {
	const n = 64
	k2 := proofbound.IntervalSquare(kappa)
	halfPi := proofbound.HalfPiInterval()
	f := make([]Interval, n+1)
	for j := range n + 1 {
		phi := scale(halfPi, int64(j), n)
		s, _, ok := proofbound.RadSinCosSpan(phi)
		if !ok {
			return Interval{}, fmt.Errorf(`%w: a loop fillet's ellipse length has no sine enclosure`, decaderr.ErrUnsupported)
		}
		v, ok := proofbound.SqrtInterval(proofbound.IntervalAdd(pointInt(1), proofbound.IntervalMul(k2, proofbound.IntervalSquare(s))))
		if !ok {
			return Interval{}, fmt.Errorf(`%w: a loop fillet's ellipse length has no root enclosure`, decaderr.ErrUnsupported)
		}
		f[j] = v
	}
	lower, upper := new(big.Rat), new(big.Rat)
	for j := range n {
		lower.Add(lower, f[j].Lo)
		upper.Add(upper, f[j+1].Hi)
	}
	step := scale(halfPi, 1, n)
	sum := proofbound.Interval(new(big.Rat).Mul(lower, step.Lo), new(big.Rat).Mul(upper, step.Hi))
	return proofbound.IntervalMul(r, sum), nil
}

// MeridianLength encloses the length of an LF5 or LF6 meridian, the quarter
// circle of radius r: r·π/2.
func MeridianLength(r Interval) Interval {
	return proofbound.IntervalMul(r, proofbound.HalfPiInterval())
}
