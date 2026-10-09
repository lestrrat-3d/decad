package filletband

import (
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// Extreme encloses the largest value of g·p over every patch of the band, p a
// point of the patch in F's frame and g = (gu, gv, gz) an exact direction in
// that frame (docs/loop-fillet-design.md §5.4). The band's ball centres run at
// the side level sideZ, r from ℓ into F's material, and the band climbs from
// sideZ toward F, along −m times F's normal.
//
// Each patch reads as a sinusoid K + A·cos φ + B·sin φ over its quarter
// φ ∈ [0, π/2], φ = 0 on the side contour and φ = π/2 on the cap contour,
// whose largest value is an endpoint or the interior stationary point
// (sinusoidMax). An LF1 patch's ruling at φ runs between its two end curves,
// so its end the sign of g along the walk picks gives the extreme; a torus
// patch's azimuth extreme is |g⊥| where g⊥'s direction lies in its window and
// the nearer window end otherwise (windowMax).
func Extreme(pieces []Piece, r, sideZ Interval, m int64, g [3]*big.Rat) (Interval, error) {
	mul, add := proofbound.IntervalMul, proofbound.IntervalAdd
	gzp := point(new(big.Rat).Mul(g[2], big.NewRat(-m, 1)))
	b := mul(r, gzp)
	gz := mul(point(g[2]), sideZ)
	var out Interval
	for i, p := range pieces {
		var e Interval
		switch p.Kind {
		case Cylinder:
			gt := add(mul(point(g[0]), p.Tau[0]), mul(point(g[1]), p.Tau[1]))
			gn := add(mul(point(g[0]), p.Nu[0]), mul(point(g[1]), p.Nu[1]))
			start, err := ratPoint(p.Start)
			if err != nil {
				return Interval{}, err
			}
			c0 := add(add(point(new(big.Rat).Add(new(big.Rat).Mul(g[0], start[0]), new(big.Rat).Mul(g[1], start[1]))), mul(r, gn)), gz)
			rgn := mul(r, gn)
			var cands []Interval
			if gt.Hi.Sign() >= 0 {
				// The ruling's far end, ℓ − κ₁·t along the walk.
				k := add(c0, mul(gt, proofbound.IntervalSub(p.Length, mul(p.Kappa1, r))))
				a := proofbound.IntervalSub(mul(mul(gt, p.Kappa1), r), rgn)
				cands = append(cands, sinusoidMax(k, a, b))
			}
			if gt.Lo.Sign() <= 0 {
				// The ruling's near end, κ₀·t along the walk.
				k := add(c0, mul(mul(gt, p.Kappa0), r))
				a := proofbound.IntervalSub(proofbound.IntervalNeg(mul(mul(gt, p.Kappa0), r)), rgn)
				cands = append(cands, sinusoidMax(k, a, b))
			}
			e = hullMax(cands)
		default:
			centre := p.Center
			if p.Kind == HornTorus {
				centre = p.Vertex
			}
			c, err := ratPoint(centre)
			if err != nil {
				return Interval{}, err
			}
			gc := add(point(new(big.Rat).Add(new(big.Rat).Mul(g[0], c[0]), new(big.Rat).Mul(g[1], c[1]))), gz)
			mx, err := windowMax(p, g)
			if err != nil {
				return Interval{}, err
			}
			rm := mul(r, mx)
			var k, a Interval
			switch p.Kind {
			case InnerTorus:
				k, a = add(gc, mul(proofbound.IntervalSub(p.Radius, r), mx)), rm
			case OuterTorus:
				k, a = add(gc, mul(add(p.Radius, r), mx)), proofbound.IntervalNeg(rm)
			default:
				k, a = add(gc, rm), proofbound.IntervalNeg(rm)
			}
			e = sinusoidMax(k, a, b)
		}
		if i == 0 {
			out = e
			continue
		}
		out = hullMax([]Interval{out, e})
	}
	if len(pieces) == 0 {
		return Interval{}, fmt.Errorf(`%w: a loop fillet band holds no patch`, decaderr.ErrDegenerate)
	}
	return out, nil
}

// hullMax encloses the largest of several enclosed values.
func hullMax(vs []Interval) Interval {
	lo, hi := vs[0].Lo, vs[0].Hi
	for _, v := range vs[1:] {
		if v.Lo.Cmp(lo) > 0 {
			lo = v.Lo
		}
		if v.Hi.Cmp(hi) > 0 {
			hi = v.Hi
		}
	}
	return proofbound.Interval(lo, hi)
}

// sinusoidMax encloses max over φ ∈ [0, π/2] of K + A·cos φ + B·sin φ. The
// value is at least the larger endpoint, K + A or K + B. It is at most
// K + max(A, B) where both coefficients are non-positive, and otherwise at
// most K + √(A⁺² + B⁺²), A⁺ and B⁺ the coefficients' positive parts, which it
// reaches at the interior stationary point where both are positive.
func sinusoidMax(k, a, b Interval) Interval {
	lo := ratMax(a.Lo, b.Lo)
	if a.Lo.Sign() > 0 && b.Lo.Sign() > 0 {
		if s, ok := proofbound.SqrtInterval(point(new(big.Rat).Add(new(big.Rat).Mul(a.Lo, a.Lo), new(big.Rat).Mul(b.Lo, b.Lo)))); ok {
			lo = s.Lo
		}
	}
	var hi *big.Rat
	if a.Hi.Sign() <= 0 && b.Hi.Sign() <= 0 {
		hi = ratMax(a.Hi, b.Hi)
	} else {
		ap, bp := ratMax(a.Hi, new(big.Rat)), ratMax(b.Hi, new(big.Rat))
		s, ok := proofbound.SqrtInterval(point(new(big.Rat).Add(new(big.Rat).Mul(ap, ap), new(big.Rat).Mul(bp, bp))))
		if !ok {
			hi = new(big.Rat).Add(ap, bp)
		} else {
			hi = s.Hi
		}
	}
	return proofbound.Interval(new(big.Rat).Add(k.Lo, lo), new(big.Rat).Add(k.Hi, hi))
}

func ratMax(a, b *big.Rat) *big.Rat {
	if a.Cmp(b) >= 0 {
		return a
	}
	return b
}

// windowMax encloses the largest value of (gu, gv)·ρ̂ over the unit radial
// directions ρ̂ of a torus patch's azimuth window: |g⊥| where g⊥'s own
// direction lies in the window, and otherwise the larger of the window's two
// ends. A whole turn's window holds every direction. Membership is decided
// exactly over the window's two exact radial vectors, lowest angle first.
func windowMax(p Piece, g [3]*big.Rat) (Interval, error) {
	gp := ratVec{g[0], g[1]}
	n2 := gp.dot(gp)
	if n2.Sign() == 0 {
		return pointInt(0), nil
	}
	norm, err := sqrtOf(n2)
	if err != nil {
		return Interval{}, err
	}
	if p.WholeTurn {
		return norm, nil
	}
	lo, hi := p.Window[0], p.Window[1]
	var in bool
	if lo.cross(hi).Sign() > 0 || (lo.cross(hi).Sign() == 0 && lo.dot(hi).Sign() < 0) {
		// A window of at most a half turn.
		in = lo.cross(gp).Sign() >= 0 && gp.cross(hi).Sign() >= 0
	} else {
		in = hi.cross(gp).Sign() <= 0 || gp.cross(lo).Sign() <= 0
	}
	if in {
		return norm, nil
	}
	ulo, err := unit(lo)
	if err != nil {
		return Interval{}, err
	}
	uhi, err := unit(hi)
	if err != nil {
		return Interval{}, err
	}
	gv := ratVec2(gp)
	dlo := proofbound.IntervalAdd(proofbound.IntervalMul(gv[0], ulo[0]), proofbound.IntervalMul(gv[1], ulo[1]))
	dhi := proofbound.IntervalAdd(proofbound.IntervalMul(gv[0], uhi[0]), proofbound.IntervalMul(gv[1], uhi[1]))
	return hullMax([]Interval{dlo, dhi}), nil
}
