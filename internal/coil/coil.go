// Package coil holds the pure arithmetic of docs/helix-design.md: the
// station fractions and their trig, a profile vertex's axis coordinates, the
// region's axis-frame moments, the four readings' closed forms and the cell
// departure terms. Every closed form reads exact rationals or rational
// intervals and returns an enclosure; Held, Mul and Add carry a station
// point's float evaluation beside a proven bound on its distance from that
// enclosure.
package coil

import (
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// Iv is the rational interval every reading here is carried in.
type Iv = proofbound.RatInterval

// Point is the exact interval of one rational.
func Point(r *big.Rat) Iv { return proofbound.PointInterval(r) }

// Measured is the interval held ± bound of one float reading with its proven
// bound. A nil result means the reading is not finite.
func Measured(held, bound float64) (Iv, bool) {
	h, b := proofarith.FloatRat(held), proofarith.FloatRat(bound)
	if h == nil || b == nil {
		return Iv{}, false
	}
	b.Abs(b)
	return proofbound.Interval(new(big.Rat).Sub(h, b), new(big.Rat).Add(h, b)), true
}

// Axis is the resolved in-plane axis (Table CP): a point (AU, AV) on it and
// its direction (DU, DV), each an interval holding the true value, and Side,
// the sign that puts the profile on the positive side of e_r = Side·(−DV, DU).
type Axis struct {
	AU, AV, DU, DV Iv
	Side           int
}

// Coords returns a plane-local point's axis coordinates: ρ, its signed
// distance from the axis along e_r, and ζ, its coordinate along the axis
// direction from the anchor.
func (a Axis) Coords(u, v *big.Rat) (Iv, Iv) {
	du := proofbound.IntervalSub(Point(u), a.AU)
	dv := proofbound.IntervalSub(Point(v), a.AV)
	rho := proofbound.IntervalSub(proofbound.IntervalMul(dv, a.DU), proofbound.IntervalMul(du, a.DV))
	if a.Side < 0 {
		rho = proofbound.IntervalNeg(rho)
	}
	zeta := proofbound.IntervalAdd(proofbound.IntervalMul(du, a.DU), proofbound.IntervalMul(dv, a.DV))
	return rho, zeta
}

// Radial returns e_r's two plane-local components.
func (a Axis) Radial() (Iv, Iv) {
	if a.Side < 0 {
		return a.DV, proofbound.IntervalNeg(a.DU)
	}
	return proofbound.IntervalNeg(a.DV), a.DU
}

// StationCount is ⌈turns · perTurn⌉ (§5.2). ok is false when the count does
// not fit an int.
func StationCount(turns *big.Rat, perTurn int64) (int64, bool) {
	scaled := new(big.Rat).Mul(turns, big.NewRat(perTurn, 1))
	q, r := new(big.Int).QuoRem(scaled.Num(), scaled.Denom(), new(big.Int))
	if r.Sign() > 0 {
		q.Add(q, big.NewInt(1))
	}
	if !q.IsInt64() {
		return 0, false
	}
	return q.Int64(), true
}

// Fraction is station j's exact turn fraction turns·j/n.
func Fraction(turns *big.Rat, j, n int64) *big.Rat {
	return new(big.Rat).Mul(turns, big.NewRat(j, n))
}

// TurnSinCos encloses sin(2πt) and cos(2πt). A multiple of a quarter turn is
// answered exactly; every other turn reads proofbound.TurnSinCosInterval,
// whose series margin keeps even an octant boundary a few grid steps wide.
func TurnSinCos(t *big.Rat) (Iv, Iv) {
	four := new(big.Rat).Mul(t, big.NewRat(4, 1))
	if four.IsInt() {
		q := new(big.Int).Mod(four.Num(), big.NewInt(4)).Int64()
		zero, one, neg := new(big.Rat), big.NewRat(1, 1), big.NewRat(-1, 1)
		switch q {
		case 0:
			return Point(zero), Point(one)
		case 1:
			return Point(one), Point(zero)
		case 2:
			return Point(zero), Point(neg)
		default:
			return Point(neg), Point(zero)
		}
	}
	return proofbound.TurnSinCosInterval(t)
}

// Moments are the region's axis-frame integrals of Table CM: A_Ω = ∫dA,
// Q = ∫ρ dA, I = ∫ρ² dA and M = ∫ρζ dA.
type Moments struct {
	Area, Q, I, M Iv
}

// RegionMoments integrates a polygon with holes given every vertex's axis
// coordinates and its exact plane-local area. loops lists vertex indices per
// loop, the outer loop counter-clockwise and every hole clockwise in the
// plane (docs/sketch-seam-design.md), so each loop's shoelace sum nets a
// hole out with no special case. The (ζ, ρ) basis has determinant Side
// against (u, v), so the sums are scaled by Side to come out positive.
func RegionMoments(rho, zeta []Iv, loops [][]int, area *big.Rat, side int) Moments {
	q := Point(new(big.Rat))
	i := Point(new(big.Rat))
	m := Point(new(big.Rat))
	two := big.NewRat(2, 1)
	for _, idx := range loops {
		n := len(idx)
		for j := range n {
			a, b := idx[j], idx[(j+1)%n]
			x0, y0, x1, y1 := zeta[a], rho[a], zeta[b], rho[b]
			c := proofbound.IntervalSub(proofbound.IntervalMul(x0, y1), proofbound.IntervalMul(x1, y0))
			q = proofbound.IntervalAdd(q, proofbound.IntervalMul(proofbound.IntervalAdd(y0, y1), c))
			sq := proofbound.IntervalAdd(proofbound.IntervalAdd(proofbound.IntervalSquare(y0), proofbound.IntervalMul(y0, y1)),
				proofbound.IntervalSquare(y1))
			i = proofbound.IntervalAdd(i, proofbound.IntervalMul(sq, c))
			mixed := proofbound.IntervalAdd(
				proofbound.IntervalAdd(proofbound.IntervalMul(x0, y1), proofbound.IntervalMul(x1, y0)),
				proofbound.IntervalScale(proofbound.IntervalAdd(proofbound.IntervalMul(x0, y0), proofbound.IntervalMul(x1, y1)), two),
			)
			m = proofbound.IntervalAdd(m, proofbound.IntervalMul(mixed, c))
		}
	}
	s := big.NewRat(int64(side), 1)
	return Moments{
		Area: Point(area),
		Q:    proofbound.IntervalScale(q, new(big.Rat).Quo(s, big.NewRat(6, 1))),
		I:    proofbound.IntervalScale(i, new(big.Rat).Quo(s, big.NewRat(12, 1))),
		M:    proofbound.IntervalScale(m, new(big.Rat).Quo(s, big.NewRat(24, 1))),
	}
}

// PolygonArea is the exact shoelace area of a polygon with holes in its own
// plane-local coordinates.
func PolygonArea(u, v []*big.Rat, loops [][]int) *big.Rat {
	sum := new(big.Rat)
	for _, idx := range loops {
		n := len(idx)
		for j := range n {
			a, b := idx[j], idx[(j+1)%n]
			sum.Add(sum, new(big.Rat).Mul(u[a], v[b]))
			sum.Sub(sum, new(big.Rat).Mul(u[b], v[a]))
		}
	}
	return sum.Quo(sum, big.NewRat(2, 1))
}

// Theta encloses the total angle Θ = 2π·turns.
func Theta(turns *big.Rat) Iv {
	return proofbound.IntervalScale(proofbound.TwoPiInterval(), turns)
}

// Volume encloses Θ·Q (CP4, Pappus).
func Volume(m Moments, turns *big.Rat) Iv {
	return proofbound.IntervalMul(Theta(turns), m.Q)
}

// CentroidCoefficients encloses the centroid's three axis-frame
// coefficients: the solid's centroid is C + R·e_r + T·e_t + N·n, with
// R = (I/Q)·sin Θ/Θ, T = σ·(I/Q)·(1 − cos Θ)/Θ and N = M/Q + pitch·turns/2.
// σ is +1 for a right-hand coil. ok is false when Q's enclosure reaches zero.
func CentroidCoefficients(m Moments, pitch, turns *big.Rat, sigma int) (Iv, Iv, Iv, bool) {
	ratio, ok := proofbound.IntervalQuo(m.I, m.Q)
	if !ok {
		return Iv{}, Iv{}, Iv{}, false
	}
	mean, ok := proofbound.IntervalQuo(m.M, m.Q)
	if !ok {
		return Iv{}, Iv{}, Iv{}, false
	}
	sinT, cosT := TurnSinCos(turns)
	theta := Theta(turns)
	sinOver, ok := proofbound.IntervalQuo(sinT, theta)
	if !ok {
		return Iv{}, Iv{}, Iv{}, false
	}
	oneMinus := proofbound.IntervalSub(Point(big.NewRat(1, 1)), cosT)
	cosOver, ok := proofbound.IntervalQuo(oneMinus, theta)
	if !ok {
		return Iv{}, Iv{}, Iv{}, false
	}
	r := proofbound.IntervalMul(ratio, sinOver)
	t := proofbound.IntervalMul(ratio, cosOver)
	if sigma < 0 {
		t = proofbound.IntervalNeg(t)
	}
	slide := new(big.Rat).Mul(pitch, turns)
	slide.Quo(slide, big.NewRat(2, 1))
	n := proofbound.IntervalAdd(mean, Point(slide))
	return r, t, n, true
}

// abs is |x| over an interval that does not straddle zero.
func abs(x Iv) Iv {
	if x.Hi.Sign() <= 0 {
		return proofbound.IntervalNeg(x)
	}
	return x
}

// SegmentArea encloses the area one LineSeg sweeps (Table CM's Area row):
// Θ·∫₀¹ sqrt(L²·ρ(λ)² + k²·Δρ²) dλ with ρ(λ) = ρ_v + λ·Δρ and k = pitch/2π.
//
// Where Δρ's enclosure excludes zero, the substitution u = L·ρ gives
// Θ/(L·|Δρ|)·[F(u_hi) − F(u_lo)] + Θ·k²·|Δρ|/(2L)·[asinh(u_hi/m) −
// asinh(u_lo/m)] with m = k·|Δρ| and F(u) = (u/2)·sqrt(u² + m²). Where it
// does not — a segment parallel to the axis, exactly or within the axis
// frame's own rounding — the integrand lies between L·ρ and
// L·ρ + m²/(2·L·ρ_min), which encloses the area between Θ·L·ρ̄ and that plus
// Θ·m²/(2·L·ρ_min); at an exact Δρ = 0 the term vanishes and the band reads
// Θ·L·ρ exactly. ok is false when L or ρ_min is not proven positive.
func SegmentArea(rhoV, zetaV, rhoW, zetaW Iv, pitch, turns *big.Rat) (Iv, bool) {
	theta := Theta(turns)
	dr := proofbound.IntervalSub(rhoW, rhoV)
	dz := proofbound.IntervalSub(zetaW, zetaV)
	l2 := proofbound.IntervalAdd(proofbound.IntervalSquare(dr), proofbound.IntervalSquare(dz))
	if l2.Lo.Sign() <= 0 {
		return Iv{}, false
	}
	l, ok := proofbound.SqrtInterval(l2)
	if !ok || l.Lo.Sign() <= 0 {
		return Iv{}, false
	}
	// k = pitch/2π, enclosed through π's own enclosure.
	k, ok := proofbound.IntervalQuo(Point(pitch), proofbound.TwoPiInterval())
	if !ok {
		return Iv{}, false
	}
	two := big.NewRat(2, 1)
	half := big.NewRat(1, 2)
	if dr.Lo.Sign() <= 0 && dr.Hi.Sign() >= 0 {
		rhoMin := rhoV.Lo
		if rhoW.Lo.Cmp(rhoMin) < 0 {
			rhoMin = rhoW.Lo
		}
		if rhoMin.Sign() <= 0 {
			return Iv{}, false
		}
		mean := proofbound.IntervalScale(proofbound.IntervalAdd(rhoV, rhoW), half)
		base := proofbound.IntervalMul(theta, proofbound.IntervalMul(l, mean))
		if dr.Lo.Sign() == 0 && dr.Hi.Sign() == 0 {
			return base, true
		}
		mUpper := proofbound.IntervalMul(k, Point(proofbound.IntervalAbsUpper(dr)))
		m2 := proofbound.IntervalSquare(mUpper)
		den := new(big.Rat).Mul(two, new(big.Rat).Mul(l.Lo, rhoMin))
		extra := new(big.Rat).Quo(new(big.Rat).Mul(theta.Hi, m2.Hi), den)
		return proofbound.Interval(base.Lo, new(big.Rat).Add(base.Hi, extra)), true
	}
	absDr := abs(dr)
	rhoLo, rhoHi := rhoV, rhoW
	if dr.Hi.Sign() < 0 {
		rhoLo, rhoHi = rhoW, rhoV
	}
	if rhoLo.Lo.Sign() <= 0 {
		return Iv{}, false
	}
	m := proofbound.IntervalMul(k, absDr)
	m2 := proofbound.IntervalSquare(m)
	f := func(rho Iv) (Iv, Iv, bool) {
		u := proofbound.IntervalMul(l, rho)
		root, ok := proofbound.SqrtInterval(proofbound.IntervalAdd(proofbound.IntervalSquare(u), m2))
		if !ok {
			return Iv{}, Iv{}, false
		}
		ratio, ok := proofbound.IntervalQuo(u, m)
		if !ok {
			return Iv{}, Iv{}, false
		}
		return proofbound.IntervalScale(proofbound.IntervalMul(u, root), half), proofbound.AsinhInterval(ratio), true
	}
	fLo, aLo, ok := f(rhoLo)
	if !ok {
		return Iv{}, false
	}
	fHi, aHi, ok := f(rhoHi)
	if !ok {
		return Iv{}, false
	}
	first, ok := proofbound.IntervalQuo(proofbound.IntervalMul(theta, proofbound.IntervalSub(fHi, fLo)),
		proofbound.IntervalMul(l, absDr))
	if !ok {
		return Iv{}, false
	}
	coef, ok := proofbound.IntervalQuo(
		proofbound.IntervalMul(theta, proofbound.IntervalMul(proofbound.IntervalSquare(k), absDr)),
		proofbound.IntervalScale(l, two),
	)
	if !ok {
		return Iv{}, false
	}
	second := proofbound.IntervalMul(coef, proofbound.IntervalSub(aHi, aLo))
	return proofbound.IntervalAdd(first, second), true
}

// HelixLength encloses the length Θ·sqrt(ρ² + k²) of the helix a profile
// vertex at radius ρ traces, written turns·sqrt((2πρ)² + pitch²).
func HelixLength(rho Iv, pitch, turns *big.Rat) (Iv, bool) {
	circ := proofbound.IntervalMul(proofbound.TwoPiInterval(), rho)
	sq := proofbound.IntervalAdd(proofbound.IntervalSquare(circ), Point(new(big.Rat).Mul(pitch, pitch)))
	root, ok := proofbound.SqrtInterval(sq)
	if !ok {
		return Iv{}, false
	}
	return proofbound.IntervalScale(root, turns), true
}

// SagUpper bounds how far a helix arc of radius at most rhoMax over one
// station step of dt turns sits from its chord at the matching parameter, the
// sag leg of CellDepartureUpper:
// the arc's circular part has curvature vector of length ρ, so linear
// interpolation over Δθ = 2π·dt departs by at most ρ·Δθ²/8 = ρ·π²·dt²/2,
// read with π's upper end. The slide is linear in θ and departs by nothing.
func SagUpper(rhoMax, dt *big.Rat) *big.Rat {
	pi := proofbound.PiUpper
	out := new(big.Rat).Mul(pi, pi)
	out.Mul(out, new(big.Rat).Mul(dt, dt))
	out.Mul(out, rhoMax)
	return out.Quo(out, big.NewRat(2, 1))
}

// CellDepartureUpper bounds docs/helix-design.md §5.4's analytic departure of
// one wall cell: profile segment v → w, at radii rhoV and rhoW, over one
// station step of dt turns at the given pitch. Every point of the two
// triangles on the cell's four TRUE corners lies within the returned distance
// of the true helicoidal cell under §5.4's shifted correspondence, and every
// point of that true cell is reached by it. The sum is
//
//	sag   = ρ_max·h²/2
//	twist = |Δρ|·h/2·((1 − c) + c·(h + k/ρ_min))
//	shift = c²·Δρ²·h²/(8·ρ_min)
//
// with h = π·dt, k = pitch/2π and c = min(1, ρ_min/|Δρ|), each read at the
// end of its interval that makes the sum larger. ok is false when ρ_min is
// not proven positive.
func CellDepartureUpper(rhoV, rhoW Iv, pitch, dt *big.Rat) (*big.Rat, bool) {
	rhoMin := rhoV.Lo
	if rhoW.Lo.Cmp(rhoMin) < 0 {
		rhoMin = rhoW.Lo
	}
	if rhoMin.Sign() <= 0 {
		return nil, false
	}
	rhoMax := rhoV.Hi
	if rhoW.Hi.Cmp(rhoMax) > 0 {
		rhoMax = rhoW.Hi
	}
	out := SagUpper(rhoMax, dt)
	dr := proofbound.IntervalAbsUpper(proofbound.IntervalSub(rhoW, rhoV))
	if dr.Sign() == 0 {
		return out, true
	}
	h := new(big.Rat).Mul(proofbound.PiUpper, dt)
	k := new(big.Rat).Quo(pitch, new(big.Rat).Mul(big.NewRat(2, 1), proofbound.PiLower))
	one := big.NewRat(1, 1)
	c := new(big.Rat).Quo(rhoMin, dr)
	if c.Cmp(one) > 0 {
		c = one
	}
	// twist = |Δρ|·h/2·((1 − c) + c·(h + k/ρ_min))
	inner := new(big.Rat).Add(h, new(big.Rat).Quo(k, rhoMin))
	inner.Mul(inner, c)
	inner.Add(inner, new(big.Rat).Sub(one, c))
	twist := new(big.Rat).Mul(dr, h)
	twist.Mul(twist, inner)
	twist.Quo(twist, big.NewRat(2, 1))
	// shift = c²·Δρ²·h²/(8·ρ_min)
	cdh := new(big.Rat).Mul(c, dr)
	cdh.Mul(cdh, h)
	shift := new(big.Rat).Mul(cdh, cdh)
	shift.Quo(shift, new(big.Rat).Mul(big.NewRat(8, 1), rhoMin))
	out.Add(out, twist)
	return out.Add(out, shift), true
}

// CellProof is one wall cell's plane-coordinate mesh-proof terms
// (docs/helix-design.md §8.1, §8.2), each an upper bound rounded up. Every
// cell of one segment shares them: they depend on the segment's radii and
// run and on the station step alone.
type CellProof struct {
	// Ruling bounds the segment's length L, the rulings' length.
	Ruling float64
	// Helix bounds the arc length of one station step at the segment's
	// outer radius, 2h·sqrt(ρ_max² + k²); every chord of the cell and the
	// true cell's s-derivative are at most this long.
	Helix float64
	// Swept bounds the volume the homotopy from the triangles on the cell's
	// true corners to the true cell under §5.4's shifted correspondence
	// sweeps: CellDepartureUpper times the longest λ- and s-derivatives any
	// surface of that homotopy has.
	Swept float64
	// Density bounds ∫|J_S − J_B|, the area-density gap between the true cell
	// and the bilinear patch through its true corners at matched parameters:
	// 2·sag·Helix + Ruling·2ρ_max·h·(h + h²/6).
	Density float64
}

// CellProofUpper evaluates CellProof for one cell of segment v → w over a
// station step of dt turns. ok is false when a radius is not proven
// positive or the segment has no proven length.
//
// Swept's two derivative bounds come from H = S(λ, θ(s) + ε) with
// ε = c·m·2Δρ·sin h/ρ(λ) and q = c·|Δρ|/ρ_min ≤ 1: |∂_s ε| ≤ 2h·q, so
// |∂_s H| ≤ Helix·(1 + q); |∂_λ ε| ≤ 2h·q·(1 + |Δρ|/(4ρ_min)), so
// |∂_λ H| ≤ Ruling + Helix·q·(1 + |Δρ|/(4ρ_min)). The triangles' own
// derivatives are a ruling and a chord, inside both.
func CellProofUpper(rhoV, zetaV, rhoW, zetaW Iv, pitch, dt *big.Rat) (CellProof, bool) {
	dep, ok := CellDepartureUpper(rhoV, rhoW, pitch, dt)
	if !ok {
		return CellProof{}, false
	}
	rhoMin := rhoV.Lo
	if rhoW.Lo.Cmp(rhoMin) < 0 {
		rhoMin = rhoW.Lo
	}
	rhoMax := rhoV.Hi
	if rhoW.Hi.Cmp(rhoMax) > 0 {
		rhoMax = rhoW.Hi
	}
	dr := proofbound.IntervalSub(rhoW, rhoV)
	dz := proofbound.IntervalSub(zetaW, zetaV)
	l, ok := proofbound.SqrtInterval(proofbound.IntervalAdd(proofbound.IntervalSquare(dr), proofbound.IntervalSquare(dz)))
	if !ok {
		return CellProof{}, false
	}
	h := new(big.Rat).Mul(proofbound.PiUpper, dt)
	k := new(big.Rat).Quo(pitch, new(big.Rat).Mul(big.NewRat(2, 1), proofbound.PiLower))
	rate, ok := proofbound.SqrtInterval(Point(new(big.Rat).Add(new(big.Rat).Mul(rhoMax, rhoMax), new(big.Rat).Mul(k, k))))
	if !ok {
		return CellProof{}, false
	}
	one := big.NewRat(1, 1)
	helix := new(big.Rat).Mul(big.NewRat(2, 1), h)
	helix.Mul(helix, rate.Hi)

	// q = c·|Δρ|/ρ_min with c = min(1, ρ_min/|Δρ|), CellDepartureUpper's own c.
	drAbs := proofbound.IntervalAbsUpper(dr)
	q := new(big.Rat).Quo(drAbs, rhoMin)
	if q.Cmp(one) > 0 {
		q = one
	}
	alongS := new(big.Rat).Add(one, q)
	alongS.Mul(alongS, helix)
	alongL := new(big.Rat).Quo(drAbs, new(big.Rat).Mul(big.NewRat(4, 1), rhoMin))
	alongL.Add(alongL, one)
	alongL.Mul(alongL, q)
	alongL.Mul(alongL, helix)
	alongL.Add(alongL, l.Hi)
	swept := new(big.Rat).Mul(dep, alongL)
	swept.Mul(swept, alongS)

	// 2·sag·Helix + L·2ρ_max·h·(h + h²/6)
	density := new(big.Rat).Mul(big.NewRat(2, 1), SagUpper(rhoMax, dt))
	density.Mul(density, helix)
	tangent := new(big.Rat).Add(h, new(big.Rat).Quo(new(big.Rat).Mul(h, h), big.NewRat(6, 1)))
	tangent.Mul(tangent, h)
	tangent.Mul(tangent, rhoMax)
	tangent.Mul(tangent, big.NewRat(2, 1))
	tangent.Mul(tangent, l.Hi)
	density.Add(density, tangent)
	return CellProof{
		Ruling:  proofbound.RatFloatUp(l.Hi),
		Helix:   proofbound.RatFloatUp(helix),
		Swept:   proofbound.RatFloatUp(swept),
		Density: proofbound.RatFloatUp(density),
	}, true
}

// Held is the float nearest an interval's midpoint, carried with the
// outward distance from it to the interval's far end: the bounded float every
// point of the interval lies within. ok is false when either is not finite.
func Held(x Iv) (proofbound.BoundedScalar, bool) {
	held, _ := Mid(x).Float64()
	if proofbound.IsNonFinite(held) {
		return proofbound.BoundedScalar{}, false
	}
	bound := proofbound.IntervalFloatError(x, held)
	if proofbound.IsNonFinite(bound) {
		return proofbound.BoundedScalar{}, false
	}
	return proofbound.MeasuredScalar(held, bound), true
}

// Mul is a·b over bounded floats: the held product, rounded once by an
// explicit conversion so no step fuses it into a later addition, and a bound
// that charges both operands' bounds and that product's own exact rounding.
func Mul(a, b proofbound.BoundedScalar) proofbound.BoundedScalar {
	value := float64(a.Value * b.Value)
	return proofbound.MeasuredScalar(value, proofbound.AbsSumUpper(
		proofbound.ProductUpper(math.Abs(a.Value), b.Bound),
		proofbound.ProductUpper(math.Abs(b.Value), a.Bound),
		proofbound.ProductUpper(a.Bound, b.Bound),
		proofarith.MulRoundError(a.Value, b.Value, value),
	))
}

// Add is a + b over bounded floats, rounded once by an explicit conversion,
// charging both bounds and the sum's own exact rounding.
func Add(a, b proofbound.BoundedScalar) proofbound.BoundedScalar {
	value := float64(a.Value + b.Value)
	return proofbound.MeasuredScalar(value, proofbound.AbsSumUpper(a.Bound, b.Bound, proofarith.AddRoundError(a.Value, b.Value, value)))
}

// Mid is an interval's midpoint.
func Mid(x Iv) *big.Rat {
	m := new(big.Rat).Add(x.Lo, x.Hi)
	return m.Quo(m, big.NewRat(2, 1))
}
