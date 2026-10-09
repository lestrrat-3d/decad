package coil

import (
	"context"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// ArcAreaPieces is the number of sub-intervals docs/helix-design.md §11.1
// cuts one arc's wall integral into. The bracket's width is fourth order in
// the sub-interval, so 1024 pieces hold a round-wire spring's wall to a
// relative width near 1e-10.
const ArcAreaPieces = 1024

// arcGridShift is the dyadic grid every per-piece enclosure is rounded
// outward onto, which keeps the rationals summed over the pieces small. The
// grid's own gap, 2^-160 per end per piece, is far below the bracket's width.
const arcGridShift = 160

// ArcArea encloses the area the whole circular profile segment seg sweeps
// (docs/helix-design.md §11.1): Θ·∫√h dα over the arc, with α its natural
// angle about its centre and
//
//	h(α) = r²·ρ(α)² + k²·r²·B(α)²,  ρ(α) = ρ_c + r·A(α)
//
// where A = p·cos α + q·sin α and B = q·cos α − p·sin α, (p, q) = e_r, ρ_c
// is the centre's radial coordinate and k = pitch/2π. The area element of
// the screw sweep is |∂α × ∂θ| = √h, since ∂α = r·B·e_r + ζ'·n with
// ρ'² + ζ'² = r², and ∂θ = ρ·σe_t + k·n.
//
// Over each piece the integrand is expanded about h_m = g², g the float
// nearest √h at the piece's midpoint:
//
//	√h = g + (h − g²)/(2g) − (h − g²)²/(8g³) + R,  |R| ≤ |h − g²|³/(16·h_lo^{5/2})
//
// h and h² are trig polynomials of degree two and four, so the first three
// terms integrate exactly through A and B at the piece's ends (A' = B,
// B' = −A). |h'| = |a₁·B + 2(a₂ − a₃)·A·B| ≤ |a₁| + |a₂ − a₃|, so over the
// piece |h − g²| ≤ |h_mid − g²| + (|a₁| + |a₂ − a₃|)·Δ/2 and h stays at or
// above h_mid's lower end less that drift. ok is false when the segment is
// not circular, a station's trig cannot be enclosed, or h is not proven
// positive on a piece. ctx is polled once per piece.
func ArcArea(ctx context.Context, seg Segment, axis Axis, pitch, turns *big.Rat) (Iv, bool, error) {
	return arcArea(ctx, seg, axis, pitch, turns, ArcAreaPieces)
}

func arcArea(ctx context.Context, seg Segment, axis Axis, pitch, turns *big.Rat, pieces int) (Iv, bool, error) {
	ends, delta, ok := arcStations(seg, 2*pieces)
	delta = gridOut(delta)
	if !ok {
		return Iv{}, false, nil
	}
	r := seg.Radius
	var cu, cv *big.Rat
	switch s := seg.Record.(type) {
	case sectionrecord.ArcSeg:
		cu, cv = new(big.Rat).SetFloat64(s.Center.U), new(big.Rat).SetFloat64(s.Center.V)
	case sectionrecord.CircleSeg:
		cu, cv = new(big.Rat).SetFloat64(s.Center.U), new(big.Rat).SetFloat64(s.Center.V)
	default:
		return Iv{}, false, nil
	}
	rhoC, _ := axis.Coords(cu, cv)
	p, q := axis.Radial()
	k, ok := proofbound.IntervalQuo(Point(pitch), proofbound.TwoPiInterval())
	if !ok {
		return Iv{}, false, nil
	}
	r2 := proofbound.IntervalSquare(r)
	a0 := gridOut(proofbound.IntervalMul(r2, proofbound.IntervalSquare(rhoC)))
	a1 := gridOut(proofbound.IntervalScale(proofbound.IntervalMul(proofbound.IntervalMul(r2, r), rhoC), big.NewRat(2, 1)))
	a2 := gridOut(proofbound.IntervalSquare(r2))
	a3 := gridOut(proofbound.IntervalMul(proofbound.IntervalSquare(k), r2))
	drift := new(big.Rat).Add(proofbound.IntervalAbsUpper(a1), proofbound.IntervalAbsUpper(proofbound.IntervalSub(a2, a3)))
	drift.Mul(drift, delta.Hi)
	drift.Quo(drift, big.NewRat(2, 1))

	type ab struct{ a, b Iv }
	at := make([]ab, len(ends))
	for i, e := range ends {
		at[i] = ab{
			a: gridOut(proofbound.IntervalAdd(proofbound.IntervalMul(p, e[0]), proofbound.IntervalMul(q, e[1]))),
			b: gridOut(proofbound.IntervalSub(proofbound.IntervalMul(q, e[0]), proofbound.IntervalMul(p, e[1]))),
		}
	}
	hOf := func(x ab) Iv {
		h := proofbound.IntervalAdd(a0, proofbound.IntervalMul(a1, x.a))
		h = proofbound.IntervalAdd(h, proofbound.IntervalMul(a2, proofbound.IntervalSquare(x.a)))
		return proofbound.IntervalAdd(h, proofbound.IntervalMul(a3, proofbound.IntervalSquare(x.b)))
	}
	two := big.NewRat(2, 1)
	half := big.NewRat(1, 2)
	sum := Point(new(big.Rat))
	for i := range pieces {
		if err := ctx.Err(); err != nil {
			return Iv{}, false, err
		}
		lo, mid, hi := at[2*i], at[2*i+1], at[2*i+2]
		hMid := hOf(mid)
		hm, _ := Mid(hMid).Float64()
		if !(hm > 0) || proofbound.IsNonFinite(hm) {
			return Iv{}, false, nil
		}
		g := new(big.Rat).SetFloat64(math.Sqrt(hm))
		g2 := new(big.Rat).Mul(g, g)

		// Exact integrals of A, A², B², A³, A·B², A⁴, B⁴ and A²·B² over the
		// piece, from A = cos φ, B = −sin φ.
		diff := func(f func(x ab) Iv) Iv { return proofbound.IntervalSub(f(hi), f(lo)) }
		bb := diff(func(x ab) Iv { return x.b })
		b3 := diff(func(x ab) Iv { return proofbound.IntervalMul(proofbound.IntervalSquare(x.b), x.b) })
		abd := diff(func(x ab) Iv { return proofbound.IntervalMul(x.a, x.b) })
		ab4 := diff(func(x ab) Iv {
			return proofbound.IntervalMul(proofbound.IntervalMul(x.a, x.b),
				proofbound.IntervalSub(proofbound.IntervalSquare(x.a), proofbound.IntervalSquare(x.b)))
		})
		third := big.NewRat(1, 3)
		eighth := big.NewRat(1, 8)
		iA := proofbound.IntervalNeg(bb)
		iA2 := proofbound.IntervalSub(proofbound.IntervalScale(delta, half), proofbound.IntervalScale(abd, half))
		iB2 := proofbound.IntervalAdd(proofbound.IntervalScale(delta, half), proofbound.IntervalScale(abd, half))
		iA3 := proofbound.IntervalAdd(iA, proofbound.IntervalScale(b3, third))
		iAB2 := proofbound.IntervalNeg(proofbound.IntervalScale(b3, third))
		threeEighths := proofbound.IntervalScale(delta, big.NewRat(3, 8))
		iA4 := proofbound.IntervalSub(proofbound.IntervalSub(threeEighths, proofbound.IntervalScale(abd, half)), proofbound.IntervalScale(ab4, eighth))
		iB4 := proofbound.IntervalSub(proofbound.IntervalAdd(threeEighths, proofbound.IntervalScale(abd, half)), proofbound.IntervalScale(ab4, eighth))
		iA2B2 := proofbound.IntervalAdd(proofbound.IntervalScale(delta, eighth), proofbound.IntervalScale(ab4, eighth))

		mul := proofbound.IntervalMul
		add := proofbound.IntervalAdd
		ih := add(add(mul(a0, delta), mul(a1, iA)), add(mul(a2, iA2), mul(a3, iB2)))
		ih2 := add(mul(proofbound.IntervalSquare(a0), delta), mul(proofbound.IntervalSquare(a1), iA2))
		ih2 = add(ih2, add(mul(proofbound.IntervalSquare(a2), iA4), mul(proofbound.IntervalSquare(a3), iB4)))
		cross := add(add(mul(mul(a0, a1), iA), mul(mul(a0, a2), iA2)), add(mul(mul(a0, a3), iB2), mul(mul(a1, a2), iA3)))
		cross = add(cross, add(mul(mul(a1, a3), iAB2), mul(mul(a2, a3), iA2B2)))
		ih2 = add(ih2, proofbound.IntervalScale(cross, two))

		// T = Δ·g + (∫h − Δ·g²)/(2g) − (∫h² − 2g²·∫h + g⁴·Δ)/(8g³)
		lin := proofbound.IntervalScale(delta, g)
		first := proofbound.IntervalScale(proofbound.IntervalSub(ih, proofbound.IntervalScale(delta, g2)), new(big.Rat).Quo(half, g))
		sq := proofbound.IntervalSub(ih2, proofbound.IntervalScale(ih, new(big.Rat).Mul(two, g2)))
		sq = proofbound.IntervalAdd(sq, proofbound.IntervalScale(delta, new(big.Rat).Mul(g2, g2)))
		g3 := new(big.Rat).Mul(g2, g)
		second := proofbound.IntervalScale(sq, new(big.Rat).Quo(eighth, g3))
		piece := proofbound.IntervalSub(add(lin, first), second)

		// The remainder over the piece.
		spread := proofbound.IntervalAbsUpper(proofbound.IntervalSub(hMid, Point(g2)))
		dev := new(big.Rat).Add(spread, drift)
		hLo := new(big.Rat).Sub(hMid.Lo, drift)
		if hLo.Sign() <= 0 {
			return Iv{}, false, nil
		}
		root, _ := proofbound.SqrtFixed(hLo)
		if root.Lo.Sign() <= 0 {
			return Iv{}, false, nil
		}
		den := new(big.Rat).Mul(hLo, hLo)
		den.Mul(den, root.Lo)
		den.Mul(den, big.NewRat(16, 1))
		rem := new(big.Rat).Mul(dev, dev)
		rem.Mul(rem, dev)
		rem.Mul(rem, delta.Hi)
		rem.Quo(rem, den)
		piece = proofbound.IntervalWiden(piece, rem)
		sum = add(sum, gridOut(piece))
	}
	theta := Theta(turns)
	return proofbound.IntervalMul(theta, sum), true, nil
}

// arcStations returns cos α and sin α at n + 1 equally spaced points of the
// segment's natural counter-clockwise sweep, and the enclosure of one gap
// between two of them, doubled: the width of one piece of n/2. A circle's
// stations are exact turn fractions read by TurnSinCos; an arc's turn its
// Start direction step by step through the enclosed sweep's n-th part.
func arcStations(seg Segment, n int) ([][2]Iv, Iv, bool) {
	out := make([][2]Iv, n+1)
	switch s := seg.Record.(type) {
	case sectionrecord.CircleSeg:
		for i := range n + 1 {
			sin, cos := TurnSinCos(big.NewRat(int64(i), int64(n)))
			out[i] = [2]Iv{gridOut(cos), gridOut(sin)}
		}
		return out, proofbound.IntervalScale(proofbound.TwoPiInterval(), big.NewRat(2, int64(n))), true
	case sectionrecord.ArcSeg:
		dx := new(big.Rat).Sub(new(big.Rat).SetFloat64(s.Start.U), new(big.Rat).SetFloat64(s.Center.U))
		dy := new(big.Rat).Sub(new(big.Rat).SetFloat64(s.Start.V), new(big.Rat).SetFloat64(s.Center.V))
		c0, ok1 := proofbound.IntervalQuo(Point(dx), seg.Radius)
		s0, ok2 := proofbound.IntervalQuo(Point(dy), seg.Radius)
		if !ok1 || !ok2 {
			return nil, Iv{}, false
		}
		c0, s0 = gridOut(c0), gridOut(s0)
		// Station i is the Start direction turned by i steps of sw/n: one
		// enclosed sine and cosine of the step, then a rotation per station,
		// each product's interval carried and rounded outward.
		sw := gridOut(seg.Sweep)
		sinStep, cosStep, ok := proofbound.RadSinCosSpan(proofbound.IntervalScale(sw, big.NewRat(1, int64(n))))
		if !ok {
			return nil, Iv{}, false
		}
		sinStep, cosStep = gridOut(sinStep), gridOut(cosStep)
		ca, sa := c0, s0
		for i := range n + 1 {
			out[i] = [2]Iv{ca, sa}
			ca, sa = gridOut(proofbound.IntervalSub(proofbound.IntervalMul(ca, cosStep), proofbound.IntervalMul(sa, sinStep))),
				gridOut(proofbound.IntervalAdd(proofbound.IntervalMul(sa, cosStep), proofbound.IntervalMul(ca, sinStep)))
		}
		return out, proofbound.IntervalScale(sw, big.NewRat(2, int64(n))), true
	default:
		return nil, Iv{}, false
	}
}

// gridOut rounds an interval outward onto the 2^-arcGridShift grid.
func gridOut(x Iv) Iv {
	lo := proofbound.RatFloorGrid(x.Lo, arcGridShift)
	hi := new(big.Rat).Neg(proofbound.RatFloorGrid(new(big.Rat).Neg(x.Hi), arcGridShift))
	return proofbound.Interval(lo, hi)
}
