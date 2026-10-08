package capband_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/capband"
	"github.com/stretchr/testify/require"
)

// gaussLegendre returns the n-point Gauss-Legendre nodes and weights on [0, 1].
func gaussLegendre(n int) ([]float64, []float64) {
	legendre := func(z float64) (float64, float64) {
		p1, p2 := 1.0, 0.0
		for j := 1; j <= n; j++ {
			p1, p2 = ((2*float64(j)-1)*z*p1-(float64(j)-1)*p2)/float64(j), p1
		}
		return p1, float64(n) * (z*p1 - p2) / (z*z - 1)
	}
	x := make([]float64, n)
	w := make([]float64, n)
	for i := range n {
		z := math.Cos(math.Pi * (float64(i) + 0.75) / (float64(n) + 0.5))
		for range 100 {
			p, dp := legendre(z)
			next := z - p/dp
			if math.Abs(next-z) < 1e-16 {
				z = next
				break
			}
			z = next
		}
		_, dp := legendre(z)
		x[i] = (1 - z) / 2
		w[i] = 1 / ((1 - z*z) * dp * dp)
	}
	return x, w
}

// ruledPatchArea integrates the area of the ruled patch a Cone patch stands
// for: rulings from the side arc (SideRadius over Th0..Th1 at SideZ) to the cap
// arc (CapRadius over CapTh0..CapTh1 at CapZ), both angles linear in one
// parameter. Tensor Gauss-Legendre on a smooth integrand converges to float64
// roundoff long before n = 64.
func ruledPatchArea(g capband.Patch, n int) float64 {
	xs, ws := gaussLegendre(n)
	as, ac := g.Th1-g.Th0, g.CapTh1-g.CapTh0
	r0, r1, h := g.SideRadius, g.CapRadius, g.CapZ-g.SideZ
	total := 0.0
	for i, u := range xs {
		ts, tc := g.Th0+u*as, g.CapTh0+u*ac
		sx, sy := r0*math.Cos(ts), r0*math.Sin(ts)
		cx, cy := r1*math.Cos(tc), r1*math.Sin(tc)
		dx, dy := cx-sx, cy-sy
		dsx, dsy := -as*r0*math.Sin(ts), as*r0*math.Cos(ts)
		dcx, dcy := -ac*r1*math.Sin(tc), ac*r1*math.Cos(tc)
		for j, t := range xs {
			pu, pv := (1-t)*dsx+t*dcx, (1-t)*dsy+t*dcy
			nx, ny, nz := pv*h, -pu*h, pu*dy-pv*dx
			total += ws[i] * ws[j] * math.Sqrt(nx*nx+ny*ny+nz*nz)
		}
	}
	return total
}

// skewedPatch builds a Cone patch whose cap window is the side window turned
// by turn and narrowed by trim at each end, its corner skews read from the two
// directrices' own end points.
func skewedPatch(t *testing.T, th0, th1, turn, trim float64) capband.Patch {
	t.Helper()
	g := capband.Patch{
		Circular:   true,
		SideRadius: 10, CapRadius: 9,
		Th0: th0, Th1: th1,
		CapTh0: th0 + turn + trim, CapTh1: th1 + turn - trim,
		SideZ: 19, CapZ: 20,
	}
	at := func(r, th float64) capband.Point { return capband.Point{U: r * math.Cos(th), V: r * math.Sin(th)} }
	var ok bool
	g.SkewStart, ok = capband.CornerSkewUpper(0, 0, at(g.SideRadius, g.Th0), at(g.CapRadius, g.CapTh0), g.CapTh0-g.Th0)
	require.True(t, ok)
	g.SkewEnd, ok = capband.CornerSkewUpper(0, 0, at(g.SideRadius, g.Th1), at(g.CapRadius, g.CapTh1), g.CapTh1-g.Th1)
	require.True(t, ok)
	return g
}

// TestConeAreaBoundEnclosesSkewedRuledPatch checks the Cone area bound covers
// the ruled patch between two windows that differ, against a converged
// quadrature of that patch. The turned rows keep the two windows' widths
// equal, so the window-width allowance the bound once carried
// (|Δα|·(R0+R1)·√(ΔR²+H²), calibrated against reachable bodies) read zero
// there while the patch sits whole square millimetres off its frustum sector.
//
// Shown to fail: with coneSkewAreaAllow returning zero, every row's published
// bound falls below its residual; with the width allowance in its place, the
// two turned rows do.
func TestConeAreaBoundEnclosesSkewedRuledPatch(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name               string
		th0, th1           float64
		turn, trim         float64
		widthAllowanceZero bool
	}{
		{name: `turned, same width`, th0: 0, th1: 1, turn: 0.5, widthAllowanceZero: true},
		{name: `turned major arc, same width`, th0: 0.3, th1: 5.5, turn: -0.4, widthAllowanceZero: true},
		{name: `trimmed at both corners`, th0: 0, th1: math.Pi / 2, trim: 0.2},
		{name: `trimmed and turned`, th0: 1, th1: 2.2, turn: 0.3, trim: 0.05},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			g := skewedPatch(t, tc.th0, tc.th1, tc.turn, tc.trim)
			area, bound := capband.AreaOf(g)
			ruled := ruledPatchArea(g, 64)
			require.InDelta(t, ruled, ruledPatchArea(g, 96), 1e-12*ruled, `the quadrature has converged`)
			residual := math.Abs(ruled - area)
			require.Greater(t, residual, 1e-3, `the premise: the ruled patch is not the frustum sector`)
			require.GreaterOrEqual(t, bound, residual, `the published bound %v covers the ruled patch's %v`, bound, ruled)
			require.GreaterOrEqual(t, capband.SkewAreaAllow(g), residual-1e-9*area,
				`the skew term alone covers what the frustum formula misses`)
			if tc.widthAllowanceZero {
				width := math.Abs(math.Abs(g.Th1-g.Th0) - math.Abs(g.CapTh1-g.CapTh0))
				require.Less(t, width*(g.SideRadius+g.CapRadius)*math.Hypot(g.CapRadius-g.SideRadius, g.CapZ-g.SideZ), residual,
					`the window-width allowance misses a turned window`)
			}
		})
	}
}

// TestCornerSkewUpperEnclosesTheTurn checks the corner skew encloses the angle
// between the two ends from above, is zero on one ray, and refuses a skew it
// cannot place on the held windows' branch.
func TestCornerSkewUpperEnclosesTheTurn(t *testing.T) {
	t.Parallel()
	at := func(r, th float64) capband.Point { return capband.Point{U: 3 + r*math.Cos(th), V: -2 + r*math.Sin(th)} }
	for _, turn := range []float64{1e-9, 0.01, 0.4, -0.7, 1.5} {
		skew, ok := capband.CornerSkewUpper(3, -2, at(10, 0.2), at(9, 0.2+turn), turn)
		require.True(t, ok)
		require.GreaterOrEqual(t, skew, math.Abs(turn)*(1-1e-12))
		require.Less(t, skew, math.Abs(turn)*(1+1e-9)+1e-15)
	}
	skew, ok := capband.CornerSkewUpper(0, 0, capband.Point{U: 10, V: 0}, capband.Point{U: 9, V: 0}, 0)
	require.True(t, ok)
	require.Zero(t, skew, `two ends on one ray from the centre have no skew`)

	_, ok = capband.CornerSkewUpper(0, 0, capband.Point{U: 10, V: 0}, capband.Point{U: 0, V: 9}, math.Pi/2)
	require.False(t, ok, `a quarter turn is outside the bound's own range`)
	_, ok = capband.CornerSkewUpper(0, 0, capband.Point{U: 10, V: 0}, capband.Point{U: 9, V: 0.1}, 2*math.Pi)
	require.False(t, ok, `held windows a turn apart name another branch`)
	_, ok = capband.CornerSkewUpper(0, 0, capband.Point{}, capband.Point{U: 9, V: 0}, 0)
	require.False(t, ok, `an end at the centre has no direction`)
}

// skewTermAt reports whether the corner-skew term coneSkewAreaAllow derives,
// evaluated at side radius r with every other input g holds, exceeds
// published, decided exactly over rationals. The term is
// L·A + B, L = √((R1−r)² + H²), A = r·(Φs+Φe)/2 + αc·R1·Φ²/4 and
// B = R1·Φ²·(αs·r + αc·R1)/4 + H·αc·R1·Φ/2, so L·A + B > p holds exactly when
// p − B < 0 or (p − B)² < L²·A², with no square root taken.
func skewTermAt(g capband.Patch, r *big.Rat, published float64) bool {
	rat := func(f float64) *big.Rat { return new(big.Rat).SetFloat64(f) }
	mul := func(xs ...*big.Rat) *big.Rat {
		out := big.NewRat(1, 1)
		for _, x := range xs {
			out.Mul(out, x)
		}
		return out
	}
	add := func(xs ...*big.Rat) *big.Rat {
		out := new(big.Rat)
		for _, x := range xs {
			out.Add(out, x)
		}
		return out
	}
	r1, ps, pe := rat(g.CapRadius), rat(g.SkewStart), rat(g.SkewEnd)
	h := new(big.Rat).Abs(new(big.Rat).Sub(rat(g.CapZ), rat(g.SideZ)))
	phi := ps
	if pe.Cmp(phi) > 0 {
		phi = pe
	}
	sum := add(ps, pe)
	alphaC := add(rat(math.Abs(g.CapTh1-g.CapTh0)), rat(g.CapThAllow))
	alphaS := add(alphaC, sum)
	half, quarter := big.NewRat(1, 2), big.NewRat(1, 4)
	a := add(mul(half, r, sum), mul(quarter, alphaC, r1, phi, phi))
	b := add(mul(quarter, r1, phi, phi, add(mul(alphaS, r), mul(alphaC, r1))), mul(half, h, alphaC, r1, phi))
	dR := new(big.Rat).Sub(r1, r)
	gap := new(big.Rat).Sub(rat(published), b)
	if gap.Sign() < 0 {
		return true
	}
	return mul(gap, gap).Cmp(mul(add(mul(dR, dR), mul(h, h)), a, a)) < 0
}

// TestConeSkewTermCoversTheHeldSideRadiusSpan checks the corner-skew term
// covers the term its derivation states at every side radius the held one
// allows, |r − R0| ≤ Held.SideRadius, not only at the top of that span. The
// slant √((R1−r)² + H²) shrinks as r grows toward R1, so on an outward-arm
// patch (R1 > R0, short slant) the term is largest at the BOTTOM of the
// span; on a tall inward patch it is largest at the top.
//
// Shown to fail: with the term read at R0 + Held.SideRadius alone, the
// outward row's bottom and middle radii exceed the published term; with the
// slant allowance kept and the radius read at the held R0, the tall row's top
// radius does.
func TestConeSkewTermCoversTheHeldSideRadiusSpan(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name         string
		side, cap, h float64
	}{
		{name: `outward arm, short slant`, side: 1000, cap: 1000.25, h: 0.25},
		{name: `inward arm, tall band`, side: 1, cap: 0.5, h: 1000},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			e := 4 * (math.Nextafter(tc.side, math.Inf(1)) - tc.side)
			g := capband.Patch{
				Circular:   true,
				SideRadius: tc.side, CapRadius: tc.cap,
				Th0: 0, Th1: 1, CapTh0: 0.01, CapTh1: 1.01,
				SideZ: 0, CapZ: tc.h,
				CapThAllow: 1e-15,
				SkewStart:  0.01, SkewEnd: 0.01,
				Held: capband.HeldAllow{SideRadius: e},
			}
			published := capband.SkewAreaAllow(g)
			r0, re := new(big.Rat).SetFloat64(tc.side), new(big.Rat).SetFloat64(e)
			for _, r := range []*big.Rat{new(big.Rat).Sub(r0, re), r0, new(big.Rat).Add(r0, re)} {
				require.False(t, skewTermAt(g, r, published),
					`the published skew term %v covers the term at side radius %s`, published, r.FloatString(20))
			}
		})
	}
}
