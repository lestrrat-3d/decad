package retainedblend

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
)

// This file sizes the end curve of a C3 arc end
// (agent-docs/boss-root-overflow-design.md §5.5.4). The wall's row, identified
// by its parameter s = 1, meets G* at its tangent point with ρ = R, α = 0 and
// ψ = 0 (external) or π (internal), never through an acos. Every other row
// keeps R4's reading (arc_end_cyl.go), and each piece of the curve takes its
// margin and angle allowance from its own least q, which falls to zero at the
// wall's row; the piece that ends there takes the box of the cone [0, π/2].

// tangentSection is what the margin of a C3 end's pieces reads besides the
// profile.
type tangentSection struct {
	// eps is G's tilt, zHi the profile's largest |z|, sFar the scale S_far and
	// eD the rounding of d.
	eps, zHi, sFar, eD float64
	// base is the q-free part X of the margin over the whole profile.
	base float64
}

// minMargin is a lower bound of every piece's margin.
func (e endSection) minMargin() float64 {
	if e.cyl != nil && e.cyl.ts != nil {
		return e.cyl.ts.base
	}
	return e.margin
}

// up is x rounded up past a few ulps of rounding.
func up(x float64) float64 { return x * (1 + 8*ulp) }

// hUp is H(h) = 2·asin(min(1, √(h/2))), rounded up: the modulus of continuity of
// acos on [−1, 1], |acos x̃ − acos x| ≤ H(|x̃ − x|), with equality at x = 1.
func hUp(h float64) float64 {
	if !(h > 0) {
		return 0
	}
	return up(2 * math.Asin(math.Min(1, up(math.Sqrt(up(h/2))))))
}

// tangentTilt is the largest D + e_d the section reads, as a share of the
// smaller of d' and r_G that keeps the second-order remainder inside the
// (1 + 2⁻¹⁰) factors.
const tangentTilt = 0x1p-20

// tangentEndSection is cylEndSection for a C3 end (§5.5.4).
func (a arcRim) tangentEndSection(spec blendSpec, c *cylEnd) (endSection, error) {
	p, fillet := a.profile(spec)
	rho0, z0 := p(0)
	rho1, z1 := p(1)
	rhoLo := math.Min(rho0, rho1)
	// The strict range checks read the rows before the wall's. The wall's row
	// takes the boundary of the range by construction, and its radius may
	// differ from the boundary by the end's own error.
	lo, hi := math.Abs(c.d-c.rG), c.d+c.rG
	reach := c.tan.reach
	if !(rho0 > lo && rho0 < hi) || !(rho1 >= lo-reach && rho1 <= hi+reach) {
		return endSection{}, fmt.Errorf("%w: a blend circle does not reach an end face", decaderr.ErrUnsupported)
	}
	if !(rhoLo*rhoLo+c.d*c.d > c.rG*c.rG) {
		return endSection{}, fmt.Errorf("%w: a blend circle meets an end face where the end curve folds back",
			decaderr.ErrUnsupported)
	}
	zHi := math.Max(math.Abs(z0), math.Abs(z1))
	ts := &tangentSection{eps: c.eps, zHi: zHi, sFar: c.sFar, eD: c.tan.eD}
	dTilt := 2 * c.eps * (zHi + c.rG)
	if !(dTilt+ts.eD <= tangentTilt*math.Min(c.d, c.rG)) {
		return endSection{}, fmt.Errorf("%w: an arc end's tilt moves its section too far", decaderr.ErrUnsupported)
	}
	// The end's own error e (§5.5.3) stays small enough for σ's second-order remainders (§5.5.6).
	if !(c.tan.e <= tangentTilt*math.Min(rhoLo, math.Min(c.d, c.rG))) {
		return endSection{}, fmt.Errorf("%w: an arc end's tags or cliff joint move its crossing too far",
			decaderr.ErrUnsupported)
	}
	e := endSection{p: p, kappa: a.kappa, side: a.side, cyl: &cylCurve{rG: c.rG, d: c.d, tan: c.tan, ts: ts}}
	e.slope = profileSlope(spec, fillet)
	// Each piece reads its own margin. sectionGrid's up-front test reads the
	// first piece's, [0, 1/2], and certify's early gap test the q-free part.
	rLo := math.Min(rhoLo, c.rG)
	ts.base = up(64*ulp*c.sFar*(c.sFar/rLo) + dTilt)
	e.margin = e.pieceMargin(0, 0.5)
	return e, nil
}

// pieceMargin is the margin of the piece [t0, t1] of a tangent end's curve:
// m = X + min(X/q_lo, r_G·H(err_ψ) + ρ_hi·(H(err_α) + asin↑(D/(d' − D)))),
// with X = 64·ulp·S·(S/r_lo) + D and D = 2ε(z_hi + r_G). The piece that ends at
// the wall's row has q_lo = 0 and takes the second term alone.
func (e endSection) pieceMargin(t0, t1 float64) float64 {
	c := e.cyl
	ts := c.ts
	rho0, _ := e.p(t0)
	rho1, _ := e.p(t1)
	rhoLo, rhoHi := math.Min(rho0, rho1), math.Max(rho0, rho1)
	rLo := math.Min(rhoLo, c.rG)
	dTilt := 2 * ts.eps * (ts.zHi + c.rG)
	x := up(64*ulp*ts.sFar*(ts.sFar/rLo) + dTilt)
	dm := dTilt + ts.eD
	errPsi, errAlpha := c.tangentErrors(rhoLo, rhoHi, dm)
	turn := math.Asin(math.Min(1, up(dm/(c.d-dm))))
	second := up(c.rG*hUp(errPsi) + rhoHi*(hUp(errAlpha)+up(turn)))
	if t1 >= 1 {
		return up(x + second)
	}
	_, _, q0 := e.at(t0)
	_, _, q1 := e.at(t1)
	qLo := math.Min(q0, q1) * (1 - 8*ulp)
	// The first branch holds only where the crossing's curvature remainder
	// stays inside its (1 + 2⁻¹⁰) factor.
	if !(qLo > 0) || !(x <= 0x1p-12*qLo*qLo*math.Min(rhoLo, c.rG)) {
		return up(x + second)
	}
	return up(x + math.Min((1+0x1p-10)*x/qLo, second))
}

// tangentErrors are err_ψ and err_α, the bounds of the errors of cos ψ's and
// cos α's arguments over rows of radius in [rhoLo, rhoHi], for D + e_d = dm
// read as moves of d' and r_G.
func (c *cylCurve) tangentErrors(rhoLo, rhoHi, dm float64) (errPsi, errAlpha float64) {
	f := 1 + 0x1p-10
	errPsi = 16*ulp*(c.d*c.d+c.rG*c.rG+rhoHi*rhoHi)/(2*c.d*c.rG) + f*dm*2*(1/c.rG+1/c.d)
	errAlpha = 16*ulp*(rhoHi*rhoHi+c.d*c.d+c.rG*c.rG)/(2*rhoLo*c.d) + f*dm*2*c.rG/(rhoLo*c.d)
	return up(errPsi), up(errAlpha)
}

// pieceAngle is the angle allowance of the piece [t0, t1]: R4's, read at the
// piece's own least q, and H(err_ψ) for the piece that ends at the wall's row.
func (e endSection) pieceAngle(t0, t1 float64) float64 {
	c := e.cyl
	ts := c.ts
	rho0, _ := e.p(t0)
	rho1, _ := e.p(t1)
	rhoLo, rhoHi := math.Min(rho0, rho1), math.Max(rho0, rho1)
	if t1 >= 1 {
		dm := 2*ts.eps*(ts.zHi+c.rG) + ts.eD
		errPsi, _ := c.tangentErrors(rhoLo, rhoHi, dm)
		return hUp(errPsi)
	}
	_, _, q0 := e.at(t0)
	_, _, q1 := e.at(t1)
	q := math.Min(q0, q1) * (1 - 8*ulp)
	if !(q > 0) {
		return math.Pi
	}
	ed := ts.eD
	dc := (8*ulp*(rhoHi*rhoHi+c.rG*c.rG+c.d*c.d)+2*c.d*ed+ed*ed)/(2*rhoLo*c.rG) +
		4*ulp*ts.sFar*(rhoHi*rhoHi+c.d*c.d)/(2*rhoLo*rhoLo*c.rG)
	return up(2 * (dc/(q*q) + 6*ulp/q + 2*ulp))
}

// tangentCone is the cone of the tangent directions over the piece [t0, t1] of a
// tangent end's curve, q0 and q1 the piece's ends' q.
func (e endSection) tangentCone(t0, t1, q0, q1 float64) (lo, hi float64) {
	if t1 >= 1 {
		return 0, math.Pi / 2
	}
	ang := e.pieceAngle(t0, t1)
	lo = math.Max(0, math.Atan(e.slope(t0)*math.Min(q0, q1))-ang)
	hi = math.Min(math.Pi/2, math.Atan(e.slope(t1)*e.qHi(t0, t1, q0, q1))+ang)
	return lo, hi
}

// psiAt is the crossing angle ψ of the row at parameter s.
func (e endSection) psiAt(s float64) float64 {
	rho, _ := e.p(s)
	if e.cyl.tan != nil && s >= 1 {
		return e.cyl.tan.psiWall()
	}
	return e.cyl.psi(rho)
}

// alphaAt is the azimuth distance α of the row at parameter s from G's centre.
func (e endSection) alphaAt(s float64) float64 {
	rho, _ := e.p(s)
	if e.cyl.tan != nil && s >= 1 {
		return 0
	}
	return e.cyl.azimuth(rho)
}
