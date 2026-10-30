// Package saddlecanal computes and proves the rolling-ball fillet of the
// closed saddle where a bore crosses a larger perpendicular cylinder
// (agent-docs/retained-edge-blend-design.md §4.10). The ball lies outside the
// bore, and inside or outside the larger cylinder: inside at the convex mouth
// of a cross hole in a shaft, outside where two bores cross in a block or a
// boss roots on a rod.
//
// Everything is in the fixed local pose of
// agent-docs/cross-cylinder-bore-chamfer-design.md: the larger cylinder
// x² + y² = R² about the z axis, the bore y² + z² = r² about the x axis, the
// mouth at x > 0. With σ1 the ball's side of the larger cylinder (−1 inside,
// +1 outside), a ball of radius ρ touching both walls has its centre on the
// two offset cylinders of radii R' = R + σ1·ρ and r' = r + ρ, so on the
// closed curve
//
//	c(θ) = (√(R'² − r'²cos²θ), r'·cos θ, r'·sin θ).
//
// It touches the larger wall along a1 = −σ1·(c_x, c_y, 0)/R' and the bore
// along a2 = −(0, cos θ, sin θ); both are normal to c′, and the arc between
// them has angle φ with cos φ = σ1·r'·cos²θ/R'. The fillet is the pipe
//
//	F(θ, t) = c(θ) + ρ·w/|w|,  w = (1 − t)·a1 + t·a2,  t ∈ [0, 1],
//
// whose rows t = 0 and t = 1 are the two tangency curves T1 and T2. The
// strip blends each half of the arc on its own (Canal.Rows), so w runs from
// a1 to the bisector of a1 and a2 over t ∈ [0, 1/2] and on to a2 over
// t ∈ [1/2, 1].
package saddlecanal

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

// Stations is the number of stations θ_k = 2πk/Stations around the mouth. It
// is a multiple of four, so no station cell holds an extremum of sin or cos
// inside it.
const Stations = 256

// Canal is one fillet: the larger cylinder's radius R, the bore's radius
// Bore and the ball's radius Rho, in millimetres. Outside is set when the
// ball lies outside the larger cylinder (σ1 = +1); the ball always lies
// outside the bore.
type Canal struct {
	R, Bore, Rho float64
	Outside      bool
}

// sigma1 is σ1: −1 when the ball lies inside the larger cylinder, +1 outside.
func (c Canal) sigma1() float64 {
	if c.Outside {
		return 1
	}
	return -1
}

// Offsets returns the float radii of the two offset cylinders the ball's
// centre lies on: r' = r + ρ about the bore's axis and R' = R + σ1·ρ about
// the larger cylinder's.
func (c Canal) Offsets() (float64, float64) {
	return c.Bore + c.Rho, c.R + c.sigma1()*c.Rho
}

// offsetIV encloses r' and R'.
func (c Canal) offsetIV() (iv, iv) {
	rp := pt(c.Bore).add(pt(c.Rho))
	if c.Outside {
		return rp, pt(c.R).add(pt(c.Rho))
	}
	return rp, pt(c.R).sub(pt(c.Rho))
}

// Exists reports whether the ball fits, decided exactly on the float
// parameters: 0 < Rho, 0 < Bore and Bore + Rho < R + σ1·Rho, so r' < R' and
// the centre curve is one closed loop per mouth. Outside the larger cylinder
// that is Bore < R, which every saddle meets.
func (c Canal) Exists() bool {
	if !(c.Rho > 0) || !(c.Bore > 0) || math.IsInf(c.R, 0) || math.IsNaN(c.R) || math.IsInf(c.Rho, 0) {
		return false
	}
	rho := proofarith.FloatRat(c.Rho)
	lhs := new(big.Rat).Add(proofarith.FloatRat(c.Bore), rho)
	rhs := new(big.Rat).Sub(proofarith.FloatRat(c.R), rho)
	if c.Outside {
		rhs = new(big.Rat).Add(proofarith.FloatRat(c.R), rho)
	}
	return lhs.Cmp(rhs) < 0
}

// arcStep is the largest angle step of a row of the strip: the sagitta of an
// arc of radius ρ over it is 2⁻¹⁰·ρ.
var arcStep = 2 * math.Acos(1-0x1p-10)

// widestArc is φ*, the largest angle of the arc from a1 to a2 over θ: its
// cosine σ1·r'·cos²θ/R' is least at θ = 0 inside the larger cylinder,
// arccos(−r'/R'), and at θ = π/2 outside it, π/2.
func (c Canal) widestArc() float64 {
	if c.Outside {
		return math.Pi / 2
	}
	rp, Rp := c.Offsets()
	return math.Acos(-rp / Rp)
}

// Rows returns the strip's row parameters t_0 = 0 < … < t_m = 1, m even and
// t_{m/2} = 1/2. Each half of the arc, from a1 to the bisector b̂ and from b̂
// to a2, is the normalised blend of its ends over its own half of t. The rows
// split the widest half-arc, φ*/2 (widestArc), into equal angles no larger
// than arcStep: the normalised blend of two unit vectors at angle β reaches
// angle s·β at τ = sin(sβ)/(sin(sβ) + sin((1 − s)β)). Every station shares
// the rows.
func (c Canal) Rows() ([]float64, error) {
	half := c.widestArc() / 2
	h := int(math.Ceil(half / arcStep))
	if h < 1 || h > 512 {
		return nil, fmt.Errorf("%w: the saddle fillet's arc has no row plan", decaderr.ErrUnsupported)
	}
	m := 2 * h
	rows := make([]float64, m+1)
	for j := 1; j < h; j++ {
		s := float64(j) / float64(h)
		a, b := math.Sin(s*half), math.Sin((1-s)*half)
		tau := a / (a + b)
		rows[j] = tau / 2
		rows[h+j] = 0.5 + tau/2
	}
	rows[h], rows[m] = 0.5, 1
	for j := 1; j <= m; j++ {
		if !(rows[j] > rows[j-1]) {
			return nil, fmt.Errorf("%w: the saddle fillet's rows are not increasing", decaderr.ErrUnsupported)
		}
	}
	return rows, nil
}

// Theta returns the float angle of station k.
func Theta(k int) float64 { return 2 * math.Pi * float64(k) / Stations }

// Point returns F(θ_k, t) in local float coordinates. Rows 0 and 1 are the
// tangency points on the larger cylinder and the bore.
func (c Canal) Point(k int, t float64) r3.Vec {
	s, co := math.Sincos(Theta(k))
	rp, Rp := c.Offsets()
	x := math.Sqrt(Rp*Rp - rp*rp*co*co)
	centre := r3.Vec{X: x, Y: rp * co, Z: rp * s}
	a1 := r3.Vec{X: x / Rp, Y: rp * co / Rp}.Scale(-c.sigma1())
	a2 := r3.Vec{Y: -co, Z: -s}
	b := a1.Add(a2)
	b = b.Scale(1 / b.Len())
	var w r3.Vec
	if t <= 0.5 {
		w = a1.Scale(1 - 2*t).Add(b.Scale(2 * t))
	} else {
		w = b.Scale(2 - 2*t).Add(a2.Scale(2*t - 1))
	}
	return centre.Add(w.Scale(c.Rho / w.Len()))
}
