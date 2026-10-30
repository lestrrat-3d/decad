// Package saddlecanal computes and proves the rolling-ball fillet of the
// closed saddle where a bore crosses a larger perpendicular cylinder, with the
// ball inside the larger cylinder and outside the bore: the convex mouth of a
// cross hole in a shaft (agent-docs/retained-edge-blend-design.md §4.10).
//
// Everything is in the fixed local pose of
// agent-docs/cross-cylinder-bore-chamfer-design.md: the larger cylinder
// x² + y² = R² about the z axis, the bore y² + z² = r² about the x axis, the
// mouth at x > 0. A ball of radius ρ touching both walls from the material
// has its centre on the two offset cylinders of radii R' = R − ρ and
// r' = r + ρ, so on the closed curve
//
//	c(θ) = (√(R'² − r'²cos²θ), r'·cos θ, r'·sin θ).
//
// It touches the larger wall along a1 = (c_x, c_y, 0)/R' and the bore along
// a2 = −(0, cos θ, sin θ); both are normal to c′. The fillet is the pipe
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
// Bore and the ball's radius Rho, in millimetres.
type Canal struct{ R, Bore, Rho float64 }

// Exists reports whether the ball fits, decided exactly on the float
// parameters: 0 < Rho, 0 < Bore and Bore + Rho < R − Rho, so r' < R' and the
// centre curve is one closed loop per mouth.
func (c Canal) Exists() bool {
	if !(c.Rho > 0) || !(c.Bore > 0) || math.IsInf(c.R, 0) || math.IsNaN(c.R) {
		return false
	}
	rho := proofarith.FloatRat(c.Rho)
	lhs := new(big.Rat).Add(proofarith.FloatRat(c.Bore), rho)
	rhs := new(big.Rat).Sub(proofarith.FloatRat(c.R), rho)
	return lhs.Cmp(rhs) < 0
}

// arcStep is the largest angle step of a row of the strip: the sagitta of an
// arc of radius ρ over it is 2⁻¹⁰·ρ.
var arcStep = 2 * math.Acos(1-0x1p-10)

// Rows returns the strip's row parameters t_0 = 0 < … < t_m = 1, m even and
// t_{m/2} = 1/2. Each half of the arc, from a1 to the bisector b̂ and from b̂
// to a2, is the normalised blend of its ends over its own half of t. The rows
// split the widest half-arc, at θ = 0 with angle φ*/2, φ* = arccos(−r'/R'),
// into equal angles no larger than arcStep: the normalised blend of two unit
// vectors at angle β reaches angle s·β at τ = sin(sβ)/(sin(sβ) + sin((1 − s)β)).
// Every station shares the rows.
func (c Canal) Rows() ([]float64, error) {
	rp, Rp := c.Bore+c.Rho, c.R-c.Rho
	half := math.Acos(-rp/Rp) / 2
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
	rp, Rp := c.Bore+c.Rho, c.R-c.Rho
	x := math.Sqrt(Rp*Rp - rp*rp*co*co)
	centre := r3.Vec{X: x, Y: rp * co, Z: rp * s}
	a1 := r3.Vec{X: x / Rp, Y: rp * co / Rp}
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
