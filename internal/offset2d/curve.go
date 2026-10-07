// Package offset2d computes held section offsets and carrier intersections
// for Fillet, Shell, and cap-loop Chamfer.
package offset2d

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/sectionaudit"
)

// Curve is a held offset carrier: a line given by a point and unit direction,
// or a circle given by its center and radius.
type Curve struct {
	IsLine         bool
	PX, PY, DX, DY float64
	CX, CY, Radius float64
}

// Normalize returns the held unit vector and length of (x, y).
func Normalize(x, y float64) (float64, float64, float64) {
	l := math.Hypot(x, y)
	if l == 0 {
		return 0, 0, 0
	}
	return x / l, y / l, l
}

// Intersect returns the root of two offset carriers nearest the corner.
// A missing intersection returns ok false.
func Intersect(a, b Curve, px, py float64) (float64, float64, bool) {
	var cands [][2]float64
	switch {
	case a.IsLine && b.IsLine:
		cands = lineLine(a, b)
	case a.IsLine && !b.IsLine:
		cands = lineCircle(a, b.CX, b.CY, b.Radius)
	case !a.IsLine && b.IsLine:
		cands = lineCircle(b, a.CX, a.CY, a.Radius)
	default:
		cands = sectionaudit.CircleCircle(a.CX, a.CY, a.Radius, b.CX, b.CY, b.Radius)
	}
	if len(cands) == 0 {
		return 0, 0, false
	}
	best, bestD := cands[0], math.Inf(1)
	for _, c := range cands {
		dd := (c[0]-px)*(c[0]-px) + (c[1]-py)*(c[1]-py)
		if dd < bestD {
			bestD, best = dd, c
		}
	}
	return best[0], best[1], true
}

// lineLine intersects two lines (point + unit direction).
func lineLine(a, b Curve) [][2]float64 {
	den := a.DX*b.DY - a.DY*b.DX
	if math.Abs(den) <= sectionaudit.Tolerance {
		return nil
	}
	s := ((b.PX-a.PX)*b.DY - (b.PY-a.PY)*b.DX) / den
	return [][2]float64{{a.PX + s*a.DX, a.PY + s*a.DY}}
}

// lineCircle intersects a line (point + unit direction) with a circle.
func lineCircle(l Curve, cx, cy, r float64) [][2]float64 {
	// |P + s·d − C|² = r², d unit: s² + 2 s (P−C)·d + |P−C|² − r² = 0.
	fx, fy := l.PX-cx, l.PY-cy
	bb := fx*l.DX + fy*l.DY
	cc := fx*fx + fy*fy - r*r
	disc := bb*bb - cc
	if disc < 0 {
		return nil
	}
	sq := math.Sqrt(math.Max(disc, 0))
	var out [][2]float64
	for _, s := range []float64{-bb + sq, -bb - sq} {
		out = append(out, [2]float64{l.PX + s*l.DX, l.PY + s*l.DY})
	}
	return out
}
