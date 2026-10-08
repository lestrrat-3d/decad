package capband

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/capcontour"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// ErrContourUnbounded refuses a cap contour point whose exact offset cannot
// be enclosed by the recorded carriers.
var ErrContourUnbounded = fmt.Errorf(`%w: this evaluator cannot prove a bound on the cap-loop chamfer's own offset contour at a corner, so no cap-level coordinate it emits there can be published with a proven displacement`, decaderr.ErrUnsupported)

// ErrHeldUnbounded refuses a circular band patch whose held angle or radius
// cannot be enclosed against the value its closed surface reads.
var ErrHeldUnbounded = fmt.Errorf(
	`%w: a cap-loop chamfer's circular band patch holds an angle or radius `+
		`this evaluator cannot bound against the point it names, so no measurement integrated over it can be published with a proven bound`,
	decaderr.ErrUnsupported)

// ContourDisplacement bounds every cap-level point of one band against the
// exact offset of its recorded side contour. dDelta encloses the rounding of
// the caller's stated setback, so the offset is read over its whole span.
func ContourDisplacement(walks []survey2d.SideWalk, joins []capcontour.Join, d, dDelta, tol float64) (float64, error) {
	if _, ok := capcontour.OffsetSpan(d, dDelta); !ok {
		return 0, ErrContourUnbounded
	}
	for _, w := range walks {
		if w.IsCircular() {
			if _, err := BandRadius(w, d, tol); err != nil {
				return 0, err
			}
		}
	}
	delta, ok := capcontour.Displacement(walks, joins, d, dDelta)
	if !ok {
		return 0, ErrContourUnbounded
	}
	return delta, nil
}

// WholeCircleDisplacement bounds the cap contour of a cornerless full circle.
func WholeCircleDisplacement(w survey2d.SideWalk, d, dDelta, tol float64) (float64, error) {
	held, err := BandRadius(w, d, tol)
	if err != nil {
		return 0, err
	}
	radius, ok := OffsetRadiusSpan(w, d, dDelta)
	if !ok {
		return 0, ErrContourUnbounded
	}
	return proofbound.IntervalFloatError(radius, held), nil
}

// OffsetRadiusSpan encloses the cap radius over every offset amount in the
// caller's stated setback span and every radius the wall's record denotes:
// the held radius within its RadiusBound (capcontour.OffsetCircleRadius).
func OffsetRadiusSpan(w survey2d.SideWalk, d, dDelta float64) (proofbound.RatInterval, bool) {
	span, ok := capcontour.OffsetSpan(d, dDelta)
	if !ok {
		return proofbound.RatInterval{}, false
	}
	return capcontour.OffsetCircleRadius(w, span)
}

// WallHeldAllow bounds a circular wall patch's side and cap angles and radii
// against the recorded side walk and the cap face's held feet.
func WallHeldAllow(w survey2d.SideWalk, capRadius float64, start, end Point, capTh0, capTh1 float64) (HeldAllow, error) {
	a0, ok0 := AngleAllow(w.CU, w.CV, Point{U: w.StartU, V: w.StartV}, proofbound.WalkEndBoundAllow(w.StartBound), w.Th0)
	a1, ok1 := AngleAllow(w.CU, w.CV, Point{U: w.EndU, V: w.EndV}, proofbound.WalkEndBoundAllow(w.EndBound), w.Th1)
	c0, okC0 := AngleAllow(w.CU, w.CV, start, 0, capTh0)
	c1, okC1 := AngleAllow(w.CU, w.CV, end, 0, capTh1)
	r1, _, okR := RadiusAllow(w.CU, w.CV, capRadius, start, end)
	if !ok0 || !ok1 || !okC0 || !okC1 || !okR || !(w.RadiusBound >= 0) || proofbound.IsNonFinite(w.RadiusBound) {
		return HeldAllow{}, ErrHeldUnbounded
	}
	return HeldAllow{Th0: a0, Th1: a1, CapTh0: c0, CapTh1: c1, SideRadius: w.RadiusBound, CapRadius: r1}, nil
}

// ApexHeldAllow bounds a reflex corner patch's cap angles and radius. Its
// side directrix is the corner point and has no angle or radius allowance.
func ApexHeldAllow(cU, cV, dc float64, foot0, foot1 Point, th0, th1 float64) (HeldAllow, error) {
	c0, ok0 := AngleAllow(cU, cV, foot0, 0, th0)
	c1, ok1 := AngleAllow(cU, cV, foot1, 0, th1)
	r1, _, okR := RadiusAllow(cU, cV, dc, foot0, foot1)
	if !ok0 || !ok1 || !okR {
		return HeldAllow{}, ErrHeldUnbounded
	}
	return HeldAllow{CapTh0: c0, CapTh1: c1, CapRadius: r1}, nil
}
