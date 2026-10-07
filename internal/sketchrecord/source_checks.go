package sketchrecord

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/sketch/geom"
)

// falsifyTol is the relative threshold for checking a fragment's sketch node.
// TExact's stated meaning is reproduction to machine precision, so an honest
// flag leaves a residual within round-off (~1e-13 relative); a flag worth
// disproving misses by the sampling error it hid. 1e-9 sits between the two —
// far above round-off, far below any sampling-scale miss — so the falsifier can
// reject a lie without ever false-rejecting an exact cut.
const falsifyTol = 1e-9

// falsifyRange is the seam's one check, and it can only reject
// (docs/sketch-seam-design.md §1). TExact's checkable meaning is that
// evaluating the source entity at the certified range reproduces the
// fragment's Polyline endpoints; a large residual therefore disproves the
// flag — the fragment is rejected and the discrepancy is a sketch bug to
// report upstream. A small residual proves nothing — a sampled cut can lie
// arbitrarily close to the curve — so this check never admits a fragment on
// its own; admission is TExact's alone.
//
// Polyline[0] and Polyline[len-1] are the only polyline content decad reads,
// and only ever to check — never to record.
func falsifyRange(e sketch.BoundaryEdge) error {
	if len(e.Polyline) < 2 {
		return fmt.Errorf(`%w: a %T fragment carries no polyline endpoints to check its certified range against`, ErrUnrecordableProfile, e.Entity)
	}
	// The polyline is in walk order; the range is in the entity's natural
	// direction. Reorder the observations, never the range.
	obs0, obs1 := e.Polyline[0], e.Polyline[len(e.Polyline)-1]
	if e.Reversed {
		obs0, obs1 = obs1, obs0
	}
	if err := falsifyBound(e, e.TStart, obs0); err != nil {
		return err
	}
	return falsifyBound(e, e.TEnd, obs1)
}

// falsifyBound checks one certified bound against its observed endpoint.
func falsifyBound(e sketch.BoundaryEdge, t float64, obs [2]float64) error {
	x, y, err := evalEntityAt(e.Entity, t)
	if err != nil {
		return fmt.Errorf(`%w: the source %T cannot be evaluated at its certified range: %s`, ErrUnrecordableProfile, e.Entity, err)
	}
	if !pointsWithinFalsifyTolerance(Point2{U: x, V: y}, Point2{U: obs[0], V: obs[1]}) {
		return fmt.Errorf(`%w: the certified range on a %T is disproven — eval(%v) = (%v, %v) does not reproduce the fragment endpoint (%v, %v); report upstream as a sketch bug`,
			ErrUnrecordableProfile, e.Entity, t, x, y, obs[0], obs[1])
	}
	return nil
}

// pointsWithinFalsifyTolerance reports whether two observations of a fragment
// endpoint agree at falsifyTol relative to their coordinate scale.
func pointsWithinFalsifyTolerance(a, b Point2) bool {
	scale := 1.0
	for _, m := range []float64{math.Abs(a.U), math.Abs(a.V), math.Abs(b.U), math.Abs(b.V)} {
		scale = math.Max(scale, m)
	}
	return math.Hypot(a.U-b.U, a.V-b.V) <= falsifyTol*scale
}

// evalEntityAt evaluates a sketch entity at the arrangement's normalized
// t ∈ [0, 1], per geom's published parameterization (geom.BoundaryEdge): the
// curve is reconstituted through the entity's own Geometry snapshot and
// geom's own evaluators and readings — nothing is re-derived here.
func evalEntityAt(ent sketch.Entity, t float64) (float64, float64, error) {
	switch ent := ent.(type) {
	case *sketch.Line:
		g := ent.Geometry()
		return g.Start.X + t*(g.End.X-g.Start.X), g.Start.Y + t*(g.End.Y-g.Start.Y), nil
	case *sketch.Circle:
		// angle = 2π·t from the absolute +x axis (a circle has no start).
		g := ent.Geometry()
		a := 2 * math.Pi * t
		return g.Center.X + g.Radius*math.Cos(a), g.Center.Y + g.Radius*math.Sin(a), nil
	case *sketch.Arc:
		// angle = StartAngle + t·Sweep — geom's own derived readings.
		g := ent.Geometry()
		a := g.StartAngle() + t*g.Sweep()
		r := g.Radius()
		return g.Center.X + r*math.Cos(a), g.Center.Y + r*math.Sin(a), nil
	case *sketch.Ellipse:
		// eccentric angle 2π·t in the rotated local frame.
		g := ent.Geometry()
		x, y := ellipsePoint(g.Center, g.Rx, g.Ry, g.Rotation, 2*math.Pi*t)
		return x, y, nil
	case *sketch.EllipticalArc:
		// eccentric angle = StartParam + t·Sweep — geom's own readings.
		g := ent.Geometry()
		x, y := ellipsePoint(g.Center, g.Rx, g.Ry, g.Rotation, g.StartParam()+t*g.Sweep())
		return x, y, nil
	case *sketch.Conic:
		g := ent.Geometry()
		x, y := g.Eval(t)
		return x, y, nil
	case *sketch.Spline:
		g := ent.Geometry()
		return geom.EvalCubicBSpline(pointPairs(g.Control), t)
	case *sketch.ClosedSpline:
		g := ent.Geometry()
		return geom.EvalPeriodicCubicBSpline(pointPairs(g.Control), t)
	case *sketch.FitSpline:
		g := ent.Geometry()
		return geom.EvalFitSpline(pointPairs(g.Fit), t)
	case *sketch.NURBS:
		// normalized: knot u = lo + (hi−lo)·t over Domain().
		g := ent.Geometry()
		lo, hi := g.Domain()
		x, y := g.Eval(lo + (hi-lo)*t)
		return x, y, nil
	default:
		return 0, 0, fmt.Errorf(`decad: no evaluator for a %T`, ent)
	}
}

// ellipsePoint returns the parametric ellipse point at eccentric angle theta:
// Center + R(rot)·(Rx·cos θ, Ry·sin θ).
func ellipsePoint(center *geom.Point, rx, ry, rot, theta float64) (float64, float64) {
	lx, ly := rx*math.Cos(theta), ry*math.Sin(theta)
	cosr, sinr := math.Cos(rot), math.Sin(rot)
	return center.X + cosr*lx - sinr*ly, center.Y + sinr*lx + cosr*ly
}

// pointPairs converts geom points to the [][2]float64 form geom's spline
// evaluators take.
func pointPairs(ps []*geom.Point) [][2]float64 {
	out := make([][2]float64, len(ps))
	for i, p := range ps {
		out[i] = [2]float64{p.X, p.Y}
	}
	return out
}
