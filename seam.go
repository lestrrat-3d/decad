package decad

import (
	"fmt"
	"math"
	"slices"

	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/sketch/geom"
	"github.com/lestrrat-3d/units"
)

// This file is the seam conversion of docs/sketch-seam-design.md: the one
// place a live sketch profile becomes the structural records a Recipe Step
// carries. sketch answers every 2D question and decad consumes the answers —
// nothing here re-derives a trim, projects a point, or fits a curve.

// RecordProfile converts a sketch profile into the structural records a
// Recipe Step carries: the region as a [ProfileRecord] — the entity's own
// defining data per boundary edge, plus the recorded range — and the sketch
// plane, read through s.Plane().Frame(), as the [PlaneRecord] that lifts the
// plane-local region into world space. The feature calls run exactly this
// conversion; it is exported so a consumer can record — and therefore vet — a
// profile without a Document.
//
// Every gate is sketch's own answer (docs/api-design.md §7): p must be a
// profile of s (Profile.Sketch, else [ErrForeignProfile]) and a current one
// (Profile.IsStale, else [ErrStaleProfile]); a profile whose Valid is false is
// never silently swept ([ErrInvalidProfile]). The one further rejection is not
// a validity judgement: a valid profile whose boundary cannot be recorded
// exactly — a Partial fragment whose cut sketch reports sampled
// (BoundaryEdge.TExact == false), or a certified range the seam's reject-only
// falsifier disproves — is [ErrUnrecordableProfile]. decad never repairs,
// projects or fits a point sketch handed over, and it never solves for one.
func RecordProfile(s *sketch.Sketch, p *sketch.Profile) (ProfileRecord, PlaneRecord, error) {
	if s == nil || p == nil {
		return ProfileRecord{}, PlaneRecord{}, fmt.Errorf(`%w: RecordProfile requires a sketch and a profile`, ErrDegenerate)
	}
	if p.Sketch() != s {
		return ProfileRecord{}, PlaneRecord{}, fmt.Errorf(`%w: the profile was built from a different sketch, so its plane-local coordinates are another plane's`, ErrForeignProfile)
	}
	if p.IsStale() {
		return ProfileRecord{}, PlaneRecord{}, fmt.Errorf(`%w: the sketch has changed since this profile was built; rebuild with Sketch.Profiles`, ErrStaleProfile)
	}
	if !p.Valid {
		return ProfileRecord{}, PlaneRecord{}, fmt.Errorf(`%w: a self-intersecting or degenerate region is never silently swept`, ErrInvalidProfile)
	}

	frame, err := s.Plane().Frame()
	if err != nil {
		return ProfileRecord{}, PlaneRecord{}, fmt.Errorf(`decad: failed to resolve the sketch plane: %w`, err)
	}
	plane := PlaneRecord{Origin: frame.Origin(), U: frame.U(), V: frame.V()}

	outer, err := recordLoop(p.Outer)
	if err != nil {
		return ProfileRecord{}, PlaneRecord{}, err
	}
	var holes []LoopRecord
	for _, h := range p.Holes {
		loop, err := recordLoop(h)
		if err != nil {
			return ProfileRecord{}, PlaneRecord{}, err
		}
		holes = append(holes, loop)
	}
	return ProfileRecord{Outer: outer, Holes: holes}, plane, nil
}

// recordLoop converts one boundary loop, edge by edge, in walk order.
func recordLoop(edges []sketch.BoundaryEdge) (LoopRecord, error) {
	segs := make([]CurveSegment, 0, len(edges))
	for _, e := range edges {
		seg, err := recordEdge(e)
		if err != nil {
			return LoopRecord{}, err
		}
		segs = append(segs, seg)
	}
	return LoopRecord{Segments: segs}, nil
}

// recordEdge converts one boundary edge into its entity's own variant.
//
// Admission (docs/sketch-seam-design.md §1): a whole edge records from the
// entity's own defining data — TExact is never consulted, because there is no
// trim to certify (the whole *EllipticalArc edge's contingent flag is a fact
// about its pinned ends, not topology distrust). A Partial fragment records
// exactly when sketch certifies its cut — TExact — and the certified range is
// then checked by the reject-only falsifier. The entity kind never decides
// admission; it only selects the variant.
func recordEdge(e sketch.BoundaryEdge) (CurveSegment, error) {
	if e.Partial {
		if !e.TExact {
			return nil, fmt.Errorf(`%w: a %T fragment is bounded by a sampled cut (TExact = false); an approximate range is never recorded as an exact trim`, ErrUnrecordableProfile, e.Entity)
		}
		if err := falsifyRange(e); err != nil {
			return nil, err
		}
	}

	// Reversed is baked into the segment as the order of its range — TStart
	// and TEnd swapped, so TStart > TEnd says the walk runs against the
	// curve's natural sense — and a closed kind's CCW flips with it. The
	// entity's fields are never reordered.
	t0, t1 := e.TStart, e.TEnd
	ccw := true
	if e.Reversed {
		t0, t1 = t1, t0
		ccw = false
	}

	switch ent := e.Entity.(type) {
	case *sketch.Line:
		g := ent.Geometry()
		return LineSeg{Start: point2(g.Start), End: point2(g.End), TStart: t0, TEnd: t1}, nil
	case *sketch.Circle:
		g := ent.Geometry()
		return CircleSeg{Center: point2(g.Center), Radius: units.Millimeters(g.Radius), CCW: ccw, TStart: t0, TEnd: t1}, nil
	case *sketch.Arc:
		g := ent.Geometry()
		return ArcSeg{Center: point2(g.Center), Start: point2(g.Start), End: point2(g.End), TStart: t0, TEnd: t1}, nil
	case *sketch.Ellipse:
		g := ent.Geometry()
		return EllipseSeg{
			Center: point2(g.Center),
			Rx:     units.Millimeters(g.Rx), Ry: units.Millimeters(g.Ry), Rotation: units.Radians(g.Rotation),
			CCW: ccw, TStart: t0, TEnd: t1,
		}, nil
	case *sketch.EllipticalArc:
		g := ent.Geometry()
		return EllipticalArcSeg{
			Center: point2(g.Center), Start: point2(g.Start), End: point2(g.End),
			Rx: units.Millimeters(g.Rx), Ry: units.Millimeters(g.Ry), Rotation: units.Radians(g.Rotation),
			TStart: t0, TEnd: t1,
		}, nil
	case *sketch.Conic:
		g := ent.Geometry()
		return ConicSeg{Start: point2(g.Start), Apex: point2(g.Apex), End: point2(g.End), Rho: g.Rho, TStart: t0, TEnd: t1}, nil
	case *sketch.Spline:
		g := ent.Geometry()
		return SplineSeg{Control: points2(g.Control), TStart: t0, TEnd: t1}, nil
	case *sketch.ClosedSpline:
		g := ent.Geometry()
		return ClosedSplineSeg{Control: points2(g.Control), CCW: ccw, TStart: t0, TEnd: t1}, nil
	case *sketch.FitSpline:
		g := ent.Geometry()
		return FitSplineSeg{Fit: points2(g.Fit), TStart: t0, TEnd: t1}, nil
	case *sketch.NURBS:
		g := ent.Geometry()
		return NURBSSeg{
			Degree:  g.Degree,
			Control: points2(g.Control),
			Knots:   slices.Clone(g.Knots),
			Weights: slices.Clone(g.Weights),
			TStart:  t0, TEnd: t1,
		}, nil
	default:
		return nil, fmt.Errorf(`%w: no CurveSegment variant records a %T; a new entity kind upstream needs a new variant before decad accepts a profile that uses it`, ErrUnrecordableProfile, e.Entity)
	}
}

// falsifyTol is the reject threshold of the falsifier, relative to the
// fragment's own coordinate scale. TExact's stated meaning is reproduction to
// machine precision, so an honest flag leaves a residual within round-off
// (~1e-13 relative); a flag worth disproving misses by the sampling error it
// hid. 1e-9 sits between the two — far above round-off, far below any
// sampling-scale miss — so the falsifier can reject a lie without ever
// false-rejecting an exact cut.
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
	scale := 1.0
	for _, m := range []float64{math.Abs(x), math.Abs(y), math.Abs(obs[0]), math.Abs(obs[1])} {
		scale = math.Max(scale, m)
	}
	if math.Hypot(x-obs[0], y-obs[1]) > falsifyTol*scale {
		return fmt.Errorf(`%w: the certified range on a %T is disproven — eval(%v) = (%v, %v) does not reproduce the fragment endpoint (%v, %v); report upstream as a sketch bug`,
			ErrUnrecordableProfile, e.Entity, t, x, y, obs[0], obs[1])
	}
	return nil
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

// point2 converts a plane-local geom point to its record form.
func point2(p *geom.Point) Point2 { return Point2{U: p.X, V: p.Y} }

// points2 converts a slice of plane-local geom points to record form.
func points2(ps []*geom.Point) []Point2 {
	out := make([]Point2, len(ps))
	for i, p := range ps {
		out[i] = point2(p)
	}
	return out
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
