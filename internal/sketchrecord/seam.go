package sketchrecord

import (
	"fmt"
	"math"
	"slices"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/sketch/geom"
	"github.com/lestrrat-3d/units"
)

type (
	Point2           = sectionrecord.Point2
	LoopRecord       = sectionrecord.LoopRecord
	CurveSegment     = sectionrecord.CurveSegment
	LineSeg          = sectionrecord.LineSeg
	CircleSeg        = sectionrecord.CircleSeg
	ArcSeg           = sectionrecord.ArcSeg
	EllipseSeg       = sectionrecord.EllipseSeg
	EllipticalArcSeg = sectionrecord.EllipticalArcSeg
	ConicSeg         = sectionrecord.ConicSeg
	SplineSeg        = sectionrecord.SplineSeg
	ClosedSplineSeg  = sectionrecord.ClosedSplineSeg
	FitSplineSeg     = sectionrecord.FitSplineSeg
	NURBSSeg         = sectionrecord.NURBSSeg
)

var ErrUnrecordableProfile = decaderr.ErrUnrecordableProfile

// These entry points let the root package reuse the record seam for both
// public recording and its private boolean and surface operations.
func RecordChainSegments(edges []sketch.BoundaryEdge) ([]CurveSegment, error) {
	return recordChainSegments(edges)
}
func RecordLoop(name string, edges []sketch.BoundaryEdge) (LoopRecord, error) {
	return recordLoop(name, edges)
}
func RecordEdge(edge sketch.BoundaryEdge) (CurveSegment, error) { return recordEdge(edge) }
func EdgeJoin(edge sketch.BoundaryEdge, segment CurveSegment) (LoopJoin, error) {
	return edgeJoin(edge, segment)
}
func FalsifyLoopJoins(name string, joins []LoopJoin) error { return falsifyLoopJoins(name, joins) }
func FalsifyChainJoins(joins []LoopJoin) error             { return falsifyChainJoins(joins) }

// recordChainSegments converts one authenticated chain's walk, edge by edge,
// in walk order, then disproves a walk whose recorded segments do not join at
// an INTERIOR junction (falsifyChainJoins) — the chain's own counterpart of
// recordLoop, which additionally checks the junction that closes a loop. A
// chain's two free ends state no junction to check at all.
func recordChainSegments(edges []sketch.BoundaryEdge) ([]CurveSegment, error) {
	segs := make([]CurveSegment, 0, len(edges))
	joins := make([]LoopJoin, 0, len(edges))
	for i, e := range edges {
		seg, err := recordEdge(e)
		if err != nil {
			return nil, fmt.Errorf("chain edge %d: %w", i, err)
		}
		join, err := edgeJoin(e, seg)
		if err != nil {
			return nil, fmt.Errorf("chain edge %d: %w", i, err)
		}
		segs = append(segs, seg)
		joins = append(joins, join)
	}
	if err := falsifyChainJoins(joins); err != nil {
		return nil, err
	}
	return segs, nil
}

// falsifyChainJoins is falsifyLoopJoins restricted to a chain's INTERIOR
// junctions: junction i, between edge i and edge i+1, for every i but the
// last (docs/sketch-seam-design.md §2.2). A chain of one segment has no
// interior junction and this is a no-op for it. It only ever rejects, on the
// identical source-aware rule falsifyLoopJoins applies.
func falsifyChainJoins(joins []LoopJoin) error {
	for i := 0; i+1 < len(joins); i++ {
		from, to := joins[i], joins[i+1]
		if from.closed || to.closed {
			// A whole closed curve never reaches a chain (docs/sketch-seam-design.md
			// §2.2), but the guard is kept for the same reason falsifyLoopJoins
			// keeps it: a defensive no-op rather than an assumption.
			continue
		}
		if loopJoinPointsAgree(from.end, to.start) {
			continue
		}
		return fmt.Errorf(
			`%w: chain edge %d ends at (%v, %v) but edge %d starts at (%v, %v), so the recorded chain does not join; sketch admitted the walk on its own proximity threshold, and decad records no walk its own segments do not bound`,
			ErrUnrecordableProfile, i, from.end.point.U, from.end.point.V, i+1, to.start.point.U, to.start.point.V,
		)
	}
	return nil
}

// recordLoop converts one named boundary loop, edge by edge, in walk order,
// then disproves a loop whose recorded segments do not join (falsifyLoopJoins).
func recordLoop(name string, edges []sketch.BoundaryEdge) (LoopRecord, error) {
	segs := make([]CurveSegment, 0, len(edges))
	joins := make([]LoopJoin, 0, len(edges))
	for i, e := range edges {
		seg, err := recordEdge(e)
		if err != nil {
			return LoopRecord{}, fmt.Errorf("%s edge %d: %w", name, i, err)
		}
		join, err := edgeJoin(e, seg)
		if err != nil {
			return LoopRecord{}, fmt.Errorf("%s edge %d: %w", name, i, err)
		}
		segs = append(segs, seg)
		joins = append(joins, join)
	}
	if err := falsifyLoopJoins(name, joins); err != nil {
		return LoopRecord{}, err
	}
	return LoopRecord{Segments: segs}, nil
}

// LoopJoin is one recorded edge's junction coordinates: the point the previous
// segment's walk must end at, and the point the next segment's walk must start
// from. closed marks a whole closed curve — a circle, an ellipse, a closed
// spline — which is a LoopRecord on its own (docs/sketch-seam-design.md §2) and
// so meets no neighbour at all.
type LoopJoin struct {
	start, end loopJoinPoint
	closed     bool
}

// loopJoinPoint carries both a junction coordinate and the observation that
// supplied it. A record endpoint and a sketch node can describe the same
// vertex without sharing every floating-point bit: a curve's node is evaluated
// from its parameter, while a whole edge records its defining point verbatim.
type loopJoinPoint struct {
	point  Point2
	source loopJoinSource
}

type loopJoinSource uint8

const (
	recordJoinSource loopJoinSource = iota
	sketchNodeJoinSource
	// walkJoinSource is an endpoint the evaluator computed from a recorded
	// segment — a narrowed line's walked end. Two such points, or one beside a
	// record endpoint, come from different arithmetic, so they compare with
	// the range falsifier's tolerance rather than bit for bit.
	walkJoinSource
)

// RecordedJoin states one segment of a loop decad assembled itself from
// recorded segments, with no sketch arrangement behind it: start and end are
// the segment's walk ends, computedStart and computedEnd mark an end the
// evaluator computed (a narrowed line's walked end) rather than one the
// record states, and closed marks a whole closed curve. A junction of two
// stated ends compares exactly; a junction with a computed end compares with
// the range falsifier's tolerance. FalsifyLoopJoins only ever rejects.
func RecordedJoin(start, end Point2, computedStart, computedEnd, closed bool) LoopJoin {
	source := func(computed bool) loopJoinSource {
		if computed {
			return walkJoinSource
		}
		return recordJoinSource
	}
	return LoopJoin{
		start:  loopJoinPoint{point: start, source: source(computedStart)},
		end:    loopJoinPoint{point: end, source: source(computedEnd)},
		closed: closed,
	}
}

// edgeJoin reads one boundary edge's junction coordinates, taking each side
// from the party that owns it.
//
// A whole edge's junction points are the RECORD's own — the entity's defining
// points, verbatim, exactly the values the segment carries. That is what makes
// the check below answerable: the recorded loop either states two neighbours at
// the same coordinate or it does not, and nothing is evaluated, projected or
// solved for.
//
// A Partial fragment supplies sketch's Polyline endpoint only at a genuinely
// cut bound. At TStart == 0 or TEnd == 1, its natural endpoint is already in
// the record, so that record coordinate supplies the join instead. The
// Polyline observations remain checks only: decad does not recompute a cut it
// is forbidden to re-derive (core §7), and no Polyline point enters the record
// (§2). A whole edge's Polyline is never read at all.
func edgeJoin(e sketch.BoundaryEdge, seg CurveSegment) (LoopJoin, error) {
	if e.Partial {
		if len(e.Polyline) < 2 {
			return LoopJoin{}, fmt.Errorf(`%w: a %T fragment carries no polyline endpoints to check its junctions against`, ErrUnrecordableProfile, e.Entity)
		}
		first, last := e.Polyline[0], e.Polyline[len(e.Polyline)-1]
		atStart := loopJoinPoint{
			point:  Point2{U: first[0], V: first[1]},
			source: sketchNodeJoinSource,
		}
		atEnd := loopJoinPoint{
			point:  Point2{U: last[0], V: last[1]},
			source: sketchNodeJoinSource,
		}

		// The polyline holds sketch's snapped arrangement node. A bound sketch
		// never cut already has its exact natural endpoint in the record, so
		// use that coordinate for closure. The range is natural while the
		// polyline is in walk order, so Reversed maps the natural endpoints to
		// the opposite walk endpoints.
		if naturalStart, naturalEnd, closed, ok := wholeSegmentEnds(seg); ok && !closed {
			if e.TStart == 0 {
				join := loopJoinPoint{point: naturalStart, source: recordJoinSource}
				if e.Reversed {
					atEnd = join
				} else {
					atStart = join
				}
			}
			if e.TEnd == 1 {
				join := loopJoinPoint{point: naturalEnd, source: recordJoinSource}
				if e.Reversed {
					atStart = join
				} else {
					atEnd = join
				}
			}
		}
		return LoopJoin{
			start: atStart,
			end:   atEnd,
		}, nil
	}
	start, end, closed, ok := wholeSegmentEnds(seg)
	if !ok {
		return LoopJoin{}, fmt.Errorf(`%w: a whole %T edge records as a %T, whose walk endpoints this seam cannot name`, ErrUnrecordableProfile, e.Entity, seg)
	}
	if e.Reversed {
		start, end = end, start
	}
	return LoopJoin{
		start:  loopJoinPoint{point: start, source: recordJoinSource},
		end:    loopJoinPoint{point: end, source: recordJoinSource},
		closed: closed,
	}, nil
}

// wholeSegmentEnds returns a whole edge's endpoints in the curve's NATURAL
// direction, read straight off the recorded segment. Every open variant states
// its own ends: the analytic four pin them as fields, a clamped spline or NURBS
// interpolates its first and last control point exactly, and a fit spline
// interpolates its first and last fit point. The three closed kinds state no
// ends — each bounds a region on its own — and report closed instead.
func wholeSegmentEnds(seg CurveSegment) (start, end Point2, closed, ok bool) {
	ends := func(points []Point2) (Point2, Point2, bool, bool) {
		if len(points) == 0 {
			return Point2{}, Point2{}, false, false
		}
		return points[0], points[len(points)-1], false, true
	}
	switch seg := seg.(type) {
	case LineSeg:
		return seg.Start, seg.End, false, true
	case ArcSeg:
		return seg.Start, seg.End, false, true
	case EllipticalArcSeg:
		return seg.Start, seg.End, false, true
	case ConicSeg:
		return seg.Start, seg.End, false, true
	case SplineSeg:
		return ends(seg.Control)
	case NURBSSeg:
		return ends(seg.Control)
	case FitSplineSeg:
		return ends(seg.Fit)
	case CircleSeg, EllipseSeg, ClosedSplineSeg:
		return Point2{}, Point2{}, true, true
	default:
		return Point2{}, Point2{}, false, false
	}
}

// falsifyLoopJoins disproves a recorded loop that does not close
// (docs/sketch-seam-design.md §3). LoopRecord's contract is that each segment's
// walk ends where the next one's starts and the last closes onto the first
// (§2). A same-source mismatch proves the record's own segments do not meet. A
// mixed-source mismatch beyond the range falsifier's tolerance contradicts a
// certified cut node. In either case the profile is ErrUnrecordableProfile.
//
// A junction whose points came from the same source compares exactly. That
// retains the authored-gap check for whole edges and checks two partial
// fragments against their shared source observation. A mixed junction at a cut
// bound compares a record endpoint to the fragment's sketch node with the same
// relative tolerance that falsifyRange applies to that node. A certified curve
// node is evaluated from its parameter, so it can differ from the defining
// endpoint by round-off even when both describe the same sketch vertex.
//
// It only ever rejects. A loop whose recorded coordinates do meet is not
// thereby admitted — admission is sketch's Valid, its TExact, and §1's range
// falsifier, exactly as before.
func falsifyLoopJoins(name string, joins []LoopJoin) error {
	for i, from := range joins {
		next := (i + 1) % len(joins)
		to := joins[next]
		if from.closed || to.closed {
			// A whole closed curve is a loop on its own (§2): it meets no
			// neighbour, so this pair states no junction to disprove.
			continue
		}
		if loopJoinPointsAgree(from.end, to.start) {
			continue
		}
		return fmt.Errorf(
			`%w: %s edge %d ends at (%v, %v) but edge %d starts at (%v, %v), so the recorded loop does not close; sketch admitted the region on its own proximity threshold, and decad records no region its own segments do not bound`,
			ErrUnrecordableProfile, name, i, from.end.point.U, from.end.point.V, next, to.start.point.U, to.start.point.V,
		)
	}
	return nil
}

// loopJoinPointsAgree compares two points from one source exactly. A mixed
// source pair compares the record's defining point to the sketch cut node that
// a certified fragment already exposed to falsifyRange, so it shares that
// check's relative tolerance. Two walked ends come from two different
// carriers' arithmetic and take the same tolerance.
func loopJoinPointsAgree(a, b loopJoinPoint) bool {
	if a.source == b.source && a.source != walkJoinSource {
		return a.point == b.point
	}
	return pointsWithinFalsifyTolerance(a.point, b.point)
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
			return nil, fmt.Errorf(`%w: a %T fragment has an uncertified trim (TExact = false); an uncertified range is never recorded as an exact trim`, ErrUnrecordableProfile, e.Entity)
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
