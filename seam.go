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
// Admission consumes only a fresh snapshot authenticated against sketch's own
// answers (docs/api-design.md §7): p must name s (Profile.Sketch, else
// [ErrForeignProfile]), be current (Profile.IsStale, else [ErrStaleProfile]),
// and exactly match one fresh result from s.Profiles. Every boundary entity
// must be non-nil and owned by s. A caller-altered snapshot is
// [ErrInvalidProfile], and a foreign boundary entity is [ErrForeignProfile].
// A profile whose Valid is false is also never silently swept
// ([ErrInvalidProfile]). The one further rejection is not a validity judgement:
// an authenticated valid profile whose boundary cannot be recorded exactly —
// a Partial fragment sketch could not certify
// (BoundaryEdge.TExact == false), a certified range the seam's reject-only
// falsifier disproves, or a loop whose source-aware junction check finds a
// contradiction — is [ErrUnrecordableProfile]. decad never repairs, projects
// or fits a point sketch handed over, and it never solves for one.
//
// That last rejection is the one a caller meets by drawing rather than by
// hitting an upstream limit. sketch admits a region on its own proximity
// threshold, so entities whose ends miss by a fraction of a nanometre still
// arrange into one valid profile. At a whole-to-whole join, decad records each
// entity's own points verbatim, so a loop whose recorded coordinates do not
// meet bounds no region. A certified mixed join at a genuinely cut bound
// instead compares the record endpoint with sketch's evaluated node at the
// range falsifier's relative tolerance. An uncut Partial bound uses the
// record's own endpoint, so it remains an exact check. Ends merely driven
// together by a coincidence constraint remain a whole-to-whole exact check:
// the solver converges to within its residual, not to the same coordinate.
func RecordProfile(s *sketch.Sketch, p *sketch.Profile) (ProfileRecord, PlaneRecord, error) {
	profile, plane, _, err := recordProfile(s, p)
	return profile, plane, err
}

// recordProfile returns the authenticated profile's area with its structural
// record so feature callers never read a caller-mutable field after admission.
func recordProfile(s *sketch.Sketch, p *sketch.Profile) (ProfileRecord, PlaneRecord, float64, error) {
	if s == nil || p == nil {
		return ProfileRecord{}, PlaneRecord{}, 0, fmt.Errorf(`%w: RecordProfile requires a sketch and a profile`, ErrDegenerate)
	}
	if p.Sketch() != s {
		return ProfileRecord{}, PlaneRecord{}, 0, fmt.Errorf(`%w: the profile was built from a different sketch, so its plane-local coordinates are another plane's`, ErrForeignProfile)
	}
	if p.IsStale() {
		return ProfileRecord{}, PlaneRecord{}, 0, fmt.Errorf(`%w: the sketch has changed since this profile was built; rebuild with Sketch.Profiles`, ErrStaleProfile)
	}
	if !p.Valid {
		return ProfileRecord{}, PlaneRecord{}, 0, fmt.Errorf(`%w: a self-intersecting or degenerate region is never silently swept`, ErrInvalidProfile)
	}

	trusted, err := authenticateProfile(s, p)
	if err != nil {
		return ProfileRecord{}, PlaneRecord{}, 0, err
	}

	frame, err := s.Plane().Frame()
	if err != nil {
		return ProfileRecord{}, PlaneRecord{}, 0, fmt.Errorf(`decad: failed to resolve the sketch plane: %w`, err)
	}
	plane := PlaneRecord{Origin: frame.Origin(), U: frame.U(), V: frame.V()}

	outer, err := recordLoop("outer", trusted.Outer)
	if err != nil {
		return ProfileRecord{}, PlaneRecord{}, 0, err
	}
	var holes []LoopRecord
	for i, h := range trusted.Holes {
		loop, err := recordLoop(fmt.Sprintf("hole %d", i), h)
		if err != nil {
			return ProfileRecord{}, PlaneRecord{}, 0, err
		}
		holes = append(holes, loop)
	}
	return ProfileRecord{Outer: outer, Holes: holes}, plane, trusted.Area, nil
}

// authenticateProfile rejects caller changes to the exported Profile fields
// and returns a fresh snapshot built by sketch. Boundary ownership is checked
// first so a foreign or typed-nil entity never reaches Geometry.
func authenticateProfile(s *sketch.Sketch, p *sketch.Profile) (*sketch.Profile, error) {
	owned := make(map[sketch.Entity]struct{})
	for _, ent := range s.Entities() {
		owned[ent] = struct{}{}
	}
	if err := authenticateBoundaryLoop(owned, p.Outer); err != nil {
		return nil, err
	}
	for _, hole := range p.Holes {
		if err := authenticateBoundaryLoop(owned, hole); err != nil {
			return nil, err
		}
	}

	var trusted *sketch.Profile
	for _, candidate := range s.Profiles() {
		if !sameProfileSnapshot(p, candidate) {
			continue
		}
		if trusted != nil {
			return nil, fmt.Errorf(`%w: the profile snapshot matches more than one current region; rebuild with Sketch.Profiles`, ErrInvalidProfile)
		}
		trusted = candidate
	}
	if trusted == nil {
		return nil, fmt.Errorf(`%w: the profile snapshot was altered after Sketch.Profiles returned it; rebuild and pass the profile unchanged`, ErrInvalidProfile)
	}
	return trusted, nil
}

func authenticateBoundaryLoop(owned map[sketch.Entity]struct{}, edges []sketch.BoundaryEdge) error {
	for _, edge := range edges {
		if isNilSketchEntity(edge.Entity) {
			return fmt.Errorf(`%w: the profile boundary contains a nil entity; rebuild with Sketch.Profiles`, ErrInvalidProfile)
		}
		if _, ok := owned[edge.Entity]; !ok {
			return fmt.Errorf(`%w: the profile boundary contains an entity not owned by its source sketch`, ErrForeignProfile)
		}
	}
	return nil
}

func isNilSketchEntity(ent sketch.Entity) bool {
	switch ent := ent.(type) {
	case nil:
		return true
	case *sketch.Line:
		return ent == nil
	case *sketch.Circle:
		return ent == nil
	case *sketch.Arc:
		return ent == nil
	case *sketch.Ellipse:
		return ent == nil
	case *sketch.EllipticalArc:
		return ent == nil
	case *sketch.Conic:
		return ent == nil
	case *sketch.Spline:
		return ent == nil
	case *sketch.ClosedSpline:
		return ent == nil
	case *sketch.FitSpline:
		return ent == nil
	case *sketch.NURBS:
		return ent == nil
	default:
		return false
	}
}

func sameProfileSnapshot(a, b *sketch.Profile) bool {
	if a.Sketch() != b.Sketch() || a.Revision() != b.Revision() ||
		a.Area != b.Area || a.Valid != b.Valid || a.SelfIntersecting != b.SelfIntersecting ||
		(a.Entities == nil) != (b.Entities == nil) || !slices.Equal(a.Entities, b.Entities) ||
		(a.Holes == nil) != (b.Holes == nil) || !sameBoundaryLoop(a.Outer, b.Outer) ||
		len(a.Holes) != len(b.Holes) {
		return false
	}
	for i := range a.Holes {
		if !sameBoundaryLoop(a.Holes[i], b.Holes[i]) {
			return false
		}
	}
	return true
}

func sameBoundaryLoop(a, b []sketch.BoundaryEdge) bool {
	if (a == nil) != (b == nil) || len(a) != len(b) {
		return false
	}
	for i := range a {
		if !sameBoundaryEdge(a[i], b[i]) {
			return false
		}
	}
	return true
}

func sameBoundaryEdge(a, b sketch.BoundaryEdge) bool {
	return a.Entity == b.Entity &&
		a.Partial == b.Partial &&
		a.Reversed == b.Reversed &&
		a.TStart == b.TStart &&
		a.TEnd == b.TEnd &&
		a.TExact == b.TExact &&
		(a.Polyline == nil) == (b.Polyline == nil) &&
		slices.Equal(a.Polyline, b.Polyline)
}

// recordLoop converts one named boundary loop, edge by edge, in walk order,
// then disproves a loop whose recorded segments do not join (falsifyLoopJoins).
func recordLoop(name string, edges []sketch.BoundaryEdge) (LoopRecord, error) {
	segs := make([]CurveSegment, 0, len(edges))
	joins := make([]loopJoin, 0, len(edges))
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

// loopJoin is one recorded edge's junction coordinates: the point the previous
// segment's walk must end at, and the point the next segment's walk must start
// from. closed marks a whole closed curve — a circle, an ellipse, a closed
// spline — which is a LoopRecord on its own (docs/sketch-seam-design.md §2) and
// so meets no neighbour at all.
type loopJoin struct {
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
)

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
func edgeJoin(e sketch.BoundaryEdge, seg CurveSegment) (loopJoin, error) {
	if e.Partial {
		if len(e.Polyline) < 2 {
			return loopJoin{}, fmt.Errorf(`%w: a %T fragment carries no polyline endpoints to check its junctions against`, ErrUnrecordableProfile, e.Entity)
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
		// use that coordinate for closure. The natural-to-walk mapping matches
		// falsifyRange's observation reorder below.
		if naturalStart, naturalEnd, closed, ok := wholeSegmentEnds(seg); ok && !closed {
			if e.Reversed {
				naturalStart, naturalEnd = naturalEnd, naturalStart
			}
			if e.TStart == 0 {
				atStart = loopJoinPoint{point: naturalStart, source: recordJoinSource}
			}
			if e.TEnd == 1 {
				atEnd = loopJoinPoint{point: naturalEnd, source: recordJoinSource}
			}
		}
		return loopJoin{
			start: atStart,
			end:   atEnd,
		}, nil
	}
	start, end, closed, ok := wholeSegmentEnds(seg)
	if !ok {
		return loopJoin{}, fmt.Errorf(`%w: a whole %T edge records as a %T, whose walk endpoints this seam cannot name`, ErrUnrecordableProfile, e.Entity, seg)
	}
	if e.Reversed {
		start, end = end, start
	}
	return loopJoin{
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
func falsifyLoopJoins(name string, joins []loopJoin) error {
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
// check's relative tolerance.
func loopJoinPointsAgree(a, b loopJoinPoint) bool {
	if a.source == b.source {
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
