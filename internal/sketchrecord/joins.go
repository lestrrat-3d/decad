package sketchrecord

import (
	"fmt"

	"github.com/lestrrat-3d/sketch"
)

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
