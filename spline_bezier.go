package decad

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/curveconvert"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

func isFreeformSegment(segment CurveSegment) bool { return curveconvert.IsFreeformSegment(segment) }

func freeformBezierSpans(segment CurveSegment, work *freeform.FreeformWork) ([]survey2d.BezierSpan, bool, error) {
	return curveconvert.FreeformBezierSpans(segment, work)
}

func ratPointsOf(points []Point2) ([]survey2d.RatPoint, error) {
	return curveconvert.RatPointsOf(points)
}

func splineBezierSpans(segment SplineSeg, work *freeform.FreeformWork) ([]survey2d.BezierSpan, error) {
	return curveconvert.SplineBezierSpans(segment, work)
}

func nurbsBezierSpans(segment NURBSSeg, work *freeform.FreeformWork) ([]survey2d.BezierSpan, error) {
	return curveconvert.NURBSBezierSpans(segment, work)
}

func closedSplineBezierSpans(segment ClosedSplineSeg, work *freeform.FreeformWork) ([]survey2d.BezierSpan, error) {
	return curveconvert.ClosedSplineBezierSpans(segment, work)
}

func shiftFreeformSpans(spans []survey2d.BezierSpan, anchor Point2) error {
	return curveconvert.ShiftFreeformSpans(spans, anchor)
}

func freeformEndpoints(spans []survey2d.BezierSpan, reversed bool) (Point2, Point2, error) {
	return curveconvert.FreeformEndpoints(spans, reversed)
}

func freeformEndpointBounds(spans []survey2d.BezierSpan, reversed bool, start, end Point2) (proofbound.WalkEndBound, proofbound.WalkEndBound) {
	return curveconvert.FreeformEndpointBounds(spans, reversed, start, end)
}

func point2Of(point survey2d.RatPoint) (Point2, bool) { return curveconvert.Point2Of(point) }

// reconstructionOf reads that model off a checked record. Every segment counts,
// whatever its kind: the chord total is the arrangement's own element count, and
// the arrangement is global.
//
// An entity SEVERAL segments name counts once, because momentRecordScene interns
// the entities it builds and the arrangement therefore holds one set of chords
// per distinct entity. That is the ordinary shape of a recorded region — one
// crossing cuts a circle into two fragments, and both name the same circle — so
// counting per fragment squares a chord total the scene never holds and refuses
// records whose reconstruction costs milliseconds.
//
// Only the kinds momentRecordScene keys on a fixed-size struct are interned
// here. A free-form key renders every control point into a string
// (freeformEntityKey), which is a per-element pass and an allocation this charge
// exists to precede, so free-form segments stay counted per fragment. That is
// conservative in the safe direction: an over-count of the scene, never an
// under-count of it.
func reconstructionOf(record ProfileRecord) freeform.FreeformReconstruction {
	var chords uint64
	var seen map[momentEntityKey]struct{}
	for _, loop := range append([]LoopRecord{record.Outer}, record.Holes...) {
		for _, segment := range loop.Segments {
			if chords > freeform.ReconstructionChordCeiling {
				break
			}
			if key, keyed := analyticEntityKey(segment); keyed {
				if _, duplicate := seen[key]; duplicate {
					continue
				}
				if seen == nil {
					seen = make(map[momentEntityKey]struct{})
				}
				seen[key] = struct{}{}
			}
			chords = freeform.ReconstructionCostAdd(chords, reconstructionChords(segment))
		}
	}
	return freeform.FreeformReconstruction{Chords: chords, Arrangement: freeform.ReconstructionCostMul(chords, chords)}
}

// chargeReconstruction levies the record-level part of that charge — the
// scene arrangement the validation runs to list its candidate profiles, and the
// one its rescaled retry runs — and returns the per-arrangement charge the
// candidate loop then levies for itself.
func chargeReconstruction(record ProfileRecord, work *freeform.FreeformWork) (uint64, error) {
	demand := reconstructionOf(record)
	if err := work.ReconstructionStep(freeform.ReconstructionCostMul(2, demand.Arrangement)); err != nil {
		return 0, err
	}
	return demand.Arrangement, nil
}

// reconstructionChords is how many polyline chords sketch will create for the
// entity one recorded segment names. It is sampleParams's own table, keyed on
// the kind: a line is its own single chord, a free-form curve is chorded per
// control point over a floor, and everything curved and analytic is chorded per
// turn.
func reconstructionChords(segment CurveSegment) uint64 {
	switch segment := segment.(type) {
	case LineSeg:
		return 1
	case ArcSeg:
		return arcChords(segment)
	case SplineSeg:
		// An open cubic B-spline is sampled per SPAN, so the count comes off the
		// control count less the degree — the one free-form kind whose sample
		// count is not simply its control count.
		return freeform.FreeformChords(len(segment.Control) - 3)
	case ClosedSplineSeg:
		return freeform.FreeformChords(len(segment.Control))
	case NURBSSeg:
		return freeform.FreeformChords(len(segment.Control))
	case FitSplineSeg:
		return freeform.FreeformChords(len(segment.Fit))
	default:
		// A circle, an ellipse, a conic and an elliptical arc all reach the same
		// per-turn branch. A whole turn is its own upper bound, and a partial
		// sweep is charged for a whole one rather than re-deriving a sweep the
		// record states as no field of its own.
		return freeform.AnalyticChordsPerTurn
	}
}

// arcChords is the per-turn count scaled by the arc's own sweep — an ArcSeg is
// swept counter-clockwise from Start to End about Center, which is the sweep
// sketch derives from the same three pinned points. A sweep that is not a
// positive angle inside one turn is charged a whole turn, which the sampler
// itself cannot exceed.
func arcChords(seg ArcSeg) uint64 {
	sweep := math.Atan2(seg.End.V-seg.Center.V, seg.End.U-seg.Center.U) -
		math.Atan2(seg.Start.V-seg.Center.V, seg.Start.U-seg.Center.U)
	if sweep <= 0 {
		sweep += 2 * math.Pi
	}
	if !(sweep > 0) || sweep > 2*math.Pi {
		return freeform.AnalyticChordsPerTurn
	}
	chords := uint64(math.Ceil(freeform.AnalyticChordsPerTurn * sweep / (2 * math.Pi)))
	if chords < 2 {
		return 2
	}
	return chords
}
