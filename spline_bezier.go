package decad

import (
	"github.com/lestrrat-3d/decad/internal/curveconvert"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentvalidate"
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

func point2Of(point survey2d.RatPoint) (Point2, bool) { return curveconvert.Point2Of(point) }

func reconstructionOf(record ProfileRecord) freeform.FreeformReconstruction {
	return momentvalidate.ReconstructionOf(momentProfile(record))
}

func chargeReconstruction(record ProfileRecord, work *freeform.FreeformWork) (uint64, error) {
	return momentvalidate.ChargeReconstruction(momentProfile(record), work)
}

func reconstructionChords(segment CurveSegment) uint64 {
	return momentvalidate.ReconstructionChords(segment)
}
