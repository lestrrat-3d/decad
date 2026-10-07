package decad

import (
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/splinebezier"
)

func isFreeformSegment(segment CurveSegment) bool { return splinebezier.IsFreeformSegment(segment) }

func freeformBezierSpans(segment CurveSegment, work *freeform.FreeformWork) ([]freeform.BezierSpan, bool, error) {
	return splinebezier.FreeformBezierSpans(segment, work)
}

func ratPointsOf(points []Point2) ([]freeform.RatPoint, error) {
	return splinebezier.RatPointsOf(points)
}

func splineBezierSpans(segment SplineSeg, work *freeform.FreeformWork) ([]freeform.BezierSpan, error) {
	return splinebezier.SplineBezierSpans(segment, work)
}

func nurbsBezierSpans(segment NURBSSeg, work *freeform.FreeformWork) ([]freeform.BezierSpan, error) {
	return splinebezier.NURBSBezierSpans(segment, work)
}

func closedSplineBezierSpans(segment ClosedSplineSeg, work *freeform.FreeformWork) ([]freeform.BezierSpan, error) {
	return splinebezier.ClosedSplineBezierSpans(segment, work)
}

func shiftFreeformSpans(spans []freeform.BezierSpan, anchor Point2) error {
	return splinebezier.ShiftFreeformSpans(spans, anchor)
}

func freeformEndpoints(spans []freeform.BezierSpan, reversed bool) (Point2, Point2, error) {
	return splinebezier.FreeformEndpoints(spans, reversed)
}

func point2Of(point freeform.RatPoint) (Point2, bool) { return splinebezier.Point2Of(point) }

func reconstructionOf(record ProfileRecord) freeform.FreeformReconstruction {
	return momentinput.ReconstructionOf(momentProfile(record))
}

func chargeReconstruction(record ProfileRecord, work *freeform.FreeformWork) (uint64, error) {
	return momentinput.ChargeReconstruction(momentProfile(record), work)
}

func reconstructionChords(segment CurveSegment) uint64 {
	return momentinput.ReconstructionChords(segment)
}
