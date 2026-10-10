package decad

import (
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// Structural curve records stay in internal packages. Their aliases keep the
// evaluator's local names short without adding them to decad's public API.

type planeRecord = sectionrecord.PlaneRecord

// Point2 is a plane-local coordinate in millimetres.
type Point2 struct {
	U float64 `json:"u"`
	V float64 `json:"v"`
}

func point2FromRecordSlice(points []sectionrecord.Point2) []Point2 {
	if points == nil {
		return nil
	}
	out := make([]Point2, len(points))
	for i, point := range points {
		out[i] = point
	}
	return out
}

func point2ToRecordSlice(points []Point2) []sectionrecord.Point2 {
	if points == nil {
		return nil
	}
	out := make([]sectionrecord.Point2, len(points))
	for i, point := range points {
		out[i] = point
	}
	return out
}

type profileRecord = momentinput.Profile

type loopRecord = sectionrecord.LoopRecord

type chainRecord = sectionrecord.ChainRecord

type curveSegment = sectionrecord.CurveSegment

type lineSeg = sectionrecord.LineSeg

type circleSeg = sectionrecord.CircleSeg

type arcSeg = sectionrecord.ArcSeg

type ellipseSeg = sectionrecord.EllipseSeg

type ellipticalArcSeg = sectionrecord.EllipticalArcSeg

type splineSeg = sectionrecord.SplineSeg

type nurbsSeg = sectionrecord.NURBSSeg

type closedSplineSeg = sectionrecord.ClosedSplineSeg

type fitSplineSeg = sectionrecord.FitSplineSeg

func cloneLoopRecord(loop loopRecord) loopRecord  { return sectionrecord.CloneLoopRecord(loop) }
func validateNURBSSegment(segment nurbsSeg) error { return sectionrecord.ValidateNURBSSegment(segment) }
func validateSegment(segment curveSegment) error  { return sectionrecord.ValidateSegment(segment) }
func normalizeSegment(segment curveSegment) (curveSegment, error) {
	return sectionrecord.NormalizeSegment(segment)
}
