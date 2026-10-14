package decad

import (
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// Structural curve records stay in internal packages. Their aliases keep the
// evaluator's local names short without adding them to decad's public API.

type planeRecord = sectionrecord.PlaneRecord

// planePoint is the evaluator's plane-local coordinate. Point2 is its public
// spelling; both share one underlying struct, so a value assigns either way.
type planePoint = sectionrecord.Point2

// Point2 is a plane-local coordinate in millimetres.
type Point2 struct {
	U float64 `json:"u"`
	V float64 `json:"v"`
}

func point2FromRecordSlice(points []sectionrecord.Point2) []planePoint {
	if points == nil {
		return nil
	}
	out := make([]planePoint, len(points))
	copy(out, points)
	return out
}

func point2ToRecordSlice(points []planePoint) []sectionrecord.Point2 {
	if points == nil {
		return nil
	}
	out := make([]sectionrecord.Point2, len(points))
	copy(out, points)
	return out
}

type profileRecord = momentinput.Profile

// prismPayload is the evaluator's own record of a straight extrude: the
// internal/prismrecord payload, which prismrecord.Build evaluates over
// topology nodes and which implements topo.Payload (docs/evaluator-design.md
// §5, §8).
type prismPayload = prismrecord.Payload

type loopRecord = sectionrecord.LoopRecord

type chainRecord = sectionrecord.ChainRecord

type curveSegment = sectionrecord.CurveSegment

type lineSeg = sectionrecord.LineSeg

type circleSeg = sectionrecord.CircleSeg

type arcSeg = sectionrecord.ArcSeg

type fitSplineSeg = sectionrecord.FitSplineSeg

func cloneLoopRecord(loop loopRecord) loopRecord { return sectionrecord.CloneLoopRecord(loop) }
func normalizeSegment(segment curveSegment) (curveSegment, error) {
	return sectionrecord.NormalizeSegment(segment)
}
