package circularmoments

import "github.com/lestrrat-3d/decad/internal/record"

// RecordSegment transfers the recorded fields used by circular proofs.
func RecordSegment(segment record.CurveSegment) CurveSegment {
	switch segment := segment.(type) {
	case record.CircleSeg:
		return CircleSeg{
			Center: RecordPoint(segment.Center), Radius: segment.Radius, CCW: segment.CCW,
			TStart: segment.TStart, TEnd: segment.TEnd,
		}
	case record.ArcSeg:
		return ArcSeg{
			Center: RecordPoint(segment.Center), Start: RecordPoint(segment.Start), End: RecordPoint(segment.End),
			TStart: segment.TStart, TEnd: segment.TEnd,
		}
	default:
		return nil
	}
}

// RecordPoint transfers a recorded plane-local point to a circular proof.
func RecordPoint(point record.Point2) Point2 { return Point2{U: point.U, V: point.V} }
